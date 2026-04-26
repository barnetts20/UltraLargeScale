#include "FVolumeTextureUtils.h"
#include "FOctree.h"

TArray<uint8> FVolumeTextureUtils::GenerateVolumeMipDataFromOctree(TArray<TSharedPtr<FOctreeNode>> InVolumeNodes, int InResolution, double InExtent, double InMaxDensity)
{
	double StartTime = FPlatformTime::Seconds();

	// --- Octree + setup ---
	const int TargetDepth = FMath::FloorLog2(InResolution);
	const double NodeExtentAtDepth = InExtent / FMath::Pow(2.0, TargetDepth);
	const double OctreeExtent = InExtent;

	// Precompute reciprocal for fast coordinate mapping
	const double Scale = 1.0 / (2.0 * NodeExtentAtDepth);

	// --- Texture allocation ---
	const int BytesPerVoxel = 4; // BGRA8
	const int64 TotalVoxels = (int64)InResolution * InResolution * InResolution;
	const int64 TotalBytes = TotalVoxels * BytesPerVoxel;

	TArray<uint8> TextureData;
	TextureData.SetNumZeroed(TotalBytes);

	// --- Parallel fill ---
	// Use chunked ParallelFor to reduce scheduling overhead
	const int ChunkSize = 512;
	const int NumChunks = (InVolumeNodes.Num() + ChunkSize - 1) / ChunkSize;

	ParallelFor(NumChunks, [&](int ChunkIdx) {
		const int Start = ChunkIdx * ChunkSize;
		const int End = FMath::Min(Start + ChunkSize, InVolumeNodes.Num());

		for (int NodeIndex = Start; NodeIndex < End; NodeIndex++)
		{
			const auto& Node = InVolumeNodes[NodeIndex];
			if (!Node.IsValid()) continue;

			// --- Compute voxel coords (multiply instead of divide) ---
			int VolumeX = static_cast<int>((Node->Center.X + OctreeExtent) * Scale);
			int VolumeY = static_cast<int>((Node->Center.Y + OctreeExtent) * Scale);
			int VolumeZ = static_cast<int>((Node->Center.Z + OctreeExtent) * Scale);

			if (VolumeX < 0 || VolumeX >= InResolution ||
				VolumeY < 0 || VolumeY >= InResolution ||
				VolumeZ < 0 || VolumeZ >= InResolution)
			{
				continue;
			}

			// --- Linear voxel index ---
			int64 VoxelIndex = ((int64)VolumeZ * InResolution * InResolution) + ((int64)VolumeY * InResolution) + VolumeX;
			int64 ByteIndex = VoxelIndex * BytesPerVoxel;

			// --- ScaleFactor ---
			uint8 DensityByte = 0;
			if (InMaxDensity > 0.0f)
			{
				float Norm = (float)Node->Data.Density / InMaxDensity;
				DensityByte = (uint8)FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f);
			}

			// --- Composition ---
			FVector Comp = Node->Data.Composition;

			TextureData[ByteIndex + 0] = (uint8)FMath::Clamp(Comp.X * 255.0f, 0.0f, 255.0f); // B
			TextureData[ByteIndex + 1] = (uint8)FMath::Clamp(Comp.Y * 255.0f, 0.0f, 255.0f); // G
			TextureData[ByteIndex + 2] = (uint8)FMath::Clamp(Comp.Z * 255.0f, 0.0f, 255.0f); // R
			TextureData[ByteIndex + 3] = DensityByte; // A
		}
		}, EParallelForFlags::BackgroundPriority);

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("OctreeTextureProcessor::Base volume mip data @%dx^3 generation duration: %.3f seconds"), InResolution, TotalDuration);

	return TextureData;
}

TArray<uint8> FVolumeTextureUtils::SampleNoiseToVolume(
	FastNoise::SmartNode<> InNoise,
	int InSeed,
	int InResolution,
	double InExtent,
	TSharedPtr<FOctree> InOctree,
	int InOctreeDepth,
	float InNoisePower,
	int InChannel,
	FVector InWorldOffset)
{
	FVector NoiseScale(1, 1, 1);
	double StartTime = FPlatformTime::Seconds();

	const int BytesPerVoxel = 4;
	const int64 TotalBytes = (int64)InResolution * InResolution * InResolution * BytesPerVoxel;

	TArray<uint8> TextureData;
	TextureData.SetNumZeroed(TotalBytes);

	const double VoxelSize = (2.0 * InExtent) / InResolution;
	const double InvExtent = 1.0 / InExtent;

	// --- Octree setup ---
	bool bWriteOctree = InOctree.IsValid() && InOctreeDepth > 0;

	// Octree resolution: number of nodes per axis at InOctreeDepth.
	// Independent of InResolution � we accumulate density from multiple
	// texture voxels into each coarser octree node.
	const int OctreeRes = bWriteOctree ? (1 << InOctreeDepth) : 0;
	const int VoxelsPerOctreeNode = bWriteOctree ? FMath::Max(1, InResolution / OctreeRes) : 1;

	// Accumulation grid at octree resolution
	const int64 OctreeNodeCount = bWriteOctree ? (int64)OctreeRes * OctreeRes * OctreeRes : 0;
	TArray<float> OctreeDensityAccum;
	TArray<int32> OctreeDensityCount;
	if (bWriteOctree)
	{
		OctreeDensityAccum.SetNumZeroed(OctreeNodeCount);
		OctreeDensityCount.SetNumZeroed(OctreeNodeCount);
	}

	// --- Chunking for noise sampling ---
	const int ChunkRes = 32;
	const int SubSamplesPerAxis = FMath::Max(1, InResolution / ChunkRes);
	const int NumChunks = ChunkRes * ChunkRes * ChunkRes;
	const int SamplesPerChunk = SubSamplesPerAxis * SubSamplesPerAxis * SubSamplesPerAxis;

	// --- Pass 1: Noise sampling + texture write + accumulate octree density ---
	ParallelFor(NumChunks, [&](int ChunkIdx)
		{
			int cx = ChunkIdx % ChunkRes;
			int cy = (ChunkIdx / ChunkRes) % ChunkRes;
			int cz = ChunkIdx / (ChunkRes * ChunkRes);

			int xStart = cx * SubSamplesPerAxis;
			int yStart = cy * SubSamplesPerAxis;
			int zStart = cz * SubSamplesPerAxis;

			// --- Build coordinate arrays for batch sampling ---
			TArray<float> XCoords, YCoords, ZCoords, NoiseOut;
			XCoords.SetNumUninitialized(SamplesPerChunk);
			YCoords.SetNumUninitialized(SamplesPerChunk);
			ZCoords.SetNumUninitialized(SamplesPerChunk);
			NoiseOut.SetNumUninitialized(SamplesPerChunk);

			int SampleIdx = 0;
			for (int lz = 0; lz < SubSamplesPerAxis; ++lz)
			{
				int z = zStart + lz;
				for (int ly = 0; ly < SubSamplesPerAxis; ++ly)
				{
					int y = yStart + ly;
					for (int lx = 0; lx < SubSamplesPerAxis; ++lx)
					{
						int x = xStart + lx;

						double wx = -InExtent + (x + 0.5) * VoxelSize;
						double wy = -InExtent + (y + 0.5) * VoxelSize;
						double wz = -InExtent + (z + 0.5) * VoxelSize;

						// Normalize to noise-space [-1, +1] for this volume,
						// then add the cell's WorldOffset so adjacent cells
						// sample contiguous regions of the same noise field.
						XCoords[SampleIdx] = (float)(wx * InvExtent * (1.0 / NoiseScale.X) + InWorldOffset.X);
						YCoords[SampleIdx] = (float)(wy * InvExtent * (1.0 / NoiseScale.Y) + InWorldOffset.Y);
						ZCoords[SampleIdx] = (float)(wz * InvExtent * (1.0 / NoiseScale.Z) + InWorldOffset.Z);

						SampleIdx++;
					}
				}
			}

			// --- Batch SIMD noise sample ---
			InNoise->GenPositionArray3D(
				NoiseOut.GetData(),
				SamplesPerChunk,
				XCoords.GetData(),
				YCoords.GetData(),
				ZCoords.GetData(),
				0.0f, 0.0f, 0.0f,
				InSeed
			);

			// --- Write to texture + accumulate for octree ---
			SampleIdx = 0;
			for (int lz = 0; lz < SubSamplesPerAxis; ++lz)
			{
				int z = zStart + lz;
				for (int ly = 0; ly < SubSamplesPerAxis; ++ly)
				{
					int y = yStart + ly;
					for (int lx = 0; lx < SubSamplesPerAxis; ++lx)
					{
						int x = xStart + lx;

						float rawNoise = NoiseOut[SampleIdx];
						float density = FMath::Pow(FMath::Clamp(rawNoise, 0.0f, 1.0f), InNoisePower);
						uint8 densityByte = (uint8)FMath::Clamp(density * 255.0f, 0.0f, 255.0f);

						int64 idx = ((int64)z * InResolution * InResolution + (int64)y * InResolution + x) * BytesPerVoxel;

						if (InChannel == -1)
						{
							TextureData[idx + 0] = densityByte;
							TextureData[idx + 1] = densityByte;
							TextureData[idx + 2] = densityByte;
							TextureData[idx + 3] = densityByte;
						}
						else
						{
							int c = FMath::Clamp(InChannel, 0, 3);
							TextureData[idx + c] = densityByte;
						}

						if (bWriteOctree)
						{
							int ox = FMath::Min(x / VoxelsPerOctreeNode, OctreeRes - 1);
							int oy = FMath::Min(y / VoxelsPerOctreeNode, OctreeRes - 1);
							int oz = FMath::Min(z / VoxelsPerOctreeNode, OctreeRes - 1);
							int64 oidx = (int64)oz * OctreeRes * OctreeRes + (int64)oy * OctreeRes + ox;

							OctreeDensityAccum[oidx] += density;
							OctreeDensityCount[oidx]++;
						}

						SampleIdx++;
					}
				}
			}
		}, EParallelForFlags::BackgroundPriority);

	double SampleDuration = FPlatformTime::Seconds() - StartTime;

	// --- Pass 2: Build FPointData from accumulated grid + bulk insert ---
	if (bWriteOctree)
	{
		double InsertStart = FPlatformTime::Seconds();

		const double OctreeNodeSize = (2.0 * InExtent) / OctreeRes;

		int32 NonEmptyCount = 0;
		for (int64 i = 0; i < OctreeNodeCount; ++i)
		{
			if (OctreeDensityCount[i] > 0) NonEmptyCount++;
		}

		TArray<FPointData> AllPointData;
		AllPointData.Reserve(NonEmptyCount);

		for (int oz = 0; oz < OctreeRes; ++oz)
		{
			for (int oy = 0; oy < OctreeRes; ++oy)
			{
				for (int ox = 0; ox < OctreeRes; ++ox)
				{
					int64 oidx = (int64)oz * OctreeRes * OctreeRes + (int64)oy * OctreeRes + ox;
					if (OctreeDensityCount[oidx] == 0) continue;

					float avgDensity = OctreeDensityAccum[oidx] / (float)OctreeDensityCount[oidx];

					double wx = -InExtent + (ox + 0.5) * OctreeNodeSize;
					double wy = -InExtent + (oy + 0.5) * OctreeNodeSize;
					double wz = -InExtent + (oz + 0.5) * OctreeNodeSize;

					FVoxelData VoxelData;
					VoxelData.Density = avgDensity;

					AllPointData.Add(FPointData(FVector(wx, wy, wz), InOctreeDepth, VoxelData));
				}
			}
		}

		OctreeDensityAccum.Empty();
		OctreeDensityCount.Empty();

		TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
		TArray<TSharedPtr<FOctreeNode>> VolumeChunks;
		InOctree->BulkInsertPositions(AllPointData, InsertedNodes, VolumeChunks);

		double InsertDuration = FPlatformTime::Seconds() - InsertStart;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SampleNoiseToVolume octree BulkInsert (%d nodes at depth %d) took %.3f sec"),
			NonEmptyCount, InOctreeDepth, InsertDuration);
	}

	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SampleNoiseToVolume @%d^3 took %.3f sec (sample: %.3f, octree depth: %d, channel: %d, batch SIMD)"),
		InResolution, Duration, SampleDuration, InOctreeDepth, InChannel);

	return TextureData;
}

void FVolumeTextureUtils::SampleNoiseToSubRegion(
	TArray<uint8>& InOutVolumeData,
	int InResolution,
	double InExtent,
	double InNoiseNormExtent,
	FastNoise::SmartNode<> InNoise,
	int InSeed,
	FIntVector InVoxelMin,
	FIntVector InVoxelMax,
	float InNoisePower,
	int InChannel,
	FVector InWorldOffset)
{
	double StartTime = FPlatformTime::Seconds();

	constexpr int BytesPerVoxel = 4;
	// VoxelSize is derived from the buffer-layout extent (the full volume).
	const double VoxelSize = (2.0 * InExtent) / InResolution;
	// InvNoiseExtent normalizes world positions into noise space per-cell.
	const double InvNoiseExtent = 1.0 / InNoiseNormExtent;
	const int Channel = FMath::Clamp(InChannel, 0, 3);

	const int MinX = FMath::Max(InVoxelMin.X, 0);
	const int MinY = FMath::Max(InVoxelMin.Y, 0);
	const int MinZ = FMath::Max(InVoxelMin.Z, 0);
	const int MaxX = FMath::Min(InVoxelMax.X, InResolution);
	const int MaxY = FMath::Min(InVoxelMax.Y, InResolution);
	const int MaxZ = FMath::Min(InVoxelMax.Z, InResolution);

	const int RangeX = MaxX - MinX;
	const int RangeY = MaxY - MinY;
	const int RangeZ = MaxZ - MinZ;
	if (RangeX <= 0 || RangeY <= 0 || RangeZ <= 0) return;

	ParallelFor(RangeZ, [&](int LocalZ)
		{
			const int z = MinZ + LocalZ;
			const int SamplesPerSlice = RangeX * RangeY;

			TArray<float> XCoords, YCoords, ZCoords, NoiseOut;
			XCoords.SetNumUninitialized(SamplesPerSlice);
			YCoords.SetNumUninitialized(SamplesPerSlice);
			ZCoords.SetNumUninitialized(SamplesPerSlice);
			NoiseOut.SetNumUninitialized(SamplesPerSlice);

			int SampleIdx = 0;
			for (int y = MinY; y < MaxY; ++y)
			{
				for (int x = MinX; x < MaxX; ++x)
				{
					// World position from the buffer layout (full neighborhood).
					double wx = -InExtent + (x + 0.5) * VoxelSize;
					double wy = -InExtent + (y + 0.5) * VoxelSize;
					double wz = -InExtent + (z + 0.5) * VoxelSize;

					// Normalize by the per-cell extent, then add the cell offset.
					// This matches how the particle generators compute noise coords.
					XCoords[SampleIdx] = (float)(wx * InvNoiseExtent + InWorldOffset.X);
					YCoords[SampleIdx] = (float)(wy * InvNoiseExtent + InWorldOffset.Y);
					ZCoords[SampleIdx] = (float)(wz * InvNoiseExtent + InWorldOffset.Z);
					SampleIdx++;
				}
			}

			InNoise->GenPositionArray3D(
				NoiseOut.GetData(),
				SamplesPerSlice,
				XCoords.GetData(),
				YCoords.GetData(),
				ZCoords.GetData(),
				0.0f, 0.0f, 0.0f,
				InSeed);

			SampleIdx = 0;
			uint8* DataPtr = InOutVolumeData.GetData();
			for (int y = MinY; y < MaxY; ++y)
			{
				for (int x = MinX; x < MaxX; ++x)
				{
					float density = FMath::Pow(FMath::Clamp(NoiseOut[SampleIdx], 0.0f, 1.0f), InNoisePower);
					uint8 densityByte = (uint8)FMath::Clamp(density * 255.0f, 0.0f, 255.0f);

					int64 idx = ((int64)z * InResolution * InResolution + (int64)y * InResolution + x) * BytesPerVoxel;
					DataPtr[idx + Channel] = densityByte;
					SampleIdx++;
				}
			}
		}, EParallelForFlags::BackgroundPriority);

	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SampleNoiseToSubRegion [%d,%d,%d]->[%d,%d,%d] took %.3f sec"),
		MinX, MinY, MinZ, MaxX, MaxY, MaxZ, Duration);
}

void FVolumeTextureUtils::ClearSubRegion(
	TArray<uint8>& InOutVolumeData,
	int InResolution,
	FIntVector InVoxelMin,
	FIntVector InVoxelMax)
{
	constexpr int BytesPerVoxel = 4;

	const int MinX = FMath::Max(InVoxelMin.X, 0);
	const int MinY = FMath::Max(InVoxelMin.Y, 0);
	const int MinZ = FMath::Max(InVoxelMin.Z, 0);
	const int MaxX = FMath::Min(InVoxelMax.X, InResolution);
	const int MaxY = FMath::Min(InVoxelMax.Y, InResolution);
	const int MaxZ = FMath::Min(InVoxelMax.Z, InResolution);

	const int RowBytes = (MaxX - MinX) * BytesPerVoxel;
	if (RowBytes <= 0) return;

	uint8* DataPtr = InOutVolumeData.GetData();
	for (int z = MinZ; z < MaxZ; ++z)
	{
		for (int y = MinY; y < MaxY; ++y)
		{
			int64 idx = ((int64)z * InResolution * InResolution + (int64)y * InResolution + MinX) * BytesPerVoxel;
			FMemory::Memzero(DataPtr + idx, RowBytes);
		}
	}
}
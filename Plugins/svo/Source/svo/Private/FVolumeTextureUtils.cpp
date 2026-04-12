#include "FOctree.h"
#include "FVolumeTextureUtils.h"

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
	int InChannel)
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
	// Pre-populate volume chunks serially so parallel inserts can use InCurrent,
	// bypassing the octree mutex entirely (same pattern as BulkInsertPositions).
	bool bWriteOctree = InOctree.IsValid() && InOctreeDepth > 0;
	TArray<TSharedPtr<FOctreeNode>> VolumeChunks;
	const int ChunkRes = bWriteOctree ? (1 << InOctree->VolumeDepth) : 32;

	if (bWriteOctree)
	{
		TArray<TArray<FPointData>> DummyChunkData;
		InOctree->PrePopulateVolumeLayer(VolumeChunks, DummyChunkData);
	}

	const int SubSamplesPerAxis = FMath::Max(1, InResolution / ChunkRes);
	const int NumChunks = ChunkRes * ChunkRes * ChunkRes;

	ParallelFor(NumChunks, [&](int ChunkIdx)
		{
			int cx = ChunkIdx % ChunkRes;
			int cy = (ChunkIdx / ChunkRes) % ChunkRes;
			int cz = ChunkIdx / (ChunkRes * ChunkRes);

			int xStart = cx * SubSamplesPerAxis;
			int yStart = cy * SubSamplesPerAxis;
			int zStart = cz * SubSamplesPerAxis;

			TSharedPtr<FOctreeNode> ChunkNode = (bWriteOctree && ChunkIdx < VolumeChunks.Num())
				? VolumeChunks[ChunkIdx]
				: nullptr;

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

						float nx = (float)(wx * InvExtent * (1.0 / NoiseScale.X));
						float ny = (float)(wy * InvExtent * (1.0 / NoiseScale.Y));
						float nz = (float)(wz * InvExtent * (1.0 / NoiseScale.Z));

						float rawNoise = InNoise->GenSingle3D(nx, ny, nz, InSeed); //We should be batching these samples
						float density = FMath::Pow(FMath::Clamp(rawNoise, 0.0f, 1.0f), InNoisePower);
						uint8 densityByte = (uint8)FMath::Clamp(density * 255.0f, 0.0f, 255.0f);

						int64 idx = ((int64)z * InResolution * InResolution + (int64)y * InResolution + x) * BytesPerVoxel;
						// TODO: Revisit channel semantics when we have proper RGB data from cluster
						// light temperature/intensity propagation. For now, broadcast density to all
						// channels so the texture previews and raymarch correctly in grayscale.
						TextureData[idx + 0] = densityByte; // B
						TextureData[idx + 1] = densityByte; // G
						TextureData[idx + 2] = densityByte; // R
						TextureData[idx + 3] = densityByte; // A

						if (bWriteOctree && ChunkNode.IsValid())
						{
							FVoxelData VoxelData;
							VoxelData.Density = density;
							InOctree->InsertPosition(FVector(wx, wy, wz), InOctreeDepth, VoxelData, ChunkNode);
						}
					}
				}
			}
		}, EParallelForFlags::BackgroundPriority);

	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SampleNoiseToVolume @%d^3 took %.3f sec (octree write: %s)"),
		InResolution, Duration, bWriteOctree ? TEXT("yes") : TEXT("no"));

	return TextureData;
}
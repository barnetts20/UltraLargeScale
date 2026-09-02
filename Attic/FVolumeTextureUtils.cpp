#include "FVolumeTextureUtils.h"
#include "FOctree.h"

#pragma region GenerateVolumeMipDataFromOctree
TArray<uint8> FVolumeTextureUtils::GenerateVolumeMipDataFromOctree(TArray<TSharedPtr<FOctreeNode>> InVolumeNodes, int InResolution, double InExtent, double InMaxDensity)
{
	double StartTime = FPlatformTime::Seconds();

	// Octree + setup
	const int TargetDepth = FMath::FloorLog2(InResolution);
	const double NodeExtentAtDepth = InExtent / FMath::Pow(2.0, TargetDepth);
	const double OctreeExtent = InExtent;

	// Precompute reciprocal for fast coordinate mapping
	const double Scale = 1.0 / (2.0 * NodeExtentAtDepth);

	// Texture allocation
	const int BytesPerVoxel = 4; // BGRA8
	const int64 TotalVoxels = (int64)InResolution * InResolution * InResolution;
	const int64 TotalBytes = TotalVoxels * BytesPerVoxel;

	TArray<uint8> TextureData;
	TextureData.SetNumZeroed(TotalBytes);

	// Parallel fill
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

			// Compute voxel coords (multiply instead of divide)
			int VolumeX = static_cast<int>((Node->Center.X + OctreeExtent) * Scale);
			int VolumeY = static_cast<int>((Node->Center.Y + OctreeExtent) * Scale);
			int VolumeZ = static_cast<int>((Node->Center.Z + OctreeExtent) * Scale);

			if (VolumeX < 0 || VolumeX >= InResolution ||
				VolumeY < 0 || VolumeY >= InResolution ||
				VolumeZ < 0 || VolumeZ >= InResolution)
			{
				continue;
			}

			// Linear voxel index
			int64 VoxelIndex = ((int64)VolumeZ * InResolution * InResolution) + ((int64)VolumeY * InResolution) + VolumeX;
			int64 ByteIndex = VoxelIndex * BytesPerVoxel;

			// Density
			uint8 DensityByte = 0;
			if (InMaxDensity > 0.0f)
			{
				float Norm = (float)Node->Data.Density / InMaxDensity;
				DensityByte = (uint8)FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f);
			}

			// Composition
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

#pragma endregion

#pragma region ClearSubRegion
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

#pragma endregion
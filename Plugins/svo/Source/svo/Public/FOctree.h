// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
#include "UObject/SavePackage.h"
#include "RHICommandList.h"
#include "RenderResource.h"
#include "RHIResources.h"       // For FRHITexture3D
#include "RHIUtilities.h"   
#include "AssetRegistry/AssetRegistryModule.h"
#include "FastNoise/FastNoise.h"
#include "Engine/VolumeTexture.h"
#include "CoreMinimal.h"

class SVO_API FOctreeNode : public TSharedFromThis<FOctreeNode>
{
public:
	TArray<uint8> Index;
	int Depth;

	TWeakPtr<FOctreeNode> Parent;
	TSharedPtr<FOctreeNode> Children[8];
	
	FInt64Vector Center;
	int64 Extent;

	FVoxelData Data = FVoxelData(); //Default constructor with 0 Density -1 ObectId

	FOctreeNode() {
		Index = TArray<uint8>();
		Center = FInt64Vector();
		Extent = 0;
		Depth = Index.Num();
	};

	FOctreeNode(FInt64Vector InCenter, int64 InExtent, TArray<uint8> InIndex, TWeakPtr<FOctreeNode> InParent) {
		Index = InIndex;
		Center = InCenter;
		Extent = InExtent;
		Parent = InParent;
		Depth = Index.Num();
	};
};

class SVO_API FOctree : public TSharedFromThis<FOctree>
{
public:

	int64 Extent; //Must be power of 2, eg 1024 2048 etc
	TSharedPtr<FOctreeNode> Root;

	double DepthMaxDensity = 0;
	int MaxDepth;
	int VolumeDepth = 5;

	// In FOctree class, add a mutex
	mutable FCriticalSection OctreeMutex;
	std::atomic<bool> bIsResetting{ false };
	#pragma region BulkInsert
	void BulkInsertPositions(
		TArray<FPointData> InPointData,
		TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes,
		TArray<TSharedPtr<FOctreeNode>>& OutVolumeChunks)
	{
		if (bIsResetting.load()) {
			return; // Early exit if shutting down
		}

		double StartTime = FPlatformTime::Seconds();

		// ---------------- Prepopulation ----------------
		double PreStart = FPlatformTime::Seconds();
		TArray<TArray<FPointData>> ChunkPointData;
		PrePopulateVolumeLayer(OutVolumeChunks, ChunkPointData);
		double PreEnd = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: PrePopulateVolumeLayer took %.3f sec"), PreEnd - PreStart);

		// ---------------- Point Distribution ----------------
		double DistStart = FPlatformTime::Seconds();
		for (const FPointData& Point : InPointData) {
			int32 ChunkIndex = FindChunkIndexForPosition(Point.GetInt64Position(), OutVolumeChunks);
			if (ChunkIndex >= 0 && ChunkIndex < ChunkPointData.Num()) {
				ChunkPointData[ChunkIndex].Add(Point);
			}
		}
		double DistEnd = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: Point distribution took %.3f sec"), DistEnd - DistStart);

		// ---------------- Parallel Chunk Inserts ----------------
		double InsertStart = FPlatformTime::Seconds();
		const int32 NumChunks = OutVolumeChunks.Num();
		TArray<TArray<TSharedPtr<FOctreeNode>>> PerChunkResults;
		PerChunkResults.SetNum(NumChunks);

		ParallelFor(NumChunks, [&](int32 i) {
			if (bIsResetting.load()) {
				return; // Early exit if shutting down
			}

			TSharedPtr<FOctreeNode> Chunk = OutVolumeChunks[i];
			TArray<TSharedPtr<FOctreeNode>> ChunkResults;
			ChunkResults.Reserve(ChunkPointData[i].Num());

			for (const FPointData& Point : ChunkPointData[i]) {
				if (bIsResetting.load()) {
					return; // Early exit if shutting down
				}

				TSharedPtr<FOctreeNode> Result = InsertPosition(Point.GetInt64Position(), Point.InsertDepth, Point.Data, Chunk);
				if (Result.IsValid()) {
					ChunkResults.Add(Result);
				}
			}

			PerChunkResults[i] = MoveTemp(ChunkResults);
			}, EParallelForFlags::BackgroundPriority);
		double InsertEnd = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: Parallel insert took %.3f sec"), InsertEnd - InsertStart);

		// ---------------- Recombine Results ----------------
		double RecombineStart = FPlatformTime::Seconds();
		OutInsertedNodes.Empty();

		int32 TotalResults = 0;
		for (const auto& Arr : PerChunkResults) {
			TotalResults += Arr.Num();
		}
		OutInsertedNodes.Reserve(TotalResults);

		for (int32 i = 0; i < NumChunks; ++i) {
			if (PerChunkResults[i].Num() > 0) {
				OutInsertedNodes.Append(PerChunkResults[i]);
			}
		}
		double RecombineEnd = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: Recombine results took %.3f sec"), RecombineEnd - RecombineStart);

		// ---------------- Total ----------------
		double EndTime = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: Total duration %.3f sec"), EndTime - StartTime);
	}

	int32 FindChunkIndexForPosition(FInt64Vector Position, const TArray<TSharedPtr<FOctreeNode>>& VolumeChunks) {
		// Calculate which chunk this position belongs to based on volume depth grid
		int64 NodesPerSide = 1LL << VolumeDepth;
		int64 ChunkSize = (Extent * 2) / NodesPerSide;
		int64 HalfExtent = Extent;

		// Convert world position to grid coordinates
		int64 GridX = (Position.X + HalfExtent) / ChunkSize;
		int64 GridY = (Position.Y + HalfExtent) / ChunkSize;
		int64 GridZ = (Position.Z + HalfExtent) / ChunkSize;

		// Clamp to valid range
		GridX = FMath::Clamp(GridX, 0LL, NodesPerSide - 1);
		GridY = FMath::Clamp(GridY, 0LL, NodesPerSide - 1);
		GridZ = FMath::Clamp(GridZ, 0LL, NodesPerSide - 1);

		// Convert 3D grid coordinates to linear index
		return GridX + GridY * NodesPerSide + GridZ * NodesPerSide * NodesPerSide;
	}

	void PrePopulateVolumeLayer(TArray<TSharedPtr<FOctreeNode>>& OutVolumeChunks, TArray<TArray<FPointData>>& OutChunkPointData) {
		int64 NodesPerSide = 1LL << VolumeDepth;
		int64 ChunkExtent = Extent >> VolumeDepth;
		int64 TotalNodeCount = NodesPerSide * NodesPerSide * NodesPerSide;
		OutVolumeChunks.SetNum(TotalNodeCount);
		OutChunkPointData.SetNum(TotalNodeCount);

		for (int64 x = 0; x < NodesPerSide; ++x) {
			for (int64 y = 0; y < NodesPerSide; ++y) {
				for (int64 z = 0; z < NodesPerSide; ++z) {
					// Calculate index directly
					int32 idx = x + y * NodesPerSide + z * NodesPerSide * NodesPerSide;

					FInt64Vector ChunkCenter = FInt64Vector(
						(x - NodesPerSide / 2) * ChunkExtent * 2 + ChunkExtent,
						(y - NodesPerSide / 2) * ChunkExtent * 2 + ChunkExtent,
						(z - NodesPerSide / 2) * ChunkExtent * 2 + ChunkExtent
					);

					TSharedPtr<FOctreeNode> ChunkNode = InsertPosition(ChunkCenter, VolumeDepth, FVoxelData(0, FVector::ZeroVector, -1, 2));
					
					OutVolumeChunks[idx] = ChunkNode;
					OutChunkPointData[idx] = TArray<FPointData>();
				}
			}
		}
	}
	
	TSharedPtr<FOctreeNode> InsertPosition(FInt64Vector InPosition, int InDepth, FVoxelData InData, TSharedPtr<FOctreeNode> InCurrent = nullptr) {
		if (bIsResetting.load()) {
			return nullptr; // Early exit if shutting down
		}
		if (InData.TypeId == -1 && InPosition == FInt64Vector::ZeroValue) return nullptr; //Ignore typeless inserts into 0,0,0
		
		TSharedPtr<FOctreeNode> Current = Root;
		int64 CurrentExtent = Extent;

		if (InCurrent) {
			Current = InCurrent;
			CurrentExtent = Current->Extent;
			//double DensityWeight = static_cast<double>(CurrentExtent) / static_cast<double>(LeafExtent);
			double DensityWeight = FMath::Pow(2.0, MaxDepth - InDepth);
			Current->Data.Density += InData.Density * DensityWeight; //Roughly scale density contribution by insert depth
			Current->Data.Composition += InData.Composition * InData.Density * DensityWeight;
			DepthMaxDensity = FMath::Max(DepthMaxDensity, Current->Data.Density);
		}
		else {
			FScopeLock Lock(&OctreeMutex);
		}


		for (int Depth = Current->Depth; Depth < InDepth; Depth++) {
			//Check/Set Max for layer
			CurrentExtent /= 2;

			uint8 ChildIndex = 0;
			if (InPosition.X >= Current->Center.X) ChildIndex |= 1;
			if (InPosition.Y >= Current->Center.Y) ChildIndex |= 2;
			if (InPosition.Z >= Current->Center.Z) ChildIndex |= 4;

			if (!Current->Children[ChildIndex].IsValid()) {
				// Compute new center for the child
				int64 OffsetX = ((ChildIndex & 1) ? 1 : -1) * CurrentExtent;
				int64 OffsetY = ((ChildIndex & 2) ? 1 : -1) * CurrentExtent;
				int64 OffsetZ = ((ChildIndex & 4) ? 1 : -1) * CurrentExtent;

				FInt64Vector ChildCenter = FInt64Vector(
					Current->Center.X + OffsetX,
					Current->Center.Y + OffsetY,
					Current->Center.Z + OffsetZ
				);

				TArray<uint8> ChildIndexArray = Current->Index;
				ChildIndexArray.Add(ChildIndex);

				Current->Children[ChildIndex] = MakeShared<FOctreeNode, ESPMode::ThreadSafe>(
					ChildCenter, CurrentExtent, ChildIndexArray, Current
				);
			}

			Current = Current->Children[ChildIndex];
		}

		InData.Density += Current->Data.Density;
		Current->Data = InData;
		return Current;
	}
	#pragma endregion

	void CollectLeafNodes(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		bool bIsLeaf = true;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				bIsLeaf = false;
				CollectLeafNodes(Child, OutNodes, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter != -1 && InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false;

		if (bIsLeaf && bPassesFilter) {
			OutNodes.Add(InNode);
		}
	}

	void CollectPopulatedNodes(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				CollectPopulatedNodes(Child, OutNodes, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InNode->Data.Density <= 0) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter != -1 && InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false;
		
		if (bPassesFilter) {
			OutNodes.Add(InNode);
		}
	}

	TArray<TSharedPtr<FOctreeNode>> GetLeafNodes(int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Leaves;
		if (Root.IsValid()) {
			CollectLeafNodes(Root, Leaves, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Leaves;
	}
	
	TArray<TSharedPtr<FOctreeNode>> GetPopulatedNodes(int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid()) {
			CollectPopulatedNodes(Root, Nodes, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Nodes;
	}

	void CollectNodesInRange(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FInt64Vector& InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		//Query Bounds
		const FInt64Vector QueryMin = InCenter - FInt64Vector(InExtent, InExtent, InExtent);
		const FInt64Vector QueryMax = InCenter + FInt64Vector(InExtent, InExtent, InExtent);

		//Node Bounds
		const FInt64Vector NodeMin = InNode->Center - FInt64Vector(InNode->Extent, InNode->Extent, InNode->Extent);
		const FInt64Vector NodeMax = InNode->Center + FInt64Vector(InNode->Extent, InNode->Extent, InNode->Extent);

		// Early reject if node doesn't intersect the query bounds
		const bool bIntersects =
			NodeMin.X <= QueryMax.X && NodeMax.X >= QueryMin.X &&
			NodeMin.Y <= QueryMax.Y && NodeMax.Y >= QueryMin.Y &&
			NodeMin.Z <= QueryMax.Z && NodeMax.Z >= QueryMin.Z;

		if (!bIntersects) return;

		// Recurse into children
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children)
		{
			if (Child.IsValid())
			{
				CollectNodesInRange(Child, OutNodes, InCenter, InExtent, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		// Filtering checks
		bool bPassesFilter = true;
		if (InNode->Data.Density <= 0) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter != -1 && InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false;

		if (bPassesFilter && InNode->Data.Density > 0)
		{
			OutNodes.Add(InNode);
		}
	}

	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(FInt64Vector InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			CollectNodesInRange(Root, Nodes, InCenter, InExtent, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Nodes;
	}

	FOctree(int64 InExtent) {
		Extent = InExtent;
		MaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(Extent)));
		DepthMaxDensity = 0;
		Root = MakeShared<FOctreeNode>(FInt64Vector(), Extent, TArray<uint8>(), nullptr);
	}
};

class SVO_API FOctreeTextureProcessor {
public:
	static TArray<uint8> GenerateVolumeMipDataFromOctree(TSharedPtr<FOctree> InOctree, TArray<TSharedPtr<FOctreeNode>> InVolumeNodes, int32 Resolution)
	{
		double StartTime = FPlatformTime::Seconds();

		// --- Octree + setup ---
		const int32 TargetDepth = FMath::FloorLog2(Resolution);

		const int64 NodeExtentAtDepth = (InOctree->Extent >> TargetDepth);
		const int64 OctreeExtent = InOctree->Extent;

		// Precompute reciprocal for fast coordinate mapping
		const double Scale = 1.0 / (2.0 * NodeExtentAtDepth);

		// --- Texture allocation ---
		const int64 TotalVoxels = (int64)Resolution * Resolution * Resolution;
		const int64 BytesPerVoxel = 4; // BGRA8
		const int64 TotalBytes = TotalVoxels * BytesPerVoxel;

		TArray<uint8> TextureData;
		TextureData.SetNumZeroed(TotalBytes);

		// --- Parallel fill ---
		// Use chunked ParallelFor to reduce scheduling overhead
		const int32 ChunkSize = 512;
		const int32 NumChunks = (InVolumeNodes.Num() + ChunkSize - 1) / ChunkSize;

		ParallelFor(NumChunks, [&](int32 ChunkIdx){
			const int32 Start = ChunkIdx * ChunkSize;
			const int32 End = FMath::Min(Start + ChunkSize, InVolumeNodes.Num());

			for (int32 NodeIndex = Start; NodeIndex < End; NodeIndex++)
			{
				const auto& Node = InVolumeNodes[NodeIndex];
				if (!Node.IsValid()) continue;

				// --- Compute voxel coords (multiply instead of divide) ---
				int32 VolumeX = static_cast<int32>((Node->Center.X + OctreeExtent) * Scale);
				int32 VolumeY = static_cast<int32>((Node->Center.Y + OctreeExtent) * Scale);
				int32 VolumeZ = static_cast<int32>((Node->Center.Z + OctreeExtent) * Scale);

				if (VolumeX < 0 || VolumeX >= Resolution ||
					VolumeY < 0 || VolumeY >= Resolution ||
					VolumeZ < 0 || VolumeZ >= Resolution)
				{
					continue;
				}

				// --- Linear voxel index ---
				int64 VoxelIndex = ((int64)VolumeZ * Resolution * Resolution) +
					((int64)VolumeY * Resolution) +
					VolumeX;
				int64 ByteIndex = VoxelIndex * BytesPerVoxel;

				// --- Density ---
				uint8 DensityByte = 0;
				if (InOctree->DepthMaxDensity > 0.0f)
				{
					float Norm = (float)Node->Data.Density / InOctree->DepthMaxDensity;
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
		UE_LOG(LogTemp, Log, TEXT("OctreeTextureProcessor::Base volume mip data @%dx^3 generation duration: %.3f seconds"), Resolution, TotalDuration);

		return TextureData;
	}

	static TArray<uint8> UpscaleVolumeDensityData(const TArray<uint8>& InMipData, int32 InRes, int32 OutRes, FastNoise::SmartNode<> InNoise = FastNoise::NewFromEncodedNodeTree("AAAAAAAA"), double InDomainScale = 1, FVector InDomainOffset = FVector(0, 0, 0), double InNoiseEffect = 1, int InSeed = 69)
	{
		bool HasNoise = false;
		float NoiseEffect = 1;
		TArray<float> NoiseData;
		NoiseData.SetNumUninitialized(OutRes * OutRes * OutRes);
		InNoise->GenUniformGrid3D(NoiseData.GetData(), 0, 0, 0, OutRes, OutRes, OutRes, 1, InSeed);

		check(InRes > 1 && OutRes > InRes);

		const int32 InVoxels = InRes * InRes * InRes;
		const int32 OutVoxels = OutRes * OutRes * OutRes;
		const int32 BytesPerVoxel = 4;

		check(InMipData.Num() == InVoxels * BytesPerVoxel);

		TArray<uint8> OutData;
		OutData.SetNumZeroed(OutVoxels * BytesPerVoxel);

		const float Scale = float(InRes - 1) / float(OutRes - 1);
		const int32 OutSlice = OutRes * OutRes;
		const int32 InSlice = InRes * InRes;

		auto Sample = [&](int32 x, int32 y, int32 z, int32 channel) -> float
			{
				int32 index = (x + y * InRes + z * InSlice) * BytesPerVoxel + channel;
				return float(InMipData[index]) / 255.0f; // normalize [0,1]
			};

		auto Write = [&](int32 x, int32 y, int32 z, const float* values)
			{
				int32 index = (x + y * OutRes + z * OutSlice) * BytesPerVoxel;
				for (int c = 0; c < BytesPerVoxel; ++c)
				{
					OutData[index + c] = uint8(FMath::Clamp(values[c] * 255.0f, 0.0f, 255.0f));
				}
			};

		ParallelFor(
			OutRes,
			[&](int32 z)
			{
				float fz = z * Scale;
				int32 z0 = FMath::FloorToInt(fz);
				int32 z1 = FMath::Min(z0 + 1, InRes - 1);
				float tz = fz - z0;

				for (int32 y = 0; y < OutRes; y++)
				{
					float fy = y * Scale;
					int32 y0 = FMath::FloorToInt(fy);
					int32 y1 = FMath::Min(y0 + 1, InRes - 1);
					float ty = fy - y0;

					for (int32 x = 0; x < OutRes; x++)
					{
						float fx = x * Scale;
						int32 x0 = FMath::FloorToInt(fx);
						int32 x1 = FMath::Min(x0 + 1, InRes - 1);
						float tx = fx - x0;

						float values[4] = { 0,0,0,0 };
						float noise = NoiseData[x + y * OutRes + z * OutSlice];
						for (int c = 0; c < BytesPerVoxel; ++c)
						{
							// 8 neighbors, and apply noise pre filter
							float c000 = Sample(x0, y0, z0, c);
							c000 += c000 * NoiseData[x0 + y0 * OutRes + z0 * OutSlice] * InNoiseEffect;
							float c100 = Sample(x1, y0, z0, c);
							c100 += c100 * NoiseData[x1 + y0 * OutRes + z0 * OutSlice] * InNoiseEffect;
							float c010 = Sample(x0, y1, z0, c);
							c010 += c010 * NoiseData[x0 + y1 * OutRes + z0 * OutSlice] * InNoiseEffect;
							float c110 = Sample(x1, y1, z0, c);
							c110 += c110 * NoiseData[x1 + y1 * OutRes + z0 * OutSlice] * InNoiseEffect;
							float c001 = Sample(x0, y0, z1, c);
							c001 += c001 * NoiseData[x0 + y0 * OutRes + z1 * OutSlice] * InNoiseEffect;
							float c101 = Sample(x1, y0, z1, c);
							c101 += c101 * NoiseData[x1 + y0 * OutRes + z1 * OutSlice] * InNoiseEffect;
							float c011 = Sample(x0, y1, z1, c);
							c011 += c011 * NoiseData[x0 + y1 * OutRes + z1 * OutSlice] * InNoiseEffect;
							float c111 = Sample(x1, y1, z1, c);
							c111 += c111 * NoiseData[x1 + y1 * OutRes + z1 * OutSlice] * InNoiseEffect;

							// Trilinear interpolation
							float c00 = FMath::Lerp(c000, c100, tx);
							float c10 = FMath::Lerp(c010, c110, tx);
							float c01 = FMath::Lerp(c001, c101, tx);
							float c11 = FMath::Lerp(c011, c111, tx);

							float c0 = FMath::Lerp(c00, c10, ty);
							float c1 = FMath::Lerp(c01, c11, ty);

							values[c] = FMath::Lerp(c0, c1, tz);
						}
						Write(x, y, z, values);
					}
				}
			},
			EParallelForFlags::BackgroundPriority
		);

		return OutData;
	}

	// --- Specialized fixed upscale: InRes^3 -> 256^3 -> 4096^2 pseudo volume ---
	// InRes = low-res input (e.g. 32^3, 64^3, etc.)
	// Always outputs a 4096x4096 2D pseudo volume (256^3 slices)
	static TArray<uint8> UpscalePseudoVolumeDensityData(const TArray<uint8>& InMipData, int32 InRes)
	{
		double StartTime = FPlatformTime::Seconds();
		
		check(InRes > 1);

		constexpr int32 Out3DRes = 256;												// fixed target 3D resolution
		constexpr int32 BytesPerVoxel = 4;											// RGBA8
		constexpr int32 tilesPerSide = 16;											// 16x16 = 256 slices
		constexpr int32 Out2DRes = 4096;   // 4096
		constexpr int32 OutBytes = 67108864; // 4096 * 4096 * 4

		const int32 InSlice = InRes * InRes;

		// --- Precompute coords for 256 steps (x,y,z) ---
		struct SampleCoord { int i0, i1; float t; };
		static TArray<SampleCoord> Precomputed;
		static int32 CachedInRes = 0;
		if (Precomputed.Num() == 0 || CachedInRes != InRes)
		{
			Precomputed.SetNum(Out3DRes);
			for (int i = 0; i < Out3DRes; i++)
			{
				float f = i * (float(InRes - 1) / float(Out3DRes - 1));
				int i0 = FMath::FloorToInt(f);
				int i1 = FMath::Min(i0 + 1, InRes - 1);
				Precomputed[i] = { i0, i1, f - i0 };
			}
			CachedInRes = InRes;
		}

		// --- Allocate output ---
		TArray<uint8> OutData;
		OutData.SetNumZeroed(OutBytes);

		const uint8* InPtr = InMipData.GetData();
		uint8* OutPtrBase = OutData.GetData();

		// Lambda to compute base index into input voxel array
		auto InIndexBase = [InRes, InSlice, BytesPerVoxel](int x, int y, int z)->int32 {
			return ((z * InSlice) + (y * InRes) + x) * BytesPerVoxel;
		};

		// --- Parallel slice processing ---
		ParallelFor(Out3DRes, [&](int32 z)
			{
				const auto& Zc = Precomputed[z];
				const int z0 = Zc.i0;
				const int z1 = Zc.i1;
				const float tz = Zc.t;

				// Tile origin for this z slice
				const int tileX = z % tilesPerSide;
				const int tileY = z / tilesPerSide;
				const int tileOriginX = tileX * Out3DRes;
				const int tileOriginY = tileY * Out3DRes;

				for (int y = 0; y < Out3DRes; ++y)
				{
					const auto& Yc = Precomputed[y];
					const int y0 = Yc.i0;
					const int y1 = Yc.i1;
					const float ty = Yc.t;

					// Destination row pointer
					const int destY = tileOriginY + y;
					uint8* DestRow = OutPtrBase + (int64(destY) * Out2DRes + tileOriginX) * BytesPerVoxel;

					for (int x = 0; x < Out3DRes; ++x)
					{
						const auto& Xc = Precomputed[x];
						const int x0 = Xc.i0;
						const int x1 = Xc.i1;
						const float tx = Xc.t;

						// Input voxel neighbors
						const int base000 = InIndexBase(x0, y0, z0);
						const int base100 = InIndexBase(x1, y0, z0);
						const int base010 = InIndexBase(x0, y1, z0);
						const int base110 = InIndexBase(x1, y1, z0);
						const int base001 = InIndexBase(x0, y0, z1);
						const int base101 = InIndexBase(x1, y0, z1);
						const int base011 = InIndexBase(x0, y1, z1);
						const int base111 = InIndexBase(x1, y1, z1);

						float outC[4];

						for (int c = 0; c < BytesPerVoxel; ++c)
						{
							float c000 = float(InPtr[base000 + c]) * (1.f / 255.f);
							float c100 = float(InPtr[base100 + c]) * (1.f / 255.f);
							float c010 = float(InPtr[base010 + c]) * (1.f / 255.f);
							float c110 = float(InPtr[base110 + c]) * (1.f / 255.f);
							float c001 = float(InPtr[base001 + c]) * (1.f / 255.f);
							float c101 = float(InPtr[base101 + c]) * (1.f / 255.f);
							float c011 = float(InPtr[base011 + c]) * (1.f / 255.f);
							float c111 = float(InPtr[base111 + c]) * (1.f / 255.f);

							// Trilinear
							const float c00 = FMath::Lerp(c000, c100, tx);
							const float c10 = FMath::Lerp(c010, c110, tx);
							const float c01 = FMath::Lerp(c001, c101, tx);
							const float c11 = FMath::Lerp(c011, c111, tx);
							const float c0 = FMath::Lerp(c00, c10, ty);
							const float c1 = FMath::Lerp(c01, c11, ty);
							outC[c] = FMath::Lerp(c0, c1, tz);
						}

						// Write 4 channels
						DestRow[x * BytesPerVoxel + 0] = uint8(outC[0] * 255.0f);
						DestRow[x * BytesPerVoxel + 1] = uint8(outC[1] * 255.0f);
						DestRow[x * BytesPerVoxel + 2] = uint8(outC[2] * 255.0f);
						DestRow[x * BytesPerVoxel + 3] = uint8(outC[3] * 255.0f);
					}
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("UpscaleToPseudoVolume (%d^3 -> 256^3 -> 4096^2) took %.3f sec"), InRes, Duration);

		return OutData;
	}

	//MUST ALWAYS BE CALLED OFF GAME THREAD
	static UTexture2D* GeneratePseudoVolumeTextureFromMipData(const TArray<uint8>& InMipData)
	{
		double StartTime = FPlatformTime::Seconds();
		
		// Calculate expected 2D pseudo-volume layout parameters, will always output 256^3 to match noise texture dimensions
		const int32 BytesPerVoxel = 4;
		int32 tilesPerSide = 16;
		int32 OutRes = 4096;
		const int32 ExpectedBytes = 67108864; // 4096 * 4096 * 4

		if (InMipData.Num() != ExpectedBytes)
		{
			UE_LOG(LogTemp, Error, TEXT("Size mismatch: Expected %lld bytes, got %d bytes"), ExpectedBytes, InMipData.Num());
			return nullptr;
		}

		#pragma region Create Placeholder Texture on Game Thread
		// --- 1) Create dummy UTexture2D on GameThread (with 1x1 mip) ---
		UTexture2D* PseudoVolumeTexture = nullptr;
		FEvent* DummyDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		DummyDoneEvent->Reset();
		AsyncTask(ENamedThreads::GameThread, [OutRes, &PseudoVolumeTexture, DummyDoneEvent]()
			{
				PseudoVolumeTexture = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
				if (!PseudoVolumeTexture) { DummyDoneEvent->Trigger(); return; }
				PseudoVolumeTexture->NeverStream = true;
				FTexturePlatformData* PlatformData = new FTexturePlatformData();
				PlatformData->SizeX = 1;
				PlatformData->SizeY = 1;
				PlatformData->PixelFormat = PF_B8G8R8A8;
				FTexture2DMipMap* Mip = new FTexture2DMipMap();
				Mip->SizeX = 1; Mip->SizeY = 1;
				Mip->BulkData.Lock(LOCK_READ_WRITE);
				void* ReallocPtr = Mip->BulkData.Realloc(1 * 1 * GPixelFormats[PF_B8G8R8A8].BlockBytes);
				if (ReallocPtr) { FMemory::Memzero(ReallocPtr, GPixelFormats[PF_B8G8R8A8].BlockBytes); }
				Mip->BulkData.Unlock();
				PlatformData->Mips.Add(Mip);
				PseudoVolumeTexture->SetPlatformData(PlatformData);
				PseudoVolumeTexture->SRGB = false;
				PseudoVolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
				PseudoVolumeTexture->CompressionNone = true;
				PseudoVolumeTexture->Filter = TF_Nearest;
				PseudoVolumeTexture->MipGenSettings = TMGS_NoMipmaps;
				PseudoVolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
				PseudoVolumeTexture->NeverStream = true;
				PseudoVolumeTexture->DeferCompression = true;
				PseudoVolumeTexture->UnlinkStreaming();
				PseudoVolumeTexture->UpdateResource();
				DummyDoneEvent->Trigger();
			});
		DummyDoneEvent->Wait();
		FPlatformProcess::ReturnSynchEventToPool(DummyDoneEvent);
		if (!PseudoVolumeTexture)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create dummy texture"));
			return nullptr;
		}
		#pragma endregion

		#pragma region RHI Async Texture Creation
		void* MipData[1] = { const_cast<uint8*>(InMipData.GetData()) };
		FGraphEventRef CompletionEvent;
		FTexture2DRHIRef RHITex = RHIAsyncCreateTexture2D(OutRes, OutRes, PF_B8G8R8A8, 1, TexCreate_ShaderResource, ERHIAccess::SRVMask, MipData, 1, TEXT("PseudoVolumeTexture"), CompletionEvent);
		if (CompletionEvent.IsValid()) { CompletionEvent->Wait(); }
		FRHITexture* RHITexture = RHITex.GetReference();
		if (!RHITexture)
		{
			UE_LOG(LogTemp, Warning, TEXT("Async RHI texture creation failed"));
			return nullptr;
		}
		#pragma endregion
	
		#pragma region RHI Switch Texture Reference on Render Thread
		FEvent* LinkDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		LinkDoneEvent->Reset();
		ENQUEUE_RENDER_COMMAND(LinkTextureCmd)([PseudoVolumeTexture, RHITexture, LinkDoneEvent](FRHICommandListImmediate& RHICmdList)
			{
				RHIUpdateTextureReference(PseudoVolumeTexture->TextureReference.TextureReferenceRHI, static_cast<FTexture2DRHIRef>(RHITexture));
				PseudoVolumeTexture->RefreshSamplerStates();
				LinkDoneEvent->Trigger();
			});
		LinkDoneEvent->Wait();
		FPlatformProcess::ReturnSynchEventToPool(LinkDoneEvent);
		#pragma endregion

		double TotalDuration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("OctreeTextureProcessor::Async pseudovolume texture linking duration: %.3f seconds"), TotalDuration);
		
		return PseudoVolumeTexture;
	}
};
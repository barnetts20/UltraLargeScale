#pragma once

#pragma region Includes/ForwardDec
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
#pragma endregion

class SVO_API FOctreeNode : public TSharedFromThis<FOctreeNode>
{
public:
#pragma region Public Parameters
	TArray<uint8> Index;
	int Depth;
	TWeakPtr<FOctreeNode> Parent;
	TSharedPtr<FOctreeNode> Children[8];
	FVector Center;
	double Extent;
	FVoxelData Data = FVoxelData(); //Default constructor with 0 ScaleFactor -1 ObectId
#pragma endregion

#pragma region Constructor/Destructor
	FOctreeNode() {
		Index = TArray<uint8>();
		Center = FVector::ZeroVector;
		Extent = 0;
		Depth = Index.Num();
	};

	FOctreeNode(FVector InCenter, double InExtent, TArray<uint8> InIndex, TWeakPtr<FOctreeNode> InParent) {
		Index = InIndex;
		Center = InCenter;
		Extent = InExtent;
		Parent = InParent;
		Depth = Index.Num();
	};
#pragma endregion
};

class SVO_API FOctree : public TSharedFromThis<FOctree>
{
public:
#pragma region Public Parameters
	double Extent; //Must be power of 2, eg 1024 2048 etc
	TSharedPtr<FOctreeNode> Root;

	int MaxDepth;
	// Default depth at which BulkInsertPositions pre-populates the chunk grid
	// for parallel insert distribution. 2 ⇒ 8^2 = 64 chunks, which balances
	// parallel granularity against per-cross prepopulate cost. The previous
	// default (5 ⇒ 32,768 chunks) was tuned for one-shot full-volume bulk
	// loads and is far too heavy for streaming-style boundary-cross updates
	// where only a handful of cells enter per cross.
	int VolumeDepth = 2;
#pragma endregion

#pragma region Locks
	mutable FCriticalSection OctreeMutex;
	std::atomic<bool> bIsResetting{ false };
#pragma endregion

#pragma region BulkInsert
	void BulkInsertPositions(TArray<FPointData> InPointData, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes, TArray<TSharedPtr<FOctreeNode>>& OutVolumeChunks) {
		if (bIsResetting.load()) {
			return; // Early exit if shutting down
		}

		double StartTime = FPlatformTime::Seconds();

		// ---------------- Prepopulation ----------------
		TArray<TArray<FPointData>> ChunkPointData;
		PrePopulateVolumeLayer(OutVolumeChunks, ChunkPointData);

		// ---------------- Point Distribution ----------------
		for (const FPointData& Point : InPointData) {
			int ChunkIndex = FindChunkIndexForPosition(Point.GetPosition(), OutVolumeChunks);
			if (ChunkIndex >= 0 && ChunkIndex < ChunkPointData.Num()) {
				ChunkPointData[ChunkIndex].Add(Point);
			}
		}

		// ---------------- Parallel Chunk Inserts ----------------
		const int NumChunks = OutVolumeChunks.Num();
		TArray<TArray<TSharedPtr<FOctreeNode>>> PerChunkResults;
		PerChunkResults.SetNum(NumChunks);

		ParallelFor(NumChunks, [&](int i) {
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

				TSharedPtr<FOctreeNode> Result = InsertPosition(Point.GetPosition(), Point.InsertDepth, Point.Data, Chunk);
				if (Result.IsValid()) {
					ChunkResults.Add(Result);
				}
			}

			PerChunkResults[i] = MoveTemp(ChunkResults);
			}, EParallelForFlags::BackgroundPriority);

		// ---------------- Recombine Results ----------------
		OutInsertedNodes.Empty();

		int TotalResults = 0;
		for (const auto& Arr : PerChunkResults) {
			TotalResults += Arr.Num();
		}
		OutInsertedNodes.Reserve(TotalResults);

		for (int i = 0; i < NumChunks; ++i) {
			if (PerChunkResults[i].Num() > 0) {
				OutInsertedNodes.Append(PerChunkResults[i]);
			}
		}

		// ---------------- Total ----------------
		double EndTime = FPlatformTime::Seconds();
		UE_LOG(LogTemp, Log, TEXT("BulkInsert: Total duration %.3f sec"), EndTime - StartTime);
	}

	int64 FindChunkIndexForPosition(FVector Position, const TArray<TSharedPtr<FOctreeNode>>& VolumeChunks) const {
		const int NodesPerSide = 1 << VolumeDepth;
		const double ChunkSize = (Extent * 2.0) / NodesPerSide;
		const double HalfExtent = Extent;

		const double GridX = (Position.X + HalfExtent) / ChunkSize;
		const double GridY = (Position.Y + HalfExtent) / ChunkSize;
		const double GridZ = (Position.Z + HalfExtent) / ChunkSize;

		// Convert to int and clamp
		const int iGridX = FMath::Clamp(FMath::FloorToInt(GridX), 0, NodesPerSide - 1);
		const int iGridY = FMath::Clamp(FMath::FloorToInt(GridY), 0, NodesPerSide - 1);
		const int iGridZ = FMath::Clamp(FMath::FloorToInt(GridZ), 0, NodesPerSide - 1);

		return (int64)iGridX + (int64)iGridY * NodesPerSide + (int64)iGridZ * NodesPerSide * NodesPerSide;
	}

	void PrePopulateVolumeLayer(TArray<TSharedPtr<FOctreeNode>>& OutVolumeChunks, TArray<TArray<FPointData>>& OutChunkPointData) {
		const int NodesPerSide = 1 << VolumeDepth;
		const double ChunkExtent = Extent / (1 << VolumeDepth);
		const int TotalNodeCount = NodesPerSide * NodesPerSide * NodesPerSide;
		OutVolumeChunks.SetNum(TotalNodeCount);
		OutChunkPointData.SetNum(TotalNodeCount);

		for (int x = 0; x < NodesPerSide; ++x) {
			for (int y = 0; y < NodesPerSide; ++y) {
				for (int z = 0; z < NodesPerSide; ++z) {
					int idx = x + y * NodesPerSide + z * NodesPerSide * NodesPerSide;

					FVector ChunkCenter = FVector(
						(x - NodesPerSide / 2) * ChunkExtent * 2.0 + ChunkExtent,
						(y - NodesPerSide / 2) * ChunkExtent * 2.0 + ChunkExtent,
						(z - NodesPerSide / 2) * ChunkExtent * 2.0 + ChunkExtent
					);

					TSharedPtr<FOctreeNode> ChunkNode = InsertPosition(ChunkCenter, VolumeDepth, FVoxelData(0, 0, FVector::ZeroVector, -1, -1));

					OutVolumeChunks[idx] = ChunkNode;
					OutChunkPointData[idx] = TArray<FPointData>();
				}
			}
		}
	}

	TSharedPtr<FOctreeNode> InsertPosition(FVector InPosition, int InDepth, FVoxelData InData, TSharedPtr<FOctreeNode> InCurrent = nullptr) {
		if (bIsResetting.load()) {
			return nullptr; // Early exit if shutting down
		}
		if (InData.TypeId == -1 && InPosition == FVector::ZeroVector) return nullptr; //Ignore typeless inserts into 0,0,0

		TSharedPtr<FOctreeNode> Current = Root;
		double CurrentExtent = Extent;

		if (InCurrent) {
			Current = InCurrent;
			CurrentExtent = Current->Extent;
		}
		else {
			//Before we build our chunks, insert needs to be serial to avoid collisions
			FScopeLock Lock(&OctreeMutex);
		}

		for (int Depth = Current->Depth; Depth < InDepth; Depth++) {
			CurrentExtent /= 2;

			uint8 ChildIndex = 0;
			if (InPosition.X >= Current->Center.X) ChildIndex |= 1;
			if (InPosition.Y >= Current->Center.Y) ChildIndex |= 2;
			if (InPosition.Z >= Current->Center.Z) ChildIndex |= 4;

			if (!Current->Children[ChildIndex].IsValid()) {
				double OffsetX = ((ChildIndex & 1) ? 1 : -1) * CurrentExtent;
				double OffsetY = ((ChildIndex & 2) ? 1 : -1) * CurrentExtent;
				double OffsetZ = ((ChildIndex & 4) ? 1 : -1) * CurrentExtent;

				FVector ChildCenter = FVector(
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

		// Collision-aware write. Existing convention: ObjectId == -1 means
		// "node has no payload yet". First insert lands in ObjectId; later
		// inserts at the same node append to AdditionalObjectIds. Other
		// fields (ScaleFactor, Density, Composition, TypeId) take the first
		// inserter's values — this matches the legacy single-ObjectId
		// readers' expectations and avoids a meaningless "merge" of
		// per-particle aux data. If you need full provenance for collision
		// cases, walk the indices and look the data up in your own buffers.
		if (Current->Data.ObjectId == -1)
		{
			Current->Data = InData;
		}
		else if (InData.ObjectId != -1 && InData.ObjectId != Current->Data.ObjectId)
		{
			Current->Data.AdditionalObjectIds.Add(InData.ObjectId);
		}
		// Carry over any AdditionalObjectIds the incoming InData itself was
		// already aggregating (re-inserts of merged data, etc.).
		for (int32 ExtraId : InData.AdditionalObjectIds)
		{
			if (ExtraId != Current->Data.ObjectId
				&& !Current->Data.AdditionalObjectIds.Contains(ExtraId))
			{
				Current->Data.AdditionalObjectIds.Add(ExtraId);
			}
		}
		return Current;
	}

	// Remove a specific ObjectId from a node. If it was the primary
	// ObjectId, promote the next AdditionalObjectIds entry into the slot.
	// Used by streaming code on cell-exit to retire one cell's slot from
	// nodes that may still hold other cells' slots after collisions.
	// Returns true if the node has no remaining ObjectIds (caller may want
	// to also clear TypeId / mark the node empty).
	bool RemoveObjectIdFromNode(const TSharedPtr<FOctreeNode>& InNode, int32 InObjectId) {
		if (!InNode.IsValid()) return false;

		if (InNode->Data.ObjectId == InObjectId)
		{
			if (InNode->Data.AdditionalObjectIds.Num() > 0)
			{
				InNode->Data.ObjectId = InNode->Data.AdditionalObjectIds[0];
				InNode->Data.AdditionalObjectIds.RemoveAt(0);
			}
			else
			{
				InNode->Data.ObjectId = -1;
				InNode->Data.TypeId = -1;
				return true;
			}
		}
		else
		{
			InNode->Data.AdditionalObjectIds.RemoveSingle(InObjectId);
		}
		return false;
	}
#pragma endregion

#pragma region Fetch Operations
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

	void CollectNodesAtDepth(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InTargetDepth) const {
		if (!InNode.IsValid()) return;
		if (InNode->Depth == InTargetDepth) {
			OutNodes.Add(InNode);
			return; // Don't recurse deeper
		}
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				CollectNodesAtDepth(Child, OutNodes, InTargetDepth);
			}
		}
	}

	// Add to FOctree class in FOctree.h
	float SampleDensityAtPosition(const FVector& InPosition) const
	{
		if (!Root.IsValid()) return 0.0f;

		// Optional: Quick bounds check
		if (FMath::Abs(InPosition.X) > Extent ||
			FMath::Abs(InPosition.Y) > Extent ||
			FMath::Abs(InPosition.Z) > Extent)
		{
			return 0.0f;
		}

		float AccumulatedDensity = 0.0f;
		TSharedPtr<FOctreeNode> Current = Root;

		// Traverse down the tree
		while (Current.IsValid())
		{
			// Accumulate density from this level
			// Note: This assumes FVoxelData has a float 'Density' member
			AccumulatedDensity += Current->Data.Density;

			// Determine which child contains the position
			uint8 ChildIndex = 0;
			if (InPosition.X >= Current->Center.X) ChildIndex |= 1;
			if (InPosition.Y >= Current->Center.Y) ChildIndex |= 2;
			if (InPosition.Z >= Current->Center.Z) ChildIndex |= 4;

			if (Current->Children[ChildIndex].IsValid())
			{
				Current = Current->Children[ChildIndex];
			}
			else
			{
				// Reached the deepest existing node for this position
				break;
			}
		}

		return AccumulatedDensity;
	}

	TArray<TSharedPtr<FOctreeNode>> GetNodesAtDepth(int InTargetDepth) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid()) {
			CollectNodesAtDepth(Root, Nodes, InTargetDepth);
		}
		return Nodes;
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

	void CollectNodesInRange(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FVector& InCenter, const double InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		const FVector QueryMin = InCenter - FVector(InExtent, InExtent, InExtent);
		const FVector QueryMax = InCenter + FVector(InExtent, InExtent, InExtent);
		const FVector NodeMin = InNode->Center - FVector(InNode->Extent, InNode->Extent, InNode->Extent);
		const FVector NodeMax = InNode->Center + FVector(InNode->Extent, InNode->Extent, InNode->Extent);

		const bool bIntersects = NodeMin.X <= QueryMax.X && NodeMax.X >= QueryMin.X && NodeMin.Y <= QueryMax.Y && NodeMax.Y >= QueryMin.Y && NodeMin.Z <= QueryMax.Z && NodeMax.Z >= QueryMin.Z;
		if (!bIntersects) return;

		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children)
		{
			if (Child.IsValid())
			{
				CollectNodesInRange(Child, OutNodes, InCenter, InExtent, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter != -1 && InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false;

		if (bPassesFilter)
		{
			OutNodes.Add(InNode);
		}
	}

	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(FVector InCenter, double InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			CollectNodesInRange(Root, Nodes, InCenter, InExtent, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Nodes;
	}

	void CollectNodesByScreenSpace(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FVector& InCenter, const double InExtent, double ScreenSpaceThreshold, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		const FVector QueryMin = InCenter - FVector(InExtent, InExtent, InExtent);
		const FVector QueryMax = InCenter + FVector(InExtent, InExtent, InExtent);
		const FVector NodeMin = InNode->Center - FVector(InNode->Extent, InNode->Extent, InNode->Extent);
		const FVector NodeMax = InNode->Center + FVector(InNode->Extent, InNode->Extent, InNode->Extent);

		const bool bIntersects = NodeMin.X <= QueryMax.X && NodeMax.X >= QueryMin.X && NodeMin.Y <= QueryMax.Y && NodeMax.Y >= QueryMin.Y && NodeMin.Z <= QueryMax.Z && NodeMax.Z >= QueryMin.Z;
		if (!bIntersects) return;

		const double Distance = FVector::Dist(InNode->Center, InCenter);
		if (Distance > 0.0)
		{
			if ((InNode->Extent * (1.0 + InNode->Data.ScaleFactor)) / Distance < ScreenSpaceThreshold) return;
		}

		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children)
		{
			if (Child.IsValid())
			{
				CollectNodesByScreenSpace(Child, OutNodes, InCenter, InExtent, ScreenSpaceThreshold,
					InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter != -1 && InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false;

		if (bPassesFilter)
		{
			OutNodes.Add(InNode);
		}
	}

	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(const FVector& InCenter, const double InExtent, const double ScreenSpaceThreshold, const int InMinDepth = -1, const int InMaxDepth = -1, const int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			CollectNodesByScreenSpace(Root, Nodes, InCenter, InExtent, ScreenSpaceThreshold, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Nodes;
	}
#pragma endregion

#pragma region Constructor/Destructor 
	FOctree(double InExtent) {
		Extent = InExtent;
		MaxDepth = static_cast<int>(FMath::Log2(Extent));
		Root = MakeShared<FOctreeNode>(FVector::ZeroVector, Extent, TArray<uint8>(), nullptr);
	}

	// Center-aware overload. Used by ASectorActor to corner-align the tree
	// so the sector's 3x3x3 coarse-cell neighborhood occupies a 3-of-4
	// segment of the tree's depth-2 grid (the 4th cell being the buffer
	// ring on each axis). With InCenter = (Sector::Extent, Sector::Extent,
	// Sector::Extent) and InExtent = 4 * Sector::Extent the tree spans
	// [-3*SE, +5*SE] and depth-2 node centers fall on coarse cell centers.
	// InCenter defaults to ZeroVector for legacy callers.
	FOctree(double InExtent, FVector InCenter) {
		Extent = InExtent;
		MaxDepth = static_cast<int>(FMath::Log2(Extent));
		Root = MakeShared<FOctreeNode>(InCenter, Extent, TArray<uint8>(), nullptr);
	}

	// Drop all nodes and rebuild an empty root. Used by streaming code that
	// rebuilds the spatial index every boundary cross via BulkInsertPositions
	// (insert-first pipeline). bIsResetting is flipped briefly so any in-
	// flight BulkInsert / GetNodesInRange calls early-out instead of touching
	// half-detached nodes. Preserves the existing root center.
	void Reset() {
		bIsResetting.store(true);
		{
			FScopeLock Lock(&OctreeMutex);
			const FVector CenterToKeep = Root.IsValid() ? Root->Center : FVector::ZeroVector;
			Root.Reset();
			Root = MakeShared<FOctreeNode>(CenterToKeep, Extent, TArray<uint8>(), nullptr);
		}
		bIsResetting.store(false);
	}
#pragma endregion
};

//class SVO_API FOctreeTextureProcessor {
//public:
//	#pragma region Extract Mip Data From Volume Nodes
//	static TArray<uint8> GenerateVolumeMipDataFromOctree(TArray<TSharedPtr<FOctreeNode>> InVolumeNodes, int InResolution, double InExtent, double InMaxDensity)
//	{
//		double StartTime = FPlatformTime::Seconds();
//
//		// --- Octree + setup ---
//		const int TargetDepth = FMath::FloorLog2(InResolution);
//		const double NodeExtentAtDepth = InExtent / FMath::Pow(2.0, TargetDepth);
//		const double OctreeExtent = InExtent;
//
//		// Precompute reciprocal for fast coordinate mapping
//		const double Scale = 1.0 / (2.0 * NodeExtentAtDepth);
//
//		// --- Texture allocation ---
//		const int BytesPerVoxel = 4; // BGRA8
//		const int64 TotalVoxels = (int64)InResolution * InResolution * InResolution;
//		const int64 TotalBytes = TotalVoxels * BytesPerVoxel;
//
//		TArray<uint8> TextureData;
//		TextureData.SetNumZeroed(TotalBytes);
//
//		// --- Parallel fill ---
//		// Use chunked ParallelFor to reduce scheduling overhead
//		const int ChunkSize = 512;
//		const int NumChunks = (InVolumeNodes.Num() + ChunkSize - 1) / ChunkSize;
//
//		ParallelFor(NumChunks, [&](int ChunkIdx){
//			const int Start = ChunkIdx * ChunkSize;
//			const int End = FMath::Min(Start + ChunkSize, InVolumeNodes.Num());
//
//			for (int NodeIndex = Start; NodeIndex < End; NodeIndex++)
//			{
//				const auto& Node = InVolumeNodes[NodeIndex];
//				if (!Node.IsValid()) continue;
//
//				// --- Compute voxel coords (multiply instead of divide) ---
//				int VolumeX = static_cast<int>((Node->Center.X + OctreeExtent) * Scale);
//				int VolumeY = static_cast<int>((Node->Center.Y + OctreeExtent) * Scale);
//				int VolumeZ = static_cast<int>((Node->Center.Z + OctreeExtent) * Scale);
//
//				if (VolumeX < 0 || VolumeX >= InResolution ||
//					VolumeY < 0 || VolumeY >= InResolution ||
//					VolumeZ < 0 || VolumeZ >= InResolution)
//				{
//					continue;
//				}
//
//				// --- Linear voxel index ---
//				int64 VoxelIndex = ((int64)VolumeZ * InResolution * InResolution) + ((int64)VolumeY * InResolution) + VolumeX;
//				int64 ByteIndex = VoxelIndex * BytesPerVoxel;
//
//				// --- ScaleFactor ---
//				uint8 DensityByte = 0;
//				if (InMaxDensity > 0.0f)
//				{
//					float Norm = (float)Node->Data.Density / InMaxDensity;
//					DensityByte = (uint8)FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f);
//				}
//
//				// --- Composition ---
//				FVector Comp = Node->Data.Composition;
//
//				TextureData[ByteIndex + 0] = (uint8)FMath::Clamp(Comp.X * 255.0f, 0.0f, 255.0f); // B
//				TextureData[ByteIndex + 1] = (uint8)FMath::Clamp(Comp.Y * 255.0f, 0.0f, 255.0f); // G
//				TextureData[ByteIndex + 2] = (uint8)FMath::Clamp(Comp.Z * 255.0f, 0.0f, 255.0f); // R
//				TextureData[ByteIndex + 3] = DensityByte; // A
//			}
//		}, EParallelForFlags::BackgroundPriority);
//
//		double TotalDuration = FPlatformTime::Seconds() - StartTime;
//		UE_LOG(LogTemp, Log, TEXT("OctreeTextureProcessor::Base volume mip data @%dx^3 generation duration: %.3f seconds"), InResolution, TotalDuration);
//
//		return TextureData;
//	}
//	#pragma endregion
//	
//	#pragma region Upscale Mip Data From InResolution^3 to 4096^2 Pseudo-volume
//	static TArray<uint8> UpscalePseudoVolumeDensityData(const TArray<uint8>& InMipData, int InResolution)
//	{
//		double StartTime = FPlatformTime::Seconds();
//
//		#pragma region 256^3 Constants
//		constexpr int Out3DRes = 256;												// fixed target 3D resolution
//		constexpr int BytesPerVoxel = 4;											// RGBA8
//		constexpr int tilesPerSide = 16;											// 16x16 = 256 slices
//		constexpr int Out2DRes = 4096;   // 4096
//		constexpr int OutBytes = 67108864; // 4096 * 4096 * 4
//		const int InSlice = InResolution * InResolution;
//		#pragma endregion
//
//		#pragma region Precompute Coordinates
//		struct SampleCoord { int i0, i1; float t; };
//		static TArray<SampleCoord> Precomputed;
//		static int CachedInRes = 0;
//		if (Precomputed.Num() == 0 || CachedInRes != InResolution)
//		{
//			Precomputed.SetNum(Out3DRes);
//			for (int i = 0; i < Out3DRes; i++)
//			{
//				float f = i * (float(InResolution - 1) / float(Out3DRes - 1));
//				int i0 = FMath::FloorToInt(f);
//				int i1 = FMath::Min(i0 + 1, InResolution - 1);
//				Precomputed[i] = { i0, i1, f - i0 };
//			}
//			CachedInRes = InResolution;
//		}
//		#pragma endregion
//
//		#pragma region Allocate Output/Index Lambda
//		TArray<uint8> OutData;
//		OutData.SetNumZeroed(OutBytes);
//
//		const uint8* InPtr = InMipData.GetData();
//		uint8* OutPtrBase = OutData.GetData();
//
//		// Lambda to compute base index into input voxel array
//		auto InIndexBase = [InResolution, InSlice, BytesPerVoxel](int x, int y, int z)->int {
//			return ((z * InSlice) + (y * InResolution) + x) * BytesPerVoxel;
//		};
//		#pragma endregion
//
//		#pragma region Parallel Slice Processing
//		// --- Parallel slice processing ---
//		ParallelFor(Out3DRes, [&](int z)
//			{
//				const auto& Zc = Precomputed[z];
//				const int z0 = Zc.i0;
//				const int z1 = Zc.i1;
//				const float tz = Zc.t;
//
//				// Tile origin for this z slice
//				const int tileX = z % tilesPerSide;
//				const int tileY = z / tilesPerSide;
//				const int tileOriginX = tileX * Out3DRes;
//				const int tileOriginY = tileY * Out3DRes;
//
//				for (int y = 0; y < Out3DRes; ++y)
//				{
//					const auto& Yc = Precomputed[y];
//					const int y0 = Yc.i0;
//					const int y1 = Yc.i1;
//					const float ty = Yc.t;
//
//					// Destination row pointer
//					const int destY = tileOriginY + y;
//					uint8* DestRow = OutPtrBase + (int64(destY) * Out2DRes + tileOriginX) * BytesPerVoxel;
//
//					for (int x = 0; x < Out3DRes; ++x)
//					{
//						const auto& Xc = Precomputed[x];
//						const int x0 = Xc.i0;
//						const int x1 = Xc.i1;
//						const float tx = Xc.t;
//
//						// Input voxel neighbors
//						const int base000 = InIndexBase(x0, y0, z0);
//						const int base100 = InIndexBase(x1, y0, z0);
//						const int base010 = InIndexBase(x0, y1, z0);
//						const int base110 = InIndexBase(x1, y1, z0);
//						const int base001 = InIndexBase(x0, y0, z1);
//						const int base101 = InIndexBase(x1, y0, z1);
//						const int base011 = InIndexBase(x0, y1, z1);
//						const int base111 = InIndexBase(x1, y1, z1);
//
//						float outC[4];
//
//						for (int c = 0; c < BytesPerVoxel; ++c)
//						{
//							float c000 = float(InPtr[base000 + c]) * (1.f / 255.f);
//							float c100 = float(InPtr[base100 + c]) * (1.f / 255.f);
//							float c010 = float(InPtr[base010 + c]) * (1.f / 255.f);
//							float c110 = float(InPtr[base110 + c]) * (1.f / 255.f);
//							float c001 = float(InPtr[base001 + c]) * (1.f / 255.f);
//							float c101 = float(InPtr[base101 + c]) * (1.f / 255.f);
//							float c011 = float(InPtr[base011 + c]) * (1.f / 255.f);
//							float c111 = float(InPtr[base111 + c]) * (1.f / 255.f);
//
//							// Trilinear
//							const float c00 = FMath::Lerp(c000, c100, tx);
//							const float c10 = FMath::Lerp(c010, c110, tx);
//							const float c01 = FMath::Lerp(c001, c101, tx);
//							const float c11 = FMath::Lerp(c011, c111, tx);
//							const float c0 = FMath::Lerp(c00, c10, ty);
//							const float c1 = FMath::Lerp(c01, c11, ty);
//							outC[c] = FMath::Lerp(c0, c1, tz);
//						}
//
//						// Write 4 channels
//						DestRow[x * BytesPerVoxel + 0] = uint8(outC[0] * 255.0f);
//						DestRow[x * BytesPerVoxel + 1] = uint8(outC[1] * 255.0f);
//						DestRow[x * BytesPerVoxel + 2] = uint8(outC[2] * 255.0f);
//						DestRow[x * BytesPerVoxel + 3] = uint8(outC[3] * 255.0f);
//					}
//				}
//			}, EParallelForFlags::BackgroundPriority);
//		#pragma endregion;
//
//		double Duration = FPlatformTime::Seconds() - StartTime;
//		UE_LOG(LogTemp, Log, TEXT("UpscaleToPseudoVolume (%d^3 -> 256^3 -> 4096^2) took %.3f sec"), InResolution, Duration);
//
//		return OutData;
//	}
//	#pragma endregion
//	
//	#pragma region Async Texture Generation - Must be called off game thread
//	static UTexture2D* GeneratePseudoVolumeTextureFromMipData(const TArray<uint8>& InMipData)
//	{
//		double StartTime = FPlatformTime::Seconds();
//		
//		// Calculate expected 2D pseudo-volume layout parameters, will always output 256^3 to match noise texture dimensions
//		const int BytesPerVoxel = 4;
//		const int tilesPerSide = 16;
//		const int OutRes = 4096;
//		const int ExpectedBytes = 67108864; // 4096 * 4096 * 4
//
//		if (InMipData.Num() != ExpectedBytes)
//		{
//			UE_LOG(LogTemp, Error, TEXT("Size mismatch: Expected %lld bytes, got %d bytes"), ExpectedBytes, InMipData.Num());
//			return nullptr;
//		}
//
//		#pragma region Create Placeholder Texture on Game Thread
//		// --- 1) Create dummy UTexture2D on GameThread (with 1x1 mip) ---
//		UTexture2D* PseudoVolumeTexture = nullptr;
//		FEvent* DummyDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
//		DummyDoneEvent->Reset();
//		AsyncTask(ENamedThreads::GameThread, [OutRes, &PseudoVolumeTexture, DummyDoneEvent]()
//			{
//				PseudoVolumeTexture = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
//				if (!PseudoVolumeTexture) { DummyDoneEvent->Trigger(); return; }
//				PseudoVolumeTexture->NeverStream = true;
//				FTexturePlatformData* PlatformData = new FTexturePlatformData();
//				PlatformData->SizeX = 1;
//				PlatformData->SizeY = 1;
//				PlatformData->PixelFormat = PF_B8G8R8A8;
//				FTexture2DMipMap* Mip = new FTexture2DMipMap();
//				Mip->SizeX = 1; Mip->SizeY = 1;
//				Mip->BulkData.Lock(LOCK_READ_WRITE);
//				void* ReallocPtr = Mip->BulkData.Realloc(1 * 1 * GPixelFormats[PF_B8G8R8A8].BlockBytes);
//				if (ReallocPtr) { FMemory::Memzero(ReallocPtr, GPixelFormats[PF_B8G8R8A8].BlockBytes); }
//				Mip->BulkData.Unlock();
//				PlatformData->Mips.Add(Mip);
//				PseudoVolumeTexture->SetPlatformData(PlatformData);
//				PseudoVolumeTexture->SRGB = false;
//				PseudoVolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
//				PseudoVolumeTexture->CompressionNone = true;
//				PseudoVolumeTexture->Filter = TF_Nearest;
//				PseudoVolumeTexture->MipGenSettings = TMGS_NoMipmaps;
//				PseudoVolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
//				PseudoVolumeTexture->NeverStream = true;
//				PseudoVolumeTexture->DeferCompression = true;
//				PseudoVolumeTexture->UnlinkStreaming();
//				PseudoVolumeTexture->UpdateResource();
//				DummyDoneEvent->Trigger();
//			});
//		DummyDoneEvent->Wait();
//		FPlatformProcess::ReturnSynchEventToPool(DummyDoneEvent);
//		if (!PseudoVolumeTexture)
//		{
//			UE_LOG(LogTemp, Error, TEXT("Failed to create dummy texture"));
//			return nullptr;
//		}
//		#pragma endregion
//
//		#pragma region RHI Async Texture Creation
//		void* MipData[1] = { const_cast<uint8*>(InMipData.GetData()) };
//		FGraphEventRef CompletionEvent;
//		FTexture2DRHIRef RHITex = RHIAsyncCreateTexture2D(OutRes, OutRes, PF_B8G8R8A8, 1, TexCreate_ShaderResource, ERHIAccess::SRVMask, MipData, 1, TEXT("PseudoVolumeTexture"), CompletionEvent);
//		if (CompletionEvent.IsValid()) { CompletionEvent->Wait(); }
//		FRHITexture* RHITexture = RHITex.GetReference();
//		if (!RHITexture)
//		{
//			UE_LOG(LogTemp, Warning, TEXT("Async RHI texture creation failed"));
//			return nullptr;
//		}
//		#pragma endregion
//	
//		#pragma region RHI Switch Texture Reference on Render Thread
//		FEvent* LinkDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
//		LinkDoneEvent->Reset();
//		ENQUEUE_RENDER_COMMAND(LinkTextureCmd)([PseudoVolumeTexture, RHITexture, LinkDoneEvent](FRHICommandListImmediate& RHICmdList)
//			{
//				RHIUpdateTextureReference(PseudoVolumeTexture->TextureReference.TextureReferenceRHI, static_cast<FTexture2DRHIRef>(RHITexture));
//				PseudoVolumeTexture->RefreshSamplerStates();
//				LinkDoneEvent->Trigger();
//			});
//		LinkDoneEvent->Wait();
//		FPlatformProcess::ReturnSynchEventToPool(LinkDoneEvent);
//		#pragma endregion
//
//		double TotalDuration = FPlatformTime::Seconds() - StartTime;
//		UE_LOG(LogTemp, Log, TEXT("OctreeTextureProcessor::Async pseudovolume texture linking duration: %.3f seconds"), TotalDuration);
//		
//		return PseudoVolumeTexture;
//	}
//	#pragma endregion
//};
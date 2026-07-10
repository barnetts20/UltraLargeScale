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
	// TODO: Shift to int64 or composite int64 morton index instead of heavyweight TArray<uint8>
	TArray<uint8> Index;
	int Depth;
	TWeakPtr<FOctreeNode> Parent;
	TSharedPtr<FOctreeNode> Children[8];
	FVector Center;
	double Extent;
	FVoxelData Data = FVoxelData();
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
	//TODO: WITH SOME MODIFICATION, WE MAY BE ABLE TO USE THE BULK INSERT PATH WHEN REGENERATING A SET OF CELLS... CHUNK DEPTH WOULD HAVE TO BE ADAPTIVE INSTEAD OF STATIC, AND IT WOULD HAVE TO OPERATE FROM A GIVEN NODE INSTEAD OF ROOT
	//TODO: BUT WE MAY GET A PERF IMPROVEMENT ON OCTREE INSERTION SO WORTH THINKING ABOUT
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

		// Clamp to MaxDepth so we never subdivide past the representable
		// precision of the tree. MakePointDataFromWorldScale can return
		// depths up to log2(TreeExtent); without this clamp, very small
		// particles would create nodes with sub-unit extents.
		const int ClampedDepth = FMath::Min(InDepth, MaxDepth);

		TSharedPtr<FOctreeNode> Current = Root;
		double CurrentExtent = Extent;

		if (InCurrent) {
			Current = InCurrent;
			CurrentExtent = Current->Extent;
		}

		for (int Depth = Current->Depth; Depth < ClampedDepth; Depth++) {
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

				Current->Children[ChildIndex] = MakeShared<FOctreeNode>(
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
		//TODO: INSERT COLLISIONS MAY NOT MAKE SENSE FOR THE USE CASE ANYMORE... WE STORE MORE THAN AN OBJECT ID IN THE NODE
		//TODO: SO ANY COLLIDED OBJECTS WOULD HAVE INCOMPLETE DATA/COULDN'T BE RECONSTRUCTED. IF WE WANT TO KEEP THIS FUNCTIONALITY, WE WOULD NEED TO MOVE THE DATA PAYLOAD OUT OF THE NODE INTO A MAP OR SOMETHING KEYED ON OBJECT ID
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

#pragma endregion

#pragma region Fetch Operations
	void CollectLeafNodes(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		// At or past max depth — this node is effectively a leaf for this query.
		// Don't recurse into children; fall through to the filter test below.
		if (InMaxDepth >= 0 && InNode->Depth >= InMaxDepth) {
			bool bPassesFilter = true;
			if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
			if (InNode->Depth > InMaxDepth) bPassesFilter = false;
			if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
			else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }
			if (bPassesFilter) OutNodes.Add(InNode);
			return;
		}

		bool bIsLeaf = true;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				bIsLeaf = false;
				CollectLeafNodes(Child, OutNodes, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
		else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }

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

	//TODO: THIS SVO WAS DEVELOPED WITH A GENERIC INTENT... SOME OF THE LOGIC MAY NOT BE NEEDED FOR OUR PURPOSES ANY LONGER WITH A DENSITY FIRST GENERATION PARADIGM
	//TODO: AT THIS POINT WE COULD POTENTIALLY START *SPECIALIZING* 
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
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) return;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				CollectPopulatedNodes(Child, OutNodes, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InNode->Data.Density <= 0) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
		else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }

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
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) return;

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
		if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
		else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }

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

	void CollectNodesByScreenSpace(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FVector& InCenter, double ScreenSpaceThresholdSq, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		if (!InNode.IsValid()) return;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) return;

		const double DistSq = FVector::DistSquared(InNode->Center, InCenter);
		{
			constexpr double Sqrt3 = 1.7320508075688772;   // AABB corner distance factor
			const double MinPossibleDist = FMath::Sqrt(DistSq) - Sqrt3 * InNode->Extent;
			if (MinPossibleDist > 0.0)
			{
				const double MaxSubtreeExtent = 2.0 * InNode->Extent;
				if (MaxSubtreeExtent * MaxSubtreeExtent < ScreenSpaceThresholdSq * MinPossibleDist * MinPossibleDist) return;
			}
		}

		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children)
		{
			if (Child.IsValid())
			{
				CollectNodesByScreenSpace(Child, OutNodes, InCenter, ScreenSpaceThresholdSq, InMinDepth, InMaxDepth, InTypeIdFilter);
			}
		}

		// Per-node test — squared form of (Extent / Distance) >= Threshold,
		// using the EXACT particle position/extent captured at insert so the
		// scan maps 1:1 to the sprite the material renders. Legacy fallback
		// for nodes without captured data (ParticleExtent == 0): the old
		// quantized-node approximation, Extent * (1 + ScaleFactor) against
		// the node center.
		const bool bHasParticle = InNode->Data.ParticleExtent > 0.0f;
		const double TestExtent = bHasParticle ? static_cast<double>(InNode->Data.ParticleExtent) : InNode->Extent * (1.0 + InNode->Data.ScaleFactor);
		const double TestDistSq = bHasParticle ? FVector::DistSquared(InNode->Data.ParticlePosition, InCenter) : DistSq;

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
		else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }
		if (bPassesFilter) OutNodes.Add(InNode);
	}

	// Deepest existing node containing InPosition, stopping at InMaxDepth.
	// Pure read — creates nothing (contrast InsertPosition). Returns null
	// only when the tree has no root. Current caller: the universe octree
	// rebase, which remaps live spawn bookkeeping (SpawnedGalaxies /
	// TrackedSpawnNodes) from old-tree nodes to their new-tree counterparts.
	TSharedPtr<FOctreeNode> FindNodeAtPosition(const FVector& InPosition, int InMaxDepth) const {
		TSharedPtr<FOctreeNode> Current = Root;
		if (!Current.IsValid()) return nullptr;
		while (Current->Depth < InMaxDepth) {
			uint8 ChildIndex = 0;
			if (InPosition.X >= Current->Center.X) ChildIndex |= 1;
			if (InPosition.Y >= Current->Center.Y) ChildIndex |= 2;
			if (InPosition.Z >= Current->Center.Z) ChildIndex |= 4;
			if (!Current->Children[ChildIndex].IsValid()) break;
			Current = Current->Children[ChildIndex];
		}
		return Current;
	}

	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(const FVector& InCenter, double ScreenSpaceThreshold, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			const double ThresholdSq = ScreenSpaceThreshold * ScreenSpaceThreshold;
			CollectNodesByScreenSpace(Root, Nodes, InCenter, ThresholdSq, InMinDepth, InMaxDepth, InTypeIdFilter);
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

	// Center-aware overload: builds the tree around an arbitrary root center
	// instead of the origin. Current callers: AUniverseActor's octree rebase
	// (root re-centered on the player's VirtualTraversal) and the initial
	// universe tree (ZeroVector).
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
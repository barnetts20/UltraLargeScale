#pragma once

// TRIMMED TO WHAT THIS HEADER USES. It previously pulled in SavePackage, the asset
// registry, VolumeTexture and four RHI headers, none of which appear anywhere below --
// the RHIResources include even carried a "For FRHITexture3D" comment naming a type this
// file never mentions. FOctree.h is included by ProceduralSpaceActor.h,
// FTierStreamingSystem.h and others, so each of those translation units was compiling the
// asset registry and the RHI to get a node struct. Anything added here that needs more
// should include it here rather than leaning on a consumer.
#include "CoreMinimal.h"
#include <DataTypes.h>

/** One node in the sparse octree: its child-index path from root, cached depth,
 *  parent and child links, cube center and half-extent, and the FVoxelData
 *  payload claimed first-writer-wins by the first insert to reach it. */
class ULTRALARGESCALE_API FOctreeNode : public TSharedFromThis<FOctreeNode>
{
public:
#pragma region Public Parameters
	/** Child-index path (0-7 per level) from root to this node. */
	TArray<uint8> Index;

	/** Cached tree depth, equal to Index.Num(). */
	int Depth;

	/** Weak link to the parent; null for the root. */
	TWeakPtr<FOctreeNode> Parent;

	/** Eight child slots, indexed (x>=cx) | (y>=cy)<<1 | (z>=cz)<<2; null when absent. */
	TSharedPtr<FOctreeNode> Children[8];

	/** Cube center in tree-local space. */
	FVector Center;

	/** Cube half-extent; each child's extent is half its parent's. */
	double Extent;

	/** Payload claimed first-writer-wins; Seed == -1 marks an unclaimed node. */
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

/** Sparse octree over a bounded cube of half-extent Extent, subdividing to
 *  MaxDepth = floor(log2(Extent)). Inserts are first-writer-wins per quantized node.
 *
 *  FOUR ENTRY POINTS, and that is the whole class. InsertPosition (FTierStreamingSystem
 *  and AStarSystemActor), GetNodesByScreenSpace (every layer's spawn scan),
 *  FindNodeAtPosition (the universe rebase), and the two constructors -- plus the Root /
 *  Extent / MaxDepth / bIsResetting members that consumers read directly.
 *
 *  A TREE IS REPLACED, NOT RESET. Streaming rebuilds by constructing a fresh FOctree and
 *  swapping the shared pointer, raising bIsResetting on the outgoing tree so in-flight
 *  work early-outs; the outgoing tree is then handed to a worker and released there,
 *  because FOctreeNode owns its children through TSharedPtr and dropping the root
 *  destroys every node recursively and synchronously on whichever thread drops it.
 *
 *  WHAT WAS REMOVED, so it is not looked for. This began as a generic SVO and carried a
 *  parallel chunked bulk-insert path (BulkInsertPositions, PrePopulateVolumeLayer,
 *  FindChunkIndexForPosition, VolumeDepth), a Reset that cleared in place, and four query
 *  families beyond the screen-space one -- SampleDensityAtPosition, GetLeafNodes,
 *  GetPopulatedNodes, GetNodesAtDepth and GetNodesInRange with their Collect helpers.
 *  None had a caller anywhere in the module, and three carried live traps: the bulk path
 *  laid its chunk grid out about the ORIGIN while FOctree(InExtent, InCenter) exists so
 *  the root need not be there, SampleDensityAtPosition's bounds test had the same
 *  mismatch, and Reset released the whole tree synchronously under a lock on the calling
 *  thread -- the exact game-thread cost the swap-and-release arrangement above exists to
 *  avoid. Anything reinstated from history has to be read against those three before it
 *  is wired up. */
class ULTRALARGESCALE_API FOctree : public TSharedFromThis<FOctree>
{
public:
#pragma region Public Parameters
	/** Half-extent of the root cube in tree-local units. */
	double Extent;

	/** Root node, created by the constructors and never replaced in place. */
	TSharedPtr<FOctreeNode> Root;

	/** Deepest representable depth, floor(log2(Extent)); inserts clamp to it. */
	int MaxDepth;

#pragma endregion

#pragma region Locks
	/** Raised on a tree that is being retired, so an in-flight InsertPosition on a worker
	 *  bails instead of populating a tree nothing will read.
	 *
	 *  SET FROM OUTSIDE, never from in here. A tree is REPLACED rather than cleared --
	 *  AGalaxyActor::FinishStarSystemPoolReturn and AUniverseActor::FinishGalaxyPoolReturn
	 *  raise this, build a fresh tree, swap the shared pointer on the game thread and hand
	 *  the outgoing one to a worker to release. It is never lowered: the tree it belongs to
	 *  is on its way out. */
	std::atomic<bool> bIsResetting{ false };
#pragma endregion

#pragma region Insert
	/** Descends from Root (or InCurrent), creating child nodes to ClampedDepth =
	 *  min(InDepth, MaxDepth), and claims the target node's payload first-writer-
	 *  wins. Returns the target node, or null while resetting or for a typeless
	 *  insert at the origin. */
	TSharedPtr<FOctreeNode> InsertPosition(FVector InPosition, int InDepth, FVoxelData InData, TSharedPtr<FOctreeNode> InCurrent = nullptr) {
		if (bIsResetting.load()) {
			return nullptr; // Early exit if shutting down
		}
		if (InData.TypeId == -1 && InPosition == FVector::ZeroVector) return nullptr; //Ignore typeless inserts into 0,0,0

		// Clamp to MaxDepth so we never subdivide past the representable
		// precision of the tree. MakePointData can return
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

		// First-writer-wins payload. Seed == -1 means the node has no
		// payload yet; the first insert claims it and later inserts at the same
		// quantized node are dropped. Collided inserts are not tracked: the node
		// stores full per-particle data (density, composition, particle index),
		// which a bare Seed cannot reconstruct, so multi-hit aggregation has
		// no consumer.
		if (Current->Data.Seed == -1)
		{
			Current->Data = InData;
		}
		return Current;
	}

#pragma endregion

#pragma region Fetch Operations
	/** Recursively gathers nodes whose projected size exceeds ScreenSpaceThresholdSq,
	 *  pruning subtrees that cannot pass, and applying the depth and TypeId filters.
	 *  See the per-node test in the body. */
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

		// Per-node test: squared form of (Extent / Distance) >= Threshold, using
		// the exact particle position/extent captured at insert so the scan maps
		// 1:1 to the sprite the material renders. Fallback for nodes without
		// captured data (ParticleExtent == 0): the quantized-node approximation,
		// Extent * (1 + ScaleFactor) against the node center.
		//
		// THE TEST IS THE POINT OF THE FUNCTION, and it was computed into
		// TestExtent/TestDistSq and then never read -- the filter below tested only depth
		// and TypeId. What survived was the SUBTREE PRUNE above, which is deliberately
		// optimistic: it bounds a whole subtree by 2 * Extent taken at its nearest corner,
		// so it discards only subtrees that cannot possibly contain a passing node. Every
		// payload node inside an unpruned region was then emitted whatever its own
		// projected size, and the threshold pin did almost nothing.
		//
		// A NODE AT ZERO DISTANCE PASSES. TestDistSq of zero means the viewer is inside the
		// node, which is the largest a thing can project, so the comparison is written as a
		// product rather than a ratio and needs no division guard.
		const bool bHasParticle = InNode->Data.ParticleExtent > 0.0f;
		const double TestExtent = bHasParticle ? static_cast<double>(InNode->Data.ParticleExtent) : InNode->Extent * (1.0 + InNode->Data.ScaleFactor);
		const double TestDistSq = bHasParticle ? FVector::DistSquared(InNode->Data.ParticlePosition, InCenter) : DistSq;

		bool bPassesFilter = true;
		if (TestExtent * TestExtent < ScreenSpaceThresholdSq * TestDistSq) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InTypeIdFilter == -1) { if (InNode->Data.TypeId < 0) bPassesFilter = false; }
		else { if (InNode->Data.TypeId != InTypeIdFilter) bPassesFilter = false; }
		if (bPassesFilter) OutNodes.Add(InNode);
	}

	/** Deepest existing node containing InPosition, stopping at InMaxDepth. Pure
	 *  read: creates nothing (contrast InsertPosition), and returns null only when
	 *  the tree has no root. Used by the universe octree rebase, which remaps live
	 *  spawn bookkeeping (SpawnedGalaxies / TrackedSpawnNodes) from old-tree nodes
	 *  to their new-tree counterparts. */
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

	/** Nodes whose projected size exceeds ScreenSpaceThreshold, matching the filters. */
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
	/** floor(log2(InExtent)) -- the depth at which a node's extent reaches one tree-local
	 *  unit. FLOOR RATHER THAN A CAST: a cast truncates toward zero, so the two agree only
	 *  for InExtent >= 1 and a sub-unit tree would report depth 0 where it has none. Log2
	 *  of zero or a negative is -inf or NaN and narrows to an undefined int, so the
	 *  argument is floored at 1 first -- a tree that small has no subdivision to offer
	 *  either way. */
	static int DeriveMaxDepth(double InExtent) {
		return FMath::FloorToInt(FMath::Log2(FMath::Max(InExtent, 1.0)));
	}

	/** Builds an empty tree of half-extent InExtent centered on the origin. */
	FOctree(double InExtent) {
		Extent = InExtent;
		MaxDepth = DeriveMaxDepth(InExtent);
		Root = MakeShared<FOctreeNode>(FVector::ZeroVector, Extent, TArray<uint8>(), nullptr);
	}

	/** Builds an empty tree of half-extent InExtent around an arbitrary center.
	 *  Used by AUniverseActor's octree rebase (root re-centered on the player's
	 *  VirtualTraversal) and initial universe-tree construction. */
	FOctree(double InExtent, FVector InCenter) {
		Extent = InExtent;
		MaxDepth = DeriveMaxDepth(InExtent);
		Root = MakeShared<FOctreeNode>(InCenter, Extent, TArray<uint8>(), nullptr);
	}

#pragma endregion
};
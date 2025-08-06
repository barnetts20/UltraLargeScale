// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
#include "CoreMinimal.h"

class SVO_API FOctreeNode : public TSharedFromThis<FOctreeNode>
{
public:
	TArray<uint8> Index;
	TWeakPtr<FOctreeNode> Parent;
	TSharedPtr<FOctreeNode> Children[8];
	
	FInt64Coordinate Center;
	int64 Extent;
	int Depth;
	FVoxelData Data = FVoxelData(); //Default constructor with -1 Density -1 ObectId

	FOctreeNode() {
		Index = TArray<uint8>();
		Center = FInt64Coordinate();
		Extent = 0;
		Depth = Index.Num();
	};

	FOctreeNode(FInt64Coordinate InCenter, int64 InExtent, TArray<uint8> InIndex, TWeakPtr<FOctreeNode> InParent) {
		Index = InIndex;
		Center = InCenter;
		Extent = InExtent;
		Parent = InParent;
		Depth = Index.Num();
	};

	~FOctreeNode();
};

class SVO_API FOctree : public TSharedFromThis<FOctree>
{
public:
	int MaxDepth;
	int64 Extent; //Must be power of 2, eg 1024 2048 etc
	TSharedPtr<FOctreeNode> Root;

	TSharedPtr<FOctreeNode> InsertPosition(FInt64Coordinate InPosition, int InDepth, FVoxelData InData) {
		TSharedPtr<FOctreeNode> Current = Root;
		int64 CurrentExtent = Extent;

		for (int Depth = 0; Depth < InDepth; ++Depth) {
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

				FInt64Coordinate ChildCenter = FInt64Coordinate(
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

		Current->Data = InData;
		return Current;
	}
	
	TSharedPtr<FOctreeNode> InsertPosition(FInt64Coordinate InPosition, FVoxelData InData) {
		return InsertPosition(InPosition, MaxDepth, InData);
	}

	void CollectLeafNodes(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		bool bIsLeaf = true;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				bIsLeaf = false;
				CollectLeafNodes(Child, OutNodes, InMinDepth, InMaxDepth, InObjectIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InObjectIdFilter != -1 && InNode->Data.ObjectId != InObjectIdFilter) bPassesFilter = false;

		if (bIsLeaf && bPassesFilter) {
			OutNodes.Add(InNode);
		}
	}

	void CollectPopulatedNodes(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		if (!InNode.IsValid()) return;
		for (const TSharedPtr<FOctreeNode>& Child : InNode->Children) {
			if (Child.IsValid()) {
				CollectPopulatedNodes(Child, OutNodes, InMinDepth, InMaxDepth, InObjectIdFilter);
			}
		}

		bool bPassesFilter = true;
		if (InNode->Data.Density == -1 && InNode->Data.ObjectId == -1) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InObjectIdFilter != -1 && InNode->Data.ObjectId != InObjectIdFilter) bPassesFilter = false;
		
		if (bPassesFilter) {
			OutNodes.Add(InNode);
		}
	}



	TArray<TSharedPtr<FOctreeNode>> GetLeafNodes(int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Leaves;
		if (Root.IsValid()) {
			CollectLeafNodes(Root, Leaves, InMinDepth, InMaxDepth, InObjectIdFilter);
		}
		return Leaves;
	}
	
	TArray<TSharedPtr<FOctreeNode>> GetPopulatedNodes(int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid()) {
			CollectPopulatedNodes(Root, Nodes, InMinDepth, InMaxDepth, InObjectIdFilter);
		}
		return Nodes;
	}

	void CollectNodesInRange(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FInt64Coordinate& InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		if (!InNode.IsValid()) return;

		//Query Bounds
		const FInt64Coordinate QueryMin = InCenter - FInt64Coordinate(InExtent, InExtent, InExtent);
		const FInt64Coordinate QueryMax = InCenter + FInt64Coordinate(InExtent, InExtent, InExtent);

		//Node Bounds
		const FInt64Coordinate NodeMin = InNode->Center - FInt64Coordinate(InNode->Extent, InNode->Extent, InNode->Extent);
		const FInt64Coordinate NodeMax = InNode->Center + FInt64Coordinate(InNode->Extent, InNode->Extent, InNode->Extent);

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
				CollectNodesInRange(Child, OutNodes, InCenter, InExtent, InMinDepth, InMaxDepth, InObjectIdFilter);
			}
		}

		// Filtering checks
		bool bPassesFilter = true;
		if (InNode->Data.Density == -1 && InNode->Data.ObjectId == -1) bPassesFilter = false;
		if (InMinDepth >= 0 && InNode->Depth < InMinDepth) bPassesFilter = false;
		if (InMaxDepth >= 0 && InNode->Depth > InMaxDepth) bPassesFilter = false;
		if (InObjectIdFilter != -1 && InNode->Data.ObjectId != InObjectIdFilter) bPassesFilter = false;

		if (bPassesFilter && InNode->Data.Density > 0)
		{
			OutNodes.Add(InNode);
		}
	}
	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(FInt64Coordinate InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			CollectNodesInRange(Root, Nodes, InCenter, InExtent, InMinDepth, InMaxDepth, InObjectIdFilter);
		}
		return Nodes;
	}

	FOctree(int64 InExtent) {
		Extent = InExtent;
		MaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(Extent)));
		Root = MakeShared<FOctreeNode>(FInt64Coordinate(), Extent, TArray<uint8>(), nullptr);
	}
};
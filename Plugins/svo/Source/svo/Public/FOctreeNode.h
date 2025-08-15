// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
#include "UObject/SavePackage.h"
#include "RHICommandList.h"
#include "RenderResource.h"
#include "RHIResources.h"       // For FRHITexture3D
#include "RHIUtilities.h"   
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/VolumeTexture.h"
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
	FVoxelData Data = FVoxelData(); //Default constructor with 0 Density -1 ObectId

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
	TArray<double> DepthMaxDensities;
	TSharedPtr<FOctreeNode> Root;

	TSharedPtr<FOctreeNode> InsertPosition(FInt64Coordinate InPosition, int InDepth, FVoxelData InData) {
		TSharedPtr<FOctreeNode> Current = Root;
		int64 CurrentExtent = Extent;

		// Calculate the leaf volume once (volume at target depth InDepth)
		int64 LeafExtent = Extent >> InDepth; // Extent at target depth
		int64 LeafVolume = LeafExtent;

		for (int Depth = 0; Depth < InDepth; ++Depth) {
			int64 CurrentVolume = CurrentExtent;

			// Weight the contribution by volume ratio
			double VolumeWeight = static_cast<double>(CurrentVolume) / static_cast<double>(LeafVolume);

			// Accumulate density weighted by volume
			Current->Data.Density += InData.Density * VolumeWeight;
			Current->Data.Composition += InData.Composition * (InData.Density * VolumeWeight);

			DepthMaxDensities[Depth] = FMath::Max(DepthMaxDensities[Depth], Current->Data.Density);


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

	void CollectNodesInRange(const TSharedPtr<FOctreeNode>& InNode, TArray<TSharedPtr<FOctreeNode>>& OutNodes, const FInt64Coordinate& InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
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
	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(FInt64Coordinate InCenter, int64 InExtent, int InMinDepth = -1, int InMaxDepth = -1, int InTypeIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Nodes;
		if (Root.IsValid())
		{
			CollectNodesInRange(Root, Nodes, InCenter, InExtent, InMinDepth, InMaxDepth, InTypeIdFilter);
		}
		return Nodes;
	}

	UVolumeTexture* CreateVolumeTextureFromOctree(int32 Resolution)
	{
		// Create the volume texture in memory (transient package)
		UVolumeTexture* NewVolumeTexture = NewObject<UVolumeTexture>(
			GetTransientPackage(),
			NAME_None,
			RF_Transient
		);

		// Get the octree data (same as in your original function)
		const int TargetDepth = FMath::FloorLog2(Resolution);
		TArray<TSharedPtr<FOctreeNode>> PopulatedNodes = this->GetPopulatedNodes(TargetDepth, TargetDepth, -1);
		const float MaxDensityAtDepth = (DepthMaxDensities.IsValidIndex(TargetDepth) ? (float)DepthMaxDensities[TargetDepth] : 1.0f);
		const int64 NodeExtentAtDepth = (this->Extent >> TargetDepth);
		const int64 OctreeExtent = this->Extent;

		// Build node map
		TMap<FIntVector, TSharedPtr<FOctreeNode>> NodeMap;
		for (const auto& Node : PopulatedNodes)
		{
			if (!Node.IsValid()) continue;
			int32 VolumeX = static_cast<int32>((Node->Center.X + OctreeExtent) / (2 * NodeExtentAtDepth));
			int32 VolumeY = static_cast<int32>((Node->Center.Y + OctreeExtent) / (2 * NodeExtentAtDepth));
			int32 VolumeZ = static_cast<int32>((Node->Center.Z + OctreeExtent) / (2 * NodeExtentAtDepth));

			if (VolumeX >= 0 && VolumeX < Resolution &&
				VolumeY >= 0 && VolumeY < Resolution &&
				VolumeZ >= 0 && VolumeZ < Resolution)
			{
				NodeMap.Add(FIntVector(VolumeX, VolumeY, VolumeZ), Node);
			}
		}

		// Create the query function that uses your actual data
		auto QueryVoxel = [&NodeMap, MaxDensityAtDepth](const int32 X, const int32 Y, const int32 Z, void* ReturnValue)
			{
				FIntVector Coord(X, Y, Z);
				uint8* Voxel = static_cast<uint8*>(ReturnValue);

				if (auto* NodePtr = NodeMap.Find(Coord))
				{
					auto Node = *NodePtr;
					uint8 DensityByte = 0;
					if (MaxDensityAtDepth > 0.0f)
					{
						float Norm = static_cast<float>(Node->Data.Density) / MaxDensityAtDepth;
						DensityByte = static_cast<uint8>(FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f));
					}

					FVector Composition = Node->Data.Composition.GetSafeNormal();
					Voxel[0] = static_cast<uint8>(Composition.X * 255); // B
					Voxel[1] = static_cast<uint8>(Composition.Y * 255); // G
					Voxel[2] = static_cast<uint8>(Composition.Z * 255); // R
					Voxel[3] = DensityByte; // A
				}
				else
				{
					Voxel[0] = 0;
					Voxel[1] = 0;
					Voxel[2] = 0;
					Voxel[3] = 0;
				}
			};

		// Set properties
		NewVolumeTexture->SRGB = false;
		NewVolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
		NewVolumeTexture->Filter = TF_Nearest;
		NewVolumeTexture->AddressMode = TA_Clamp;
		NewVolumeTexture->MipGenSettings = TMGS_NoMipmaps;
		NewVolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
		NewVolumeTexture->NeverStream = true;

		// Create the texture data
		NewVolumeTexture->UpdateSourceFromFunction(
			QueryVoxel,
			Resolution,
			Resolution,
			Resolution,
			ETextureSourceFormat::TSF_BGRA8
		);

		NewVolumeTexture->UpdateResource();
		FlushRenderingCommands();

		return NewVolumeTexture;
	}
	UVolumeTexture* SaveVolumeTextureAsAssetFromOctree(int32 Resolution, const FString& AssetPath, const FString& AssetName)
	{
		// This should be called instead of SaveVolumeTextureAsAsset
		// and should recreate the volume texture with proper Source data

		FString CleanAssetPath = AssetPath;
		if (CleanAssetPath.EndsWith(TEXT("/")))
		{
			CleanAssetPath = CleanAssetPath.LeftChop(1);
		}

		FString PackagePath = CleanAssetPath + TEXT("/") + AssetName;
		UPackage* Package = CreatePackage(*PackagePath);

		if (!Package)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create package"));
			return nullptr;
		}

		UVolumeTexture* NewVolumeTexture = NewObject<UVolumeTexture>(
			Package,
			*AssetName,
			RF_Public | RF_Standalone
		);

		// Get the octree data (same as in CreateVolumeTextureFromOctreeSimple)
		const int TargetDepth = FMath::FloorLog2(Resolution);
		TArray<TSharedPtr<FOctreeNode>> PopulatedNodes = this->GetPopulatedNodes(TargetDepth, TargetDepth, -1);
		const float MaxDensityAtDepth = (DepthMaxDensities.IsValidIndex(TargetDepth) ? (float)DepthMaxDensities[TargetDepth] : 1.0f);
		const int64 NodeExtentAtDepth = (this->Extent >> TargetDepth);
		const int64 OctreeExtent = this->Extent;

		// Build node map
		TMap<FIntVector, TSharedPtr<FOctreeNode>> NodeMap;
		for (const auto& Node : PopulatedNodes)
		{
			if (!Node.IsValid()) continue;
			int32 VolumeX = static_cast<int32>((Node->Center.X + OctreeExtent) / (2 * NodeExtentAtDepth));
			int32 VolumeY = static_cast<int32>((Node->Center.Y + OctreeExtent) / (2 * NodeExtentAtDepth));
			int32 VolumeZ = static_cast<int32>((Node->Center.Z + OctreeExtent) / (2 * NodeExtentAtDepth));

			if (VolumeX >= 0 && VolumeX < Resolution &&
				VolumeY >= 0 && VolumeY < Resolution &&
				VolumeZ >= 0 && VolumeZ < Resolution)
			{
				NodeMap.Add(FIntVector(VolumeX, VolumeY, VolumeZ), Node);
			}
		}

		// Create the query function that uses your actual data
		auto QueryVoxel = [&NodeMap, MaxDensityAtDepth](const int32 X, const int32 Y, const int32 Z, void* ReturnValue)
			{
				FIntVector Coord(X, Y, Z);
				uint8* Voxel = static_cast<uint8*>(ReturnValue);

				if (auto* NodePtr = NodeMap.Find(Coord))
				{
					auto Node = *NodePtr;
					uint8 DensityByte = 0;
					if (MaxDensityAtDepth > 0.0f)
					{
						float Norm = static_cast<float>(Node->Data.Density) / MaxDensityAtDepth;
						DensityByte = static_cast<uint8>(FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f));
					}

					FVector Composition = Node->Data.Composition.GetSafeNormal();
					Voxel[0] = static_cast<uint8>(Composition.X * 255); // B
					Voxel[1] = static_cast<uint8>(Composition.Y * 255); // G
					Voxel[2] = static_cast<uint8>(Composition.Z * 255); // R
					Voxel[3] = DensityByte; // A
				}
				else
				{
					Voxel[0] = 0;
					Voxel[1] = 0;
					Voxel[2] = 0;
					Voxel[3] = 0;
				}
			};

		// Set properties
		NewVolumeTexture->SRGB = false;
		NewVolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
		NewVolumeTexture->Filter = TF_Nearest;
		NewVolumeTexture->AddressMode = TA_Clamp;
		NewVolumeTexture->MipGenSettings = TMGS_NoMipmaps;
		NewVolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
		NewVolumeTexture->NeverStream = true;

		// This creates proper Source data that can be saved
		NewVolumeTexture->UpdateSourceFromFunction(
			QueryVoxel,
			Resolution,
			Resolution,
			Resolution,
			ETextureSourceFormat::TSF_BGRA8
		);

		NewVolumeTexture->UpdateResource();
		FlushRenderingCommands();

		Package->MarkPackageDirty();

		// Save
		FString PackageFileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.Error = GError;
		SaveArgs.SaveFlags = SAVE_NoError;

		bool bSaved = UPackage::SavePackage(Package, NewVolumeTexture, *PackageFileName, SaveArgs);

		if (bSaved)
		{
			UE_LOG(LogTemp, Warning, TEXT("Volume texture saved successfully: %s"), *PackagePath);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to save volume texture"));
		}

		return NewVolumeTexture;
	}

	FOctree(int64 InExtent) {
		Extent = InExtent;
		MaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(Extent)));
		DepthMaxDensities.SetNumZeroed(128, true);
		Root = MakeShared<FOctreeNode>(FInt64Coordinate(), Extent, TArray<uint8>(), nullptr);
	}
};
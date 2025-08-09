// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
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

		for (int Depth = 0; Depth < InDepth; ++Depth) {
			//Accumulate density of children
			Current->Data.Density += InData.Density;
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

	TArray<TSharedPtr<FOctreeNode>> GetLeafNodes(int InMinDepth = -1, int InMaxDepth = -1, int InObjectIdFilter = -1) const {
		TArray<TSharedPtr<FOctreeNode>> Leaves;
		if (Root.IsValid()) {
			CollectLeafNodes(Root, Leaves, InMinDepth, InMaxDepth, InObjectIdFilter);
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

	UVolumeTexture* CreateVolumeTextureFromOctreeSimple()
	{
		const int32 Resolution = 256;

		// Create the volume texture
		UVolumeTexture* VolumeTexture = NewObject<UVolumeTexture>(
			GetTransientPackage(),
			NAME_None,
			RF_Transient
		);

		// Cache octree data for the lambda
		TArray<TSharedPtr<FOctreeNode>> PopulatedNodes = this->GetPopulatedNodes(8, 8, -1);
		float MaxDensityAtDepth = this->DepthMaxDensities[8];
		int64 NodeExtentAtDepth = this->Extent >> 8;
		int64 OctreeExtent = this->Extent;

		// Create a lookup map for faster access
		TMap<FIntVector, TSharedPtr<FOctreeNode>> NodeMap;
		for (const auto& Node : PopulatedNodes)
		{
			int32 VolumeX = (Node->Center.X + OctreeExtent) / (2 * NodeExtentAtDepth);
			int32 VolumeY = (Node->Center.Y + OctreeExtent) / (2 * NodeExtentAtDepth);
			int32 VolumeZ = (Node->Center.Z + OctreeExtent) / (2 * NodeExtentAtDepth);

			if (VolumeX >= 0 && VolumeX < Resolution &&
				VolumeY >= 0 && VolumeY < Resolution &&
				VolumeZ >= 0 && VolumeZ < Resolution)
			{
				NodeMap.Add(FIntVector(VolumeX, VolumeY, VolumeZ), Node);
			}
		}

		// Define the voxel query function
		auto QueryVoxel = [&](const int32 X, const int32 Y, const int32 Z, void* ReturnValue)
			{
				FIntVector Coord(X, Y, Z);

				if (auto* NodePtr = NodeMap.Find(Coord))
				{
					auto Node = *NodePtr;

					// For RGBA format
					if constexpr (true) // Assuming RGBA8 format
					{
						uint8* Voxel = static_cast<uint8*>(ReturnValue);
						uint8 NormalizedDensity = FMath::Clamp(Node->Data.Density / MaxDensityAtDepth * 255.0f, 0.0f, 255.0f);

						Voxel[0] = NormalizedDensity;     // R: density
						Voxel[1] = Node->Data.TypeId;     // G: type
						Voxel[2] = Node->Data.ObjectId;   // B: object
						Voxel[3] = 255;                   // A: alpha
					}
				}
				else
				{
					// Empty voxel
					uint8* Voxel = static_cast<uint8*>(ReturnValue);
					Voxel[0] = 0;
					Voxel[1] = 0;
					Voxel[2] = 0;
					Voxel[3] = 0;
				}
			};

		// Update the volume texture from the function
		VolumeTexture->UpdateSourceFromFunction(
			QueryVoxel,
			Resolution,                          // SizeX
			Resolution,                          // SizeY  
			Resolution,                          // SizeZ
			ETextureSourceFormat::TSF_BGRA8      // Format
		);

		VolumeTexture->UpdateResource();

		return VolumeTexture;
	}

	UVolumeTexture* SaveVolumeTextureAsAsset(UVolumeTexture* VolumeTexture, const FString& AssetPath, const FString& AssetName)
	{
		if (!VolumeTexture)
		{
			UE_LOG(LogTemp, Error, TEXT("VolumeTexture is null"));
			return nullptr;
		}

		// Create package path
		FString PackagePath = AssetPath + "/" + AssetName;
		UPackage* Package = CreatePackage(*PackagePath);

		if (!Package)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create package"));
			return nullptr;
		}

		// Create a new volume texture in the package
		UVolumeTexture* NewVolumeTexture = NewObject<UVolumeTexture>(
			Package,
			*AssetName,
			RF_Public | RF_Standalone | RF_MarkAsRootSet
		);

		// Copy the source data
		if (VolumeTexture->Source.IsValid())
		{
			NewVolumeTexture->Source = VolumeTexture->Source;
		}

		// Copy platform data
		if (VolumeTexture->GetPlatformData())
		{
			NewVolumeTexture->SetPlatformData(VolumeTexture->GetPlatformData());
		}

		// Copy properties (using the correct property names from the header)
		NewVolumeTexture->SRGB = VolumeTexture->SRGB;
		NewVolumeTexture->CompressionSettings = VolumeTexture->CompressionSettings;
		NewVolumeTexture->Filter = VolumeTexture->Filter;
		NewVolumeTexture->AddressMode = VolumeTexture->AddressMode;  // Single AddressMode property

		// Update the new texture
		NewVolumeTexture->UpdateResource();

		// Mark package as dirty and save
		Package->MarkPackageDirty();
		FAssetRegistryModule::AssetCreated(NewVolumeTexture);

		// Save the package
		FString PackageFileName = FPackageName::LongPackageNameToFilename(
			PackagePath,
			FPackageName::GetAssetPackageExtension()
		);

		bool bSaved = UPackage::SavePackage(
			Package,
			NewVolumeTexture,
			RF_Public | RF_Standalone,
			*PackageFileName
		);

		if (bSaved)
		{
			UE_LOG(LogTemp, Warning, TEXT("Volume texture saved to: %s"), *PackagePath);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to save volume texture"));
		}

		return NewVolumeTexture;
	}

	UTexture2D* CreateVolumeTextureFlipbookFromOctree()
	{
		const int32 Resolution = 256;
		const int32 SlicesPerRow = 16;
		const int32 AtlasSize = Resolution * SlicesPerRow; // 4096x4096
		EPixelFormat PixelFormat = PF_R8G8B8A8;

		// Prepare atlas data asynchronously
		TFuture<TArray<uint8>> AtlasDataFuture = Async(EAsyncExecution::Thread, [this, Resolution, SlicesPerRow, AtlasSize]()
			{
				const int32 AtlasDataSize = AtlasSize * AtlasSize * 4; // RGBA
				TArray<uint8> AtlasData;
				AtlasData.SetNumZeroed(AtlasDataSize);

				// Get octree data
				TArray<TSharedPtr<FOctreeNode>> PopulatedNodes = this->GetPopulatedNodes(8, 8, -1);
				float MaxDensityAtDepth = this->DepthMaxDensities[8];

				if (MaxDensityAtDepth > 0)
				{
					int64 NodeExtentAtDepth = this->Extent >> 8;

					for (const TSharedPtr<FOctreeNode>& Node : PopulatedNodes)
					{
						int32 VolumeX = (Node->Center.X + this->Extent) / (2 * NodeExtentAtDepth);
						int32 VolumeY = (Node->Center.Y + this->Extent) / (2 * NodeExtentAtDepth);
						int32 VolumeZ = (Node->Center.Z + this->Extent) / (2 * NodeExtentAtDepth);

						if (VolumeX >= 0 && VolumeX < Resolution &&
							VolumeY >= 0 && VolumeY < Resolution &&
							VolumeZ >= 0 && VolumeZ < Resolution)
						{
							int32 SliceRow = VolumeZ / SlicesPerRow;
							int32 SliceCol = VolumeZ % SlicesPerRow;

							int32 AtlasX = SliceCol * Resolution + VolumeX;
							int32 AtlasY = SliceRow * Resolution + VolumeY;

							int32 PixelIndex = (AtlasY * AtlasSize + AtlasX) * 4;

							uint8 NormalizedDensity = FMath::Clamp(Node->Data.Density / MaxDensityAtDepth * 255.0f, 0.0f, 255.0f);

							AtlasData[PixelIndex + 0] = NormalizedDensity;
							AtlasData[PixelIndex + 1] = NormalizedDensity;// Node->Data.TypeId;
							AtlasData[PixelIndex + 2] = NormalizedDensity;// Node->Data.ObjectId;
							AtlasData[PixelIndex + 3] = 1;
						}
					}
				}

				return AtlasData;
			});

		// Wait for data preparation
		TArray<uint8> AtlasData = AtlasDataFuture.Get();

		check(IsInGameThread());

		UTexture2D* NewTexture = NewObject<UTexture2D>(
			GetTransientPackage(),
			NAME_None,
			RF_Transient
		);

		check(IsValid(NewTexture));

		NewTexture->NeverStream = true;

		{
			// Create dummy 1x1 texture first
			NewTexture->SetPlatformData(new FTexturePlatformData());
			NewTexture->GetPlatformData()->SizeX = 4096;
			NewTexture->GetPlatformData()->SizeY = 4096;
			NewTexture->GetPlatformData()->PixelFormat = PixelFormat;

			FTexture2DMipMap* Mip = new FTexture2DMipMap();
			NewTexture->GetPlatformData()->Mips.Add(Mip);
			Mip->SizeX = NewTexture->GetPlatformData()->SizeX;
			Mip->SizeY = NewTexture->GetPlatformData()->SizeY;

			const uint32 DummyMipBytes = NewTexture->GetPlatformData()->SizeX * NewTexture->GetPlatformData()->SizeY * GPixelFormats[PixelFormat].BlockBytes;
			{
				Mip->BulkData.Lock(LOCK_READ_WRITE);
				void* TextureData = Mip->BulkData.Realloc(DummyMipBytes);

				static TArray<uint8> DummyBytes;
				DummyBytes.SetNumZeroed(DummyMipBytes);
				FMemory::Memcpy(TextureData, DummyBytes.GetData(), DummyMipBytes);

				Mip->BulkData.Unlock();
			}

			NewTexture->UpdateResource();
		}

		// Async create the real atlas texture with updated signature
		FTexture2DRHIRef RHITexture2D;
		FGraphEventRef CompletionEvent;

		Async(
			EAsyncExecution::Thread,
			[&RHITexture2D, &CompletionEvent, AtlasSize, PixelFormat, AtlasData]()
			{
				// Prepare mip data array
				void* MipData[1];
				MipData[0] = (void*)AtlasData.GetData();

				RHITexture2D = RHIAsyncCreateTexture2D(
					AtlasSize,                          // SizeX
					AtlasSize,                          // SizeY
					PixelFormat,                        // Format
					1,                                  // NumMips
					TexCreate_ShaderResource,           // Flags
					ERHIAccess::SRVMask,               // InResourceState
					MipData,                           // InitialMipData
					1,                                 // NumInitialMips
					TEXT("OctreeVolumeFlipbook"),      // DebugName
					CompletionEvent                    // OutCompletionEvent
				);
			}
		).Wait();

		// Wait for completion event if needed
		if (CompletionEvent.IsValid())
		{
			CompletionEvent->Wait();
		}

		// IMPORTANT: Update the UTexture2D's platform data to match the RHI texture
		{
			// Update platform data size to match the real texture
			NewTexture->GetPlatformData()->SizeX = AtlasSize;
			NewTexture->GetPlatformData()->SizeY = AtlasSize;

			// Update the mip map size
			NewTexture->GetPlatformData()->Mips[0].SizeX = AtlasSize;
			NewTexture->GetPlatformData()->Mips[0].SizeY = AtlasSize;
		}

		// Link RHI texture to UTexture2D
		ENQUEUE_RENDER_COMMAND(UpdateTextureReference)(
			[this, NewTexture, RHITexture2D](FRHICommandListImmediate& RHICmdList)
			{
				RHIUpdateTextureReference(NewTexture->TextureReference.TextureReferenceRHI, RHITexture2D);
				NewTexture->RefreshSamplerStates();
				//SaveTextureToFile(NewTexture, FString("TestTexture"));
			}
		);

		return NewTexture;
	}

	void SaveTextureToFile(UTexture2D* Texture, FString Filename)
	{
		if (!Texture) return;

		// Get texture data
		FTexturePlatformData* PlatformData = Texture->GetPlatformData();
		if (!PlatformData) return;

		FTexture2DMipMap& Mip = (PlatformData)->Mips[0];
		void* RawData = Mip.BulkData.Lock(LOCK_READ_ONLY);

		if (RawData)
		{
			int32 Width = Texture->GetSizeX();
			int32 Height = Texture->GetSizeY();

			// Convert to array for easier handling
			TArray<FColor> ImageData;
			ImageData.SetNum(Width * Height);

			uint8* SourceData = static_cast<uint8*>(RawData);
			for (int32 i = 0; i < Width * Height; ++i)
			{
				ImageData[i].R = SourceData[i * 4 + 0]; // Density
				ImageData[i].G = SourceData[i * 4 + 1]; // TypeId
				ImageData[i].B = SourceData[i * 4 + 2]; // ObjectId
				ImageData[i].A = SourceData[i * 4 + 3]; // Z coordinate
			}

			// Save as PNG
			FString SavePath = FPaths::ProjectSavedDir() + TEXT("Screenshots/") + Filename;
			FFileHelper::CreateBitmap(*SavePath, Width, Height, ImageData.GetData());

			UE_LOG(LogTemp, Warning, TEXT("Texture saved to: %s"), *SavePath);
		}

		Mip.BulkData.Unlock();
	}

	FOctree(int64 InExtent) {
		Extent = InExtent;
		MaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(Extent)));
		DepthMaxDensities.SetNumZeroed(128, true);
		Root = MakeShared<FOctreeNode>(FInt64Coordinate(), Extent, TArray<uint8>(), nullptr);
	}
};
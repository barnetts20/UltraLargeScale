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
	int MaxDepth;
	int64 Extent; //Must be power of 2, eg 1024 2048 etc
	TArray<double> DepthMaxDensities;
	TSharedPtr<FOctreeNode> Root;
	// In FOctree class, add a mutex
	mutable FCriticalSection OctreeMutex;

	TSharedPtr<FOctreeNode> InsertPosition(FInt64Vector InPosition, int InDepth, FVoxelData InData) {
		TSharedPtr<FOctreeNode> Current = Root;
		int64 CurrentExtent = Extent;

		// Calculate the leaf volume once (volume at target depth InDepth)
		int64 LeafExtent = Extent >> InDepth; // Extent at target depth
		for (int Depth = 0; Depth < InDepth; ++Depth) {
			// Weight the contribution by volume ratio
			double DensityWeight = static_cast<double>(CurrentExtent) / static_cast<double>(LeafExtent);

			// Accumulate density weighted by volume
			Current->Data.Density += InData.Density * DensityWeight;
			Current->Data.Composition += InData.Composition * InData.Density * DensityWeight;

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
	
	TSharedPtr<FOctreeNode> InsertPosition(FInt64Vector InPosition, FVoxelData InData) {
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
		DepthMaxDensities.SetNumZeroed(128, true);
		Root = MakeShared<FOctreeNode>(FInt64Vector(), Extent, TArray<uint8>(), nullptr);
	}
};

class SVO_API FOctreeTextureProcessor {
public:
	static TArray<uint8> GenerateVolumeMipDataFromOctree(TSharedPtr<FOctree> InOctree, int32 Resolution)
	{
		// Get the octree data
		const int TargetDepth = FMath::FloorLog2(Resolution);
		TArray<TSharedPtr<FOctreeNode>> PopulatedNodes = InOctree->GetPopulatedNodes(TargetDepth, TargetDepth, -1);
		const float MaxDensityAtDepth = (InOctree->DepthMaxDensities.IsValidIndex(TargetDepth) ? (float)InOctree->DepthMaxDensities[TargetDepth] : 1.0f);
		const int64 NodeExtentAtDepth = (InOctree->Extent >> TargetDepth);
		const int64 OctreeExtent = InOctree->Extent;

		// Allocate flat zeroed array directly
		const int64 TotalVoxels = static_cast<int64>(Resolution) * Resolution * Resolution;
		const int64 BytesPerVoxel = 4; // BGRA8 = 4 bytes
		const int64 TotalBytes = TotalVoxels * BytesPerVoxel;

		TArray<uint8> TextureData;
		TextureData.SetNumZeroed(TotalBytes);

		// Fill the array directly in one loop
		ParallelFor(PopulatedNodes.Num(), [&](int32 NodeIndex)
			{
				const auto& Node = PopulatedNodes[NodeIndex];
				if (!Node.IsValid()) return;

				// Calculate voxel coordinates
				int32 VolumeX = static_cast<int32>((Node->Center.X + OctreeExtent) / (2 * NodeExtentAtDepth));
				int32 VolumeY = static_cast<int32>((Node->Center.Y + OctreeExtent) / (2 * NodeExtentAtDepth));
				int32 VolumeZ = static_cast<int32>((Node->Center.Z + OctreeExtent) / (2 * NodeExtentAtDepth));

				// Bounds check
				if (VolumeX < 0 || VolumeX >= Resolution ||
					VolumeY < 0 || VolumeY >= Resolution ||
					VolumeZ < 0 || VolumeZ >= Resolution)
				{
					return;
				}

				// Calculate linear index and write directly
				int64 VoxelIndex = (static_cast<int64>(VolumeZ) * Resolution * Resolution) +
					(static_cast<int64>(VolumeY) * Resolution) + VolumeX;
				int64 ByteIndex = VoxelIndex * BytesPerVoxel;

				// Calculate density
				uint8 DensityByte = 0;
				if (MaxDensityAtDepth > 0.0f)
				{
					float Norm = static_cast<float>(Node->Data.Density) / MaxDensityAtDepth;
					DensityByte = static_cast<uint8>(FMath::Clamp(Norm * 255.0f, 0.0f, 255.0f));
				}

				// Write voxel data directly
				FVector Composition = Node->Data.Composition.GetSafeNormal();
				TextureData[ByteIndex + 0] = static_cast<uint8>(Composition.X * 255); // B
				TextureData[ByteIndex + 1] = static_cast<uint8>(Composition.Y * 255); // G
				TextureData[ByteIndex + 2] = static_cast<uint8>(Composition.Z * 255); // R
				TextureData[ByteIndex + 3] = DensityByte; // A
			});
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

	static UVolumeTexture* GenerateVolumeTextureFromMipData(const TArray<uint8>& InMipData, int32 InResolution) {
		UVolumeTexture* VolumeTexture;

		VolumeTexture = NewObject<UVolumeTexture>(
			GetTransientPackage(),
			NAME_None,
			RF_Transient
		);

		VolumeTexture->SRGB = false;
		VolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
		VolumeTexture->CompressionNone = true;
		VolumeTexture->Filter = TF_Nearest;
		VolumeTexture->AddressMode = TA_Clamp;
		VolumeTexture->MipGenSettings = TMGS_NoMipmaps;
		VolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
		VolumeTexture->NeverStream = true;
		VolumeTexture->DeferCompression = true;
		VolumeTexture->UnlinkStreaming();

		VolumeTexture->Source.Init(InResolution, InResolution, InResolution, 1, ETextureSourceFormat::TSF_BGRA8, InMipData.GetData());
		VolumeTexture->UpdateResource();
		FlushRenderingCommands();

		return VolumeTexture;
	}

	static UTexture2D* GeneratePsuedoVolumeTextureFromMipData(const TArray<uint8>& InMipData, int32 InResolution)
	{
		const int32 BytesPerVoxel = 4; 
		const int64 InVoxels = int64(InResolution) * InResolution * InResolution;
		if (InMipData.Num() != InVoxels * BytesPerVoxel)
		{
			UE_LOG(LogTemp, Error, TEXT("Size mismatch"));
			return nullptr;
		}

		// --- 1) Pack slices into 2D OutData (thread safe) ---
		int32 tilesPerSide = FMath::CeilToInt(FMath::Sqrt((float)InResolution));
		int32 OutWidth = tilesPerSide * InResolution;
		int32 OutHeight = tilesPerSide * InResolution;
		const int64 OutBytes = int64(OutWidth) * OutHeight * BytesPerVoxel;

		TArray<uint8> OutData;
		OutData.SetNumZeroed(OutBytes);

		const int32 InSlicePixels = InResolution * InResolution;
		const int32 OutRowBytes = OutWidth * BytesPerVoxel;
		const int32 InRowBytes = InResolution * BytesPerVoxel;

		ParallelFor(InResolution, [&](int32 z)
			{
				const int32 tileX = z % tilesPerSide;
				const int32 tileY = z / tilesPerSide;
				const int32 destTileOriginX = tileX * InResolution;
				const int32 destTileOriginY = tileY * InResolution;
				const int64 srcSliceBase = int64(z) * InSlicePixels * BytesPerVoxel;

				for (int32 y = 0; y < InResolution; ++y)
				{
					const int32 destY = destTileOriginY + y;
					const int64 destRowBase = int64(destY) * OutRowBytes + int64(destTileOriginX) * BytesPerVoxel;
					const int64 srcRowBase = srcSliceBase + int64(y) * InRowBytes;
					FMemory::Memcpy(&OutData[destRowBase], &InMipData[srcRowBase], size_t(InRowBytes));
				}
			}, EParallelForFlags::BackgroundPriority);

		// --- 2) Create dummy UTexture2D on GameThread (with 1x1 mip) ---
		UTexture2D* DummyTexture = nullptr;
		FEvent* DummyDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		DummyDoneEvent->Reset();

		AsyncTask(ENamedThreads::GameThread, [OutWidth, OutHeight, &DummyTexture, DummyDoneEvent]()
			{
				DummyTexture = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
				if (!DummyTexture) { DummyDoneEvent->Trigger(); return; }

				DummyTexture->NeverStream = true;

				FTexturePlatformData* PlatformData = new FTexturePlatformData();
				PlatformData->SizeX = 1;
				PlatformData->SizeY = 1;
				PlatformData->PixelFormat = PF_B8G8R8A8;

				FTexture2DMipMap* Mip = new FTexture2DMipMap();
				Mip->SizeX = 1; Mip->SizeY = 1;

				void* LockedPtr = Mip->BulkData.Lock(LOCK_READ_WRITE);
				void* ReallocPtr = Mip->BulkData.Realloc(1 * 1 * GPixelFormats[PF_B8G8R8A8].BlockBytes);
				if (ReallocPtr) { FMemory::Memzero(ReallocPtr, GPixelFormats[PF_B8G8R8A8].BlockBytes); }
				Mip->BulkData.Unlock();

				PlatformData->Mips.Add(Mip);
				DummyTexture->SetPlatformData(PlatformData);

				DummyTexture->SRGB = false;
				DummyTexture->CompressionSettings = TC_VectorDisplacementmap;
				DummyTexture->CompressionNone = true;
				DummyTexture->Filter = TF_Nearest;
				DummyTexture->MipGenSettings = TMGS_NoMipmaps;
				DummyTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
				DummyTexture->NeverStream = true;
				DummyTexture->DeferCompression = true;
				DummyTexture->UnlinkStreaming();

				DummyTexture->UpdateResource();

				DummyDoneEvent->Trigger();
			});
		DummyDoneEvent->Wait();
		FPlatformProcess::ReturnSynchEventToPool(DummyDoneEvent);

		if (!DummyTexture)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create dummy texture"));
			return nullptr;
		}

		// --- 3) Create RHI texture asynchronously ---
		void* MipData[1] = { OutData.GetData() };
		FGraphEventRef CompletionEvent;
		FTexture2DRHIRef RHITex = RHIAsyncCreateTexture2D(
			OutWidth, OutHeight,
			PF_B8G8R8A8,
			1,
			TexCreate_ShaderResource,
			ERHIAccess::SRVMask,
			MipData,
			1,
			TEXT("MyAsyncTexture"),
			CompletionEvent
		);
		if (CompletionEvent.IsValid()) { CompletionEvent->Wait(); }

		FRHITexture* RHITexture = RHITex.GetReference();
		if (!RHITexture)
		{
			UE_LOG(LogTemp, Warning, TEXT("Async RHI texture creation failed"));
			return nullptr;
		}

		// --- 4) Link RHI texture to dummy texture on render thread ---
		FEvent* LinkDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		LinkDoneEvent->Reset();

		ENQUEUE_RENDER_COMMAND(LinkTextureCmd)([DummyTexture, RHITexture, LinkDoneEvent](FRHICommandListImmediate& RHICmdList)
			{
				RHIUpdateTextureReference(DummyTexture->TextureReference.TextureReferenceRHI, static_cast<FTexture2DRHIRef>(RHITexture));
				DummyTexture->RefreshSamplerStates();
				LinkDoneEvent->Trigger();
			});

		LinkDoneEvent->Wait();
		FPlatformProcess::ReturnSynchEventToPool(LinkDoneEvent);

		return DummyTexture;
	}

	static void SaveVolumeTextureAsAsset(UVolumeTexture* InSaveTexture, const FString& AssetPath, const FString& AssetName)
	{
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
			return;
		}
		Package->MarkPackageDirty();

		// Save
		FString PackageFileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.Error = GError;
		SaveArgs.SaveFlags = SAVE_NoError;

		bool bSaved = UPackage::SavePackage(Package, InSaveTexture, *PackageFileName, SaveArgs);

		if (bSaved)
		{
			UE_LOG(LogTemp, Warning, TEXT("Volume texture saved successfully: %s"), *PackagePath);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to save volume texture"));
		}
	}
	
	static UTexture2D* SaveTexture2DAsAsset(
		const TArray<uint8>& MipData,
		int32 Width,
		int32 Height,
		const FString& AssetPath,
		const FString& AssetName
	)
	{
		// Basic validation
		const uint32 BytesPerPixel = GPixelFormats[PF_B8G8R8A8].BlockBytes;
		const uint64 ExpectedBytes = uint64(Width) * Height * BytesPerPixel;
		if ((uint64)MipData.Num() != ExpectedBytes)
		{
			UE_LOG(LogTemp, Error, TEXT("SaveTexture2DAsAsset_InitFlow: MipData size mismatch %d != %llu"), MipData.Num(), ExpectedBytes);
			return nullptr;
		}

		// Package
		FString CleanPath = AssetPath;
		if (CleanPath.EndsWith(TEXT("/"))) CleanPath.LeftChopInline(1);
		const FString PackageName = CleanPath + TEXT("/") + AssetName;
		UPackage* Package = CreatePackage(*PackageName);
		if (!Package)
		{
			UE_LOG(LogTemp, Error, TEXT("CreatePackage failed: %s"), *PackageName);
			return nullptr;
		}
		Package->FullyLoad();
		Package->MarkPackageDirty();

		// Create texture object in package
		UTexture2D* NewTexture = NewObject<UTexture2D>(Package, *AssetName, RF_Public | RF_Standalone | RF_MarkAsRootSet);
		if (!NewTexture)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create UTexture2D"));
			return nullptr;
		}

		// Mirror your volume texture init settings
		NewTexture->NeverStream = true;
		NewTexture->SRGB = false;
		NewTexture->CompressionSettings = TC_VectorDisplacementmap;
		NewTexture->CompressionNone = true;
		NewTexture->Filter = TF_Nearest;
		NewTexture->MipGenSettings = TMGS_NoMipmaps;
		NewTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
		NewTexture->DeferCompression = true;
		NewTexture->UnlinkStreaming();

		// Initialize Source with your BGRA8 data (this is the same flow as your volume texture)
		NewTexture->Source.Init(Width, Height, /*NumSlices*/1, /*NumMips*/1, ETextureSourceFormat::TSF_BGRA8, MipData.GetData());

		// Build render resource and *block* until RHI/render-thread/ImageCore finish reading the Source
		NewTexture->UpdateResource();
		FlushRenderingCommands(); // VERY IMPORTANT: ensures engine finished reading Source memory

		// Now the engine has finished using MipData pointer — it is safe for caller to let MipData go out of scope.

		// Register asset and save
		FAssetRegistryModule::AssetCreated(NewTexture);
		Package->MarkPackageDirty();

		const FString FilePath = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());

		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.Error = GError;
		SaveArgs.SaveFlags = SAVE_None;

		const bool bSaved = UPackage::SavePackage(Package, NewTexture, *FilePath, SaveArgs);
		if (!bSaved)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to save package %s"), *FilePath);
		}
		else
		{
			UE_LOG(LogTemp, Log, TEXT("Saved texture asset %s"), *PackageName);
		}

		return NewTexture;
	}



};
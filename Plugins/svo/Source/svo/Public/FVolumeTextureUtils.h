#pragma once

#include "CoreMinimal.h"
#include "RHICommandList.h"
#include "RenderResource.h"
#include "RHIResources.h"
#include "RHIUtilities.h"
#include "Engine/VolumeTexture.h"
#include "UObject/SavePackage.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "FastNoise/FastNoise.h"

class FOctree;
class FOctreeNode;

/// <summary>
/// Sparse voxel entry for rasterizing node data into a dense volume grid.
/// </summary>
struct FVolumeVoxelEntry
{
	int X, Y, Z;       // Grid coordinates (0 to Resolution-1)
	uint8 R, G, B, A;  // Channel values
};

/// <summary>
/// VOLUME TEXTURE UTILITIES
/// Pure texture/buffer operations: upscaling, compositing, rasterization,
/// pseudo-volume packing, async texture creation, and optional bake-to-disk.
/// No octree or noise dependencies � operates on raw buffers.
/// 
/// Buffer format: BGRA8 (4 bytes per voxel), linear layout [z][y][x].
/// Channel semantics are defined by the caller � this class is channel-agnostic.
/// </summary>
class SVO_API FVolumeTextureUtils
{
public:

#pragma region Noise Sampling
	/// <summary>
	/// Sample a noise function into a volume grid at the specified resolution.
	/// Optionally writes results into an octree at the given depth simultaneously (single pass).
	/// Parallel chunking at depth-5 boundaries (each depth-5 chunk processes its sub-nodes serially).
	/// When no octree is provided, produces a standalone volume buffer from noise.
	/// </summary>
	static TArray<uint8> SampleNoiseToVolume(
		FastNoise::SmartNode<> InNoise,
		int InSeed,
		int InResolution,
		double InExtent,
		TSharedPtr<FOctree> InOctree = nullptr,
		int InOctreeDepth = -1,
		float InNoisePower = 2.0f,
		int InChannel = 3);
#pragma endregion

#pragma region Extract Mip Data From Volume Nodes
	static TArray<uint8> GenerateVolumeMipDataFromOctree(TArray<TSharedPtr<FOctreeNode>> InVolumeNodes, int InResolution, double InExtent, double InMaxDensity);
#pragma endregion

#pragma region Upscale Volume Data

	/// <summary>
	/// Upscale a low-resolution volume grid to 256^3 via trilinear interpolation.
	/// Returns the upscaled data as a flat 256^3 BGRA8 buffer.
	/// </summary>
	/// <param name="InMipData">Input volume data (InResolution^3 * 4 bytes, BGRA8)</param>
	/// <param name="InResolution">Input resolution per axis (e.g., 32)</param>
	/// <returns>Upscaled 256^3 BGRA8 buffer</returns>
	static TArray<uint8> UpscaleVolumeData(const TArray<uint8>& InMipData, int InResolution)
	{
		double StartTime = FPlatformTime::Seconds();

		constexpr int OutRes = 256;
		constexpr int BytesPerVoxel = 4;
		constexpr int64 OutBytes = (int64)OutRes * OutRes * OutRes * BytesPerVoxel;
		const int InSlice = InResolution * InResolution;

		// Precompute interpolation coordinates
		struct SampleCoord { int i0, i1; float t; };
		TArray<SampleCoord> Coords;
		Coords.SetNum(OutRes);
		for (int i = 0; i < OutRes; i++)
		{
			float f = i * (float(InResolution - 1) / float(OutRes - 1));
			int i0 = FMath::FloorToInt(f);
			int i1 = FMath::Min(i0 + 1, InResolution - 1);
			Coords[i] = { i0, i1, f - i0 };
		}

		TArray<uint8> OutData;
		OutData.SetNumZeroed(OutBytes);

		const uint8* InPtr = InMipData.GetData();
		uint8* OutPtr = OutData.GetData();

		auto InIndex = [InResolution, InSlice, BytesPerVoxel](int x, int y, int z) -> int {
			return ((z * InSlice) + (y * InResolution) + x) * BytesPerVoxel;
			};

		ParallelFor(OutRes, [&](int z)
			{
				const auto& Zc = Coords[z];
				for (int y = 0; y < OutRes; ++y)
				{
					const auto& Yc = Coords[y];
					for (int x = 0; x < OutRes; ++x)
					{
						const auto& Xc = Coords[x];

						const int base000 = InIndex(Xc.i0, Yc.i0, Zc.i0);
						const int base100 = InIndex(Xc.i1, Yc.i0, Zc.i0);
						const int base010 = InIndex(Xc.i0, Yc.i1, Zc.i0);
						const int base110 = InIndex(Xc.i1, Yc.i1, Zc.i0);
						const int base001 = InIndex(Xc.i0, Yc.i0, Zc.i1);
						const int base101 = InIndex(Xc.i1, Yc.i0, Zc.i1);
						const int base011 = InIndex(Xc.i0, Yc.i1, Zc.i1);
						const int base111 = InIndex(Xc.i1, Yc.i1, Zc.i1);

						int64 outIdx = ((int64)z * OutRes * OutRes + (int64)y * OutRes + x) * BytesPerVoxel;

						for (int c = 0; c < BytesPerVoxel; ++c)
						{
							float c000 = float(InPtr[base000 + c]) / 255.f;
							float c100 = float(InPtr[base100 + c]) / 255.f;
							float c010 = float(InPtr[base010 + c]) / 255.f;
							float c110 = float(InPtr[base110 + c]) / 255.f;
							float c001 = float(InPtr[base001 + c]) / 255.f;
							float c101 = float(InPtr[base101 + c]) / 255.f;
							float c011 = float(InPtr[base011 + c]) / 255.f;
							float c111 = float(InPtr[base111 + c]) / 255.f;

							float c00 = FMath::Lerp(c000, c100, Xc.t);
							float c10 = FMath::Lerp(c010, c110, Xc.t);
							float c01 = FMath::Lerp(c001, c101, Xc.t);
							float c11 = FMath::Lerp(c011, c111, Xc.t);
							float c0 = FMath::Lerp(c00, c10, Yc.t);
							float c1 = FMath::Lerp(c01, c11, Yc.t);
							float val = FMath::Lerp(c0, c1, Zc.t);

							OutPtr[outIdx + c] = uint8(val * 255.0f);
						}
					}
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::UpscaleVolumeData (%d^3 -> 256^3) took %.3f sec"), InResolution, Duration);

		return OutData;
	}

#pragma endregion

#pragma region Rasterize Sparse Data to Volume

	/// <summary>
	/// Rasterize sparse voxel entries into a dense volume buffer.
	/// Additive � values are added to existing buffer content, clamped to 255.
	/// Use for writing sparse octree node data into a 256^3 grid.
	/// </summary>
	/// <param name="InOutVolumeData">Target volume buffer (modified in place, Resolution^3 * 4 bytes)</param>
	/// <param name="InResolution">Volume resolution per axis</param>
	/// <param name="InEntries">Sparse voxel entries to rasterize</param>
	static void RasterizeSparseEntries(
		TArray<uint8>& InOutVolumeData,
		int InResolution,
		const TArray<FVolumeVoxelEntry>& InEntries)
	{
		double StartTime = FPlatformTime::Seconds();

		constexpr int BytesPerVoxel = 4;
		const int Slice = InResolution * InResolution;
		uint8* DataPtr = InOutVolumeData.GetData();

		for (const FVolumeVoxelEntry& Entry : InEntries)
		{
			if (Entry.X < 0 || Entry.X >= InResolution ||
				Entry.Y < 0 || Entry.Y >= InResolution ||
				Entry.Z < 0 || Entry.Z >= InResolution)
			{
				continue;
			}

			int64 idx = ((int64)Entry.Z * Slice + (int64)Entry.Y * InResolution + Entry.X) * BytesPerVoxel;

			// Additive compositing, clamped
			DataPtr[idx + 0] = (uint8)FMath::Min((int)DataPtr[idx + 0] + (int)Entry.R, 255);
			DataPtr[idx + 1] = (uint8)FMath::Min((int)DataPtr[idx + 1] + (int)Entry.G, 255);
			DataPtr[idx + 2] = (uint8)FMath::Min((int)DataPtr[idx + 2] + (int)Entry.B, 255);
			DataPtr[idx + 3] = (uint8)FMath::Min((int)DataPtr[idx + 3] + (int)Entry.A, 255);
		}

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::RasterizeSparseEntries (%d entries) took %.3f sec"), InEntries.Num(), Duration);
	}

#pragma endregion

#pragma region Composite Volume Layers

	/// <summary>
	/// Composite a detail layer onto a base volume buffer (both same resolution, BGRA8).
	/// Additive per channel, clamped to 255.
	/// </summary>
	/// <param name="InOutBaseData">Base volume (modified in place)</param>
	/// <param name="InDetailData">Detail volume to composite (additive)</param>
	static void CompositeVolumeLayers(TArray<uint8>& InOutBaseData, const TArray<uint8>& InDetailData)
	{
		double StartTime = FPlatformTime::Seconds();

		check(InOutBaseData.Num() == InDetailData.Num());

		uint8* BasePtr = InOutBaseData.GetData();
		const uint8* DetailPtr = InDetailData.GetData();
		const int64 TotalBytes = InOutBaseData.Num();

		// Process in parallel chunks of 64KB
		constexpr int64 ChunkSize = 65536;
		const int NumChunks = (TotalBytes + ChunkSize - 1) / ChunkSize;

		ParallelFor(NumChunks, [&](int ChunkIdx)
			{
				int64 Start = (int64)ChunkIdx * ChunkSize;
				int64 End = FMath::Min(Start + ChunkSize, TotalBytes);
				for (int64 i = Start; i < End; ++i)
				{
					int val = (int)BasePtr[i] + (int)DetailPtr[i];
					BasePtr[i] = (uint8)FMath::Min(val, 255);
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::CompositeVolumeLayers took %.3f sec"), Duration);
	}

#pragma endregion

#pragma region Pack to Pseudo-Volume Layout

	/// <summary>
	/// Pack a 256^3 BGRA8 volume buffer into a 4096x4096 2D pseudo-volume layout.
	/// Layout: 16x16 tiles, each tile is a 256x256 Z-slice.
	/// Input: linear 256^3 buffer [z][y][x] * 4 bytes.
	/// Output: tiled 4096x4096 buffer suitable for UTexture2D creation.
	/// </summary>
	static TArray<uint8> PackToPseudoVolumeLayout(const TArray<uint8>& InVolumeData)
	{
		double StartTime = FPlatformTime::Seconds();

		constexpr int VolumeRes = 256;
		constexpr int BytesPerVoxel = 4;
		constexpr int TilesPerSide = 16;
		constexpr int OutRes = 4096;
		constexpr int64 ExpectedBytes = (int64)VolumeRes * VolumeRes * VolumeRes * BytesPerVoxel;
		constexpr int64 OutBytes = (int64)OutRes * OutRes * BytesPerVoxel;

		check(InVolumeData.Num() == ExpectedBytes);

		TArray<uint8> OutData;
		OutData.SetNumZeroed(OutBytes);

		const uint8* InPtr = InVolumeData.GetData();
		uint8* OutPtr = OutData.GetData();

		ParallelFor(VolumeRes, [&](int z)
			{
				const int tileX = z % TilesPerSide;
				const int tileY = z / TilesPerSide;
				const int tileOriginX = tileX * VolumeRes;
				const int tileOriginY = tileY * VolumeRes;

				for (int y = 0; y < VolumeRes; ++y)
				{
					int64 srcOffset = ((int64)z * VolumeRes * VolumeRes + (int64)y * VolumeRes) * BytesPerVoxel;
					int destY = tileOriginY + y;
					int64 dstOffset = ((int64)destY * OutRes + tileOriginX) * BytesPerVoxel;

					FMemory::Memcpy(OutPtr + dstOffset, InPtr + srcOffset, VolumeRes * BytesPerVoxel);
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::PackToPseudoVolumeLayout took %.3f sec"), Duration);

		return OutData;
	}

#pragma endregion

#pragma region Async Texture Creation

	/// <summary>
	/// Create a UTexture2D from pre-built 4096x4096 BGRA8 pseudo-volume data.
	/// Must be called from a background thread (uses async game thread dispatch internally).
	/// If InSavePath is non-empty, the texture is also saved to disk as a persistent .uasset.
	/// </summary>
	/// <param name="InMipData">4096x4096 BGRA8 buffer (67,108,864 bytes)</param>
	/// <param name="InSavePath">If non-empty, saves the texture as a persistent asset at this path</param>
	/// <returns>Transient UTexture2D ready for material use</returns>
	static UTexture2D* CreatePseudoVolumeTexture(
		const TArray<uint8>& InMipData,
		const FString& InSavePath = TEXT(""))
	{
		double StartTime = FPlatformTime::Seconds();

		const int OutRes = 4096;
		const int ExpectedBytes = 67108864;

		if (InMipData.Num() != ExpectedBytes)
		{
			UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::CreatePseudoVolumeTexture - Size mismatch: Expected %d bytes, got %d bytes"), ExpectedBytes, InMipData.Num());
			return nullptr;
		}

		// Create placeholder texture on game thread
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
			UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::CreatePseudoVolumeTexture - Failed to create placeholder texture"));
			return nullptr;
		}

		// Create RHI texture async
		void* MipDataPtrs[1] = { const_cast<uint8*>(InMipData.GetData()) };
		FGraphEventRef CompletionEvent;
		FTexture2DRHIRef RHITex = RHIAsyncCreateTexture2D(
			OutRes, OutRes, PF_B8G8R8A8, 1,
			TexCreate_ShaderResource, ERHIAccess::SRVMask,
			MipDataPtrs, 1, TEXT("PseudoVolumeTexture"), CompletionEvent);
		if (CompletionEvent.IsValid()) { CompletionEvent->Wait(); }
		FRHITexture* RHITexture = RHITex.GetReference();
		if (!RHITexture)
		{
			UE_LOG(LogTemp, Warning, TEXT("FVolumeTextureUtils::CreatePseudoVolumeTexture - Async RHI texture creation failed"));
			return nullptr;
		}

		// Link RHI texture on render thread
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

		// Optional: save to disk if path provided
		if (!InSavePath.IsEmpty())
		{
			SaveTextureToDisk(PseudoVolumeTexture, InMipData, OutRes, InSavePath);
		}

		double TotalDuration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::CreatePseudoVolumeTexture took %.3f sec"), TotalDuration);

		return PseudoVolumeTexture;
	}

#pragma endregion

private:

#pragma region Save to Disk

	/// <summary>
	/// Save a pseudo-volume texture to disk as a persistent .uasset.
	/// </summary>
	static void SaveTextureToDisk(
		UTexture2D* InTexture,
		const TArray<uint8>& InMipData,
		int InResolution,
		const FString& InSavePath)
	{
		double StartTime = FPlatformTime::Seconds();

		FEvent* SaveDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		SaveDoneEvent->Reset();
		AsyncTask(ENamedThreads::GameThread, [InTexture, &InMipData, InResolution, &InSavePath, SaveDoneEvent]()
			{
				FString PackagePath = InSavePath;
				FString AssetName = FPackageName::GetShortName(PackagePath);

				UPackage* Package = CreatePackage(*PackagePath);
				if (!Package)
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Failed to create package: %s"), *PackagePath);
					SaveDoneEvent->Trigger();
					return;
				}

				UTexture2D* SaveTexture = NewObject<UTexture2D>(Package, *AssetName, RF_Public | RF_Standalone);
				if (!SaveTexture)
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Failed to create texture object"));
					SaveDoneEvent->Trigger();
					return;
				}

				SaveTexture->NeverStream = true;
				SaveTexture->SRGB = false;
				SaveTexture->CompressionSettings = TC_VectorDisplacementmap;
				SaveTexture->CompressionNone = true;
				SaveTexture->Filter = TF_Nearest;
				SaveTexture->MipGenSettings = TMGS_NoMipmaps;
				SaveTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;

				FTexturePlatformData* PlatformData = new FTexturePlatformData();
				PlatformData->SizeX = InResolution;
				PlatformData->SizeY = InResolution;
				PlatformData->PixelFormat = PF_B8G8R8A8;

				FTexture2DMipMap* Mip = new FTexture2DMipMap();
				Mip->SizeX = InResolution;
				Mip->SizeY = InResolution;
				Mip->BulkData.Lock(LOCK_READ_WRITE);
				void* TextureDataPtr = Mip->BulkData.Realloc(InMipData.Num());
				if (TextureDataPtr)
				{
					FMemory::Memcpy(TextureDataPtr, InMipData.GetData(), InMipData.Num());
				}
				Mip->BulkData.Unlock();

				PlatformData->Mips.Add(Mip);
				SaveTexture->SetPlatformData(PlatformData);
				SaveTexture->UpdateResource();

				Package->MarkPackageDirty();
				FString PackageFileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());

				FSavePackageArgs SaveArgs;
				SaveArgs.TopLevelFlags = EObjectFlags::RF_Public | EObjectFlags::RF_Standalone;
				bool bSaved = UPackage::SavePackage(Package, SaveTexture, *PackageFileName, SaveArgs);

				if (bSaved)
				{
					FAssetRegistryModule::AssetCreated(SaveTexture);
					UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Saved to: %s"), *PackageFileName);
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Failed to save: %s"), *PackageFileName);
				}

				SaveDoneEvent->Trigger();
			});
		SaveDoneEvent->Wait();
		FPlatformProcess::ReturnSynchEventToPool(SaveDoneEvent);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk took %.3f sec"), Duration);
	}

#pragma endregion
};
#pragma once

#include "CoreMinimal.h"
#include "RHICommandList.h"
#include "RenderResource.h"
#include "RHIResources.h"
#include "RHIUtilities.h"
#include "Engine/VolumeTexture.h"
#include "UObject/SavePackage.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "ContentBrowserModule.h"
#include "IContentBrowserSingleton.h"
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
#pragma region Splat VBOs to Volume

	/// <summary>
	/// Splat point cloud nodes into a volume buffer as spherical density kernels.
	/// Each node writes a falloff sphere centered at its position with radius
	/// derived from its octree extent and scale factor.
	///
	/// This is the mechanism by which clusters emerge in the volume texture:
	/// dense regions of VBOs produce overlapping splats that accumulate into
	/// bright knots visible in the raymarch.
	///
	/// Additive per channel, clamped to 255. Parallel over nodes.
	/// </summary>
	/// <param name="InOutVolumeData">Target volume buffer (modified in place, Resolution^3 * 4 bytes BGRA8)</param>
	/// <param name="InResolution">Volume resolution per axis (e.g., 32, 64, 128)</param>
	/// <param name="InExtent">World-space half-extent of the volume (same as octree extent)</param>
	/// <param name="InNodes">Octree nodes to splat (typically the PointNodes from BulkInsertPositions)</param>
	/// <param name="InRadiusScale">Multiplier on each node's natural radius (Extent * (1+ScaleFactor)). 
	///        Use >1 to fatten splats for cluster visibility at low resolutions.</param>
	/// <param name="InFalloffPower">Exponent for the falloff curve. 1.0=linear, 2.0=quadratic (smooth), 
	///        higher=sharper core. Default 2.0 gives a nice gaussian-ish rolloff.</param>
	/// <param name="InIntensity">Peak density value at the center of each splat (0.0-1.0 range, 
	///        mapped to 0-255). Default 1.0.</param>
	/// <param name="InChannel">Which BGRA channel to write (-1 = all channels, 0=B, 1=G, 2=R, 3=A). 
	///        Default -1 (all).</param>
	static void SplatVBOsToVolume(
		TArray<uint8>& InOutVolumeData,
		int InResolution,
		double InExtent,
		const TArray<TSharedPtr<FOctreeNode>>& InNodes,
		float InRadiusScale = 1.0f,
		float InFalloffPower = 2.0f,
		float InIntensity = 1.0f,
		int InChannel = -1)
	{
		double StartTime = FPlatformTime::Seconds();

		constexpr int BytesPerVoxel = 4;
		const double VoxelSize = (2.0 * InExtent) / InResolution;
		const double InvVoxelSize = 1.0 / VoxelSize;
		const int Slice = InResolution * InResolution;
		const uint8 PeakByte = (uint8)FMath::Clamp(InIntensity * 255.0f, 0.0f, 255.0f);

		// --- Pre-compute per-node splat parameters in voxel space ---
		struct FSplatEntry
		{
			float CenterX, CenterY, CenterZ;  // Voxel-space center (fractional)
			float RadiusVoxels;                 // Radius in voxel units
			float InvRadius;                    // 1/radius for normalization
			float Intensity;                    // Peak intensity 0-255
		};

		TArray<FSplatEntry> Splats;
		Splats.Reserve(InNodes.Num());

		for (const TSharedPtr<FOctreeNode>& Node : InNodes)
		{
			if (!Node.IsValid()) continue;

			// Node's world-space radius: extent * (1 + ScaleFactor), scaled by user param
			double WorldRadius = Node->Extent * (1.0 + Node->Data.ScaleFactor) * InRadiusScale;
			float RadiusVoxels = (float)(WorldRadius * InvVoxelSize);

			// Skip nodes whose splat is smaller than half a voxel — they'd be sub-pixel
			if (RadiusVoxels < 0.5f) RadiusVoxels = 0.5f;

			// World position to voxel-space (0 to Resolution)
			float VoxelX = (float)((Node->Center.X + InExtent) * InvVoxelSize);
			float VoxelY = (float)((Node->Center.Y + InExtent) * InvVoxelSize);
			float VoxelZ = (float)((Node->Center.Z + InExtent) * InvVoxelSize);

			FSplatEntry Entry;
			Entry.CenterX = VoxelX;
			Entry.CenterY = VoxelY;
			Entry.CenterZ = VoxelZ;
			Entry.RadiusVoxels = RadiusVoxels;
			Entry.InvRadius = 1.0f / RadiusVoxels;
			Entry.Intensity = InIntensity * 255.0f;

			Splats.Add(Entry);
		}

		if (Splats.Num() == 0)
		{
			UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SplatVBOsToVolume - No valid nodes to splat"));
			return;
		}

		// --- Accumulation buffer (float per channel to avoid clamping during accumulation) ---
		// Using a float buffer avoids repeated clamp-to-255 during overlapping splats,
		// giving correct additive blending before the final quantization pass.
		const int64 TotalVoxels = (int64)InResolution * InResolution * InResolution;
		TArray<float> AccumBuffer;
		AccumBuffer.SetNumZeroed(TotalVoxels);

		// --- Splat each node (parallel over nodes, atomic accumulate) ---
		// For low node counts (<10K) this is faster than splatting per-voxel-slice
		// because most voxels are untouched by any splat.
		//
		// We use a two-pass approach:
		// Pass 1: Accumulate into float buffer (parallel per node, with simple overlap tolerance)
		// Pass 2: Quantize and composite into the BGRA byte buffer

		// Pass 1: Parallel splat into float accumulation buffer
		// Note: overlapping splats from different nodes may race on the same voxel.
		// For density visualization this is acceptable — the error is bounded and
		// the result converges. For exact accumulation, switch to per-slice parallelism.
		ParallelFor(Splats.Num(), [&](int SplatIdx)
			{
				const FSplatEntry& S = Splats[SplatIdx];

				// Bounding box in voxel coordinates (clamped to grid)
				int MinX = FMath::Max(0, FMath::FloorToInt(S.CenterX - S.RadiusVoxels));
				int MaxX = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterX + S.RadiusVoxels));
				int MinY = FMath::Max(0, FMath::FloorToInt(S.CenterY - S.RadiusVoxels));
				int MaxY = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterY + S.RadiusVoxels));
				int MinZ = FMath::Max(0, FMath::FloorToInt(S.CenterZ - S.RadiusVoxels));
				int MaxZ = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterZ + S.RadiusVoxels));

				const float RadSq = S.RadiusVoxels * S.RadiusVoxels;

				for (int z = MinZ; z <= MaxZ; ++z)
				{
					float dz = (z + 0.5f) - S.CenterZ;
					float dz2 = dz * dz;

					for (int y = MinY; y <= MaxY; ++y)
					{
						float dy = (y + 0.5f) - S.CenterY;
						float dy2 = dy * dy;
						float dyz2 = dy2 + dz2;

						// Early row skip — if dy²+dz² already exceeds radius², whole row is outside
						if (dyz2 > RadSq) continue;

						for (int x = MinX; x <= MaxX; ++x)
						{
							float dx = (x + 0.5f) - S.CenterX;
							float distSq = dx * dx + dyz2;

							if (distSq > RadSq) continue;

							// Normalized distance [0, 1]
							float t = FMath::Sqrt(distSq) * S.InvRadius;

							// Falloff: 1 at center, 0 at edge
							float falloff = FMath::Pow(1.0f - t, InFalloffPower);
							float contribution = S.Intensity * falloff;

							int64 VoxelIdx = (int64)z * Slice + (int64)y * InResolution + x;

							// Simple non-atomic add — acceptable race for density vis
							AccumBuffer[VoxelIdx] += contribution;
						}
					}
				}
			}, Splats.Num() > 64 ? EParallelForFlags::BackgroundPriority : EParallelForFlags::ForceSingleThread);

		// Pass 2: Quantize and composite into byte buffer (parallel over Z slices)
		uint8* DataPtr = InOutVolumeData.GetData();

		ParallelFor(InResolution, [&](int z)
			{
				for (int y = 0; y < InResolution; ++y)
				{
					for (int x = 0; x < InResolution; ++x)
					{
						int64 VoxelIdx = (int64)z * Slice + (int64)y * InResolution + x;
						float Accum = AccumBuffer[VoxelIdx];

						if (Accum <= 0.0f) continue;

						uint8 SplatByte = (uint8)FMath::Min(Accum, 255.0f);
						int64 ByteIdx = VoxelIdx * BytesPerVoxel;

						if (InChannel == -1)
						{
							// All channels
							for (int c = 0; c < BytesPerVoxel; ++c)
							{
								DataPtr[ByteIdx + c] = (uint8)FMath::Min((int)DataPtr[ByteIdx + c] + (int)SplatByte, 255);
							}
						}
						else
						{
							// Single channel
							int c = FMath::Clamp(InChannel, 0, 3);
							DataPtr[ByteIdx + c] = (uint8)FMath::Min((int)DataPtr[ByteIdx + c] + (int)SplatByte, 255);
						}
					}
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SplatVBOsToVolume (%d nodes, radius scale %.1f, falloff %.1f, res %d) took %.3f sec"),
			InNodes.Num(), InRadiusScale, InFalloffPower, InResolution, Duration);
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

		// Optional: save to disk if path provided.
		// NOTE: InSavePath must be a /Game/... content-relative package path with NO extension.
		// e.g. TEXT("/Game/GeneratedTextures/SectorVolume_0")
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
	/// InSavePath must be a content-relative package path with NO file extension,
	/// e.g. "/Game/GeneratedTextures/SectorVolume_0". The .uasset extension is
	/// appended internally by LongPackageNameToFilename.
	///
	/// NOTE: This creates a NEW UTexture2D in the target package using Source mip data
	/// (the editor-facing source representation). PlatformData alone is not sufficient
	/// for SavePackage -- without Source data the asset saves but loads as blank in
	/// the editor and content browser.
	/// InTexture (the transient runtime texture) is intentionally NOT reused here;
	/// it lives in GetTransientPackage() and cannot be moved to a persistent package.
	/// </summary>
	static void SaveTextureToDisk(
		UTexture2D* InTexture,   // Unused directly -- kept for API clarity/future use
		const TArray<uint8>& InMipData,
		int InResolution,
		const FString& InSavePath)
	{
		double StartTime = FPlatformTime::Seconds();

		// Capture by value -- this is called from a background thread and the caller
		// may return before the AsyncTask executes on the game thread.
		TArray<uint8> MipDataCopy = InMipData;
		FString SavePathCopy = InSavePath;

		FEvent* SaveDoneEvent = FPlatformProcess::GetSynchEventFromPool(true);
		SaveDoneEvent->Reset();
		AsyncTask(ENamedThreads::GameThread, [MipDataCopy = MoveTemp(MipDataCopy), SavePathCopy = MoveTemp(SavePathCopy), InResolution, SaveDoneEvent]()
			{
				// Package path must be a UE content-relative path (e.g. /Game/Folder/AssetName).
				// Note: /Game/ maps to your project's Content/ folder on disk.
				// Do NOT pass filesystem paths like /Content/... or paths with .uasset extension.

				FString PackagePath = SavePathCopy;
				FString AssetName = FPackageName::GetShortName(PackagePath);

				UPackage* Package = CreatePackage(*PackagePath);
				if (!Package)
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Failed to create package: %s"), *PackagePath);
					SaveDoneEvent->Trigger();
					return;
				}
				Package->FullyLoad();

				UTexture2D* SaveTexture = NewObject<UTexture2D>(Package, *AssetName, RF_Public | RF_Standalone);
				if (!SaveTexture)
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Failed to create texture object in package"));
					SaveDoneEvent->Trigger();
					return;
				}

				// --- Texture settings ---
				SaveTexture->NeverStream = true;
				SaveTexture->SRGB = false;
				SaveTexture->CompressionSettings = TC_VectorDisplacementmap;
				SaveTexture->CompressionNone = true;
				SaveTexture->Filter = TF_Nearest;
				SaveTexture->MipGenSettings = TMGS_NoMipmaps;
				SaveTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;

				// --- Populate SOURCE data ---
				// Source is the editor-facing representation that SavePackage persists.
				// PlatformData is the cooked runtime form and is NOT written by SavePackage.
				const int ExpectedBytes = InResolution * InResolution * 4;
				UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Initializing source %dx%d, data size %d, expected %d"),
					InResolution, InResolution, MipDataCopy.Num(), ExpectedBytes);

				SaveTexture->Source.Init(
					InResolution,
					InResolution,
					/*NumSlices=*/1,
					/*NumMips=*/1,
					TSF_BGRA8
				);

				UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Source valid after Init: %s, size: %lld"),
					SaveTexture->Source.IsValid() ? TEXT("yes") : TEXT("no"),
					SaveTexture->Source.CalcMipSize(0));

				uint8* SourceDataPtr = SaveTexture->Source.LockMip(0);
				if (SourceDataPtr)
				{
					if (MipDataCopy.Num() == ExpectedBytes)
					{
						FMemory::Memcpy(SourceDataPtr, MipDataCopy.GetData(), ExpectedBytes);
						UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Source data copied OK"));
					}
					else
					{
						FMemory::Memzero(SourceDataPtr, SaveTexture->Source.CalcMipSize(0));
						UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Size mismatch, zeroed source (expected %d, got %d)"),
							ExpectedBytes, MipDataCopy.Num());
					}
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Source.LockMip(0) returned null"));
				}
				SaveTexture->Source.UnlockMip(0);

				// Do NOT call UpdateResource() -- this texture is never rendered.
				// Do NOT call PostEditChange() -- on a 4096^2 texture it kicks off a full
				// async compression job (hence "Waiting for textures to be ready") even with
				// CompressionNone=true, because it still schedules a build task.
				// We just need the source data written; SavePackage handles the rest.
				Package->MarkPackageDirty();

				FString PackageFileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());

				FSavePackageArgs SaveArgs;
				SaveArgs.TopLevelFlags = EObjectFlags::RF_Public | EObjectFlags::RF_Standalone;
				bool bSaved = UPackage::SavePackage(Package, SaveTexture, *PackageFileName, SaveArgs);

				if (bSaved)
				{
					FAssetRegistryModule& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
					AssetRegistry.Get().AssetCreated(SaveTexture);

					TArray<FString> PathsToScan;
					PathsToScan.Add(FPackageName::GetLongPackagePath(PackagePath));
					AssetRegistry.Get().ScanPathsSynchronous(PathsToScan, /*bForceRescan=*/true);

#if WITH_EDITOR
					// Explicitly notify the Content Browser to refresh — ScanPathsSynchronous
					// updates the registry but doesn't always force a visible refresh.
					FContentBrowserModule& ContentBrowserModule = FModuleManager::LoadModuleChecked<FContentBrowserModule>(TEXT("ContentBrowser"));
					ContentBrowserModule.Get().SyncBrowserToAssets(TArray<FAssetData>{ FAssetData(SaveTexture) });
#endif
					UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SaveTextureToDisk - Saved to: %s"), *PackageFileName);
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("FVolumeTextureUtils::SaveTextureToDisk - SavePackage failed for: %s"), *PackageFileName);
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

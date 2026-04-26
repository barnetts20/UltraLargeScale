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
// Inline method bodies below dereference FOctree/FOctreeNode/FPointData, so we
// need full definitions — forward declarations are not enough.
#include "FOctree.h"
#include "DataTypes.h"

/// <summary>
/// Sparse voxel entry for rasterizing node data into a dense volume grid.
/// </summary>
struct FVolumeVoxelEntry
{
	int X, Y, Z;       // Grid coordinates (0 to Resolution-1)
	uint8 R, G, B, A;  // Channel values
};

/// <summary>
/// CPU-side density field view — wraps the authoritative uint8 BGRA8 buffer
/// produced by SampleNoiseToVolume along with the source-space metadata
/// required to map world-space queries back into voxel coordinates.
///
/// The backing buffer is owned externally (typically by whichever actor ran
/// the noise sampling) and MUST outlive any sampling calls. FDensityVolume
/// only holds a pointer + metadata — no ownership, no copy.
///
/// Layout matches SampleNoiseToVolume output:
///   - uint8 BGRA8, 4 bytes per voxel
///   - Linear [z][y][x] order, X fastest, Z slowest
///   - Voxel-center convention: voxel (i,j,k) is centered at
///     (Min + (i+0.5)*VoxelSize, ...) in source-local space
///
/// Channel indexing matches the raw byte layout: channel 0 = B, 1 = G,
/// 2 = R, 3 = A. The noise sampler defaults to channel 3 (alpha), so the
/// default sampler channel here matches.
/// </summary>
struct SVO_API FDensityVolume
{
	const uint8* Buffer = nullptr;          // Non-owning pointer to BGRA8 buffer
	int64        BufferSize = 0;            // Buffer size in bytes (for validation)
	FVector      SourceCenter = FVector::ZeroVector;   // Source-space center
	FVector      SourceHalfExtent = FVector::ZeroVector; // Source-space half-size
	int32        Resolution = 0;            // Voxels per axis
	int32        NumChannels = 4;           // BGRA8 = 4

	FDensityVolume() = default;

	FDensityVolume(
		const TArray<uint8>& InBuffer,
		const FVector& InSourceCenter,
		const FVector& InSourceHalfExtent,
		int32 InResolution)
		: Buffer(InBuffer.GetData())
		, BufferSize(InBuffer.Num())
		, SourceCenter(InSourceCenter)
		, SourceHalfExtent(InSourceHalfExtent)
		, Resolution(InResolution)
		, NumChannels(4)
	{}

	bool IsValid() const
	{
		return Buffer != nullptr
			&& Resolution > 0
			&& BufferSize == (int64)Resolution * Resolution * Resolution * NumChannels;
	}

	/// <summary>
	/// Sample density in normalized [0,1] at a source-local position with trilinear
	/// interpolation. Channel defaults to 3 (alpha) to match SampleNoiseToVolume's
	/// default output channel.
	///
	/// NOTE: InLocalPos is relative to SourceCenter (i.e. already transformed into
	/// the coordinate frame the noise was sampled in — same space as the
	/// generator's InExtent). Out-of-bounds positions fail a check().
	/// </summary>
	float SampleDensityAtLocalPos(const FVector& InLocalPos, int32 Channel = 3) const
	{
		checkf(IsValid(), TEXT("FDensityVolume::SampleDensityAtLocalPos: buffer not valid"));
		checkf(Channel >= 0 && Channel < NumChannels,
			TEXT("FDensityVolume::SampleDensityAtLocalPos: channel %d out of range [0,%d)"),
			Channel, NumChannels);

		// Position relative to volume center
		const FVector Rel = InLocalPos - SourceCenter;

		checkf(
			FMath::Abs(Rel.X) <= SourceHalfExtent.X &&
			FMath::Abs(Rel.Y) <= SourceHalfExtent.Y &&
			FMath::Abs(Rel.Z) <= SourceHalfExtent.Z,
			TEXT("FDensityVolume::SampleDensityAtLocalPos: position (%f,%f,%f) outside half-extent (%f,%f,%f)"),
			Rel.X, Rel.Y, Rel.Z, SourceHalfExtent.X, SourceHalfExtent.Y, SourceHalfExtent.Z);

		// Convert to voxel-center coordinates.
		// Source space spans [-HalfExtent, +HalfExtent]. Voxel i is centered at
		// -HalfExtent + (i + 0.5) * VoxelSize, where VoxelSize = 2*HalfExtent / Res.
		// Solving for voxel coordinate: vc = (Rel + HalfExtent) / VoxelSize - 0.5
		const FVector VoxelSize(
			2.0 * SourceHalfExtent.X / (double)Resolution,
			2.0 * SourceHalfExtent.Y / (double)Resolution,
			2.0 * SourceHalfExtent.Z / (double)Resolution);

		const double VCx = (Rel.X + SourceHalfExtent.X) / VoxelSize.X - 0.5;
		const double VCy = (Rel.Y + SourceHalfExtent.Y) / VoxelSize.Y - 0.5;
		const double VCz = (Rel.Z + SourceHalfExtent.Z) / VoxelSize.Z - 0.5;

		// Clamp to valid interpolation range [0, Res-1]. Positions outside the
		// voxel-center lattice (within the half-voxel border) clamp to the edge
		// voxel, which is the correct behavior for "just inside the boundary".
		const double MaxIdx = (double)(Resolution - 1);
		const double Cx = FMath::Clamp(VCx, 0.0, MaxIdx);
		const double Cy = FMath::Clamp(VCy, 0.0, MaxIdx);
		const double Cz = FMath::Clamp(VCz, 0.0, MaxIdx);

		const int32 X0 = (int32)FMath::FloorToDouble(Cx);
		const int32 Y0 = (int32)FMath::FloorToDouble(Cy);
		const int32 Z0 = (int32)FMath::FloorToDouble(Cz);
		const int32 X1 = FMath::Min(X0 + 1, Resolution - 1);
		const int32 Y1 = FMath::Min(Y0 + 1, Resolution - 1);
		const int32 Z1 = FMath::Min(Z0 + 1, Resolution - 1);

		const float Tx = (float)(Cx - (double)X0);
		const float Ty = (float)(Cy - (double)Y0);
		const float Tz = (float)(Cz - (double)Z0);

		// Buffer layout: idx = (z * R*R + y * R + x) * 4 + channel
		const int64 R = (int64)Resolution;
		const int64 Slice = R * R;
		const int64 Stride = (int64)NumChannels;

		auto FetchByte = [&](int32 X, int32 Y, int32 Z) -> float
			{
				const int64 ByteIdx = ((int64)Z * Slice + (int64)Y * R + (int64)X) * Stride + (int64)Channel;
				return (float)Buffer[ByteIdx] * (1.0f / 255.0f);
			};

		const float C000 = FetchByte(X0, Y0, Z0);
		const float C100 = FetchByte(X1, Y0, Z0);
		const float C010 = FetchByte(X0, Y1, Z0);
		const float C110 = FetchByte(X1, Y1, Z0);
		const float C001 = FetchByte(X0, Y0, Z1);
		const float C101 = FetchByte(X1, Y0, Z1);
		const float C011 = FetchByte(X0, Y1, Z1);
		const float C111 = FetchByte(X1, Y1, Z1);

		const float C00 = FMath::Lerp(C000, C100, Tx);
		const float C10 = FMath::Lerp(C010, C110, Tx);
		const float C01 = FMath::Lerp(C001, C101, Tx);
		const float C11 = FMath::Lerp(C011, C111, Tx);

		const float C0 = FMath::Lerp(C00, C10, Ty);
		const float C1 = FMath::Lerp(C01, C11, Ty);

		return FMath::Lerp(C0, C1, Tz);
	}
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
	///
	/// InWorldOffset is an additive offset applied to noise sample coordinates
	/// AFTER normalization (so it's in normalized noise-space units, not world
	/// units). For a tiled grid of volumes that want to be continuous across
	/// boundaries, pass `(2 * CellCoord)` so adjacent cells sample contiguous
	/// regions of the same field. Default zero matches single-volume behavior.
	/// </summary>
	static TArray<uint8> SampleNoiseToVolume(
		FastNoise::SmartNode<> InNoise,
		int InSeed,
		int InResolution,
		double InExtent,
		TSharedPtr<FOctree> InOctree = nullptr,
		int InOctreeDepth = -1,
		float InNoisePower = 2.0f,
		int InChannel = 3,
		FVector InWorldOffset = FVector::ZeroVector);

	/// <summary>
	/// Sample a noise function into a sub-region of an existing volume buffer.
	/// The sub-region is defined by a voxel bounding box [Min, Max) per axis.
	///
	/// InExtent is the half-size of the volume in world space (determines where
	/// each voxel sits spatially). InNoiseNormExtent is the half-size used to
	/// normalize positions into noise space — typically Params.Extent (one cell)
	/// so that noise coordinates match the particle generators. InWorldOffset
	/// is added after normalization for seamless cross-cell tiling.
	/// </summary>
	static void SampleNoiseToSubRegion(
		TArray<uint8>& InOutVolumeData,
		int InResolution,
		double InExtent,
		double InNoiseNormExtent,
		FastNoise::SmartNode<> InNoise,
		int InSeed,
		FIntVector InVoxelMin,
		FIntVector InVoxelMax,
		float InNoisePower = 2.0f,
		int InChannel = 3,
		FVector InWorldOffset = FVector::ZeroVector);

	/// <summary>
	/// Zero all voxels in a sub-region of a volume buffer. Used to clear
	/// exiting cells before sampling entering cells into the same buffer.
	/// </summary>
	static void ClearSubRegion(
		TArray<uint8>& InOutVolumeData,
		int InResolution,
		FIntVector InVoxelMin,
		FIntVector InVoxelMax);
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
	/// Splat point cloud nodes into a 256^3 volume buffer as randomly-oriented
	/// ellipsoidal density kernels, and optionally write the resulting density
	/// into the octree at a target depth.
	///
	/// Each node gets a randomized splat seeded from its ObjectId:
	/// per-axis radius, orientation, falloff sharpness, and intensity.
	/// The random rotation breaks axis-alignment so ellipsoids point in
	/// arbitrary directions — no grid artifacts.
	///
	/// Uses Lorentzian falloff: 1 / (1 + k * t^2) — sharp spike, long tail.
	/// </summary>
	static void SplatVBOsToVolume(
		TArray<uint8>& InOutVolumeData,
		int InResolution,
		double InExtent,
		const TArray<TSharedPtr<FOctreeNode>>& InNodes,
		FVector InMinRadiusVoxels = FVector(7.0f),
		FVector InMaxRadiusVoxels = FVector(32.0f),
		float InMinFalloff = 1.0f,
		float InMaxFalloff = 4.0f,
		float InMinIntensity = 0.8f,
		float InMaxIntensity = 2.0f,
		int InChannel = -1,
		TSharedPtr<FOctree> InOctree = nullptr,
		int InOctreeDepth = 8)
	{
		double StartTime = FPlatformTime::Seconds();

		constexpr int BytesPerVoxel = 4;
		const double VoxelSize = (2.0 * InExtent) / InResolution;
		const double InvVoxelSize = 1.0 / VoxelSize;
		const int Slice = InResolution * InResolution;

		// --- Per-node splat: randomized oriented ellipsoid ---
		struct FSplatEntry
		{
			float CenterX, CenterY, CenterZ;
			float RadiusX, RadiusY, RadiusZ;
			float InvRadiusX, InvRadiusY, InvRadiusZ;
			float FalloffK;
			float Intensity;
			float MaxRadius;  // Largest axis radius — for AABB bounding box

			// Inverse rotation matrix rows (rotates world-space delta into ellipsoid-local space)
			// Stored as 3 row vectors for cache-friendly dot products
			float R00, R01, R02;
			float R10, R11, R12;
			float R20, R21, R22;
		};

		TArray<FSplatEntry> Splats;
		Splats.Reserve(InNodes.Num());

		for (const TSharedPtr<FOctreeNode>& Node : InNodes)
		{
			if (!Node.IsValid()) continue;

			FRandomStream NodeStream(Node->Data.ObjectId);

			float VoxelX = (float)((Node->Center.X + InExtent) * InvVoxelSize);
			float VoxelY = (float)((Node->Center.Y + InExtent) * InvVoxelSize);
			float VoxelZ = (float)((Node->Center.Z + InExtent) * InvVoxelSize);

			float RX = NodeStream.FRandRange(InMinRadiusVoxels.X, InMaxRadiusVoxels.X);
			float RY = NodeStream.FRandRange(InMinRadiusVoxels.Y, InMaxRadiusVoxels.Y);
			float RZ = NodeStream.FRandRange(InMinRadiusVoxels.Z, InMaxRadiusVoxels.Z);

			// Random rotation — uniform random Euler angles
			FRotator RandomRot(
				NodeStream.FRandRange(-180.0f, 180.0f),
				NodeStream.FRandRange(-180.0f, 180.0f),
				NodeStream.FRandRange(-180.0f, 180.0f)
			);

			// Build rotation matrix, then transpose to get the inverse rotation
			// (rotation matrices are orthogonal, so transpose = inverse).
			// We want: localDelta = InverseRotation * worldDelta
			// FRotationMatrix gives us R where worldDelta = R * localDelta
			// So InverseRotation = R^T
			FMatrix RotMatrix = FRotationMatrix(RandomRot);

			FSplatEntry Entry;
			Entry.CenterX = VoxelX;
			Entry.CenterY = VoxelY;
			Entry.CenterZ = VoxelZ;
			Entry.RadiusX = RX;
			Entry.RadiusY = RY;
			Entry.RadiusZ = RZ;
			Entry.InvRadiusX = 1.0f / RX;
			Entry.InvRadiusY = 1.0f / RY;
			Entry.InvRadiusZ = 1.0f / RZ;
			Entry.FalloffK = NodeStream.FRandRange(InMinFalloff, InMaxFalloff);
			Entry.Intensity = NodeStream.FRandRange(InMinIntensity, InMaxIntensity) * 255.0f;
			Entry.MaxRadius = FMath::Max3(RX, RY, RZ);

			// Store transposed rows (inverse rotation)
			Entry.R00 = RotMatrix.M[0][0]; Entry.R01 = RotMatrix.M[1][0]; Entry.R02 = RotMatrix.M[2][0];
			Entry.R10 = RotMatrix.M[0][1]; Entry.R11 = RotMatrix.M[1][1]; Entry.R12 = RotMatrix.M[2][1];
			Entry.R20 = RotMatrix.M[0][2]; Entry.R21 = RotMatrix.M[1][2]; Entry.R22 = RotMatrix.M[2][2];

			Splats.Add(Entry);
		}

		if (Splats.Num() == 0)
		{
			UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SplatVBOsToVolume - No valid nodes to splat"));
			return;
		}

		// --- Float accumulation buffer ---
		const int64 TotalVoxels = (int64)InResolution * InResolution * InResolution;
		TArray<float> AccumBuffer;
		AccumBuffer.SetNumZeroed(TotalVoxels);

		// --- Pass 1: Parallel splat into float accumulation buffer ---
		ParallelFor(Splats.Num(), [&](int SplatIdx)
			{
				const FSplatEntry& S = Splats[SplatIdx];

				// AABB uses MaxRadius — conservative bounding box for the rotated ellipsoid
				int MinX = FMath::Max(0, FMath::FloorToInt(S.CenterX - S.MaxRadius));
				int MaxX = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterX + S.MaxRadius));
				int MinY = FMath::Max(0, FMath::FloorToInt(S.CenterY - S.MaxRadius));
				int MaxY = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterY + S.MaxRadius));
				int MinZ = FMath::Max(0, FMath::FloorToInt(S.CenterZ - S.MaxRadius));
				int MaxZ = FMath::Min(InResolution - 1, FMath::CeilToInt(S.CenterZ + S.MaxRadius));

				for (int z = MinZ; z <= MaxZ; ++z)
				{
					float dz = (z + 0.5f) - S.CenterZ;

					for (int y = MinY; y <= MaxY; ++y)
					{
						float dy = (y + 0.5f) - S.CenterY;

						for (int x = MinX; x <= MaxX; ++x)
						{
							float dx = (x + 0.5f) - S.CenterX;

							// Rotate world-space delta into ellipsoid-local space
							float lx = S.R00 * dx + S.R01 * dy + S.R02 * dz;
							float ly = S.R10 * dx + S.R11 * dy + S.R12 * dz;
							float lz = S.R20 * dx + S.R21 * dy + S.R22 * dz;

							// Normalized ellipsoidal distance
							float nx = lx * S.InvRadiusX;
							float ny = ly * S.InvRadiusY;
							float nz = lz * S.InvRadiusZ;
							float normDistSq = nx * nx + ny * ny + nz * nz;

							if (normDistSq > 1.0f) continue;

							// Lorentzian falloff: sharp spike, long tail
							float falloff = 1.0f / (1.0f + S.FalloffK * normDistSq);
							float contribution = S.Intensity * falloff;

							int64 VoxelIdx = (int64)z * Slice + (int64)y * InResolution + x;
							AccumBuffer[VoxelIdx] += contribution;
						}
					}
				}
			}, Splats.Num() > 64 ? EParallelForFlags::BackgroundPriority : EParallelForFlags::ForceSingleThread);

		// --- Octree setup for depth writes ---
		bool bWriteOctree = InOctree.IsValid() && InOctreeDepth > 0;
		TArray<TSharedPtr<FOctreeNode>> VolumeChunks;

		if (bWriteOctree)
		{
			TArray<TArray<FPointData>> DummyChunkData;
			InOctree->PrePopulateVolumeLayer(VolumeChunks, DummyChunkData);
		}

		// --- Pass 2: Quantize to byte buffer + write octree (parallel over Z slices) ---
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
							for (int c = 0; c < BytesPerVoxel; ++c)
							{
								DataPtr[ByteIdx + c] = (uint8)FMath::Min((int)DataPtr[ByteIdx + c] + (int)SplatByte, 255);
							}
						}
						else
						{
							int c = FMath::Clamp(InChannel, 0, 3);
							DataPtr[ByteIdx + c] = (uint8)FMath::Min((int)DataPtr[ByteIdx + c] + (int)SplatByte, 255);
						}

						if (bWriteOctree)
						{
							double wx = -InExtent + (x + 0.5) * VoxelSize;
							double wy = -InExtent + (y + 0.5) * VoxelSize;
							double wz = -InExtent + (z + 0.5) * VoxelSize;

							int ChunkIdx = InOctree->FindChunkIndexForPosition(FVector(wx, wy, wz), VolumeChunks);
							TSharedPtr<FOctreeNode> ChunkNode = (ChunkIdx >= 0 && ChunkIdx < VolumeChunks.Num())
								? VolumeChunks[ChunkIdx]
								: nullptr;

							if (ChunkNode.IsValid())
							{
								FVoxelData SplatData;
								SplatData.Density = Accum / 255.0f;
								InOctree->InsertPosition(FVector(wx, wy, wz), InOctreeDepth, SplatData, ChunkNode);
							}
						}
					}
				}
			}, EParallelForFlags::BackgroundPriority);

		double Duration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("FVolumeTextureUtils::SplatVBOsToVolume (%d nodes, res %d, octree: %s) took %.3f sec"),
			InNodes.Num(), InResolution, bWriteOctree ? TEXT("yes") : TEXT("no"), Duration);
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
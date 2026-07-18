// GalaxyDataGenerator.h
// Galaxy density field (spiral SDF + bulge/disc/background), tier generation,
// and volume texture sampling.

#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "FastNoise/FastNoise.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"


/** Owns noise composition and tier generation for the galaxy layer; mirrors
 *  UniverseDataGenerator. The galaxy actor wires tier callbacks that delegate
 *  here; this class has no knowledge of actors, Niagara, octrees, or the
 *  streaming pipeline. */

class SVO_API GalaxyDataGenerator
{
public:
	GalaxyDataGenerator() {};
	GalaxyDataGenerator(FGalaxyParams InParams) : Params(InParams) {};

	FGalaxyParams Params;
	FastNoise::SmartNode<> DensityNoise;

#pragma region Math Helpers

	/** Smooth maximum of three values (A, B, C) via the log-sum-exp form:
	 *  SmoothMax ~ max(A, B, C) with a differentiable blend zone of radius ~1/K
	 *  around each crossing (higher K = sharper, approaching hard max; lower K =
	 *  softer). K > 0, typically 2-16.
	 *
	 *  LSE form: (1/K) * log(exp(K*A) + exp(K*B) + exp(K*C)), numerically
	 *  stabilized by subtracting the running max before exponentiation so the
	 *  result never overflows regardless of K. */
	static FORCEINLINE float SmoothMax(float A, float B, float C, float K = 8.0f)
	{
		const float M = FMath::Max3(A, B, C);
		const float ExpA = FMath::Exp(K * (A - M));
		const float ExpB = FMath::Exp(K * (B - M));
		const float ExpC = FMath::Exp(K * (C - M));
		return M + FMath::Loge(ExpA + ExpB + ExpC) / K;
	}

#pragma endregion

#pragma region Signed Distance Fields & Analytic Density
	/** Arms use a two-phase approach: SampleArmSDF returns unsigned distance from
	 *  the centerline, then SampleDensity remaps through core/envelope thresholds.
	 *  The disc and bulge are rotationally symmetric so they use direct analytic
	 *  profiles instead of SDF remapping. */

	 /** Unsigned distance from the nearest arm centerline at query radius rXY
	  *  (pre-computed cylindrical radius) for InNormPos in [-1, 1] normalized
	  *  galaxy space. */
	float SampleArmSDF(const FVector& InNormPos, double rXY) const;

	/** Signed distance to the bulge ellipsoid. Positive inside.
	 *  Used as a hard boundary guard; actual bulge density uses a Hernquist profile. */
	float SampleBulgeSDF(const FVector& InNormPos) const;

	/** Analytic bulge density using a Hernquist profile in oblate coordinates.
	 *  Returns density in [0, 1]; hard zero outside BulgeCutoffRadius. */
	float SampleBulgeDensity(const FVector& InNormPos) const;

	/** Signed distance to the disc cylinder. Positive inside.
	 *  Used as a hard boundary guard in SampleDiscDensity. */
	float SampleDiscSDF(const FVector& InNormPos, double rXY, double absZ) const;

	/** Analytic disc density at the given cylindrical coordinates.
	 *  Separable exponential radial x exp(-|z/h|^falloff) vertical profile.
	 *  Returns density in [0, 1]; hard zero outside the disc cylinder. */
	float SampleDiscDensity(double rXY, double absZ) const;

#pragma endregion

#pragma region Density Sampling
	/** Composites all active layers into a single [0, 1] density value. Arms,
	 *  disc, and bulge are max-blended (union); background is additive.
	 *    Arms:  SDF-based, core/envelope remap + radial growth.
	 *    Disc:  Analytic exponential radial x vertical profile.
	 *    Bulge: Hernquist profile in oblate coordinates.
	 *    BG:    Additive halo (zeroed until compositing phase). */

	 /** Sample density at a single normalized position.
	  *  InNormPos is in [-1, 1] noise space (position / Extent).
	  *  Returns density in [0, 1]. */
	float SampleDensity(const FVector& InNormPos) const;

	/** Batch-evaluate the density field for an array of positions. */
	void SampleDensityBatch(float* OutDensity, int32 InCount, const float* InX, const float* InY, const float* InZ) const;

#pragma endregion

#pragma region Initialization

	/** Build the encoded noise graph (kept for future use) and mark ready. */
	void Initialize();

	/** Build noise from encoded tree. Kept for future FastNoise swap-in. */
	FastNoise::SmartNode<> BuildNoise() const;

	/** Sample the density field into a CPU-side BGRA8 volume texture buffer.
	 *  Uses the C++ SampleDensity path directly rather than going through
	 *  FVolumeTextureUtils::SampleNoiseToVolume. */
	TArray<uint8> SampleNoiseVolume(int InNoiseResolution) const;

#pragma endregion

#pragma region Tier Generation Callbacks
	/** Self-contained generation functions that write directly into particle
	 *  buffers; the galaxy actor's tier system calls them via
	 *  FParticleTierConfig::GenerateCallback lambdas.
	 *
	 *  A single generation function serves all tiers; Large/Mid/Small differ only
	 *  in the candidate volume (full extent vs cell-local), the tier params (scale
	 *  range, density curve), and the seed offset for stream isolation. */

	 /** Generates particles for a single tier cell (InCoord, InSlotIndex) via
	  *  batched noise rejection sampling: candidates are distributed uniformly
	  *  within InCellExtent around InNodeCenter, density-gated by SampleDensity,
	  *  and written into InBuffer at the slot region for InSlotIndex. InTierParams
	  *  supplies the scale range and density curve; InSeedOffset is added to
	  *  Params.Seed for stream isolation between tiers; OutSlotCount receives the
	  *  accepted particle count. For the Large tier (full galaxy), pass
	  *  InNodeCenter = ZeroVector and InCellExtent = Params.Extent. */
	void GenerateTierNode(const FIntVector& InCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InBuffer, const FVector& InNodeCenter, double InCellExtent, const FTierParams& InTierParams, int32 InSeedOffset, int32& OutSlotCount) const;

#pragma endregion

#pragma region Large Tier SDF Culling

	/** One active cell in the large tier culling grid. */
	struct FActiveLargeTierCell
	{
		/** Galaxy-local center (not normalized). */
		FVector Center;

		/** Half-extent of the cell (same on all axes). */
		double HalfExt;

		/** Integer grid coordinate at LargeTierCullDepth. */
		FIntVector GridCoord;
	};

	/** Subdivide the galaxy volume into a uniform grid at Params.LargeTierCullDepth
	 *  and return only cells where at least one corner has non-zero composite
	 *  density. Cells whose all 8 corners evaluate to SampleDensity == 0 are
	 *  entirely outside all SDF envelopes and can never produce accepted candidates.
	 *
	 *  Grid covers [-Extent, +Extent] on each axis (galaxy-local). Corner
	 *  positions are converted to normalized [-1, 1] space before testing. */
	TArray<FActiveLargeTierCell> CollectActiveLargeTierCells() const;

	/** Generate the full large tier slot by iterating over SDF-active cells.
	 *  Candidates are distributed proportionally: each active cell receives
	 *  ceil(SlotCapacity / ActiveCellCount) candidates, capped at SlotCapacity
	 *  total. This concentrates sampling on arms/disc/bulge and avoids wasting
	 *  rejection attempts on empty inter-arm space.
	 *
	 *  Writes into InBuffer at the slot region for InSlotIndex. Pads remaining
	 *  entries dead. Sets OutSlotCount to the accepted particle count. */
	void GenerateLargeTierSlot(
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		int32& OutSlotCount) const;

#pragma endregion
};
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PointCloudGenerator.h"
#include "ProceduralSpaceActor.h"
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"
#include "UniverseDataGenerator.generated.h"

/// Noise-graph tuning knobs consumed by UniverseDataGenerator::BuildNoise().
/// Defaults reproduce the original hardcoded values.
USTRUCT(BlueprintType)
struct SVO_API FNoiseParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float MasterScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterFalloff = 32.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterScale = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterMulti = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterRemapMax = 1.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterRemapMin = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebFalloff = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebRemapMin = -0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebRemapMax = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmp = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpFreq = 1.0f;
};

/// Per-tier streaming parameters exposed in the editor.
/// Scale ranges (MinScale / MaxScale) are NOT edited directly — they are
/// derived at runtime by FUniverseParams::DeriveScaleRanges() from
/// MaxEntityScale and the tier depth sequence.
USTRUCT(BlueprintType)
struct SVO_API FTierParams
{
	GENERATED_BODY()

	// Octree grid depth that defines this tier's cell size.
	// Cell extent = TreeExtent / (1 << GridDepth).
	// Use evenly-spaced depths (e.g. 1, 4, 7 → spacing of 3).
	// Scale ratio between adjacent tiers = 2^spacing (spacing 3 → ratio 8).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 GridDepth = 1;

	// Half-width of the 3D neighborhood streamed around the player.
	// 1 -> 3x3x3 = 27 slots.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 NeighborhoodRadius = 1;

	// Max particles per slot (candidate count before rejection).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 MaxParticlesPerSlot = 500;

	// Maps a uniform [0,1] sample to a [0,1] t-value that lerps between
	// MinScale and MaxScale. Controls the size distribution of particles
	// within this tier. Defaults to identity (linear).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution")
	FRuntimeFloatCurve ScaleDistribution;

	// Maps the raw noise density [0,1] to a modified density [0,1] before
	// the rejection gate. Controls how aggressively the noise field is
	// interpreted — e.g. a steep curve concentrates particles in high-density
	// regions, a flat curve makes them more uniform. Defaults to identity.
	// Values > 1.0 are clamped, allowing the curve to sharpen features.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution")
	FRuntimeFloatCurve DensityResponse;

	// --- Derived at runtime — not directly editable ---

	// Largest entity scale this tier represents.
	double MaxScale = 0.0;

	// Smallest entity scale this tier represents.
	double MinScale = 0.0;

	// Initialize both curves to identity: f(x) = x.
	FTierParams()
	{
		ScaleDistribution.GetRichCurve()->AddKey(0.0f, 0.0f);
		ScaleDistribution.GetRichCurve()->AddKey(1.0f, 1.0f);

		DensityResponse.GetRichCurve()->AddKey(0.0f, 0.0f);
		DensityResponse.GetRichCurve()->AddKey(1.0f, 1.0f);
	}

	/// Derive MinScale/MaxScale for an ordered array of tiers from a single
	/// MaxEntityScale value and the depth sequence between tiers.
	///
	/// Tiers must be ordered shallowest-first (Large → Mid → Small).
	/// The depth spacing between adjacent tiers determines the scale ratio:
	///   ratio = 2 ^ (nextDepth - thisDepth)
	/// The last tier mirrors the spacing of the previous pair.
	///
	/// Example with depths 1, 4, 7 and MaxEntityScale = 1e23:
	///   Large: 1.25e22 → 1e23    (ratio 8, spacing 3)
	///   Mid:   1.5625e21 → 1.25e22 (ratio 8, spacing 3)
	///   Small: 1.953125e20 → 1.5625e21 (ratio 8, mirrors spacing 3)
	static void DeriveTierScaleRanges(double MaxEntityScale, TArrayView<FTierParams*> Tiers)
	{
		const int32 NumTiers = Tiers.Num();
		if (NumTiers == 0) return;

		Tiers[0]->MaxScale = MaxEntityScale;

		for (int32 i = 0; i < NumTiers; ++i)
		{
			int32 DepthDelta;
			if (i + 1 < NumTiers)
			{
				DepthDelta = Tiers[i + 1]->GridDepth - Tiers[i]->GridDepth;
			}
			else
			{
				DepthDelta = Tiers[i]->GridDepth - Tiers[i - 1]->GridDepth;
			}

			const double Ratio = static_cast<double>(1 << FMath::Clamp(DepthDelta, 1, 20));
			Tiers[i]->MinScale = Tiers[i]->MaxScale / Ratio;

			if (i + 1 < NumTiers)
			{
				Tiers[i + 1]->MaxScale = Tiers[i]->MinScale;
			}
		}
	}
};

/// UNIVERSE GENERATION PARAM STRUCT
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	// --- Noise ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FNoiseParams DensityParams;

	// --- Tier scale derivation ---

	// The absolute largest entity scale (world units) that the sector supports.
	// All tier scale ranges cascade downward from this single value:
	//   Tier[0].MaxScale = MaxEntityScale
	//   Tier[0].MinScale = MaxEntityScale / 2^(depth[1] - depth[0])
	//   Tier[1].MaxScale = Tier[0].MinScale
	//   ... and so on.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 1e23;

	// --- Per-tier streaming configs ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	// --- Gas layer params (paired with the large tier) ---

	// Gas sprite extent as a multiplier of the per-particle cluster extent.
	// GasExtent = ClusterExtent * Lerp(GasExtentMinMultiplier, GasExtentMaxMultiplier, Density).
	// Keeps gas automatically in the same coordinate space as cluster sprites.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasExtentMinMultiplier = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasExtentMaxMultiplier = 1000.0f;

	static constexpr const char* EncodedTree = "EAAAAIA/GQAbABsAEwAAAEBAJAAgAAAAFwAAAAAAAACAP8UggD8AAAAADQADAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEXAAAAAAAAAIA/zcxMvQAAgD8kAAIAAAD//wEAAAAASEIB//8GAAAAAIA+";

	/// Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	/// depth sequence. Delegates to FTierParams::DeriveTierScaleRanges.
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		FTierParams::DeriveTierScaleRanges(MaxEntityScale, Tiers);
	}

	FUniverseParams() {
		Seed = 69;
		Extent = 274877906944;
		UnitScale = 2e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

		// Tier streaming params — depths evenly spaced by 3.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.MaxParticlesPerSlot = 8000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 4000;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 2000;

		// Scale ranges derived from MaxEntityScale (1e23) + depth spacing (3).
		// 2^3 = 8, so each tier covers one octave of scale:
		//   Large: 1.25e22 → 1e23
		//   Mid:   1.5625e21 → 1.25e22
		//   Small: 1.953125e20 → 1.5625e21
		DeriveScaleRanges();

		// Per-tier ScaleDistribution and DensityResponse curves default to
		// identity (linear) via FTierParams(). Override in BP defaults or
		// editor if a non-linear distribution is desired.
	}
};

/// UNIVERSE GENERATOR - GENERATES DATA FOR POPULATING A UNIVERSE
///
/// Owns all noise composition and particle generation logic. The sector actor
/// wires tier callbacks that delegate here; this class has no knowledge of
/// actors, Niagara, octrees, or the streaming pipeline.
class SVO_API UniverseDataGenerator {
public:
	UniverseDataGenerator() {};
	UniverseDataGenerator(FUniverseParams InParams) {
		Params = InParams;
	};

	FUniverseParams Params;
	FastNoise::SmartNode<> DensityNoise;
	TArray<FPointData> GeneratedData;

	// -----------------------------------------------------------------------
	// Noise Composition
	// -----------------------------------------------------------------------

	// Build the sector-scale density noise graph from the current Params.
	// Pure function of FNoiseParams — no actor state needed.
	void Initialize();
	FastNoise::SmartNode<> BuildNoise() const;

	// Sample the noise field into a CPU-side volume texture buffer.
	// Returns the raw BGRA8 data suitable for FDensityVolume or GPU upload.
	TArray<uint8> SampleNoiseVolume(
		int InNoiseResolution,
		const FIntVector& InCellCoord) const;

	// -----------------------------------------------------------------------
	// Tier Generation Callbacks
	// -----------------------------------------------------------------------
	// These are self-contained generation functions that write directly into
	// particle buffers. The sector actor's tier system calls them via
	// FParticleTierConfig::GenerateCallback lambdas.
	//
	// Grid geometry (NodeCenter, CellExtent) is passed in rather than
	// computed internally so the generator stays decoupled from the actor's
	// tree extent multiplier and grid-depth conventions.

	// Large tier: generates cluster + gas particles using batched noise.
	// OutSlotCount receives the number of accepted particles.
	void GenerateLargeTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InClusterBuffer,
		FNiagaraParticleBuffer& InGasBuffer,
		const FVector& InNodeCenter,
		int32& OutSlotCount) const;

	// Mid tier: generates cluster particles at mid-grid scale.
	// OutSlotCount receives the number of accepted particles.
	void GenerateMidTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;

	// Small tier: generates galaxy-scale particles.
	// OutSlotCount receives the number of accepted particles.
	void GenerateSmallTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;
};
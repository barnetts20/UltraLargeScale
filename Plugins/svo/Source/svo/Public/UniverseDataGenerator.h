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
};

/// UNIVERSE GENERATION PARAM STRUCT
///
/// Inherits Seed, Extent, UnitScale from FBaseParams.
/// NOTE: Rotation and ParentColor are also inherited from FBaseParams but are
/// unused at the sector level — they exist for child actors (Galaxy, StarSystem)
/// that receive these values from their spawning parent. They will appear in
/// the editor but have no effect on sector behavior.
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	// --- Noise ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FNoiseParams NoiseParams;

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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasMinExtent = 1e15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasMaxExtent = 5e16f;

	static constexpr const char* EncodedTree = "EAAAAIA/GQAbABsAEwAAAEBAJAAgAAAAFwAAAAAAAACAP8UggD8AAAAADQADAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEXAAAAAAAAAIA/zcxMvQAAgD8kAAIAAAD//wEAAAAASEIB//8GAAAAAIA+";

	/// Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	/// depth sequence. Call after setting depths and MaxEntityScale.
	///
	/// Tiers are ordered shallowest-first (Large → Mid → Small).
	/// The depth spacing between adjacent tiers determines the scale ratio:
	///   ratio = 2 ^ (nextDepth - thisDepth)
	/// The last tier mirrors the spacing of the previous pair.
	///
	/// Example with depths 1, 4, 7 and MaxEntityScale = 1e23:
	///   Large: 1.25e22 → 1e23    (ratio 8, spacing 3)
	///   Mid:   1.5625e21 → 1.25e22 (ratio 8, spacing 3)
	///   Small: 1.953125e20 → 1.5625e21 (ratio 8, mirrors spacing 3)
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		constexpr int32 NumTiers = UE_ARRAY_COUNT(Tiers);

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

	FUniverseParams() {
		Seed = 69;
		Extent = 2147483648;
		UnitScale = 2e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

		// Tier streaming params — depths evenly spaced by 3.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.MaxParticlesPerSlot = 8000;

		MidTier.GridDepth = 4;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 4000;

		SmallTier.GridDepth = 7;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 2000;

		DeriveScaleRanges();
	}
};

/// UNIVERSE GENERATOR — Owns noise construction, grid math helpers, and
/// per-tier particle generation methods. Used exclusively by ASectorActor
/// but encapsulates all generation logic independent of the actor.
class SVO_API UniverseDataGenerator : public PointCloudGenerator {
public:
	UniverseDataGenerator() : PointCloudGenerator(8647) {};
	UniverseDataGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	FUniverseParams Params;

	// Legacy override required by PointCloudGenerator base class.
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	// Preferred entry point: uses a pre-built CPU-side density volume for
	// rejection sampling instead of going through the octree.
	void GenerateData(const FDensityVolume& InDensityVolume);

	TArray<FPointData> GeneratedData;

	// --- Noise Construction ---

	// Build the sector density noise graph from Params.NoiseParams.
	// The returned node is immutable and safe to share across threads for
	// read-only evaluation via GenPositionArray3D.
	FastNoise::SmartNode<> BuildNoise(int InSeed) const;

	// --- Grid Coord Helpers ---
	// Static helpers parameterized by extent and tree multiplier so generation
	// methods don't need a reference back to the actor.

	static FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth,
		double InExtent, double InTreeExtentMultiplier);

	static double GetGridCellExtent(int32 InGridDepth,
		double InExtent, double InTreeExtentMultiplier);

	// --- Tier-Specific Generation Methods ---
	// Each receives the grid coord, slot index, buffer(s), and a pre-built
	// noise instance. Returns the number of accepted particles written.
	// Dead-particle padding is written internally; the caller writes SlotCounts.

	// Large tier: generates cluster + gas particles using batched noise.
	// Candidates scatter within ±Extent of the cell center; noise offset is
	// shared across all candidates in a cell (coord-derived).
	int32 GenerateLargeNode(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InClusterBuffer, FNiagaraParticleBuffer& InGasBuffer,
		const FastNoise::SmartNode<>& InNoise, int32 InGridDepth) const;

	// Mid tier: generates cluster particles at mid-tier grid scale.
	// Same noise-driven rejection as the large tier but writes a single buffer
	// with rotations.
	int32 GenerateMidNode(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FastNoise::SmartNode<>& InNoise, int32 InGridDepth) const;

	// Small tier: generates galaxy-scale particles using batched noise.
	// Candidates scatter within ±CellExtent of the cell center; noise offset
	// is computed per-candidate (may straddle coarse cell boundaries).
	int32 GenerateSmallNode(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FastNoise::SmartNode<>& InNoise, int32 InGridDepth) const;
};
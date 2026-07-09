// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"     // FBaseParams (base of FUniverseParams)
#include "FTierStreamingSystem.h"     // FTierParams (per-tier members)
#include "UniverseParams.generated.h"


/// Noise-graph tuning knobs consumed by UniverseDataGenerator::BuildNoise().
/// Defaults reproduce the original hardcoded values.
USTRUCT(BlueprintType)
struct SVO_API FUniverseDensityParams
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


/// UNIVERSE GENERATION PARAM STRUCT
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	// --- Noise ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FUniverseDensityParams DensityParams;

	// --- Tier scale derivation ---

	// The absolute largest entity scale (world units) that the sector supports.
	// All tier scale ranges cascade downward from this single value:
	//   Tier[0].MaxScale = MaxEntityScale
	//   Tier[0].MinScale = MaxEntityScale / 2^(depth[1] - depth[0])
	//   Tier[1].MaxScale = Tier[0].MinScale
	//   ... and so on.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 1e22;

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
	float GasExtentMinMultiplier = 2000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasExtentMaxMultiplier = 20000.0;

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
		UnitScale = 1.6e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

		// Tier streaming params — depths evenly spaced by 2 (ratio 4 per tier).
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.SlotCapacity = 8000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 4000;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 2000;

		// Scale ranges derived from MaxEntityScale (1e22) + depth spacing (2).
		// 2^2 = 4, so each tier covers two octaves of scale (64x total spread):
		//   Large: 2.5e21    → 1e22
		//   Mid:   6.25e20   → 2.5e21
		//   Small: 1.5625e20 → 6.25e20
		DeriveScaleRanges();

		// Per-tier ScaleDistribution and DensityResponse curves default to
		// identity (linear) via FTierParams(). Override in BP defaults or
		// editor if a non-linear distribution is desired.
	}
};
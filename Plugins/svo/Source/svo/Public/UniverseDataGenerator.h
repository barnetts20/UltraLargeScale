// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PointCloudGenerator.h"
#include "ProceduralSpaceActor.h"
#include "FVolumeTextureUtils.h"
#include "UniverseDataGenerator.generated.h"

/// Noise-graph tuning knobs consumed by ASectorActor::BuildNoise().
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

/// Per-tier streaming parameters exposed in the editor. One instance per
/// galaxy-sprite scale band (Large / Mid / Small). BuildTierConfigs reads
/// these to populate FParticleTierConfig at init time.
USTRUCT(BlueprintType)
struct SVO_API FTierParams
{
	GENERATED_BODY()

	// Octree grid depth that defines this tier's cell size.
	// Cell extent = TreeExtent / (1 << GridDepth).
	//   Large = 1 (cell = 2*Extent), Mid = 2 (cell = Extent),
	//   Small = 7 at default ScanDepth 6 (cell = Extent / 32).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 GridDepth = 1;

	// Half-width of the 3D neighborhood streamed around the player.
	// 1 -> 3x3x3 = 27 slots.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 NeighborhoodRadius = 1;

	// Max particles per slot (candidate count before rejection).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 MaxParticlesPerSlot = 500;

	// Scale range for particles generated in this tier.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MinScale = 3e23;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxScale = 3e25;
};

/// UNIVERSE GENERATION PARAM STRUCT
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	// --- Universe data generator params (used by UniverseDataGenerator) ---

	// Number of galaxy objects to place in the sector volume.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	int Count = 500;

	// Scale distribution curve shared across all tiers and the data generator.
	// Maps a uniform [0,1] sample to a [0,1] t-value that lerps between
	// the tier's MinScale and MaxScale.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FRuntimeFloatCurve ScaleDistributionCurve;

	// Position jitter applied to generated galaxy positions (data generator only).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double Jitter = .02;

	// --- Noise ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FNoiseParams NoiseParams;

	// --- Per-tier streaming configs ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	// --- Gas layer params (paired with the large tier) ---

	// Per-particle extent at density=0 (lower bound of density-driven lerp).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasMinExtent = 1e15f;

	// Per-particle extent at density=1 (upper bound of density-driven lerp).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gas")
	float GasMaxExtent = 5e16f;

	static constexpr const char* EncodedTree = "EAAAAIA/GQAbABsAEwAAAEBAJAAgAAAAFwAAAAAAAACAP8UggD8AAAAADQADAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEXAAAAAAAAAIA/zcxMvQAAgD8kAAIAAAD//wEAAAAASEIB//8GAAAAAIA+";

	FUniverseParams() {
		Seed = 69;
		Extent = 2147483648;
		UnitScale = 2e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

		// Large tier (was "Coarse")
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.MaxParticlesPerSlot = 500;
		LargeTier.MinScale = 3e23;
		LargeTier.MaxScale = 3e25;

		// Mid tier
		MidTier.GridDepth = 2;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 500;
		MidTier.MinScale = 3e20;
		MidTier.MaxScale = 3e23;

		// Small tier (was "Proximity")
		SmallTier.GridDepth = 7;   // old ScanDepth(6) + 1
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 2000;
		SmallTier.MinScale = 1e16;
		SmallTier.MaxScale = 1e18;

		ScaleDistributionCurve.GetRichCurve()->AddKey(0.0f, 0.0f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.05f, 0.0025f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.15f, 0.005f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.3f, 0.015f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.5f, 0.02f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.9f, 0.04f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.975f, 0.1f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.99f, 0.20f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.99999f, 0.40f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(1.0f, 1.0f);
	}
};

/// UNIVERSE GENERATOR - GENERATES DATA FOR POPULATING A UNIVERSE
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
};
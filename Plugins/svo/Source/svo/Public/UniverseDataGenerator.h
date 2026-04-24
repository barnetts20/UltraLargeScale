// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PointCloudGenerator.h"
#include "ProceduralSpaceActor.h"
#include "FVolumeTextureUtils.h"
#include "UniverseDataGenerator.generated.h"

/// Noise‑graph tuning knobs consumed by ASectorActor::BuildNoise().
/// Defaults reproduce the original hardcoded values.
USTRUCT(BlueprintType)
struct SVO_API FNoiseParams
{
	GENERATED_BODY()

	// Overall domain scale applied to the Voronoi source.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float MasterScale = 1.0f;

	// Cluster falloff exponent (PowInt value on the remapped Voronoi FBM).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterFalloff = 32.0f;

	// Domain scale applied after the cluster power curve.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterScale = 3.0f;

	// Multiplier on the cluster × web product.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterMulti = 50.0f;

	// Remap output max for the cluster branch (min→max swap in FastNoise Remap).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterRemapMax = 1.001f;

	// Remap output min for the cluster branch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ClusterRemapMin = 0.0f;

	// Web (filament) falloff exponent.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebFalloff = 3.0f;

	// Web remap output min.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebRemapMin = -0.1f;

	// Web remap output max.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WebRemapMax = 1.0f;

	// Domain warp amplitude.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmp = 0.25f;

	// Domain warp frequency.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpFreq = 1.0f;
};

/// <summary>
/// UNIVERSE GENERATION PARAM STRUCT
/// </summary>
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	int Count = 500;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MinClusterScale = 3e23;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MaxClusterScale = 3e25;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MinGalaxyScale = 1e16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MaxGalaxyScale = 1e18;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FRuntimeFloatCurve ScaleDistributionCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double Jitter = .02;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FNoiseParams NoiseParams;

	static constexpr const char* EncodedTree = "EAAAAIA/GQAbABsAEwAAAEBAJAAgAAAAFwAAAAAAAACAP8UggD8AAAAADQADAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEXAAAAAAAAAIA/zcxMvQAAgD8kAAIAAAD//wEAAAAASEIB//8GAAAAAIA+";

	FUniverseParams() {
		Seed = 69;
		Extent = 2147483648;
		UnitScale = 2e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

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

/// <summary>
/// UNIVERSE GENERATOR - GENERATES DATA FOR POPULATING A UNIVERSE
/// </summary>
class SVO_API UniverseDataGenerator : public PointCloudGenerator {
public:
	UniverseDataGenerator() : PointCloudGenerator(8647) {};
	UniverseDataGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	FUniverseParams Params;

	// Legacy override required by PointCloudGenerator base class. This path
	// previously sampled density from the octree; it now emits a warning and
	// falls back to uniform-density generation. Prefer the FDensityVolume
	// overload below for all new callers.
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	// Preferred entry point: uses a pre-built CPU-side density volume for
	// rejection sampling instead of going through the octree. The octree is
	// no longer required at generation time — point insertion into the octree
	// (if desired) happens downstream via BulkInsertPositions.
	void GenerateData(const FDensityVolume& InDensityVolume);

	TArray<FPointData> GeneratedData;
};
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PointCloudGenerator.h"
#include "ProceduralSpaceActor.h"
#include "FVolumeTextureUtils.h"
#include "UniverseDataGenerator.generated.h"

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
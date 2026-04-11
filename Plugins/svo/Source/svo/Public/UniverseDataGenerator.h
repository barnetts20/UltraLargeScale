// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PointCloudGenerator.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.generated.h"

/// <summary>
/// UNIVERSE GENERATION PARAM STRUCT
/// </summary>
USTRUCT(BlueprintType)
struct SVO_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	int Count = 20000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MinGalaxyScale = 1e24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MaxGalaxyScale = 1e30;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FRuntimeFloatCurve ScaleDistributionCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double Jitter = .02;

	static constexpr const char* EncodedTree = "EwAAAEBAFwAAAAAAmpmZPgAAAAAAAIA/HwAgABAAzcyMQBcAAACAvwAAgD8AAAAAzcxMPhsAFwAAAAAAAACAPwAAgL8AAIA/JAALAAAACwABAAAAAAAAAAEAAAAEAAAAAAAAgD8AAAAAQAAK16M9AAAAAAAAAAAAPwCamZk+AAAAAD8=";

	FUniverseParams() {
		Seed = 69;
		Extent = 2147483648;
		UnitScale = 1e18;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1,1,1);

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

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	TArray<FPointData> GeneratedData;
};

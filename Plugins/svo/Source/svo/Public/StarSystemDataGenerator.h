// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "PointCloudGenerator.h"
#include "StarSystemDataGenerator.generated.h"

USTRUCT(BlueprintType)
struct SVO_API FStarSystemParams : public FBaseParams {
	GENERATED_BODY()

	FLinearColor StarColor = FLinearColor(1, 1, 1, 1);

	//Volume Material Params
	FLinearColor VolumeAmbientColor = FLinearColor(1, 1, 1, 1);
	FLinearColor VolumeCoolShift = FLinearColor(.2, .5, .8);
	FLinearColor VolumeHotShift = FLinearColor(.5, 1.5, 3);
	double VolumeHueVariance = .1;
	double VolumeHueVarianceScale = .5;
	double VolumeSaturationVariance = .1;
	double VolumeTemperatureInfluence = 32;
	double VolumeTemperatureScale = 1;
	double VolumeDensity = .5;
	double VolumeWarpAmount = .05;
	double VolumeWarpScale = .33;
	FString VolumeNoise = "/svo/VolumeTextures/VT_PerlinWorley_Balanced";
};

class SVO_API StarSystemDataGenerator : public PointCloudGenerator {
public:
	StarSystemDataGenerator() : PointCloudGenerator(8647) {};
	StarSystemDataGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	FStarSystemParams SystemParams;
	enum EObjectType { Star = 0, Gas = 1, TerrestrialPlanet = 2, GasPlanet = 3, Moon = 4, Debris = 5, None = 6 };

	struct FOrbit
	{
		FVector Center;        // Orbit center (usually star origin)
		FVector Normal;        // Orbit plane normal
		double SemiMajorAxis;    // Orbit size
		double Eccentricity;     // 0-1, ellipse stretch
		double Phase;            // Starting angle offset 0 - 2pi
		EObjectType Type;
	};

	double Extent;
	double UnitScale;

	static constexpr double DepthProb[10] = {
		0.12,  // moonlets / large asteroids
		0.15,  // large moons
		0.18,  // Mars–Mercury class
		0.20,  // Earth-sized (peak)
		0.14,  // Super-Earths
		0.09,  // Sub-Neptunes
		0.06,  // Neptune-class
		0.03,  // Saturn-class
		0.02,  // Jupiter-class
		0.01   // Super-Jupiters
	};

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;
	void GenerateOrbits();
	void GeneratePlanet(const FOrbit& InPlanetOrbit, int32 InOrbitIndex);
	void GenerateDebris(const FOrbit& InDebrisOrbit, int32 InOrbitIndex);
	void GenerateUnboundDebris(); // should be used to generate low density background debris that does not follow the orbital plane or at least has a lesser relation to it
	void GenerateGas();
	FVector GetOrbitPosition(const FOrbit& Orbit) const;
	// Populate volumetric layer

	TArray<FOrbit> GeneratedOrbits;
	TArray<FPointData> GeneratedData;
};
// StarSystemDataGenerator.h
// Data generator for star system content.
// FStarSystemParams lives in StarSystemParams.h,
// mirroring FUniverseParams (UniverseParams.h) and FGalaxyParams (GalaxyParams.h).
// This class is retained for future use (full orbit/noise generation).
// First-pass actor uses analytic line layout and never calls GenerateData().

#pragma once

#include "CoreMinimal.h"
#include <FOctree.h>

class SVO_API StarSystemDataGenerator
{
public:
	StarSystemDataGenerator() : Seed(8647) {}
	StarSystemDataGenerator(int InSeed) : Seed(InSeed) {}

	// Seeded so callers can reproduce the same system.
	int Seed = 0;

	// These are set by the owning actor before any generation call.
	double Extent = 0.0;
	double UnitScale = 1.0;

	// Parent star particle's color, set by the owning actor before generation
	// from FStarSystemParams::ParentColor (FBaseParams). GenerateData tints the
	// central star with this. (GenerateData is unused in the first pass.)
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 1);

	// Ported from the former PointCloudGenerator base; used only by GenerateData.
	FRotator Rotation = FRotator(0.0, 0.0, 0.0);
	int MinInsertionDepth = 1;
	int MaxInsertionDepth = 1;

	// --- Object type enum (used as TypeId in octree nodes) ---
	enum EObjectType
	{
		Star = 0,
		Gas = 1,
		TerrestrialPlanet = 2,
		GasPlanet = 3,
		Moon = 4,
		Debris = 5,
		None = 6
	};

	struct FOrbit
	{
		FVector Center;        // Orbit center (usually star origin)
		FVector Normal;        // Orbit plane normal
		double  SemiMajorAxis; // Orbit size
		double  Eccentricity;  // 0–1, ellipse stretch
		double  Phase;         // Starting angle offset 0–2π
		EObjectType Type;
	};

	// --- Probability distribution for planet depth selection ---
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

	// --- Full generation (future use) ---
	// Not called during the first-pass analytic implementation.
	void GenerateData(TSharedPtr<FOctree> InOctree);
	void GenerateOrbits();
	void GeneratePlanet(const FOrbit& InPlanetOrbit, int32 InOrbitIndex);
	void GenerateDebris(const FOrbit& InDebrisOrbit, int32 InOrbitIndex);
	void GenerateUnboundDebris();
	void GenerateGas();
	FVector GetOrbitPosition(const FOrbit& Orbit) const;

	TArray<FOrbit>     GeneratedOrbits;
	TArray<FPointData> GeneratedData;
};
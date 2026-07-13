/** Data generator for star-system content. FStarSystemParams lives in
 *  StarSystemParams.h, mirroring FUniverseParams and FGalaxyParams. Retained for
 *  future use (full orbit/noise generation); the first-pass actor uses analytic
 *  line layout and never calls GenerateData(). */

#pragma once

#include "CoreMinimal.h"
#include <FOctree.h>

class SVO_API StarSystemDataGenerator
{
public:
	StarSystemDataGenerator() : Seed(8647) {}
	StarSystemDataGenerator(int InSeed) : Seed(InSeed) {}

	/** Seeded so callers can reproduce the same system. */
	int Seed = 0;

	/** Set by the owning actor before any generation call. */
	double Extent = 0.0;
	double UnitScale = 1.0;

	/** Parent star particle's color, set by the owning actor before generation
	 *  from FStarSystemParams::ParentColor (FBaseParams). GenerateData tints the
	 *  central star with this (GenerateData is unused in the first pass). */
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 1);

	/** Used only by GenerateData. */
	FRotator Rotation = FRotator(0.0, 0.0, 0.0);
	int MinInsertionDepth = 1;
	int MaxInsertionDepth = 1;

#pragma region Object Types
	/** Object class stored as TypeId in octree nodes. */
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

	/** A single orbit definition. */
	struct FOrbit
	{
		/** Orbit center (usually star origin). */
		FVector Center;
		/** Orbit plane normal. */
		FVector Normal;
		/** Orbit size (semi-major axis). */
		double SemiMajorAxis;
		/** Ellipse stretch, 0-1. */
		double Eccentricity;
		/** Starting angle offset, 0-2pi. */
		double Phase;
		EObjectType Type;
	};

#pragma endregion

#pragma region Planet Depth Distribution
	/** Probability distribution for planet depth selection. */
	static constexpr double DepthProb[10] = {
		0.12,  // moonlets / large asteroids
		0.15,  // large moons
		0.18,  // Mars-Mercury class
		0.20,  // Earth-sized (peak)
		0.14,  // Super-Earths
		0.09,  // Sub-Neptunes
		0.06,  // Neptune-class
		0.03,  // Saturn-class
		0.02,  // Jupiter-class
		0.01   // Super-Jupiters
	};

#pragma endregion

#pragma region Full Generation
	/** Full orbit/noise generation, not called during the first-pass analytic
	 *  implementation. */
	void GenerateData(TSharedPtr<FOctree> InOctree);
	void GenerateOrbits();
	void GeneratePlanet(const FOrbit& InPlanetOrbit, int32 InOrbitIndex);
	void GenerateDebris(const FOrbit& InDebrisOrbit, int32 InOrbitIndex);
	void GenerateUnboundDebris();
	void GenerateGas();
	FVector GetOrbitPosition(const FOrbit& Orbit) const;

	TArray<FOrbit>     GeneratedOrbits;
	TArray<FPointData> GeneratedData;

#pragma endregion
};
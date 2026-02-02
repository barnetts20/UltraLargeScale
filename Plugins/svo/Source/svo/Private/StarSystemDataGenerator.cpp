// Fill out your copyright notice in the Description page of Project Settings.


#include "StarSystemDataGenerator.h"

#pragma region Star System Generator
void StarSystemDataGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	// TODO: Star at center with massive scale (We should base this on actual math, just jamming in depth 9 for now
	FPointData StarData(FVector::ZeroVector, 9,
		FVoxelData(1, 0, FVector(SystemParams.StarColor.R, SystemParams.StarColor.G, SystemParams.StarColor.B) * 10, 1, EObjectType::Star));
	GeneratedData.Add(StarData);
	Extent = InOctree->Extent;

	MinInsertionDepth = 1;
	MaxInsertionDepth = InOctree->MaxDepth;

	GenerateOrbits();

	// Process each orbit with unique seed
	for (int32 i = 0; i < GeneratedOrbits.Num(); i++)
	{
		FRandomStream Stream(Seed + i * 1000); // Unique seed per orbit
		const FOrbit& Orbit = GeneratedOrbits[i];

		switch (Orbit.Type)
		{
		case EObjectType::TerrestrialPlanet:
		case EObjectType::GasPlanet:
			GeneratePlanet(Orbit, i);
			break;
		case EObjectType::Debris:
			GenerateDebris(Orbit, i);
			break;
		}
	}

	GenerateUnboundDebris();
	GenerateGas();
}

void StarSystemDataGenerator::GenerateOrbits()
{
	FRandomStream Stream(Seed);

	int32 NumOrbits = Stream.RandRange(6, 14);
	GeneratedOrbits.SetNumZeroed(NumOrbits);

	FVector SystemUp = Rotation.RotateVector(FVector::UpVector);

	// Start orbits at reasonable distance from star (not too close)
	double StarRadius = Extent * 0.005; // Estimate star size
	double CurrentRadius = StarRadius * 10.0; // First orbit well clear of star
	double GrowthFactor = Stream.FRandRange(1.4, 4); // Increased spacing
	double MaxInclinationDegrees = 45.0; // Reduced for more planar system

	int32 FrostLineIndex = FMath::Clamp(int32(NumOrbits * 0.4), 2, NumOrbits - 2);

	for (int32 i = 0; i < NumOrbits; i++)
	{
		FOrbit& Orbit = GeneratedOrbits[i];
		Orbit.Center = FVector::ZeroVector;

		// Orbital inclination with bias toward planar
		double InclinationDeg = Stream.FRandRange(-MaxInclinationDegrees, MaxInclinationDegrees);
		double InclinationRad = FMath::DegreesToRadians(InclinationDeg);

		FVector TiltAxis = FVector::CrossProduct(SystemUp,
			FVector(Stream.FRandRange(-1, 1), Stream.FRandRange(-1, 1), 0)).GetSafeNormal();
		if (TiltAxis.IsNearlyZero())
			TiltAxis = Rotation.RotateVector(FVector::RightVector);

		FQuat TiltRotation(TiltAxis, InclinationRad);
		Orbit.Normal = TiltRotation.RotateVector(SystemUp).GetSafeNormal();

		// Set semi-major axis with proper spacing
		Orbit.SemiMajorAxis = CurrentRadius;

		// Grow radius for next orbit
		double GrowthMultiplier = GrowthFactor * Stream.FRandRange(0.9, 1.1);
		CurrentRadius = FMath::Min(CurrentRadius * GrowthMultiplier, Extent * 0.65);

		// Low eccentricity for stability
		Orbit.Eccentricity = FMath::Pow(Stream.FRand(), 3.0) * 0.3;
		Orbit.Phase = Stream.FRandRange(0.0, 2.0 * PI);

		// Object type assignment
		double p = Stream.FRand();

		if (i < FrostLineIndex)
		{
			// Inner system: rocky planets and asteroid belts
			if (p < 0.5)
				Orbit.Type = EObjectType::TerrestrialPlanet;
			else if (p < 0.85)
				Orbit.Type = EObjectType::Debris;
			else
				Orbit.Type = EObjectType::None; // Empty orbit
		}
		else
		{
			// Outer system: gas giants, ice worlds, Kuiper belt
			if (p < 0.25)
				Orbit.Type = EObjectType::GasPlanet;
			else if (p < 0.45)
				Orbit.Type = EObjectType::TerrestrialPlanet; // Ice worlds
			else if (p < 0.80)
				Orbit.Type = EObjectType::Debris;
			else
				Orbit.Type = EObjectType::None;
		}
	}
}

void StarSystemDataGenerator::GeneratePlanet(const FOrbit& InPlanetOrbit, int32 OrbitIndex)
{
	FRandomStream Stream(Seed + OrbitIndex * 1000 + 100);

	double normDist = InPlanetOrbit.SemiMajorAxis / (Extent * 0.65);
	double scale = 0;
	FVector composition;

	if (InPlanetOrbit.Type == EObjectType::TerrestrialPlanet)
	{
		// Distance-based sizing: inner planets can be larger (Venus/Earth-like)
		if (normDist < 0.3)
			scale = Stream.FRandRange(4000, 12000); // Mercury to Earth
		else if (normDist < 0.6)
			scale = Stream.FRandRange(3000, 7000); // Mars-like
		else
			scale = Stream.FRandRange(1500, 5000); // Icy dwarfs

		// Color variation for terrestrial
		composition = FVector(
			Stream.FRandRange(0.0, 0.3),
			Stream.FRandRange(0.4, 0.7),
			Stream.FRandRange(0.6, 1.0)
		);
	}
	else if (InPlanetOrbit.Type == EObjectType::GasPlanet)
	{
		// Gas giants larger in middle/outer system
		if (normDist < 0.4)
			scale = Stream.FRandRange(40000, 100000); // Jupiter-like
		else
			scale = Stream.FRandRange(20000, 60000); // Neptune-like

		// Color variation for gas giants
		float hue = Stream.FRand();
		if (hue < 0.3)
			composition = FVector(1, 0.8, 0.6); // Jupiter browns/oranges
		else if (hue < 0.6)
			composition = FVector(0.9, 0.9, 0.7); // Saturn yellows
		else
			composition = FVector(0.6, 0.8, 1.0); // Neptune blues
	}
	else
	{
		return;
	}

	FPointData PlanetData = FPointData::MakePointDataFromWorldScale(scale, UnitScale, Extent);// MakePointDataFromScale(scale);
	PlanetData.Data.TypeId = InPlanetOrbit.Type;
	PlanetData.Data.Composition = composition;
	PlanetData.SetPosition(GetOrbitPosition(InPlanetOrbit));

	GeneratedData.Add(PlanetData);

	// Moon generation based on planet type and size
	int NumMoons = 0;
	if (InPlanetOrbit.Type == EObjectType::GasPlanet)
	{
		NumMoons = Stream.RandRange(2, 12); // Gas giants have many moons
	}
	else if (scale > 6000)
	{
		NumMoons = Stream.RandRange(0, 3); // Large terrestrials
	}
	else if (Stream.FRand() < 0.3)
	{
		NumMoons = Stream.RandRange(0, 1); // Small chance for small planets
	}

	// Generate moons
	for (int i = 0; i < NumMoons; i++)
	{
		FOrbit SubOrbit;
		SubOrbit.Center = PlanetData.GetPosition();

		// Moon orbits at multiples of planet radius (Hill sphere consideration)
		double minDistance = scale * 2.5;
		double maxDistance = scale * Stream.FRandRange(15, 50);
		SubOrbit.SemiMajorAxis = Stream.FRandRange(minDistance, maxDistance);

		SubOrbit.Eccentricity = Stream.FRandRange(0, 0.15);
		SubOrbit.Normal = Stream.GetUnitVector();
		SubOrbit.Phase = Stream.FRandRange(0, 2 * PI);

		// 10% chance for debris ring instead of moon
		if (Stream.FRand() < 0.1)
		{
			SubOrbit.Type = EObjectType::Debris;
			GenerateDebris(SubOrbit, OrbitIndex * 100 + i);
		}
		else
		{
			// Moon size scales with planet
			double moonScale = Stream.FRandRange(scale * 0.05, scale * 0.35);
			FPointData MoonData = FPointData::MakePointDataFromWorldScale(moonScale, UnitScale, Extent);
			MoonData.Data.TypeId = EObjectType::Moon;

			// Gray/brown moons with variation
			MoonData.Data.Composition = FVector(
				Stream.FRandRange(0.4, 0.7),
				Stream.FRandRange(0.4, 0.7),
				Stream.FRandRange(0.4, 0.7)
			);

			MoonData.SetPosition(GetOrbitPosition(SubOrbit));
			GeneratedData.Add(MoonData);
		}
	}
}

void StarSystemDataGenerator::GenerateDebris(const FOrbit& InDebrisOrbit, int32 OrbitIndex)
{
	FRandomStream Stream(Seed + OrbitIndex * 1000 + 500);

	// Determine debris belt characteristics
	bool isFullRing = Stream.FRand() < 0.7; // 70% full rings, 30% arcs
	double arcStartPhase = Stream.FRandRange(0, 2 * PI);
	double arcLength = isFullRing ? 2 * PI : Stream.FRandRange(PI * 0.3, PI * 1.5);

	// Number of debris objects based on orbit size
	double orbitCircumference = 2 * PI * InDebrisOrbit.SemiMajorAxis;
	int32 BaseCount = FMath::RoundToInt(orbitCircumference / (Extent * 0.01));
	int32 DebrisCount = Stream.RandRange(
		FMath::Max(50, BaseCount / 2),
		FMath::Max(200, BaseCount * 2)
	);

	// Belt thickness as fraction of orbital radius
	double radialThickness = InDebrisOrbit.SemiMajorAxis * Stream.FRandRange(0.05, 0.15);
	double verticalThickness = radialThickness * Stream.FRandRange(0.1, 0.3);

	for (int32 i = 0; i < DebrisCount; i++)
	{
		// Distribute along arc
		double phaseOffset;
		if (isFullRing)
		{
			phaseOffset = Stream.FRandRange(0, 2 * PI);
		}
		else
		{
			phaseOffset = arcStartPhase + Stream.FRandRange(0, arcLength);
		}

		// Create perturbed orbit for this debris piece
		FOrbit DebrisOrbit = InDebrisOrbit;
		DebrisOrbit.Phase = phaseOffset;

		// Add radial variation
		DebrisOrbit.SemiMajorAxis += Stream.FRandRange(-radialThickness, radialThickness);
		DebrisOrbit.Eccentricity += Stream.FRandRange(-0.05, 0.05);
		DebrisOrbit.Eccentricity = FMath::Clamp(DebrisOrbit.Eccentricity, 0.0, 0.5);

		// Get base position on orbit
		FVector BasePos = GetOrbitPosition(DebrisOrbit);

		// Add vertical scatter perpendicular to orbital plane
		FVector VerticalOffset = DebrisOrbit.Normal * Stream.FRandRange(-verticalThickness, verticalThickness);
		FVector FinalPos = BasePos + VerticalOffset;

		// Debris size variation
		double debrisScale = Stream.FRandRange(10, 500); // 10km to 500km asteroids
		if (Stream.FRand() < 0.05) // 5% chance of larger object
			debrisScale = Stream.FRandRange(500, 2000);

		FPointData DebrisData = FPointData::MakePointDataFromWorldScale(debrisScale, UnitScale, Extent);
		DebrisData.Data.TypeId = EObjectType::Debris;

		// Rocky/icy composition
		DebrisData.Data.Composition = FVector(
			Stream.FRandRange(0.3, 0.6),
			Stream.FRandRange(0.3, 0.5),
			Stream.FRandRange(0.3, 0.5)
		);

		DebrisData.SetPosition(FinalPos);
		GeneratedData.Add(DebrisData);
	}
}

void StarSystemDataGenerator::GenerateUnboundDebris()
{
	FRandomStream Stream(Seed + 9999);

	// Sparse debris field throughout system
	int32 UnboundCount = Stream.RandRange(100, 500);

	for (int32 i = 0; i < UnboundCount; i++)
	{
		// Random position within system bounds (favor outer regions)
		double r = FMath::Pow(Stream.FRand(), 0.5) * Extent * 0.7; // Square root for volume distribution
		double theta = Stream.FRandRange(0, 2 * PI);
		double phi = Stream.FRandRange(0, PI);

		FVector Pos(
			r * FMath::Sin(phi) * FMath::Cos(theta),
			r * FMath::Sin(phi) * FMath::Sin(theta),
			r * FMath::Cos(phi)
		);

		// Small debris objects
		double debrisScale = Stream.FRandRange(5, 200);

		FPointData DebrisData = FPointData::MakePointDataFromWorldScale(debrisScale, UnitScale, Extent);
		DebrisData.Data.TypeId = EObjectType::Debris;
		DebrisData.Data.Composition = FVector(0.4, 0.4, 0.4);
		DebrisData.SetPosition(Pos);

		GeneratedData.Add(DebrisData);
	}
}

void StarSystemDataGenerator::GenerateGas()
{
	// Generate nebulous gas layer for stellar system
	// This could be represented as low-density points or a separate gas field
	FRandomStream Stream(Seed + 7777);

	int32 GasParticleCount = Stream.RandRange(50, 200);

	for (int32 i = 0; i < GasParticleCount; i++)
	{
		// Distribute throughout system with concentration toward plane
		double r = FMath::Pow(Stream.FRand(), 0.3) * Extent * 0.8;
		double theta = Stream.FRandRange(0, 2 * PI);

		// Flatten to disk using exponential distribution
		double z = Stream.FRandRange(-1, 1);
		z = FMath::Sign(z) * FMath::Pow(FMath::Abs(z), 2.0) * Extent * 0.1;

		FVector Pos(
			r * FMath::Cos(theta),
			r * FMath::Sin(theta),
			z
		);

		// Very large, very low density gas clouds
		FPointData GasData = FPointData::MakePointDataFromWorldScale(Stream.FRandRange(5000, 20000), UnitScale, Extent);
		GasData.Data.GasDensity = Stream.FRandRange(0.001, 0.01); // Very diffuse
		GasData.Data.TypeId = EObjectType::Gas;
		GasData.Data.Composition = FVector(0.1, 0.1, 0.15); // Faint blue
		GasData.SetPosition(Pos);

		GeneratedData.Add(GasData);
	}
}

FVector StarSystemDataGenerator::GetOrbitPosition(const FOrbit& Orbit) const
{
	double a = Orbit.SemiMajorAxis;
	double e = Orbit.Eccentricity;
	double b = a * FMath::Sqrt(1.0 - e * e);

	// Build orbital basis vectors
	FVector UpRef = FVector::UpVector;
	FVector OrbitRight = FVector::CrossProduct(Orbit.Normal, UpRef);
	if (OrbitRight.IsNearlyZero())
		OrbitRight = FVector::CrossProduct(Orbit.Normal, FVector::ForwardVector);
	OrbitRight.Normalize();

	FVector OrbitForward = FVector::CrossProduct(Orbit.Normal, OrbitRight).GetSafeNormal();

	// Calculate position on ellipse
	double theta = Orbit.Phase;
	FVector Pos = Orbit.Center
		+ OrbitRight * (a * FMath::Cos(theta))
		+ OrbitForward * (b * FMath::Sin(theta));

	return Pos;
}
#pragma endregion
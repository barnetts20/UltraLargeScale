/** The star system's orbit types and the ellipse evaluation over them.
 *
 *  NOT A GENERATOR IN THE SENSE THE OTHER TWO LAYERS USE THE WORD. The universe and galaxy
 *  layers place entities against a density field evaluated on the GPU; this layer lays its
 *  bodies out analytically -- AStarSystemActor computes the orbits itself, fills
 *  GeneratedOrbits, and reads positions back through GetOrbitPosition. There is no field,
 *  no dispatch, and nothing here to calibrate. */

#pragma once

#include "CoreMinimal.h"
#include <FOctree.h>

class ULTRALARGESCALE_API StarSystemDataGenerator
{
public:
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

#pragma region Orbit Layout

	/** A point on the ellipse InOrbit describes, at its own Phase.
	 *
	 *  Reads nothing but its argument, so the caller may pass a temporary copy with a swept
	 *  Phase to walk the whole orbit track. */
	FVector GetOrbitPosition(const FOrbit& Orbit) const;

	/** The system's orbits. POPULATED BY THE ACTOR, not here. */
	TArray<FOrbit>     GeneratedOrbits;

#pragma endregion
};
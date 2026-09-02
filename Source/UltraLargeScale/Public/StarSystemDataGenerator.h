/** The star system's orbit types and the ellipse evaluation over them.
 *
 *  NOT A GENERATOR IN THE SENSE THE OTHER TWO LAYERS USE THE WORD. The universe and galaxy
 *  layers place entities against a density field evaluated on the GPU; this layer lays its
 *  bodies out ANALYTICALLY -- AStarSystemActor computes the orbits itself, fills
 *  GeneratedOrbits, and reads positions back through GetOrbitPosition. There is no field,
 *  no dispatch, and nothing here to calibrate.
 *
 *  THE NAME IS THE MIRROR'S, not a description. FStarSystemParams sits beside
 *  FUniverseParams and FGalaxyParams for the same reason. If this layer ever does acquire a
 *  density field, the shape to copy is GalaxyDataGenerator -- and the shared pieces are
 *  already in place: FTierBatchCell, FTierStreamingSystem::SubdivideCells and the
 *  Pack()/FillShaderParameters marshal pattern.
 *
 *  WHETHER IT SHOULD is an open question rather than a pending task. A handful of bodies on
 *  orbits is not a density distribution, and a field this layer does not need would cost a
 *  shader and buy nothing. */

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
	 *  THE ONLY THING THIS CLASS COMPUTES. AStarSystemActor lays the orbits out itself --
	 *  it fills GeneratedOrbits directly and calls this per planet -- so what lives here is
	 *  the orbit TYPE and the ellipse evaluation, not a generation pass.
	 *
	 *  It reads nothing but its argument, which is what lets the actor call it against a
	 *  temporary copy with a swept Phase to draw an orbit track. */
	FVector GetOrbitPosition(const FOrbit& Orbit) const;

	/** The system's orbits. POPULATED BY THE ACTOR, not here. */
	TArray<FOrbit>     GeneratedOrbits;

#pragma endregion
};
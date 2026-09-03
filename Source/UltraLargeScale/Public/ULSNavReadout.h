// ULSNavReadout.h
#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "UniverseParams.h"   // FUniverseFieldOffset, UniverseCellWrap

class AUniverseActor;

/** The player's position in universe-local units and the real-cm conversion of it. Sourced
 *  from AUniverseActor::VirtualTraversal, never the actor transform, and continuous across an
 *  octree rebase. */
struct FULSUniverseCoord
{
	/** AUniverseActor::VirtualTraversal, universe-local units. */
	FVector Local = FVector::ZeroVector;

	/** Real cm per universe-local unit (UniverseParams.UnitScale) at sample time. */
	double UnitScaleCm = 1.0;

	/** Local * UnitScaleCm. The only quantity the ladder should ever be handed. */
	FVector RealCm = FVector::ZeroVector;
};

/** One nested layer the player is inside, or nearest to: a galaxy, a star system. Not a
 *  sub-coordinate of the universe one but the same position re-expressed where it has
 *  resolution, so the two are equivalent by construction. */
struct FULSLayerCoord
{
	/** "Galaxy" / "System". A display label, not a class name. */
	const TCHAR* LayerName = TEXT("");

	/** The layer's procedural seed: a position says where, the seed says which one. */
	int32 Seed = 0;

	/** That actor's VirtualTraversal, in its own local units. */
	FVector Local = FVector::ZeroVector;

	/** Real cm per local unit for this layer (Params.UnitScale). */
	double UnitScaleCm = 1.0;

	/** Local * UnitScaleCm. */
	FVector RealCm = FVector::ZeroVector;

	/** max|Local axis| / Octree->Extent. Below 1 the player is inside the layer's bounds,
	 *  matching IsPlayerInsideBounds: a per-axis test, so a max and NOT a length. */
	double BoundsFraction = 0.0;

	/** True when BoundsFraction <= 1; when false this is the NEAREST candidate, shown dimmed. */
	bool bInside = false;
};

/** One frame of navigation state, read off the universe layer and its live children. Purely a
 *  read -- nothing re-derives a field quantity, and speeds are absent as a derivative. */
struct FULSNavSample
{
	/** False when there is no universe or it is not Ready; everything below is then undefined.
	 *  Zeroes are not absence: (0,0,0) is a legitimate position. */
	bool bValid = false;

	ELifecycleState State = ELifecycleState::Uninitialized;

	FULSUniverseCoord Coord;

	/** Galaxy, then star system, outermost first. Only layers whose actor is Ready appear. */
	TArray<FULSLayerCoord, TInlineAllocator<2>> Layers;

	/** The value the OffsetCell / OffsetFrac pins were pushed this frame, from the same
	 *  ComputeFieldOffset call PushFieldOffset makes; disagreement with Coord means drift. */
	FUniverseFieldOffset FieldOffset;

	/** GetEffectiveSpeedScale(): real cm travelled per world cm moved. */
	double SpeedScale = 1.0;

	/** Universe-layer VirtualTraversal at sample time (== Coord.Local, duplicated so the
	 *  subsystem can difference it without reaching back through the actor). */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** The world-space player position the parallax accumulation used this frame; differencing
	 *  this rather than pawn velocity makes real and compressed speed comparable. */
	FVector FrameOfReferenceWorld = FVector::ZeroVector;

	/** The finest streaming tier's grid coord, which keys every cell and every entity seed and
	 *  does not wrap. Unset (INT32_MIN) until that tier has streamed at least once. */
	FIntVector GridCoord = FIntVector(INT32_MIN);

	/** The field's repeat period in small cells. Needed to read FieldOffset: the index arrives
	 *  in [0, period), so one cell left of origin is period-1, not -1. Zero if unavailable. */
	int32 FieldCellPeriod = 0;

	int32 SpawnedGalaxyCount = 0;

	/** Live star systems under the galaxy chosen for Layers, not across all galaxies. */
	int32 SpawnedSystemCount = 0;
};

/** Samples the universe layer and walks down into the containing galaxy and star system.
 *  GAME THREAD ONLY -- dereferences octrees that background transition tasks swap. */
FULSNavSample ULSSampleUniverseNav(const AUniverseActor& InUniverse);
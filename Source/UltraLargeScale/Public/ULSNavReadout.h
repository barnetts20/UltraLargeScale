// ULSNavReadout.h
#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "UniverseParams.h"   // FUniverseFieldOffset

class AUniverseActor;

/** THE UNIVERSE COORDINATE.
 *
 *  A position in this project is (SECTOR, LOCAL): an integer sector on the universe cell
 *  lattice plus the player's VirtualTraversal inside it, in universe-local units. That is
 *  the same cell-plus-fraction shape the field offset and the octree grid already use,
 *  and it is chosen for the same reason -- one exact integer carries the magnitude so the
 *  double only ever has to carry a bounded remainder.
 *
 *  WHY VirtualTraversal AND NOT THE ACTOR TRANSFORM. AUniverseActor::SetActorLocation is
 *  reassigned to the player's world position every single tick, so the actor transform is
 *  a rendering convenience with no relationship to the field. VirtualTraversal is the
 *  quantity the field is actually evaluated at, and it is what ComputeFieldOffset,
 *  the tier grids, the octree queries and the spawn scan all read.
 *
 *  WHY IT SURVIVES A REBASE. RebaseOctree rebuilds the tree centred on the CURRENT
 *  VirtualTraversal; it does not reset VirtualTraversal. Octree node centres are absolute
 *  in sector-local space and move with the tree. So the coordinate below is continuous
 *  across a rebase, which is exactly the property that makes it worth writing down: a
 *  rebase is invisible to it. If that ever changes -- if a rebase starts subtracting an
 *  origin from VirtualTraversal -- this struct becomes a coordinate that silently
 *  renumbers itself mid-flight, and the symptom is that two visits to the same structure
 *  report different positions with nothing else looking wrong.
 *
 *  SECTOR IS READ, NOT DERIVED, AND IS THEREFORE ALWAYS (0,0,0) TODAY. ConfigureCell is
 *  the only writer and nothing calls it with a non-zero coordinate: the universe is
 *  level-placed, there is one of it, and nothing hands the player between sectors.
 *  VirtualTraversal is consequently free to run arbitrarily far past one sector extent --
 *  the octree rebases under it rather than the sector index advancing -- so at long
 *  traversals the authoritative Sector says 0 while Local says several thousand sector
 *  spans.
 *
 *  SectorDerived below reports that overflow, and is explicitly NOT written back into
 *  Sector. Folding it would make this readout claim a sector the octree, the tier grids
 *  and the spawn scan all disagree it is in -- and since none of them would complain, the
 *  first symptom would be a written-down coordinate that does not lead back to the same
 *  place. The fold belongs in whatever implements sector handoff, next to the
 *  corresponding octree change. Until then this pair is the honest description: the
 *  lattice index, and how far past it the traversal has gone. */
struct FULSUniverseCoord
{
	/** AUniverseActor::CellCoord, verbatim. */
	FIntVector Sector = FIntVector::ZeroValue;

	/** What Sector WOULD be if the lattice were live: Local divided by one sector span,
	 *  truncated. Display only -- see the note above. Zero until the traversal exceeds a
	 *  sector, at which point the HUD annotates the Sector row rather than replacing it. */
	FIntVector SectorDerived = FIntVector::ZeroValue;

	/** AUniverseActor::VirtualTraversal, universe-local units. */
	FVector Local = FVector::ZeroVector;

	/** Real cm per universe-local unit (UniverseParams.UnitScale) at sample time. */
	double UnitScaleCm = 1.0;

	/** Full extent of one sector in local units (2 * UniverseParams.Extent). */
	double SectorSpanUnits = 0.0;

	/** (Sector * SectorSpanUnits + Local) * UnitScaleCm. The one quantity that is
	 *  comparable across sectors, and the only one the ladder should ever be handed. */
	FVector RealCm = FVector::ZeroVector;
};

/** One nested layer the player is inside (or nearest to): a galaxy, a star system.
 *
 *  THE LAYERS ARE NOT SUB-COORDINATES OF THE UNIVERSE ONE -- they are the same position
 *  re-expressed at a scale where it has resolution. The universe coordinate is a double
 *  at magnitudes where its ulp is fractions of an AU; asking it where you are inside a
 *  star system is asking a number that cannot answer. Each layer's own VirtualTraversal
 *  is accumulated at that layer's UnitScale and stays small, which is the entire reason
 *  the scale ladder of actors exists.
 *
 *  THEY ARE ALSO NOT INDEPENDENT. AUniverseActor::FinalizeGalaxyPlacement seeds a
 *  galaxy's VirtualTraversal from (camera - spawn location), and that expression reduces
 *  to (UniverseVT - NodeCentre) * (UniverseUnitScale / GalaxyUnitScale) -- the camera
 *  position cancels. So a galaxy-local position IS the universe position, refined; if the
 *  two ever describe different places, the placement arithmetic and the parallax
 *  accumulation have diverged, and the visible symptom is a galaxy that drifts relative
 *  to the sprite it was spawned from. Displaying both is what makes that comparable by
 *  eye. */
struct FULSLayerCoord
{
	/** "Galaxy" / "System". Literal, not a class name: the class name is an
	 *  implementation detail and this is a label. */
	const TCHAR* LayerName = TEXT("");

	/** The layer's procedural seed -- the identity that actually reproduces. A position
	 *  says where; the seed says which one, and it is the seed that survives a retune of
	 *  the field while the position does not. */
	int32 Seed = 0;

	/** That actor's VirtualTraversal, in its own local units. */
	FVector Local = FVector::ZeroVector;

	/** Real cm per local unit for this layer (Params.UnitScale). */
	double UnitScaleCm = 1.0;

	/** Local * UnitScaleCm. */
	FVector RealCm = FVector::ZeroVector;

	/** max|Local axis| / Octree->Extent. Below 1 the player is inside the layer's bounds,
	 *  matching AGalaxyActor / AStarSystemActor::IsPlayerInsideBounds exactly -- which is
	 *  a per-axis test, not a radial one, so this is a max and NOT a length. A length here
	 *  would disagree with the containment the spawn scan uses by up to sqrt(3), and the
	 *  readout would say "outside" while the streaming system kept spawning. */
	double BoundsFraction = 0.0;

	/** True when BoundsFraction <= 1. When false this is the NEAREST candidate rather
	 *  than a container, and is shown dimmed. */
	bool bInside = false;
};

/** One frame of navigation state, read off the universe layer and its live children.
 *
 *  PURELY A READ. Every member is either copied from a published value or produced by
 *  calling the universe's own accessor. Nothing here re-derives a field quantity, and
 *  nothing should start to: the compute path and the material already have to agree on
 *  the field, and a third evaluator that only the HUD uses would be able to disagree with
 *  both while looking entirely reasonable on screen. If a future readout needs a field
 *  value this struct cannot get by asking, the function that computes it belongs on
 *  AUniverseActor next to the code that pushes it to the material -- not here.
 *
 *  Speeds are absent on purpose. They are a time derivative and this is a snapshot; the
 *  differencing and smoothing live in the subsystem that owns per-frame history. */
struct FULSNavSample
{
	/** False when there is no universe, or it has not reached Ready. Everything below is
	 *  undefined in that case -- the HUD prints a waiting state rather than zeroes,
	 *  because a coordinate of exactly (0,0,0) is also a legitimate position. */
	bool bValid = false;

	ELifecycleState State = ELifecycleState::Uninitialized;

	FULSUniverseCoord Coord;

	/** Galaxy, then star system, outermost first. Empty when nothing is spawned. Only
	 *  layers whose actor is Ready appear: a mid-initialization pooled actor still holds
	 *  the previous occupant's VirtualTraversal, and printing that would put the player
	 *  confidently inside a galaxy that no longer exists. */
	TArray<FULSLayerCoord, TInlineAllocator<2>> Layers;

	/** The value the material's OffsetCell / OffsetFrac pins were pushed this frame, via
	 *  AUniverseActor::ComputeFieldOffset -- the same call PushFieldOffset makes, not a
	 *  reconstruction of it. This is the field's own idea of where the player is, so a
	 *  disagreement between this and Coord is the visible form of the offset-normalisation
	 *  drifting from the proxy extent. */
	FUniverseFieldOffset FieldOffset;

	/** GetEffectiveSpeedScale(): real cm travelled per world cm moved. The spatial
	 *  compression factor. */
	double SpeedScale = 1.0;

	/** Universe-layer VirtualTraversal at sample time (== Coord.Local; duplicated so the
	 *  subsystem can difference it without reaching back through the actor). */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** The world-space player position the parallax accumulation used this frame
	 *  (CurrentFrameOfReferenceLocation). Differencing THIS rather than the pawn's
	 *  velocity is what makes real speed and compressed speed comparable: they are then
	 *  two derivatives of the same input, so their ratio is meaningful. */
	FVector FrameOfReferenceWorld = FVector::ZeroVector;

	int32 SpawnedGalaxyCount = 0;

	/** Live star systems under the galaxy chosen for Layers, not across all galaxies. */
	int32 SpawnedSystemCount = 0;
};

/** Samples the universe layer and walks down into the containing galaxy / star system.
 *
 *  Cost is one ComputeFieldOffset (which does one K2_GetVectorParameterValue on the
 *  volume MID) plus a linear pass over SpawnedGalaxies and the chosen galaxy's
 *  SpawnedStarSystems. Both maps are small by construction -- they are what the streaming
 *  budget keeps resident -- so this is a per-frame cost of tens of comparisons, not a
 *  search. Game thread only: it dereferences TSharedPtr octrees that background transition
 *  tasks swap. */
FULSNavSample ULSSampleUniverseNav(const AUniverseActor& InUniverse);
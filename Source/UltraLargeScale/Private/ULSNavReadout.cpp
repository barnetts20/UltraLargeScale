// ULSNavReadout.cpp
#include "ULSNavReadout.h"

#include "GalaxyActor.h"
#include "StarSystemActor.h"
#include "UniverseActor.h"

namespace
{
	/** Fills the common part of a layer entry from any procedural actor. THE EXTENT COMES FROM
	 *  THE OCTREE, not Params.Extent, because IsPlayerInsideBounds tests against Octree->Extent
	 *  and this fraction has to agree with it. They are the same number today, and the day one
	 *  stops being -- a rebased tree, a multiplier like PersistentTreeMultiplier -- the readout
	 *  would report "outside" for a player the spawn scan still treats as inside. */
	template <typename TActor>
	FULSLayerCoord MakeLayerCoord(const TCHAR* InLayerName, const TActor& InActor)
	{
		FULSLayerCoord Out;
		Out.LayerName = InLayerName;
		Out.Seed = InActor.Params.Seed;
		Out.Local = InActor.VirtualTraversal;
		Out.UnitScaleCm = InActor.Params.UnitScale;
		Out.RealCm = InActor.VirtualTraversal * InActor.Params.UnitScale;

		const double Extent = InActor.Octree.IsValid()
			? InActor.Octree->Extent
			: InActor.Params.Extent;

		// PER-AXIS MAX, mirroring IsPlayerInsideBounds. See the note on BoundsFraction.
		const double Largest = FMath::Max3(
			FMath::Abs(Out.Local.X), FMath::Abs(Out.Local.Y), FMath::Abs(Out.Local.Z));

		Out.BoundsFraction = (Extent > 0.0) ? (Largest / Extent) : TNumericLimits<double>::Max();
		Out.bInside = Out.BoundsFraction <= 1.0;
		return Out;
	}

	/** Picks the layer instance the player is in, or the nearest if none contains them.
	 *
	 *  NEAREST-BY-BOUNDS-FRACTION, not by distance, which matters when candidates have different
	 *  extents: a small system 2 extents away and a large one 5 away are not usefully ranked by
	 *  raw distance. IT RETURNS A NON-CONTAINING CANDIDATE ON PURPOSE -- showing nothing when the
	 *  player is between galaxies makes an empty row look like a broken walk. */
	template <typename TActor, typename TMap>
	const TActor* SelectNearest(const TMap& InSpawned, double& OutBestFraction)
	{
		const TActor* Best = nullptr;
		OutBestFraction = TNumericLimits<double>::Max();

		for (const auto& Pair : InSpawned)
		{
			const TActor* Candidate = Pair.Value.Get();

			// READY ONLY. A pooled actor mid-init still holds the VirtualTraversal of whoever
			// had it last -- a well-formed position inside a galaxy that is not there any more.
			if (!Candidate || Candidate->InitializationState != ELifecycleState::Ready)
			{
				continue;
			}

			const double Extent = Candidate->Octree.IsValid()
				? Candidate->Octree->Extent
				: Candidate->Params.Extent;
			if (Extent <= 0.0)
			{
				continue;
			}

			const FVector& VT = Candidate->VirtualTraversal;
			const double Fraction =
				FMath::Max3(FMath::Abs(VT.X), FMath::Abs(VT.Y), FMath::Abs(VT.Z)) / Extent;

			if (Fraction < OutBestFraction)
			{
				OutBestFraction = Fraction;
				Best = Candidate;
			}
		}

		return Best;
	}
}

FULSNavSample ULSSampleUniverseNav(const AUniverseActor& InUniverse)
{
	FULSNavSample Out;
	Out.State = InUniverse.InitializationState;

	// EVERYTHING BELOW READS STATE THE TICK MAINTAINS: before Ready, both VirtualTraversal and
	// CurrentFrameOfReferenceLocation are numbers rather than positions.
	if (InUniverse.InitializationState != ELifecycleState::Ready)
	{
		return Out;
	}

	const double UnitScaleCm = InUniverse.UniverseParams.UnitScale;

	Out.bValid = true;
	Out.Coord.Local = InUniverse.VirtualTraversal;
	Out.Coord.UnitScaleCm = UnitScaleCm;

	// THE ONLY ARITHMETIC IN THIS FILE, on one line so the UnitScale used is visible next to
	// the multiply. Double throughout: traversal reaches ~4e9 against a UnitScale of 1.6e17, so
	// the product reaches ~7e26, inside double range with ~15 digits left -- about half an AU
	// at the octree's far edge, which is THIS COORDINATE'S PRECISION FLOOR.
	Out.Coord.RealCm = InUniverse.VirtualTraversal * UnitScaleCm;

	// The universe's own derivation, not a copy of it. See the note on FULSNavSample.
	Out.FieldOffset = InUniverse.ComputeFieldOffset();

	Out.GridCoord = InUniverse.GetFinestTierCoord();
	Out.FieldCellPeriod = InUniverse.GetFieldCellPeriod();
	Out.SpeedScale = InUniverse.GetEffectiveSpeedScale();
	Out.VirtualTraversal = InUniverse.VirtualTraversal;
	Out.FrameOfReferenceWorld = InUniverse.CurrentFrameOfReferenceLocation;
	Out.SpawnedGalaxyCount = InUniverse.SpawnedGalaxies.Num();

	// --- Nested layers ------------------------------------------------------------ STRICTLY
	// TOP-DOWN: the star system is looked up inside the CHOSEN galaxy, not across all of them.
	// A system's VirtualTraversal is only meaningful relative to its parent, so searching all
	// galaxies would return one belonging to a galaxy the player is nowhere near, at a bounds
	// fraction that looks entirely convincing.
	double GalaxyFraction = 0.0;
	const AGalaxyActor* Galaxy =
		SelectNearest<AGalaxyActor>(InUniverse.SpawnedGalaxies, GalaxyFraction);

	if (Galaxy)
	{
		Out.Layers.Add(MakeLayerCoord(TEXT("Galaxy"), *Galaxy));
		Out.SpawnedSystemCount = Galaxy->SpawnedStarSystems.Num();

		// ONLY DESCEND FROM A CONTAINING GALAXY. A system in a galaxy the player is outside of
		// is a coordinate in someone else's frame, and printing it under the galaxy row implies
		// a containment that is not there.
		if (Galaxy->IsPlayerInsideBounds())
		{
			double SystemFraction = 0.0;
			const AStarSystemActor* System =
				SelectNearest<AStarSystemActor>(Galaxy->SpawnedStarSystems, SystemFraction);

			if (System)
			{
				Out.Layers.Add(MakeLayerCoord(TEXT("System"), *System));
			}
		}
	}

	return Out;
}
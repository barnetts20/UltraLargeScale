// ULSNavReadout.cpp
#include "ULSNavReadout.h"

#include "GalaxyActor.h"
#include "StarSystemActor.h"
#include "UniverseActor.h"

namespace
{
	/** Fills the common part of a layer entry from any procedural actor.
	 *
	 *  THE EXTENT COMES FROM THE OCTREE, not from Params.Extent, because
	 *  IsPlayerInsideBounds tests against Octree->Extent and this fraction has to agree
	 *  with that test. They are the same number today (both layers build their tree at
	 *  Params.Extent), and the day one of them stops being -- a rebased tree, a
	 *  multiplier like the universe's PersistentTreeMultiplier -- the readout would
	 *  otherwise start reporting "outside" for a player the spawn scan is still treating
	 *  as inside, with nothing else changing. */
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

	/** Picks the layer instance the player is in, or the nearest one if none contains
	 *  them.
	 *
	 *  NEAREST-BY-BOUNDS-FRACTION, not nearest by distance, and the difference matters
	 *  when two candidates have different extents: a small system 2 extents away and a
	 *  large one 5 extents away are not ranked by raw distance in any way that means
	 *  anything to a player. Fraction ranks them by "how close to being inside".
	 *
	 *  IT RETURNS A NON-CONTAINING CANDIDATE ON PURPOSE. Showing nothing when the player
	 *  is between galaxies hides the useful case -- approaching one -- and makes an empty
	 *  row indistinguishable from a broken walk. The caller dims it and the fraction says
	 *  which it is. */
	template <typename TActor, typename TMap>
	const TActor* SelectNearest(const TMap& InSpawned, double& OutBestFraction)
	{
		const TActor* Best = nullptr;
		OutBestFraction = TNumericLimits<double>::Max();

		for (const auto& Pair : InSpawned)
		{
			const TActor* Candidate = Pair.Value.Get();

			// READY ONLY. A pooled actor mid-init still holds the VirtualTraversal of
			// whoever had it last, and that value is a perfectly well-formed position
			// inside a galaxy that is not there any more.
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

	// EVERYTHING BELOW READS STATE THE TICK MAINTAINS, so it is only meaningful once the
	// tick is maintaining it. Before Ready, VirtualTraversal is whatever the last
	// occupant of this pooled actor left behind and CurrentFrameOfReferenceLocation has
	// not been primed -- both are numbers, neither is a position.
	if (InUniverse.InitializationState != ELifecycleState::Ready)
	{
		return Out;
	}

	const double UnitScaleCm = InUniverse.UniverseParams.UnitScale;
	const double SectorSpan = 2.0 * InUniverse.UniverseParams.Extent;

	Out.bValid = true;
	Out.Coord.Sector = InUniverse.CellCoord;
	Out.Coord.Local = InUniverse.VirtualTraversal;
	Out.Coord.UnitScaleCm = UnitScaleCm;
	Out.Coord.SectorSpanUnits = SectorSpan;

	// THE ONLY ARITHMETIC IN THIS FILE, and it is the sector fold: local units offset by
	// whole sectors, then converted to real cm on one line so the UnitScale that was used
	// is visible next to the multiply. Done in double throughout: at the default extent a
	// sector step is ~4.3e9 units and UnitScale is 1.6e17, so the product reaches ~7e26 --
	// well inside double range, and with ~15 significant digits left, which resolves to
	// roughly half an AU at the far edge of the octree. That is the precision floor of
	// this coordinate, and it is the reason the layers below it exist rather than a
	// defect to fix here.
	const FVector SectorOffsetUnits(
		static_cast<double>(InUniverse.CellCoord.X) * SectorSpan,
		static_cast<double>(InUniverse.CellCoord.Y) * SectorSpan,
		static_cast<double>(InUniverse.CellCoord.Z) * SectorSpan);

	Out.Coord.RealCm = (SectorOffsetUnits + InUniverse.VirtualTraversal) * UnitScaleCm;

	// The universe's own derivation, not a copy of it. See the note on FULSNavSample.
	Out.FieldOffset = InUniverse.ComputeFieldOffset();

	Out.FieldCellPeriod = InUniverse.GetFieldCellPeriod();
	Out.SpeedScale = InUniverse.GetEffectiveSpeedScale();
	Out.VirtualTraversal = InUniverse.VirtualTraversal;
	Out.FrameOfReferenceWorld = InUniverse.CurrentFrameOfReferenceLocation;
	Out.SpawnedGalaxyCount = InUniverse.SpawnedGalaxies.Num();

	// --- Nested layers ------------------------------------------------------------
	//
	// STRICTLY TOP-DOWN, and the star system is looked up inside the CHOSEN galaxy rather
	// than across all of them. Every galaxy owns its own SpawnedStarSystems keyed on its
	// own octree nodes, and a system's VirtualTraversal is only meaningful relative to its
	// parent; searching all galaxies for the nearest system would happily return one
	// belonging to a galaxy the player is nowhere near, at a bounds fraction that looks
	// entirely convincing.
	double GalaxyFraction = 0.0;
	const AGalaxyActor* Galaxy =
		SelectNearest<AGalaxyActor>(InUniverse.SpawnedGalaxies, GalaxyFraction);

	if (Galaxy)
	{
		Out.Layers.Add(MakeLayerCoord(TEXT("Galaxy"), *Galaxy));
		Out.SpawnedSystemCount = Galaxy->SpawnedStarSystems.Num();

		// ONLY DESCEND FROM A CONTAINING GALAXY. A star system belonging to a galaxy the
		// player is outside of is not "the nearest system"; it is a coordinate in a frame
		// that is not the player's, and printing it under the galaxy row implies a
		// containment that is not there.
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
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "ProceduralSpaceActor.h"
#include "FNiagaraParticleBuffer.h"
#include "UniverseParams.h"
#include "UniverseEntityGen.h"

/** Generates the data that populates a universe sector: owns tier geometry, calibration
 *  and the GPU entity dispatch. The sector actor wires tier callbacks that delegate here;
 *  this class has no knowledge of actors, Niagara, octrees, or the streaming pipeline.
 *
 *  IT NO LONGER EVALUATES A FIELD ANYWHERE. Everything that did was FastNoise, and the
 *  field this layer places against is UniverseDensityCore.ush -- which the C++ shim
 *  cannot stand in for, since its geometry depends on texture fetches the shim stubs to
 *  a neutral 0.5. */
class ULTRALARGESCALE_API UniverseDataGenerator {
public:
	UniverseDataGenerator() {};
	UniverseDataGenerator(FUniverseParams InParams) {
		Params = InParams;
	};

	FUniverseParams Params;
	TArray<FPointData> GeneratedData;

	// THE FASTNOISE STACK IS GONE, entire. What stood here was Initialize, BuildNoise,
	// SampleNoiseVolume and the three per-tier rejection samplers -- GenerateLargeTierNode,
	// GenerateMidTierNode and GenerateSmallTierNode -- kept for one pass after the GPU swap
	// so that reverting it meant rebinding three callbacks rather than restoring deleted
	// code. That pass is over: all three tiers bind GenerateBatchCallback and dispatch.
	//
	// THEY WERE NEVER A FALLBACK. They sampled a DIFFERENT FIELD from the one the ray march
	// draws, so falling back to them would have placed entities against a universe nobody is
	// looking at. The GPU path fails closed for that reason and still does.
	//
	// FUniverseParams::EncodedTree and FUniverseNoiseGraphParams came out with them; they
	// were each other's only remaining consumers.

#pragma region GPU Entity Generation

	/** The field's normalized frame, as a HALF EXTENT in caller units.
	 *
	 *  The ray march proxy's half extent, which is the Large tier's neighbourhood span --
	 *  NOT UniverseParams.Extent. Supplied by the actor rather than derived here, because
	 *  the actor owns the proxy and a second derivation of the same number is how the
	 *  render and placement end up sampling two different scalings of one field.
	 *
	 *  Zero until the actor sets it, which every GPU path below treats as unconfigured. */
	double FieldExtent = 0.0;

	/** The field's four volume textures, resolved.
	 *
	 *  THE SAME OBJECTS THE MATERIAL SAMPLES, guaranteed rather than asked for: the actor
	 *  loads all four from the authored paths in MaterialParams and hands the same bundle to
	 *  the material instance and to this. The galaxy layer keeps a separate NoiseTexture
	 *  property that must be set to match its material's, which is a correspondence nothing
	 *  checks -- placement and render can silently sample different assets and the only
	 *  symptom is entities off the structure. Resolving from the authored paths removes the
	 *  question, and it matters four times as much now that there are four of them.
	 *
	 *  ONE BUNDLE, NOT FOUR MEMBERS, so a caller cannot pass a partial set. See
	 *  FUniverseFieldTextures for why all-or-nothing is the only sound rule here.
	 *
	 *  Held raw rather than as a UPROPERTY because this class is not a UObject; the actor
	 *  owns the references that keep them alive. */
	FUniverseFieldTextures FieldTextures;

	/** ONE FIELD CELL IN CALLER UNITS: FieldExtent * CellSizeSmall.
	 *
	 *  NAMED RATHER THAN SPELLED AT EACH SITE. Calibration and the gen-cell split both need
	 *  this number, and two spellings of it put them on fields a fraction of a cell apart --
	 *  which shows as entities sitting beside the structure rather than as anything
	 *  obviously wrong. */
	double FieldCellSize() const
	{
		return FieldExtent
			* FMath::Max(static_cast<double>(Params.DensityParams.Lattice.CellSizeSmall), 1e-6);
	}

	/** The field's repeat period in small cells. Gen cell indices cross to the shader
	 *  through a float3 and must be reduced by the period the CORE derives, never another;
	 *  reducing by a different one puts placement and the render on wraps that disagree, a
	 *  long way out, with nothing logging it. See SplitCellCentre. */
	int32 FieldCellPeriod() const
	{
		return UniverseCellWrap::FieldCellPeriod(Params.DensityParams.Lattice);
	}

	/** The dispatch's InvFieldExtent uniform: what converts a caller-space offset into the
	 *  field's normalized frame.
	 *
	 *  NARROWED HERE AND NOWHERE ELSE. It reaches the shader as a float, and the guard is
	 *  the division guard rather than a range limit -- an unset FieldExtent is zero, which
	 *  every GPU path rejects before reaching this. */
	float InvFieldExtent() const
	{
		return static_cast<float>(1.0 / FMath::Max(FieldExtent, 1e-9));
	}

	/** One cell of a tier's generation grid.
	 *
	 *  THE CENTRE IS SUPPLIED, not derived. Grid-coord-to-centre lives on the actor, which
	 *  owns the grid; a generator inferring it from a buffer's slot centres puts every
	 *  candidate somewhere else entirely. It is also what lets a streamed neighbourhood and
	 *  a calibration block be the same dispatch with different contents. */
	struct FTierBatchCell
	{
		FIntVector Coord = FIntVector::ZeroValue;
		int32 SlotIndex = 0;

		/** Index into the array this cell was subdivided FROM, or its own index when
		 *  nothing was subdivided. Calibration needs it and generation does not: a tier
		 *  with one cell per slot is calibrated against the largest STREAMED cell, which
		 *  after subdivision is the largest sum over one parent's children. */
		int32 ParentIndex = 0;

		FVector Centre = FVector::ZeroVector;
		double HalfExtent = 0.0;
	};

	/** Split each cell into 8^Levels children, in place of it.
	 *
	 *  Child coords are ParentCoord * 2^Levels + an offset, so they are unique across
	 *  parents and depend on nothing but the parent -- the placement key and the probe
	 *  jitter both read them, and a child whose coord shifted with the batch would
	 *  regenerate differently.
	 *
	 *  NO BOUNDS CULL, the one substantive difference from the galaxy's version. That field
	 *  is zero outside its unit sphere, so a child past the boundary can be dropped for one
	 *  dot product instead of a full cell's worth of field evaluations. THIS FIELD IS
	 *  UNBOUNDED: there is no outside, every child can hold structure, and a cull here would
	 *  delete cells that belong. The probe pass's own envelope test does the equivalent job,
	 *  culling void children at the cost of their probes. */
	static void SubdivideCells(const TArray<FTierBatchCell>& InCells, int32 InLevels,
		TArray<FTierBatchCell>& OutCells);

	/** Marshals a tier's cells into the dispatch's FUniverseGenCell records.
	 *
	 *  ONE BUILDER FOR CALIBRATION AND GENERATION, and that is the point rather than a
	 *  convenience. A cell's centre is split into an exact field cell plus a fraction, and a
	 *  calibration that split its cells differently from generation would solve the tier's
	 *  constant against a field offset by a fraction of a cell from the one entities land
	 *  in. The two paths differ in which cells they pass and in nothing else.
	 *
	 *  The slot travels from the cell. Calibration parents are built at slot 0 and children
	 *  inherit their parent's, so the calibration path carries a single token slot without
	 *  needing to say so here. */
	void BuildGenCells(const TArray<FTierBatchCell>& InCells,
		TArray<FUniverseGenCell>& OutCells) const;

	/** A representative block of the field, for calibration.
	 *
	 *  THE GALAXY ENUMERATES ITS WHOLE GRID AND THIS CANNOT. That grid is bounded by the
	 *  galaxy volume, so "every cell the tier will ever generate" is a finite list. This
	 *  field is unbounded and the tier grid is a streaming window that moves with the
	 *  player -- there is no whole grid to measure, and the set of cells that will ever be
	 *  generated is infinite.
	 *
	 *  What makes a sample sufficient instead is HOMOGENEITY. One parameter set describes
	 *  the field everywhere, and its variation is bounded by the authored ranges rather
	 *  than by position, so a block of cells anywhere is statistically the same as a block
	 *  anywhere else. This builds one at a FIXED coord -- not the current neighbourhood --
	 *  so the answer is deterministic and cacheable rather than depending on where the
	 *  player happened to be when a tier first streamed.
	 *
	 *  THE ASSUMPTION IS CHECKABLE and worth checking: the lattice crossover band runs
	 *  about five times the mean density of either end, so if a block lands entirely inside
	 *  or outside it the constant will be off by that factor. Widening the block trades
	 *  calibration cost for a better average. */
	void BuildCalibrationGrid(const FTierParams& InTierParams,
		double InCellHalfExtent, TArray<FTierBatchCell>& OutCells) const;

	/** The tier's placement constant: accepted count per cell is this times cell mass.
	 *
	 *  Measured ONCE per tier, lazily, and cached against the tier's seed offset. Returns 0
	 *  if calibration could not run, which the caller treats as a failed batch rather than
	 *  generating with a meaningless constant. */
	float GetTierBudgetScale(const FTierParams& InTierParams, int32 InSeedOffset,
		int32 InGridDepth, double InCellHalfExtent) const;

	/** GPU generation for a whole batch of tier slots, in one dispatch.
	 *
	 *  ONE GROUP PER CELL. The group probes its cell for a rejection envelope, derives its
	 *  own candidate budget from it, and spends itself on that cell's candidates. Nothing
	 *  on this side evaluates the field -- and for this layer that is structural rather
	 *  than a preference, since the C++ shim stubs texture fetches to a neutral 0.5 and
	 *  this field's GEOMETRY depends on them.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback. Safe because tier generation
	 *  already runs on AnyBackgroundHiPriTask, so the wait costs a worker, not a frame.
	 *
	 *  FAILS CLOSED. There is no CPU path behind it, so a failure blanks the affected slots
	 *  and zeroes their counts before returning. A slot is reused as the player crosses
	 *  boundaries, so "nothing written" is not an empty slot -- it is the previous
	 *  occupant's entities still sitting there at a coord they no longer belong to, which
	 *  reads as a placement bug rather than a generation failure. */
	bool GenerateTierBatchGPU(
		const TArray<FTierBatchCell>& InQueuedCells,
		FNiagaraParticleBuffer& InBuffer,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		int32 InGridDepth,
		TArray<int32>& OutSlotCounts) const;

private:
	/** The GPU placement key seed for one tier. THE ONE PLACE that maps a tier index to a
	 *  seed, because CalibrateBlocking and GenerateBatchBlocking must be handed the
	 *  identical value or a tier calibrates against a field it will not generate.
	 *
	 *  The tier index rides MixSeed's index argument rather than being added to the seed:
	 *  additive offsets alias across sectors whose seeds land within the offset range of
	 *  each other, and offset 0 would hand the large tier the unmixed sector seed, which is
	 *  also whatever else reaches for it. */
	int32 TierKeySeed(int32 InSeedOffset) const
	{
		return ProcSeed::MixSeed(Params.Seed, UniverseSeed::Placement, InSeedOffset);
	}

	/** Calibrated placement constants, keyed by tier seed offset.
	 *
	 *  Mutable and lock-guarded because tier generation runs on background workers and two
	 *  tiers can enter this concurrently. The measurement is deterministic, so a duplicated
	 *  one is wasteful rather than wrong -- the lock is held across it anyway because a GPU
	 *  probe of a calibration block is not something to run twice. */
	mutable TMap<int32, float> TierBudgetScales;

	/** Held by pointer because FCriticalSection is neither copyable nor movable, and a
	 *  defaulted move over a mutex member fails to compile. A moved-from generator has a
	 *  null lock, which GetTierBudgetScale treats as unconfigured rather than dereferencing. */
	mutable TUniquePtr<FCriticalSection> TierBudgetScaleLock =
		MakeUnique<FCriticalSection>();

#pragma endregion
};
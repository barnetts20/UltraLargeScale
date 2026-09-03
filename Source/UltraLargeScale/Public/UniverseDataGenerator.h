// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "ProceduralSpaceActor.h"
#include "FNiagaraParticleBuffer.h"
#include "UniverseParams.h"
#include "UniverseEntityGen.h"

/** Generates the data that populates a universe sector: owns tier geometry, calibration and
 *  the GPU entity dispatch. The sector actor wires tier callbacks that delegate here; this
 *  class has no knowledge of actors, Niagara, octrees, or the streaming pipeline.
 *
 *  It evaluates no field. The field is UniverseDensityCore.ush, and the C++ shim cannot stand
 *  in for it -- the shim stubs texture fetches to 0.5, and the geometry depends on them. */
class ULTRALARGESCALE_API UniverseDataGenerator {
public:
	UniverseDataGenerator() {};
	UniverseDataGenerator(FUniverseParams InParams) {
		Params = InParams;
	};

	FUniverseParams Params;

#pragma region GPU Entity Generation

	/** The field's normalized frame, as a HALF EXTENT in caller units: the ray march proxy's
	 *  half extent, which is the Large tier's neighbourhood span -- NOT UniverseParams.Extent.
	 *  Supplied by the actor, which owns the proxy. Zero until it does, which every GPU path
	 *  below treats as unconfigured. */
	double FieldExtent = 0.0;

	/** The field's four volume textures, resolved. THE SAME OBJECTS THE MATERIAL SAMPLES: the
	 *  actor loads all four from the authored paths and hands one bundle to both, so placement
	 *  and render cannot sample different assets. ONE BUNDLE, NOT FOUR MEMBERS, so a caller
	 *  cannot pass a partial set. Held raw -- the actor owns the references. */
	FUniverseFieldTextures FieldTextures;

	/** One field cell in caller units: FieldExtent * CellSizeSmall. Named rather than spelled
	 *  at each site, since two spellings put calibration and the gen-cell split on fields a
	 *  fraction of a cell apart. */
	double FieldCellSize() const
	{
		return FieldExtent
			* FMath::Max(static_cast<double>(Params.DensityParams.Lattice.CellSizeSmall), 1e-6);
	}

	/** The field's repeat period in small cells. Gen cell indices cross to the shader through a
	 *  float3 and must be reduced by the period the CORE derives, never another. */
	int32 FieldCellPeriod() const
	{
		return UniverseCellWrap::FieldCellPeriod(Params.DensityParams.Lattice);
	}

	/** The dispatch's InvFieldExtent uniform: converts a caller-space offset into the field's
	 *  normalized frame. Narrowed to float here and nowhere else. */
	float InvFieldExtent() const
	{
		return static_cast<float>(1.0 / FMath::Max(FieldExtent, 1e-9));
	}

	/** Marshals a tier's cells into the dispatch's FUniverseGenCell records.
	 *
	 *  ONE BUILDER FOR CALIBRATION AND GENERATION. A cell's centre is split into an exact
	 *  field cell plus a fraction, and a calibration that split differently from generation
	 *  would solve the tier's constant against a field offset by a fraction of a cell. The two
	 *  paths differ only in which cells they pass. The slot travels from the cell. */
	void BuildGenCells(const TArray<FTierBatchCell>& InCells,
		TArray<FUniverseGenCell>& OutCells) const;

	/** A representative block of the field, for calibration. Built at a FIXED coord, not the
	 *  current neighbourhood, so the answer is deterministic and cacheable. A sample suffices
	 *  because the field is HOMOGENEOUS -- one parameter set describes it everywhere. The
	 *  lattice crossover band runs about five times the mean density of either end, so a block
	 *  landing entirely inside or outside it puts the constant off by that factor. */
	void BuildCalibrationGrid(const FTierParams& InTierParams,
		double InCellHalfExtent, TArray<FTierBatchCell>& OutCells) const;

	/** The tier's placement constant: accepted count per cell is this times cell mass. Measured
	 *  ONCE per tier, lazily, and cached against the tier's seed offset. Returns 0 if
	 *  calibration could not run, which the caller must treat as a failed batch. */
	float GetTierBudgetScale(const FTierParams& InTierParams, int32 InSeedOffset,
		int32 InGridDepth, double InCellHalfExtent) const;

	/** GPU generation for a whole batch of tier slots, in one dispatch. One group per cell:
	 *  the group probes its cell for a rejection envelope, derives its own candidate budget
	 *  from it, and spends itself on that cell's candidates.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback.
	 *
	 *  FAILS CLOSED: a failure blanks the affected slots and zeroes their counts before
	 *  returning, because a slot is reused as the player crosses boundaries and "nothing
	 *  written" leaves the previous occupant's entities at a coord they do not belong to. */
	bool GenerateTierBatchGPU(
		const TArray<FTierBatchCell>& InQueuedCells,
		FNiagaraParticleBuffer& InBuffer,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		int32 InGridDepth,
		TArray<int32>& OutSlotCounts) const;

private:
	/** The GPU placement key seed for one tier. THE ONE PLACE that maps a tier index to a seed:
	 *  CalibrateBlocking and GenerateBatchBlocking must be handed the identical value, or a
	 *  tier calibrates against a field it will not generate. */
	int32 TierKeySeed(int32 InSeedOffset) const
	{
		return ProcSeed::MixSeed(Params.Seed, UniverseSeed::Placement, InSeedOffset);
	}

	/** Calibrated placement constants, keyed by tier seed offset. Mutable and lock-guarded
	 *  because tier generation runs on background workers and two tiers can enter this
	 *  concurrently. The lock is held across the measurement, not just the map write. */
	mutable TMap<int32, float> TierBudgetScales;

	/** Held by pointer because FCriticalSection is neither copyable nor movable. A moved-from
	 *  generator has a null lock, which GetTierBudgetScale treats as unconfigured. */
	mutable TUniquePtr<FCriticalSection> TierBudgetScaleLock =
		MakeUnique<FCriticalSection>();

#pragma endregion
};
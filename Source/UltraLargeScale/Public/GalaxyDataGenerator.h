// GalaxyDataGenerator.h
// Tier generation for the galaxy layer. NO FIELD EVALUATION HERE: the density field lives in
// GalaxyDensityCore.ush and is evaluated only by the GPU. This class marshals parameters into
// a dispatch and reads the results back.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "DataTypes.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "FNiagaraParticleBuffer.h"

/** Owns density evaluation and tier generation for the galaxy layer; mirrors
 *  UniverseDataGenerator. The galaxy actor wires tier callbacks that delegate here;
 *  this class has no knowledge of actors, Niagara, octrees, or streaming. */
class ULTRALARGESCALE_API GalaxyDataGenerator
{
public:
	GalaxyDataGenerator();
	explicit GalaxyDataGenerator(FGalaxyParams InParams);

	// Out of line, and move-only: TUniquePtr needs GalaxyDensityParams complete at
	// the point of destruction, which is only true inside the .cpp.
	~GalaxyDataGenerator();
	GalaxyDataGenerator(GalaxyDataGenerator&&) noexcept;
	GalaxyDataGenerator& operator=(GalaxyDataGenerator&&) noexcept;
	GalaxyDataGenerator(const GalaxyDataGenerator&) = delete;
	GalaxyDataGenerator& operator=(const GalaxyDataGenerator&) = delete;

	FGalaxyParams Params;

#pragma region Initialization

	/** Clears the per-tier calibration cache. MUST run whenever Params changes: calibration is
	 *  solved against a specific field, so on a pooled actor a surviving entry places the new
	 *  galaxy's entities at a density solved for the previous one. */
	void Initialize();

#pragma endregion

#pragma region Tier Generation

public:
	/** The tier's placement constant: accepted count per cell is this times cell mass.
	 *
	 *  Measured ONCE per tier, lazily, by probing its whole grid -- SUBDIVIDED EXACTLY AS
	 *  GENERATION WILL SUBDIVIDE IT -- and reducing on the CPU. Cached against the tier's
	 *  seed offset, which is what distinguishes the three. bInCellsShareSlot selects the
	 *  reduction: the total over the grid when cells share a slot, the largest per-parent sum
	 *  when each streamed cell owns its own.
	 *
	 *  Returns 0 if calibration could not run; the caller must treat that as a failed batch
	 *  rather than generating with a meaningless constant. */
	float GetTierBudgetScale(const FTierParams& InTierParams, int32 InSeedOffset,
		bool bInCellsShareSlot) const;

	/** Every cell of a streamed tier's grid, enumerated exhaustively, bounds-culled only.
	 *  Uses the streaming system's own coordinate formula, so calibration measures the cells
	 *  the tier will actually generate with. */
	void BuildFullTierGrid(int32 InGridDepth, TArray<FTierBatchCell>& OutCells) const;

	/** GPU generation for a whole batch of tier slots, in one dispatch. One group per cell:
	 *  the group probes its cell for a rejection envelope, derives its own candidate budget
	 *  from that envelope, and spends itself on that cell's candidates.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback.
	 *
	 *  Returns false if the dispatch could not run or the readback timed out. FAILS CLOSED --
	 *  the affected slots are blanked and their counts zeroed before returning, because a
	 *  reused slot left untouched shows the previous occupant's entities. Every path logs. */
	bool GenerateTierBatchGPU(
		const TArray<FTierBatchCell>& InCells,
		FNiagaraParticleBuffer& InBuffer,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		/** Whether MANY cells feed ONE slot. Stated rather than inferred from the cell count,
		 *  because subdivision also makes cells outnumber slots and means the opposite thing:
		 *  those cells all belong to one parent, not to the whole tier. */
		bool bInCellsShareSlot,
		TArray<int32>& OutSlotCounts) const;

private:
	/** The GPU placement key seed for one tier. THE ONE PLACE that maps a tier index to a seed:
	 *  CalibrateBlocking and GenerateBatchBlocking must be handed the identical value. */
	int32 TierKeySeed(int32 InSeedOffset) const
	{
		return ProcSeed::MixSeed(Params.Seed, GalaxySeed::Placement, InSeedOffset);
	}

	/** Calibrated placement constants, keyed by tier seed offset. Mutable and lock-guarded
	 *  because tier generation runs on background workers and two tiers can enter this
	 *  concurrently. The lock is held across the measurement, not just the map write. */
	mutable TMap<int32, float> TierBudgetScales;

	/** Held by pointer because THIS CLASS IS MOVABLE and FCriticalSection is not. A moved-from
	 *  generator has a null lock, and GetTierBudgetScale returns zero rather than deref it. */
	mutable TUniquePtr<FCriticalSection> TierBudgetScaleLock =
		MakeUnique<FCriticalSection>();

#pragma endregion
};
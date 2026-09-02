// GalaxyDataGenerator.h
// Galaxy density field and tier generation.
//
// The density field itself lives in Shaders/GalaxyDensityCore.ush and is compiled by
// BOTH the shader and this module, so star placement and the rendered gas are one
// function rather than two implementations kept in agreement by hand. Only
// GalaxyDataGenerator.cpp compiles it, inside namespace GalaxyHLSL -- see
// GalaxyHLSLShim.h for why that namespace must not be opened.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "DataTypes.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "FNiagaraParticleBuffer.h"

/** Declared in GalaxyDensityCore.ush, compiled inside namespace GalaxyHLSL by
 *  GalaxyDataGenerator.cpp. Held by pointer so this header needs neither the shim nor
 *  the field itself. */
namespace GalaxyHLSL
{
	struct GalaxyDensityParams;
}

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

	/** Clamped, pre-inverted fields plus the 16 per-arm records, derived once in
	 *  Initialize(). Rebuilding it per sample would repeat 16 hashes, a tan and every
	 *  reciprocal on every candidate. */
	TUniquePtr<GalaxyHLSL::GalaxyDensityParams> Derived;

#pragma region Density Sampling

	/** Raw field value at a normalized position, InNormPos in [-1,1] (position /
	 *  Extent).
	 *
	 *  UNBOUNDED. This is an OPTICAL DEPTH, not a probability: it peaks near 260 at
	 *  the default tuning while most of the volume sits below 0.01. Feeding it
	 *  straight to a rejection test accepts every candidate above 1.0 -- the arms,
	 *  the inner disc and the whole bulge -- erasing the structure it describes.
	 *  Always route it through GalaxySpawnProbability, in GalaxyPlacement.ush, first. */
	float SampleDensity(const FVector& InNormPos) const;

#pragma endregion

#pragma region Initialization

	/** Derive the density parameters. MUST run before any sampling; SampleDensity
	 *  returns zero until it does. */
	void Initialize();

	// THERE IS NO SECOND FIELD HERE. The only thing this layer places against is
	// GalaxyDensityCore.ush, evaluated through Derived above. A noise graph built beside it
	// would be a field nobody renders, and entities placed against one are entities sitting
	// beside the structure they belong to.

#pragma endregion

#pragma region Tier Generation

	/** GENERATION IS GPU-ONLY FOR THIS LAYER, end to end. Cell culling, envelope estimation
	 *  and per-cell budgeting all happen in the dispatch, where they sample the TEXTURED
	 *  field; a CPU prepass could only reach an analytic stand-in for it, and a budget
	 *  solved against a different field from the one entities are rejected against is wrong
	 *  in a way nothing reports.
	 *
	 *  WHAT IS LEFT ON THIS SIDE IS GEOMETRY AND MARSHALLING. Nothing here evaluates the
	 *  field, so GalaxyDensityCore.ush does not have to compile as C++ to serve the runtime.
	 *  The shim stays for the bake and the parity probe, which are verification tools rather
	 *  than a second implementation of the path.
	 *
	 *  FParticleTierConfig::GenerateCallback stays on the config because the other two
	 *  layers bind it. */

public:
	/** THE CELL TYPE AND THE SUBDIVISION ARE SHARED. Both live on FTierStreamingSystem
	 *  because every layer needs the same ones, and a per-layer copy of either is a place the
	 *  child COORD -- the placement key -- can drift between layers. See FTierBatchCell.
	 *
	 *  THIS LAYER PASSES MakeSphereBoundsCull. The field is zero outside its unit sphere, so
	 *  a child whose nearest point already lies past it can hold nothing, and a sphere fills
	 *  only pi/6 of its bounding cube -- one dot product against a whole thread group's worth
	 *  of field evaluations.
	 *
	 *  Centred coords, not Ascending. Switching would reroll every star this layer places;
	 *  see ETierChildCoords. */

	 /** The tier's placement constant: accepted count per cell is this times cell mass.
	  *
	  *  Measured ONCE per tier, lazily, by probing its whole grid -- SUBDIVIDED EXACTLY AS
	  *  GENERATION WILL SUBDIVIDE IT -- and reducing on the CPU. Cached against the tier's
	  *  seed offset, which is what distinguishes the three.
	  *
	  *  Which reduction divides the capacity depends on how the tier maps cells to slots:
	  *  the total when they share one, the largest per-parent sum when each streamed cell
	  *  owns its own. Getting that wrong is what makes a void neighbourhood come back as
	  *  densely populated as an arm.
	  *
	  *  Returns 0 if calibration could not run, which the caller treats as a failed
	  *  batch rather than generating with a meaningless constant. */
	float GetTierBudgetScale(const FTierParams& InTierParams, int32 InSeedOffset,
		bool bInCellsShareSlot) const;

	/** Every cell of a streamed tier's grid, enumerated exhaustively.
	 *
	 *  The streaming system's own coordinate formula, so calibration measures the cells
	 *  the tier will actually generate with -- a different coord labelling would reseed
	 *  the probe jitter and answer a slightly different question. Bounds-culled only. */
	void BuildFullTierGrid(int32 InGridDepth, TArray<FTierBatchCell>& OutCells) const;

	/** GPU generation for a whole batch of tier slots, in one dispatch.
	 *
	 *  ONE GROUP PER CELL. The group probes its cell to get a rejection envelope,
	 *  derives its own candidate budget from that envelope, and then spends itself on
	 *  that cell's candidates. Nothing on this side evaluates the field.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback. Safe because tier generation
	 *  already runs on AnyBackgroundHiPriTask, so the wait costs a worker, not a frame.
	 *
	 *  Returns false if the dispatch could not run or the readback timed out. There is no
	 *  CPU path behind it, so it FAILS CLOSED: the affected slots are blanked and their
	 *  counts zeroed before returning. A slot is reused as the player crosses boundaries,
	 *  and leaving one untouched shows the previous occupant's entities at a coord they do
	 *  not belong to. Every such path logs. */
	bool GenerateTierBatchGPU(
		const TArray<FTierBatchCell>& InCells,
		FNiagaraParticleBuffer& InBuffer,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		/** Whether MANY cells feed ONE slot, which decides how the tier's placement
		 *  constant is calibrated -- against the total mass of its grid rather than the
		 *  largest single cell. Stated rather than inferred from the cell count, because
		 *  subdivision also makes cells outnumber slots and means the opposite thing:
		 *  those cells all belong to one parent, not to the whole tier. */
		bool bInCellsShareSlot,
		TArray<int32>& OutSlotCounts) const;

private:
	/** The GPU placement key seed for one tier. THE ONE PLACE that maps a tier index to
	 *  a seed, because CalibrateBlocking and GenerateBatchBlocking must be handed the
	 *  identical value -- see GalaxySeed::Placement.
	 *
	 *  Was `Params.Seed + InSeedOffset`. Additive offsets alias across galaxies whose
	 *  seeds land within the offset range of each other, and offset 0 handed the
	 *  large tier the unmixed galaxy seed, which is also whatever else reaches for it. */
	int32 TierKeySeed(int32 InSeedOffset) const
	{
		return ProcSeed::MixSeed(Params.Seed, GalaxySeed::Placement, InSeedOffset);
	}

	/** Calibrated placement constants, keyed by tier seed offset.
	 *
	 *  Mutable and lock-guarded because tier generation runs on background workers and
	 *  two tiers can enter this concurrently. The measurement is deterministic, so a
	 *  duplicated one is wasteful rather than wrong -- the lock is held across it anyway
	 *  because a GPU probe of a whole grid is not something to run twice. */
	mutable TMap<int32, float> TierBudgetScales;

	/** Held by pointer because THIS CLASS IS MOVABLE and FCriticalSection is not.
	 *
	 *  The generator owns a TUniquePtr member already, so it is move-only, and a
	 *  defaulted move constructor over a mutex member fails to compile -- the mutex
	 *  deletes both its copy and its move. Indirecting through TUniquePtr moves the
	 *  pointer and leaves the mutex where it is.
	 *
	 *  A moved-from generator has a null lock, which GetTierBudgetScale treats the same
	 *  way SampleDensity treats a null Derived: return zero rather than dereference. */
	mutable TUniquePtr<FCriticalSection> TierBudgetScaleLock =
		MakeUnique<FCriticalSection>();

#pragma endregion
};
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
#include "FastNoise/FastNoise.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "FVolumeTextureUtils.h"
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
	FastNoise::SmartNode<> DensityNoise;

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
	 *  Always route it through GalaxyDensityParams::SpawnProbability first. */
	float SampleDensity(const FVector& InNormPos) const;

#pragma endregion

#pragma region Initialization

	/** Derive the density parameters and build the encoded noise graph. MUST run
	 *  before any sampling; SampleDensity returns zero until it does. */
	void Initialize();

	/** Build noise from encoded tree. Kept for a future FastNoise swap-in. */
	FastNoise::SmartNode<> BuildNoise() const;

	/** Sample the field into a CPU-side BGRA8 volume buffer.
	 *
	 *  TRANSITIONAL. Once the raymarch material evaluates the field directly there is
	 *  nothing to bake, and removing this path also removes the async upload, the
	 *  upscale, and roughly 320 MB of resident volume texture. It is kept for now as
	 *  an independent cross-check: the CPU evaluates and bakes, the old material
	 *  renders the texture, and that should match the new material evaluating the
	 *  same parameters live. */
	TArray<uint8> SampleNoiseVolume(int InNoiseResolution) const;

#pragma endregion

#pragma region Tier Generation

	/** Generation is GPU-ONLY for this layer, and now GPU-only end to end.
	 *
	 *  The per-slot CPU generators went first -- GenerateTierNode, GenerateLargeTierSlot
	 *  and the MakePlacement helper they shared -- along with the
	 *  FParticleTierConfig::GenerateCallback bindings that reached them. The density
	 *  PREPASS went second: cell culling, envelope estimation and per-cell budgeting all
	 *  moved into the dispatch, where they can sample the textured field instead of an
	 *  analytic stand-in for it.
	 *
	 *  What that leaves on this side is geometry and marshalling. Nothing here evaluates
	 *  the field, so GalaxyDensityCore.ush no longer has to compile as C++ to serve the
	 *  runtime -- the shim stays for the bake and the parity probe, which are
	 *  verification tools rather than a second implementation of the path.
	 *
	 *  The other two layers still bind GenerateCallback, so it stays on the config. */

public:
	/** A cell handed to the dispatch. GEOMETRY ONLY.
	 *
	 *  It carried a DensityReference and a Candidates count until the prepass moved to
	 *  the GPU. Both are now derived by the cell's own thread group, from probes that
	 *  sample the TEXTURED field rather than an analytic stand-in for it.
	 *
	 *  THE CENTRE IS SUPPLIED, not derived. Grid-coord-to-centre lives on the actor,
	 *  which owns the grid; the generator inferring it from a buffer's slot centres put
	 *  every candidate somewhere else entirely and every batch came back with nothing
	 *  accepted. It is also what lets a streamed neighbourhood and a whole bounding grid
	 *  be the same dispatch with different contents. */
	struct FTierBatchCell
	{
		FIntVector Coord = FIntVector::ZeroValue;
		int32 SlotIndex = 0;

		/** Index into the array this cell was subdivided FROM, or its own index when
		 *  nothing was subdivided.
		 *
		 *  Calibration needs it and generation does not. A tier with one cell per slot
		 *  is calibrated against the largest STREAMED cell, which after subdivision is
		 *  the largest sum over one parent's children -- so the children have to say
		 *  which parent they belong to. SlotIndex cannot answer that: a batch of
		 *  neighbouring cells shares no slot, and the whole-grid calibration pass has no
		 *  slots at all. */
		int32 ParentIndex = 0;

		FVector Centre = FVector::ZeroVector;
		double HalfExtent = 0.0;
	};

	/** GPU generation for a whole batch of tier slots, in one dispatch.
	 *
	 *  ONE GROUP PER CELL. The group probes its cell to get a rejection envelope,
	 *  derives its own candidate budget from that envelope, and then spends itself on
	 *  that cell's candidates. Nothing on this side evaluates the field.
	 *
	 *  This is the reason for the migration: the compute path can sample the volume
	 *  texture, so entity placement sees the warp and modulation the material draws
	 *  instead of a texture-free approximation of it. It also lifts the constraint that
	 *  shaped the field in the first place -- features no longer have to be affordable
	 *  analytically to be reachable from placement.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback. That is safe because tier
	 *  generation already runs on AnyBackgroundHiPriTask, so the wait costs a worker
	 *  rather than a frame.
	 *
	 *  Returns false if the dispatch could not run or the readback timed out. There is
	 *  no CPU path behind it, so it FAILS CLOSED: the affected slots are blanked and
	 *  their counts zeroed before returning, because a slot is reused as the player
	 *  crosses boundaries and leaving it untouched would show the previous occupant's
	 *  entities at a coord they no longer belong to. Every such path logs. */
	 /** The tier's placement constant: accepted count per cell is this times cell mass.
	  *
	  *  Measured ONCE per tier, lazily, by probing its whole grid -- SUBDIVIDED EXACTLY AS
	  *  GENERATION WILL SUBDIVIDE IT -- and reducing on the CPU. Cached against the tier's
	  *  seed offset, which is what distinguishes the three.
	  *
	  *  Which reduction divides the capacity depends on how the tier maps cells to slots:
	  *  the total when they share one, the largest per-parent sum when each streamed cell
	  *  owns its own. Getting that wrong is what made a void neighbourhood come back as
	  *  densely populated as an arm.
	  *
	  *  Returns 0 if calibration could not run, which the caller treats as a failed
	  *  batch rather than generating with a meaningless constant. */
	float GetTierBudgetScale(const FTierParams& InTierParams, int32 InSeedOffset,
		bool bInCellsShareSlot) const;

	/** Split each cell into 8^Levels children, in place of it.
	 *
	 *  Deriving geometry FROM a supplied cell, not inferring it: the actor still says
	 *  where its cells are and this only descends inside them, which is what keeps the
	 *  grid the actor's business.
	 *
	 *  Child coords are ParentCoord * 2^Levels + an offset, so they are unique across
	 *  parents and depend on nothing but the parent -- the placement key and the probe
	 *  jitter both read them, and a child whose coord shifted with the batch would
	 *  regenerate differently. They do NOT correspond to positions on the streaming grid
	 *  at the deeper level, and nothing requires them to.
	 *
	 *  Children whose nearest point lies outside the field are dropped here rather than
	 *  discovered by their thread group, which costs one dot product against sixty-four
	 *  field evaluations. */
	static void SubdivideCells(const TArray<FTierBatchCell>& InCells, int32 InLevels,
		double InExtent, TArray<FTierBatchCell>& OutCells);

	/** Every cell of a streamed tier's grid, enumerated exhaustively.
	 *
	 *  The streaming system's own coordinate formula, so calibration measures the cells
	 *  the tier will actually generate with -- a different coord labelling would reseed
	 *  the probe jitter and answer a slightly different question. Bounds-culled only. */
	void BuildFullTierGrid(int32 InGridDepth, TArray<FTierBatchCell>& OutCells) const;

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
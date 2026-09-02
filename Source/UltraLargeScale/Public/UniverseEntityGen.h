// UniverseEntityGen.h
// Global shader + RDG dispatch for GPU entity placement on the universe layer.
//
// Build.cs needs: "RenderCore", "RHI", "Renderer", "Projects".
// The /UltraLargeScale shader path is already registered in FUltraLargeScaleModule.
//
// A NEAR-TWIN OF GalaxyEntityGen, and deliberately so: the two dispatches have the same
// shape, the same failure modes and the same hard-won details in the RDG setup. They are
// separate rather than templated because the parameter structs differ in every field and
// SHADER_PARAMETER_STRUCT is a macro, not a type -- unifying them would mean a base struct
// plus a derived one per layer, which is more machinery than the duplication costs.
//
// WHAT IS GENUINELY DIFFERENT, and worth knowing before reading it as a copy:
//   - Placement lives in UniversePlacement.ush, not in the field core.
//   - There is no VIEW offset. The proxy's offset is a property of the view and the field
//     is static in caller space, so the uniform offset pins are zero here; each group still
//     passes its own cell as the offset. See InvFieldExtent in the .usf.
//   - A field sample here is a fifty-four candidate walk across two lattices plus five
//     texture fetches across FOUR distinct volumes, roughly fifteen to twenty-five times a
//     galaxy sample. Probe cost therefore dominates far sooner, and GenerationSubdivision
//     wants to sit lower. The five fetches touch four working sets, so cache behaviour is
//     worse than the fetch count alone suggests.

#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"
#include "RHIGPUReadback.h"

#include <atomic>

#include "UniverseParams.h"
#include "FTierStreamingSystem.h"

/** One placed entity, as the dispatch writes it.
 *
 *  EXACTLY TWO uint4s. UniverseEntityGen.usf writes OutEntities[Base + 0] and [Base + 1]
 *  from this layout; there is no shared declaration, so the two are kept in step by this
 *  comment and the assert below.
 *
 *  AN INTEGER BUFFER carrying the floats as bit patterns, not the reverse. A float's bit
 *  pattern is always a valid integer; an arbitrary integer read as a float can be a
 *  denormal or a NaN, and neither round-trips reliably.
 *
 *  A member that straddles a 16-byte boundary packs differently on some backends and the
 *  failure is silent -- every field after the straddle reads shifted. Keeping the record a
 *  flat multiple of 16 with no implicit tail padding removes the question. */
struct FUniverseEntityOut
{
	/** CALLER UNITS, RELATIVE TO THE OWNING CELL'S CENTRE -- not an absolute position.
	 *
	 *  AN ABSOLUTE POSITION HERE WOULD NARROW A COORDINATE THAT TRACKS VirtualTraversal into
	 *  a float32 on the way back, losing precision the CPU cannot recover. The push folds
	 *  positions to cell-local anyway -- FNiagaraParticleBuffer::MakeCellLocalPositions --
	 *  so folding in the shader instead costs nothing and bounds this value by the cell.
	 *
	 *  The scatter adds the cell's own double-precision centre back. See FUniverseGenCell. */
	FVector3f Pos = FVector3f::ZeroVector;
	float Extent = 0.0f;

	/** Three decorrelated uniforms in [0,1), ten bits each. Read it through DecodeDecor
	 *  rather than by hand. */
	uint32 DecorPacked = 0;

	/** The candidate index that produced this entity, NOT its position in the buffer. The
	 *  dispatch compacts with a global atomic, so storage ORDER is scheduling dependent
	 *  while the set, and each entity's identity within it, is not. */
	uint32 Slot = 0;

	/** Index into the batch's cell array. Required, not diagnostic: entities are appended to
	 *  one shared buffer, so an entity's position in that buffer says nothing about which
	 *  cell -- and therefore which SLOT -- it belongs to. */
	uint32 CellIndex = 0;

	/** Raw field value where the entity landed, before any spawn shaping.
	 *
	 *  THE UNFADED VALUE. UniverseRayMarch multiplies by BoundsFade after sampling and
	 *  SampleAtPosition does not, so this is the field itself rather than the field as
	 *  seen from wherever the player happened to be. */
	float Density = 0.0f;

	/** Undoes PackUniverseDecor in UniversePlacement.ush. Keep the two together. */
	FVector3f DecodeDecor() const
	{
		constexpr float Inv = 1.0f / 1023.0f;

		return FVector3f(
			static_cast<float>(DecorPacked & 1023u) * Inv,
			static_cast<float>((DecorPacked >> 10) & 1023u) * Inv,
			static_cast<float>((DecorPacked >> 20) & 1023u) * Inv);
	}
};
static_assert(sizeof(FUniverseEntityOut) == 32, "FUniverseEntityOut must match the .usf layout");

/** Mirrors FUniverseGenCell in UniverseEntityGen.usf.
 *
 *  GEOMETRY ONLY. A centre, a half extent, and the grid coord that keys the cell. NOTHING
 *  ABOUT THE FIELD may be added here: the shader probes each cell and derives its own
 *  envelope and budget, and anything the CPU had to fill in would drag density evaluation
 *  back onto this side -- which for this layer means onto a path that cannot reach the
 *  volume texture at all.
 *
 *  THE CENTRE IS A CELL AND A FRACTION, NOT A POSITION, and it must stay that way.
 *
 *  Caller space tracks VirtualTraversal without bound -- the octree rebases AROUND the player
 *  rather than renumbering under them -- and a float32 holding 5e13 has a unit of last place
 *  of about 4e6. AN ABSOLUTE CENTRE HERE DOES NOT PRODUCE A WRONG POSITION. It produces a
 *  dead tier: MakeUniverseProbe builds its probes as centre +/- extent, so once the extent
 *  falls below the centre's own ulp EVERY PROBE IN THE CELL COLLAPSES ONTO THE SAME FLOAT.
 *  The envelope becomes a single-point estimate of a mostly-void field, the cell reports no
 *  mass, and the tier places nothing at all -- silently, with the raymarch still drawing the
 *  field perfectly. A quantised grid of galaxies would be the gentle version.
 *
 *  Splitting it fixes that at the root: the integer carries the magnitude exactly, and
 *  everything the shader computes from it is a small offset inside one cell where float32 has
 *  all its precision. The cell index is fed to MakeUniverseDensityParams as that group's
 *  field offset, so the sample point is reconstructed by the same mechanism the material
 *  uses.
 *
 *  CentreCell IS REDUCED MODULO THE FIELD PERIOD by the caller, for the same reason
 *  AUniverseActor::ComputeFieldOffset reduces: it crosses to the shader through a float3.
 *  Reducing by a DIFFERENT period from the one the core derives puts placement and the
 *  material on wraps that disagree -- see UniverseCellWrap::FieldCellPeriod.
 *
 *  The padding holds the record at 48 bytes and splits it so no vector straddles a
 *  sixteen-byte boundary; HalfExtent sits between the two vectors to keep that true. */
struct FUniverseGenCell
{
	/** Whole field cells containing this cell's centre, wrapped into [0, period). */
	FIntVector3 CentreCell = FIntVector3(0, 0, 0);

	/** Half extent in CALLER units, unchanged. Bounded by the grid cell size, so it never
	 *  had a precision problem of its own -- only the centre it was added to did. */
	float HalfExtent = 0.0f;

	/** Remainder of the centre within CentreCell, per axis, in [0,1). */
	FVector3f CentreFrac = FVector3f::ZeroVector;

	/** Which slot region of the particle buffer this cell's entities belong to. Every child
	 *  of a subdivided streamed cell carries its parent's.
	 *
	 *  THE SHADER NEEDS IT, not just the CPU. Without it the dispatch can only compact
	 *  globally, and a batch that accepts more than it can hold then keeps whichever
	 *  entities arrived first -- scheduling order, not field order.
	 *
	 *  SITS BETWEEN THE TWO VECTORS, like HalfExtent above, and for the same reason: with
	 *  Coord immediately after CentreFrac the third vector lands at offset 28 and straddles
	 *  the 16-byte boundary at 32. */
	uint32 SlotIndex = 0;

	/** The placement key, and the ONE coordinate here that is not wrapped. It seeds every
	 *  candidate draw and, through FVoxelData::ComposeSeed, every entity -- so generation
	 *  stays aperiodic while the density field repeats. */
	FIntVector3 Coord = FIntVector3(0, 0, 0);

	uint32 Pad = 0;
};
static_assert(sizeof(FUniverseGenCell) == 48, "FUniverseGenCell must match the .usf layout");

class FUniverseEntityGenCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FUniverseEntityGenCS);
	SHADER_USE_PARAMETER_STRUCT(FUniverseEntityGenCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint4>, OutEntities)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutCounts)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FUniverseGenCell>, InCells)

		SHADER_PARAMETER(uint32, NumCells)
		SHADER_PARAMETER(uint32, EntityCapacity)
		SHADER_PARAMETER(uint32, NumSlots)
		SHADER_PARAMETER(uint32, SlotCapacity)
		SHADER_PARAMETER(uint32, DispatchGroupsX)
		SHADER_PARAMETER(float, BudgetScale)
		SHADER_PARAMETER(int32, KeySeed)

		/** Reciprocal of the RAY MARCH PROXY'S half extent -- not the sector extent, and
		 *  not anything about the tier grid. The material marches a box of that size, so
		 *  that is what one unit of normalized space means. See the long note in the .usf
		 *  for why no field offset accompanies it. */
		SHADER_PARAMETER(float, InvFieldExtent)

		SHADER_PARAMETER(float, BudgetAnchor)
		SHADER_PARAMETER(float, EnvelopePad)

		SHADER_PARAMETER(float, PlaceSpawnExponent)
		SHADER_PARAMETER(float, PlaceExtentMin)
		SHADER_PARAMETER(float, PlaceExtentMax)
		SHADER_PARAMETER(float, PlaceExtentExponent)

		// THE FOUR FIELD VOLUMES, matching the .usf declarations and the material's Custom
		// node pin for pin. Two UNORM multinoise volumes for the region axes, two signed
		// vector volumes for the warp octaves.
		//
		// FOUR SAMPLERS RATHER THAN ONE SHARED. They are all currently the same static
		// state -- trilinear, wrap on every axis -- so this costs four descriptor slots to
		// say nothing. It buys the ability to give one asset different addressing without
		// touching the other three, which is the likely next thing to want: the warp
		// volumes and the variance volumes have different periodicity requirements and only
		// the warp ones are forced to wrap by the cell-index mask.
		SHADER_PARAMETER_TEXTURE(Texture3D, VarianceTexA)
		SHADER_PARAMETER_SAMPLER(SamplerState, VarianceTexASampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, VarianceTexB)
		SHADER_PARAMETER_SAMPLER(SamplerState, VarianceTexBSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexLarge)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexLargeSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexSmall)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexSmallSampler)

		// Raw field inputs, in MakeUniverseDensityParams order, and written by
		// FUniverseDensityParams::FillShaderParameters. Deriving in the shader rather than
		// uploading a packed UniverseDensityParams: a derived struct in a constant buffer
		// would have to match HLSL packing against the C++ layout member for member, and
		// one straddling float3 shifts everything after it with no diagnostic.
		SHADER_PARAMETER(FVector4f, InCellSizeRange)
		SHADER_PARAMETER(float, InSeed)
		SHADER_PARAMETER(FVector3f, InOffsetCell)
		SHADER_PARAMETER(FVector3f, InOffsetFrac)
		SHADER_PARAMETER(FVector4f, InWallDensityRange)
		SHADER_PARAMETER(FVector4f, InWallFalloffRange)
		SHADER_PARAMETER(FVector4f, InFilamentDensityRange)
		SHADER_PARAMETER(FVector4f, InFeatureWidthRange)
		SHADER_PARAMETER(FVector4f, InVoidFloorRange)
		SHADER_PARAMETER(FVector4f, InVoidSizeSpreadRange)
		SHADER_PARAMETER(FVector4f, InWarpAmountLargeRange)
		SHADER_PARAMETER(FVector4f, InWarpAmountSmallRange)
		SHADER_PARAMETER(FVector4f, InWarpLargeWeights)
		SHADER_PARAMETER(FVector4f, InWarpSmallWeights)
		SHADER_PARAMETER(FVector4f, InRegionScales)
		SHADER_PARAMETER(float, InBoundsFadeStart)
		END_SHADER_PARAMETER_STRUCT()

		/** Which pass this permutation compiles.
		 *
		 *    0 PROBE    -- one group per cell: cull, envelope, and weigh it
		 *    1 GENERATE -- one group per cell: draw and place candidates
		 *
		 *  Generation runs both. CALIBRATION runs the probe pass alone and reduces the
		 *  per-cell masses on the CPU, which is what lets it take a max of per-parent sums
		 *  -- a quantity no single thread group can see.
		 *
		 *  A permutation rather than separate shader classes: one parameter struct, one
		 *  IMPLEMENT_GLOBAL_SHADER, one entry point, and no chance of them drifting apart
		 *  in what they bind. */
		class FPassDim : SHADER_PERMUTATION_INT("UNIVERSE_ENTITYGEN_PASS", 2);

	using FPermutationDomain = TShaderPermutationDomain<FPassDim>;

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
	{
		return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5);
	}

	static constexpr uint32 ThreadGroupSize = 64;

	/** Pass indices, named so the dispatch site does not read as magic numbers. */
	static constexpr int32 PassProbe = 0;
	static constexpr int32 PassGenerate = 1;

	/** Probes per cell: ThreadGroupSize lanes over two rounds. MUST equal
	 *  UNIVERSE_ENTITYGEN_PROBES in the .usf, which has no way to share this.
	 *
	 *  TWO ROUNDS RATHER THAN ONE because this field's peaks are thin filaments in a
	 *  mostly-empty volume and a single round misses them at any cell size: cells a seventh
	 *  of a field cell across still under-estimate their peak by an order of magnitude. Four
	 *  rounds drives clipping to about one cell in a hundred and leaves ninety percent of the
	 *  dispatch probing, which is the wrong balance; judge any change by the C/P ratio.
	 *
	 *  Also the divisor the CPU uses to turn the evaluated-candidate counter into the
	 *  candidates-per-probe ratio.
	 *
	 *  THE RATIO MATTERS MORE HERE THAN IN THE GALAXY. Probes and candidates are both
	 *  field evaluations, and a universe field evaluation is expensive enough that the
	 *  subdivision crossover sits a level or two below the galaxy's. Watch it from the
	 *  first run. */
	static constexpr int32 ProbesPerCell = 128;
};

/** One in-flight dispatch. Owns its readback fence and its result.
 *
 *  ASYNCHRONOUS BY CONSTRUCTION. A tier callback cannot fill a particle buffer and return a
 *  count in one call, because the answer does not exist until the GPU has run and the copy
 *  has landed. Poll IsReady() over subsequent frames and NEVER BLOCK ON IT: a synchronous
 *  readback stalls the render thread for the entire pipeline depth, which is precisely the
 *  cost this path exists to avoid. */
class FUniverseEntityGenRequest
{
public:
	/** Records in the shared entity buffer, and the number of cells requested.
	 *
	 *  EntityCapacity is a budget for the whole dispatch, not a per-cell run. Counts is
	 *  NumCells * 5 + 1 elements; the extra is the global entity append cursor. */
	int32 EntityCapacity = 0;
	int32 NumCells = 0;
	int32 NumSlots = 0;

	/** True once there is something to consume, or once we know there never will be.
	 *
	 *  bSubmitted IS LOAD-BEARING. The readbacks are allocated on the calling thread but
	 *  the copies are enqueued later, on the render thread, and a readback that has not
	 *  been enqueued yet reports not-ready forever rather than erroring. */
	bool IsReady() const
	{
		if (bAborted)
		{
			return true;
		}

		return bSubmitted
			&& Readback.IsValid() && CountReadback.IsValid()
			&& Readback->IsReady() && CountReadback->IsReady();
	}

	bool Failed() const { return bAborted; }

	/** Set on the render thread once the copies are enqueued, read on the waiting worker;
	 *  and set on the game thread if the dispatch could not be issued at all. Atomic
	 *  because those are three different threads. */
	std::atomic<bool> bSubmitted{ false };
	std::atomic<bool> bAborted{ false };

	/** Set once the render thread has copied the staging buffers into the arrays below.
	 *  Lock() maps a staging buffer through the RHI and is render-thread only, so the
	 *  waiting worker cannot read the results itself -- it polls the fence, then hands the
	 *  copy to the render thread and waits again. */
	std::atomic<bool> bCopied{ false };

	/** Destinations for that copy. Owned by the request rather than by the caller because
	 *  the render thread writes them after the caller has stopped touching anything. */
	TArray<FUniverseEntityOut> Entities;
	TArray<uint32> Counts;

	// DO NOT ADD A CONSUMER THAT READS THE READBACKS DIRECTLY. Lock() maps through the RHI
	// and is render-thread only, so any such call crashes on a worker.
	// GenerateBatchBlocking marshals the copy to the render thread; a non-blocking
	// consumer has to do the same.

	TUniquePtr<FRHIGPUBufferReadback> Readback;

	/** Accepted count per cell. Separate from the entity buffer because the CPU has to
	 *  know how much of each cell's run is live, and reading a whole SlotCapacity-wide run
	 *  to find out would put back the traffic the compaction removed. */
	TUniquePtr<FRHIGPUBufferReadback> CountReadback;
};

namespace UniverseEntityGen
{
	/** Give up on the readback after this long.
	 *
	 *  A SAFETY VALVE, not a tuning knob, which is why it is a constant rather than a
	 *  UPROPERTY: if the render thread is blocked -- synchronous load, hitch, PIE teardown
	 *  -- the fence never lands, and a background worker spinning forever is a hang with no
	 *  stack pointing at the cause. Nothing about a particular sector makes the right
	 *  answer different, and exposing it invites tuning a deadlock detector instead of
	 *  fixing the deadlock. */
	inline constexpr double ReadbackTimeoutSeconds = 2.0;

	/** COUNTER BUFFER LAYOUT, in one place.
	 *
	 *  Five slots per cell -- accepted, candidates evaluated, envelope, largest candidate
	 *  density, cell mass -- then one global past the end: the entity append cursor.
	 *
	 *  DERIVE EVERY SIZE FROM THESE, never from a spelled-out expression. A buffer sized
	 *  smaller than the copy asks for runs the copy off the end and lands the shader's
	 *  global write outside the view -- and RDG's buffer POOLING masks it, handing back
	 *  oversized allocations left over from bigger dispatches, so it surfaces as an
	 *  out-of-bounds assert with no visible connection to the layout.
	 *
	 *  UniverseEntityGen.usf indexes the same layout with literals and cannot share these.
	 *  Change one and change the other. */
	inline constexpr int32 CountersPerCell = 5;

	/** The global entity append cursor, then one accepted-count per SLOT.
	 *
	 *  The per-slot counters are the shader's own ceiling as well as a diagnostic, so their
	 *  number is part of the buffer layout rather than optional instrumentation. */
	inline constexpr int32 CountGlobals = 1;

	/** The density the mass integral is expressed against: mass_i is the cell's mean of
	 *  (density / anchor)^exponent.
	 *
	 *  NOT A TUNING KNOB, AND NOT AUTHORED. It cancels out of the result entirely.
	 *  Calibration solves BudgetScale = capacity / sum(mass), so scaling the anchor scales
	 *  every mass by the same factor and BudgetScale by its inverse; the accepted count per
	 *  cell is unchanged. Only the logged scale moves.
	 *
	 *  It survives as a NUMERICAL NORMALISER, and this layer wants a different value from
	 *  the galaxy's. That field peaks in the high hundreds; this one is bounded by the sum of
	 *  its three resolved maxima -- about 3 at the default ranges -- so an anchor of ten would
	 *  push every mass to a small fraction and, raised to a spawn exponent above 1, toward
	 *  denormals. One keeps the masses near unity, which is all the anchor is for. */
	inline constexpr float kBudgetAnchor = 1.0f;

	/** How the calibration sample is drawn. All three are about the SAMPLE, not the field.
	 *
	 *  A CONTIGUOUS BLOCK SIZED IN TIER CELLS DOES NOT WORK HERE, which is the mistake these
	 *  replace. A neighbourhood-shaped block spans (2R+1) tier cells, and a tier cell shrinks
	 *  by four per grid depth -- so the Large tier's block covered nearly seven field cells
	 *  while the Small tier's covered four tenths of ONE. That tier was calibrating against a
	 *  single point of the web and treating it as the whole universe, which is why it
	 *  over-delivered by two orders of magnitude while the coarser tiers were fine.
	 *
	 *  SCATTERED, NOT CONTIGUOUS. What calibration needs is the DISTRIBUTION of cell masses,
	 *  not a picture of one region, so cells drawn at random across several field cells beat
	 *  a solid block of the same count -- and the cost stays fixed per tier instead of
	 *  exploding for the fine ones. */
	inline constexpr int32 kCalibrationParents = 96;

	/** How wide the scatter is, in FIELD cells -- the unit the field's structure is actually
	 *  expressed in. Wide enough to cross several walls and voids; every extra cell of span
	 *  is free, since the sample count is fixed. */
	inline constexpr float kCalibrationSpanFieldCells = 8.0f;

	/** Which point of the sorted per-parent masses the constant divides by.
	 *
	 *  NOT THE MAXIMUM, and this is a judgement rather than an optimisation. The field's
	 *  dynamic range is far wider than a slot can represent: a filament threading a fine
	 *  tier's cell is genuinely hundreds of times denser than a void, and no single constant
	 *  gives that cell room without leaving the voids at zero. Something has to clip.
	 *
	 *  Dividing by the max puts the knee above everything, so nothing clips and nothing but
	 *  the densest cells has a visible population. Dividing by a high percentile puts it just
	 *  below the tail: the top couple of percent of cells pin at capacity, and every other
	 *  cell tracks density honestly. The second is the better picture, and it is a deliberate
	 *  choice rather than a failure to reach the first. */
	inline constexpr float kCalibrationPercentile = 0.98f;

	inline constexpr int32 CountElementsFor(int32 InNumCells, int32 InNumSlots)
	{
		return InNumCells * CountersPerCell + CountGlobals + InNumSlots;
	}

	/** Entities actually WRITTEN, and the count to read the entity array against. Bounded by
	 *  capacity, because the shader caps each slot and the per-slot ceilings sum to it. */
	inline constexpr int32 GlobalCursorIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell;
	}

	/** One slot's TRUE demand, which deliberately over-counts past SlotCapacity. The ratio
	 *  against capacity is how far the tier's calibrated constant is out, and it is the only
	 *  observer of that -- a slot that is merely full looks identical whether it wanted one
	 *  more entity or twenty times more. */
	inline constexpr int32 SlotCursorIndex(int32 InNumCells, int32 InSlot)
	{
		return InNumCells * CountersPerCell + CountGlobals + InSlot;
	}

	/** Weigh every cell of a tier's grid, so its placement constant can be solved.
	 *
	 *  Runs the probe pass and returns the PER-CELL masses, one per entry of InCells and in
	 *  the same order. It does not reduce them: which reduction the capacity divides by
	 *  depends on how the tier maps cells to slots, that is the caller's business, and a max
	 *  of per-parent sums is not something a single thread group can see.
	 *
	 *  ONCE PER TIER, not per batch. Dividing a target across whatever cells were in a batch
	 *  made a cell's yield depend on which neighbours streamed in with it: a void
	 *  neighbourhood came back as densely populated as a filament, and burned several times
	 *  the candidates doing it.
	 *
	 *  WHETHER THIS LAYER NEEDS IT AT ALL IS AN OPEN QUESTION. The universe field is
	 *  statistically homogeneous -- one parameter set everywhere, with regional variance
	 *  bounded by the authored ranges -- so cell masses may vary little enough that
	 *  BudgetScale could simply be authored. The lattice crossover band is the reason to
	 *  doubt that: mean density peaks around 0.30 there against 0.06 at either end, which is
	 *  a fivefold swing. Measure the spread across a calibration run before deciding; the
	 *  masses come back either way.
	 *
	 *  BACKGROUND THREAD ONLY, like GenerateBatchBlocking, and for the same reason. */
	bool CalibrateBlocking(
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FUniverseGenCell>& InCells,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		TArray<float>& OutCellMass);

	/** Dispatch, wait, and return the placed entities.
	 *
	 *  CALL FROM A BACKGROUND THREAD ONLY. It blocks, which is safe here and nowhere else:
	 *  tier generation already runs on AnyBackgroundHiPriTask inside a ParallelFor, so the
	 *  wait stalls a worker rather than a frame. Calling it from the game or render thread
	 *  deadlocks against the very work it is waiting on.
	 *
	 *  ONE dispatch for the whole batch, never one per slot -- per-slot dispatches serialise
	 *  on a GPU round-trip each and the latency compounds across a neighbourhood.
	 *
	 *  Returns false on timeout, leaving OutEntities untouched. THERE IS NO CPU PATH BEHIND
	 *  THIS, and for this layer that is structural rather than incidental: the shim stubs
	 *  texture fetches to their neutral, and this field's geometry depends on those fetches.
	 *  The caller blanks the affected slots rather than filling them another way.
	 *
	 *  RETURNS FALSE IMMEDIATELY if InFieldTextures is not complete. All four or none --
	 *  see FUniverseFieldTextures. */
	bool GenerateBatchBlocking(
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FUniverseGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InNumSlots,
		int32 InSlotCapacity,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		float InBudgetScale,
		TArray<FUniverseEntityOut>& OutEntities,
		TArray<uint32>& OutCounts);

	/** Enqueue the passes and a readback copy.
	 *
	 *  bInCalibrateOnly runs the probe pass alone -- the per-cell masses without any
	 *  entities, which is what CalibrateBlocking wants. Otherwise it runs probe then
	 *  generate, carrying the calibrated constant in as a uniform. */
	void Dispatch(
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		TArray<FUniverseGenCell> InCells,
		int32 InEntityCapacity,
		int32 InNumSlots,
		int32 InSlotCapacity,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FUniverseEntityGenRequest> OutRequest);
}
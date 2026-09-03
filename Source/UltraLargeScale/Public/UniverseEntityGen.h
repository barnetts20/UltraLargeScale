// UniverseEntityGen.h
// Global shader + RDG dispatch for GPU entity placement on the universe layer.
// Build.cs needs: "RenderCore", "RHI", "Renderer", "Projects".
//
// A near-twin of GalaxyEntityGen. What differs: placement lives in UniversePlacement.ush
// rather than the field core; there is no VIEW offset, since the field is static in caller
// space, so each group passes its own cell instead; and a field sample costs roughly fifteen
// to twenty-five times a galaxy sample, so GenerationSubdivision wants to sit lower.

#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"
#include "RHIGPUReadback.h"

#include <atomic>

#include "UniverseParams.h"
#include "FTierStreamingSystem.h"

/** One placed entity, as the dispatch writes it. EXACTLY TWO uint4s: UniverseEntityGen.usf
 *  writes OutEntities[Base + 0] and [Base + 1] from this layout, and there is no shared
 *  declaration, so the two are kept in step by this comment and the assert below.
 *
 *  An INTEGER buffer carrying the floats as bit patterns, not the reverse -- a float's bit
 *  pattern is always a valid integer, while an arbitrary integer read as a float can be a
 *  denormal or a NaN. Keep the record a flat multiple of 16 with no implicit tail padding: a
 *  member straddling a 16-byte boundary packs differently on some backends, silently. */
struct FUniverseEntityOut
{
	/** CALLER UNITS, RELATIVE TO THE OWNING CELL'S CENTRE -- not an absolute position, which
	 *  would narrow a coordinate tracking VirtualTraversal into a float32 on the way back and
	 *  lose precision the CPU cannot recover. The scatter adds the cell's own double-precision
	 *  centre back. See FUniverseGenCell. */
	FVector3f Pos = FVector3f::ZeroVector;
	float Extent = 0.0f;

	/** Three decorrelated uniforms in [0,1), ten bits each. Read through DecodeDecor. */
	uint32 DecorPacked = 0;

	/** The candidate index that produced this entity, NOT its position in the buffer. The
	 *  dispatch compacts with a global atomic, so storage ORDER is scheduling dependent while
	 *  the set, and each entity's identity within it, is not. */
	uint32 Slot = 0;

	/** Index into the batch's cell array. Required, not diagnostic: entities are appended to
	 *  one shared buffer, so position says nothing about which cell, and therefore which
	 *  SLOT, an entity belongs to. */
	uint32 CellIndex = 0;

	/** Raw field value where the entity landed, before any spawn shaping. THE UNFADED VALUE:
	 *  the raymarch multiplies by BoundsFade after sampling and SampleAtPosition does not, so
	 *  this is the field itself rather than the field as seen from the player. */
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

/** Mirrors FUniverseGenCell in UniverseEntityGen.usf. GEOMETRY ONLY -- a centre, a half
 *  extent, and the grid coord that keys the cell. NOTHING ABOUT THE FIELD may be added: the
 *  shader probes each cell and derives its own envelope and budget, and anything the CPU had
 *  to fill in would drag density evaluation onto a path that cannot reach the volume texture.
 *
 *  THE CENTRE IS A CELL AND A FRACTION, NOT A POSITION, and it must stay that way. Caller
 *  space tracks VirtualTraversal without bound, and an absolute centre does not produce a
 *  wrong position -- it produces a DEAD TIER: probes are built as centre +/- extent, so once
 *  the extent falls below the centre's own ulp every probe collapses onto the same float and
 *  the tier places nothing while the raymarch still draws the field. CentreCell IS REDUCED
 *  MODULO THE FIELD PERIOD by the caller, since it crosses through a float3; reducing by a
 *  DIFFERENT period from the one the core derives puts placement and the material on wraps
 *  that disagree. See UniverseCellWrap::FieldCellPeriod.
 *
 *  The padding holds the record at 48 bytes and splits it so no vector straddles a
 *  sixteen-byte boundary; HalfExtent sits between the two vectors to keep that true. */
struct FUniverseGenCell
{
	/** Whole field cells containing this cell's centre, wrapped into [0, period). */
	FIntVector3 CentreCell = FIntVector3(0, 0, 0);

	/** Half extent in CALLER units. Bounded by the grid cell size, so it never had a precision
	 *  problem of its own -- only the centre it was added to did. */
	float HalfExtent = 0.0f;

	/** Remainder of the centre within CentreCell, per axis, in [0,1). */
	FVector3f CentreFrac = FVector3f::ZeroVector;

	/** Which slot region of the particle buffer this cell's entities belong to; every child of
	 *  a subdivided streamed cell carries its parent's. THE SHADER NEEDS IT, not just the CPU
	 *  -- without it the dispatch can only compact globally, and an over-full batch then keeps
	 *  whichever entities arrived first, which is scheduling order.
	 *
	 *  SITS BETWEEN THE TWO VECTORS, like HalfExtent, to keep the third vector off the 16-byte
	 *  boundary at 32. */
	uint32 SlotIndex = 0;

	/** The placement key, and the ONE coordinate here that is not wrapped. It seeds every
	 *  candidate draw and, through FVoxelData::ComposeSeed, every entity, so generation stays
	 *  aperiodic while the density field repeats. */
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

		/** Reciprocal of the RAY MARCH PROXY'S half extent -- not the sector extent, and not
		 *  anything about the tier grid. The material marches a box of that size, so that is
		 *  what one unit of normalized space means. */
		SHADER_PARAMETER(float, InvFieldExtent)

		SHADER_PARAMETER(float, BudgetAnchor)
		SHADER_PARAMETER(float, EnvelopePad)

		SHADER_PARAMETER(float, PlaceSpawnExponent)
		SHADER_PARAMETER(float, PlaceExtentMin)
		SHADER_PARAMETER(float, PlaceExtentMax)
		SHADER_PARAMETER(float, PlaceExtentExponent)

		// THE FOUR FIELD VOLUMES, matching the .usf declarations and the material's Custom node
		// pin for pin: two UNORM multinoise volumes for the region axes, two signed vector
		// volumes for the warp octaves. Four samplers rather than one shared, so one asset can
		// be given different addressing without touching the other three.
		SHADER_PARAMETER_TEXTURE(Texture3D, VarianceTexA)
		SHADER_PARAMETER_SAMPLER(SamplerState, VarianceTexASampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, VarianceTexB)
		SHADER_PARAMETER_SAMPLER(SamplerState, VarianceTexBSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexLarge)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexLargeSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexSmall)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexSmallSampler)

		// Raw field inputs, in MakeUniverseDensityParams order, written by
		// FUniverseDensityParams::FillShaderParameters. Derived in the shader rather than
		// uploaded as a packed struct -- that would have to match HLSL packing against the C++
		// layout member for member, and one straddling float3 shifts everything after it.
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
		 *  Generation runs both. CALIBRATION runs the probe pass alone and reduces the per-cell
		 *  masses on the CPU, which is what lets it take a percentile of per-parent sums -- a
		 *  quantity no single thread group can see. */
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
	 *  Two rounds rather than one because this field's peaks are thin filaments in a
	 *  mostly-empty volume: cells a seventh of a field cell across still under-estimate their
	 *  peak by an order of magnitude. Four rounds drives clipping to one cell in a hundred but
	 *  leaves ninety percent of the dispatch probing. Also the divisor behind the
	 *  candidates-per-probe ratio -- judge any change by that ratio. */
	static constexpr int32 ProbesPerCell = 128;
};

/** One in-flight dispatch. Owns its readback fence and its result.
 *
 *  ASYNCHRONOUS BY CONSTRUCTION: the answer does not exist until the GPU has run and the copy
 *  has landed. Poll IsReady() over subsequent frames and NEVER BLOCK ON IT -- a synchronous
 *  readback stalls the render thread for the entire pipeline depth. */
class FUniverseEntityGenRequest
{
public:
	/** Records in the shared entity buffer, and the number of cells requested. EntityCapacity
	 *  is a budget for the whole dispatch, not a per-cell run; for Counts see CountElementsFor. */
	int32 EntityCapacity = 0;
	int32 NumCells = 0;
	int32 NumSlots = 0;

	/** True once there is something to consume, or once we know there never will be.
	 *
	 *  bSubmitted IS LOAD-BEARING: the readbacks are allocated on the calling thread but the
	 *  copies are enqueued later on the render thread, and a readback that has not been
	 *  enqueued yet reports not-ready forever rather than erroring. */
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

	/** Set on the render thread once the copies are enqueued and read on the waiting worker;
	 *  bAborted is set on the game thread if the dispatch could not be issued at all. Atomic
	 *  because those are three different threads. */
	std::atomic<bool> bSubmitted{ false };
	std::atomic<bool> bAborted{ false };

	/** Set once the render thread has copied the staging buffers into the arrays below. Lock()
	 *  maps a staging buffer through the RHI and is render-thread only, so the waiting worker
	 *  polls the fence, hands the copy to the render thread, and waits again. */
	std::atomic<bool> bCopied{ false };

	/** Destinations for that copy. Owned by the request rather than the caller, because the
	 *  render thread writes them after the caller has stopped touching anything. */
	TArray<FUniverseEntityOut> Entities;
	TArray<uint32> Counts;

	// DO NOT ADD A CONSUMER THAT READS THE READBACKS DIRECTLY: Lock() maps through the RHI and
	// is render-thread only, so any such call crashes on a worker. GenerateBatchBlocking
	// marshals the copy to the render thread, and a non-blocking consumer must do the same.

	TUniquePtr<FRHIGPUBufferReadback> Readback;

	/** Accepted count per cell. Separate from the entity buffer because the CPU has to know how
	 *  much of each cell's run is live, and reading a whole SlotCapacity-wide run to find out
	 *  would put back the traffic the compaction removed. */
	TUniquePtr<FRHIGPUBufferReadback> CountReadback;
};

namespace UniverseEntityGen
{
	/** Give up on the readback after this long. A SAFETY VALVE, not a tuning knob: if the render
	 *  thread is blocked the fence never lands, and a worker spinning forever is a hang. */
	inline constexpr double ReadbackTimeoutSeconds = 2.0;

	/** COUNTER BUFFER LAYOUT, in one place: five slots per cell -- accepted, candidates
	 *  evaluated, envelope, largest candidate density, cell mass -- then the globals.
	 *
	 *  DERIVE EVERY SIZE FROM THESE, never from a spelled-out expression. An undersized buffer
	 *  runs the copy off the end and lands the shader's global write outside the view, and
	 *  RDG's buffer POOLING masks it by handing back oversized allocations left over from
	 *  bigger dispatches. UniverseEntityGen.usf indexes the same layout with literals and
	 *  cannot share these -- change one and change the other. */
	inline constexpr int32 CountersPerCell = 5;

	/** The global entity append cursor, then one accepted-count per SLOT. The per-slot counters
	 *  are the shader's own ceiling as well as a diagnostic, so their number is part of the
	 *  layout rather than optional instrumentation. */
	inline constexpr int32 CountGlobals = 1;

	/** The density the mass integral is expressed against: mass_i is the cell's mean of
	 *  (density / anchor)^exponent.
	 *
	 *  NOT A TUNING KNOB, AND NOT AUTHORED -- it cancels out entirely, since calibration solves
	 *  BudgetScale = capacity / sum(mass) and scaling the anchor scales every mass by the same
	 *  factor and BudgetScale by its inverse. It survives as a NUMERICAL NORMALISER, and wants
	 *  a different value from the galaxy's: this field is bounded near 3 at the default ranges,
	 *  so an anchor of ten drives the masses toward denormals under a spawn exponent above 1. */
	inline constexpr float kBudgetAnchor = 1.0f;

	/** How many parent cells the calibration sample draws. SCATTERED, NOT CONTIGUOUS: what
	 *  calibration needs is the DISTRIBUTION of cell masses, not a picture of one region, so
	 *  cells drawn at random across several field cells beat a solid block of the same count.
	 *  A block sized in TIER cells does not work at all here, since a tier cell shrinks by four
	 *  per grid depth and the finest tier's block would cover a fraction of one field cell. */
	inline constexpr int32 kCalibrationParents = 96;

	/** How wide the scatter is, in FIELD cells -- the unit the field's structure is expressed
	 *  in. Wide enough to cross several walls and voids; extra span is free, since the sample
	 *  count is fixed. */
	inline constexpr float kCalibrationSpanFieldCells = 8.0f;

	/** Which point of the sorted per-parent masses the constant divides by. NOT THE MAXIMUM,
	 *  deliberately: the field's dynamic range is wider than a slot can represent, so something
	 *  has to clip. Dividing by the max leaves only the densest cells visibly populated, while
	 *  a high percentile pins the top couple of percent at capacity and lets every other cell
	 *  track density honestly. */
	inline constexpr float kCalibrationPercentile = 0.98f;

	inline constexpr int32 CountElementsFor(int32 InNumCells, int32 InNumSlots)
	{
		return InNumCells * CountersPerCell + CountGlobals + InNumSlots;
	}

	/** Entities actually WRITTEN, and the count to read the entity array against. Bounded by
	 *  capacity: the shader caps each slot, and the per-slot ceilings sum to it. */
	inline constexpr int32 GlobalCursorIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell;
	}

	/** One slot's TRUE demand, deliberately over-counting past SlotCapacity. The ratio against
	 *  capacity is how far the tier's calibrated constant is out, and its only observer -- a
	 *  full slot looks the same whether it wanted one more entity or twenty times more. */
	inline constexpr int32 SlotCursorIndex(int32 InNumCells, int32 InSlot)
	{
		return InNumCells * CountersPerCell + CountGlobals + InSlot;
	}

	/** Weigh every cell of a tier's grid, so its placement constant can be solved. Runs the
	 *  probe pass and returns the PER-CELL masses, one per entry of InCells and in the same
	 *  order. It does not reduce them -- which reduction divides the capacity depends on how
	 *  the tier maps cells to slots, and that is the caller's business.
	 *
	 *  CALL ONCE PER TIER, not per batch: reducing over a batch makes a cell's yield depend on
	 *  which neighbours streamed in with it. BACKGROUND THREAD ONLY, as it blocks. */
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
	 *  CALL FROM A BACKGROUND THREAD ONLY. It blocks: calling it from the game or render thread
	 *  deadlocks against the very work it is waiting on.
	 *
	 *  ONE dispatch for the whole batch, never one per slot -- per-slot dispatches serialise on
	 *  a GPU round-trip each and the latency compounds across a neighbourhood.
	 *
	 *  Returns false on timeout, leaving OutEntities untouched; the caller must blank the
	 *  affected slots, since there is no CPU path behind this. Returns false IMMEDIATELY if
	 *  InFieldTextures is not complete -- all four or none, see FUniverseFieldTextures. */
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
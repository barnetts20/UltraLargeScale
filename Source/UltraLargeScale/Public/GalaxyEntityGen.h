// GalaxyEntityGen.h
// Global shader + RDG dispatch for GPU entity placement.
//
// Build.cs needs: "RenderCore", "RHI", "Renderer", "Projects".
// The /UltraLargeScale shader path is already registered in FUltraLargeScaleModule.

#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"
#include "RHIGPUReadback.h"

#include <atomic>

#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"

/** One placed entity, as the dispatch writes it.
 *
 *  EXACTLY TWO uint4s. GalaxyEntityGen.usf writes OutEntities[Base + 0] and [Base + 1]
 *  from this layout; there is no shared declaration, so the two are kept in step by
 *  this comment and the assert below.
 *
 *  AN INTEGER BUFFER carrying the floats as bit patterns, not the reverse. A float's
 *  bit pattern is always a valid integer; an arbitrary integer read as a float can be a
 *  denormal or a NaN, and neither round-trips reliably.
 *
 *  A member that straddles a 16-byte boundary packs differently on some backends and
 *  the failure is silent -- every field after the straddle reads shifted. Keeping the
 *  record a flat multiple of 16 with no implicit tail padding removes the question. */
struct FGalaxyEntityOut
{
	FVector3f Pos = FVector3f::ZeroVector;
	float Extent = 0.0f;

	/** Three decorrelated uniforms in [0,1), ten bits each. Read it through
	 *  DecodeDecor rather than by hand. */
	uint32 DecorPacked = 0;

	/** The candidate index that produced this entity, NOT its position in the buffer.
	 *  The dispatch compacts with a global atomic, so storage ORDER is scheduling
	 *  dependent while the set, and each entity's identity within it, is not. */
	uint32 Slot = 0;

	/** Index into the batch's cell array. Required, not diagnostic: entities are
	 *  appended to one shared buffer, so position says nothing about which cell -- and
	 *  therefore which SLOT -- an entity belongs to. */
	uint32 CellIndex = 0;

	/** Raw field value where the entity landed, before any spawn shaping. Carried
	 *  because the slot exists, not because anything reads it yet. */
	float Density = 0.0f;

	/** Undoes PackGalaxyDecor in GalaxyPlacement.ush. Keep the two together. */
	FVector3f DecodeDecor() const
	{
		constexpr float Inv = 1.0f / 1023.0f;

		return FVector3f(
			static_cast<float>(DecorPacked & 1023u) * Inv,
			static_cast<float>((DecorPacked >> 10) & 1023u) * Inv,
			static_cast<float>((DecorPacked >> 20) & 1023u) * Inv);
	}
};
static_assert(sizeof(FGalaxyEntityOut) == 32, "FGalaxyEntityOut must match the .usf layout");

/** Mirrors FGalaxyGenCell in GalaxyEntityGen.usf.
 *
 *  GEOMETRY ONLY. A centre, a half extent, and the grid coord that keys the cell.
 *  NOTHING ABOUT THE FIELD may be added here: the shader probes each cell and derives
 *  its own envelope and budget, and anything the CPU had to fill in would drag density
 *  evaluation back onto this side.
 *
 *  The padding holds the record at 48 bytes and splits it so no vector straddles a
 *  sixteen-byte boundary. */
struct FGalaxyGenCell
{
	/** AN ABSOLUTE POSITION, which the universe's equivalent deliberately is not.
	 *
	 *  FUniverseGenCell carries a cell index plus a fraction because universe caller space
	 *  tracks VirtualTraversal without bound: a float32 centre out there has an ulp wider
	 *  than the cell, every probe collapses onto one point, and the tier places nothing.
	 *
	 *  THE GALAXY IS A BOUNDED FIELD and cannot reach that. AGalaxyActor builds its octree
	 *  at Params.Extent and never rebases, and IsPlayerInsideBounds despawns the actor once
	 *  VirtualTraversal leaves that box, so a cell centre stays within a small multiple of
	 *  Extent -- around 7e8 at the shipped scale, where float32's ulp is about 64 against a
	 *  subdivided cell half-extent of 4e7. Six orders of margin.
	 *
	 *  THE EXEMPTION IS THE BOUND, NOT THE TYPE. Give a galaxy an unbounded traversal -- a
	 *  rebase, or bounds that stop despawning -- and this becomes the universe's bug with
	 *  the universe's symptom: generation stops silently while the raymarch carries on
	 *  drawing the field. */
	FVector3f Centre = FVector3f::ZeroVector;
	float HalfExtent = 0.0f;
	FIntVector3 Coord = FIntVector3(0, 0, 0);
	uint32 Pad[5] = { 0, 0, 0, 0, 0 };
};
static_assert(sizeof(FGalaxyGenCell) == 48, "FGalaxyGenCell must match the .usf layout");

class FGalaxyEntityGenCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FGalaxyEntityGenCS);
	SHADER_USE_PARAMETER_STRUCT(FGalaxyEntityGenCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint4>, OutEntities)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutCounts)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGalaxyGenCell>, InCells)

		SHADER_PARAMETER(uint32, NumCells)
		SHADER_PARAMETER(uint32, EntityCapacity)
		SHADER_PARAMETER(uint32, DispatchGroupsX)
		SHADER_PARAMETER(float, BudgetScale)
		SHADER_PARAMETER(int32, KeySeed)
		SHADER_PARAMETER(float, InvGalaxyExtent)
		SHADER_PARAMETER(float, BudgetAnchor)
		SHADER_PARAMETER(float, EnvelopePad)

		SHADER_PARAMETER(float, PlaceSpawnExponent)
		SHADER_PARAMETER(float, PlaceExtentMin)
		SHADER_PARAMETER(float, PlaceExtentMax)
		SHADER_PARAMETER(float, PlaceExtentExponent)

		// THE FOUR FIELD VOLUMES, matching the .usf declarations and the material's Custom
		// node pin for pin. Two signed vector volumes for the warp fetches, two UNORM
		// multinoise volumes for the modulation fetches.
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexDisc)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexDiscSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, WarpTexHalo)
		SHADER_PARAMETER_SAMPLER(SamplerState, WarpTexHaloSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, NoiseTexDisc)
		SHADER_PARAMETER_SAMPLER(SamplerState, NoiseTexDiscSampler)
		SHADER_PARAMETER_TEXTURE(Texture3D, NoiseTexHalo)
		SHADER_PARAMETER_SAMPLER(SamplerState, NoiseTexHaloSampler)

		// Raw field inputs, in MakeGalaxyDensityParams order. Deriving in the shader
		// rather than uploading a packed GalaxyDensityParams: a derived struct in a
		// constant buffer would have to match HLSL packing against the C++ layout
		// member for member, and one straddling float3 shifts everything after it
		// with no diagnostic.
		SHADER_PARAMETER(FVector4f, InLateralScale)
		SHADER_PARAMETER(FVector4f, InVerticalScale)
		SHADER_PARAMETER(FVector4f, InLayerDensity)
		SHADER_PARAMETER(FVector4f, InNoiseAmount)
		SHADER_PARAMETER(FVector4f, InWarpAmount)
		SHADER_PARAMETER(FVector4f, InArmAsym)
		SHADER_PARAMETER(FVector4f, InSpiralTwist)
		SHADER_PARAMETER(FVector3f, InCentralVoid)
		SHADER_PARAMETER(FVector4f, InNoiseScale)
		SHADER_PARAMETER(FVector4f, InWarpScale)
		SHADER_PARAMETER(FVector4f, InNoiseChannelWeights)
		SHADER_PARAMETER(FVector3f, InNoiseOffset)
		SHADER_PARAMETER(float, InBoundsFadeStart)
		SHADER_PARAMETER(float, InDiscScaleLengthRatio)
		SHADER_PARAMETER(float, InDiscVerticalFalloff)
		SHADER_PARAMETER(float, InDiscFlare)
		SHADER_PARAMETER(float, InDiscWarpAmplitude)
		SHADER_PARAMETER(float, InDiscWarpPhase)
		SHADER_PARAMETER(float, InDiscWarpTwist)
		SHADER_PARAMETER(float, InDiscLopsidedAmount)
		SHADER_PARAMETER(float, InDiscLopsidedPhase)
		SHADER_PARAMETER(float, InArmCount)
		SHADER_PARAMETER(float, InArmAsymSeed)
		SHADER_PARAMETER(float, InArmProfileExponent)
		SHADER_PARAMETER(float, InArmRadialGrowth)
		SHADER_PARAMETER(float, InArmHostFalloff)
		SHADER_PARAMETER(float, InBulgeConcentration)
		SHADER_PARAMETER(float, InBackgroundConcentration)
		SHADER_PARAMETER(float, InNoiseEnable)
		SHADER_PARAMETER(FVector4f, InFieldOrientation)
		END_SHADER_PARAMETER_STRUCT()

		/** Which pass this permutation compiles.
		 *
		 *    0 PROBE    -- one group per cell: cull, envelope, and weigh it
		 *    1 GENERATE -- one group per cell: draw and place candidates
		 *
		 *  Generation runs both. CALIBRATION runs the probe pass alone and reduces the
		 *  per-cell masses on the CPU, which is what lets it take a max of per-parent
		 *  sums -- a quantity no single thread group can see.
		 *
		 *  Calibrating per tier rather than per batch is the point. Accepted count per cell
		 *  is BudgetScale x mass_i, so dividing a target across whatever cells happened to be
		 *  in a batch made a cell's yield depend on its neighbours -- a void neighbourhood
		 *  came back as full as an arm, and cost more candidates to produce.
		 *
		 *  A permutation rather than separate shader classes: one parameter struct, one
		 *  IMPLEMENT_GLOBAL_SHADER, one entry point, and no chance of them drifting apart in
		 *  what they bind. */
		class FPassDim : SHADER_PERMUTATION_INT("GALAXY_ENTITYGEN_PASS", 2);

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
	 *  GALAXY_ENTITYGEN_PROBES in the .usf, which has no way to share this.
	 *
	 *  TWO ROUNDS RATHER THAN ONE because a galaxy's features are thin -- a disc a few
	 *  percent of the extent thick, arms narrow in azimuth inside it -- so the structure
	 *  occupies a small fraction of any cell and a single round misses it. A cell whose
	 *  probes all miss reports a zero envelope and places nothing, leaving a cell-shaped
	 *  hole with the raymarch still drawing structure through it.
	 *
	 *  Also the divisor the CPU uses to turn the evaluated-candidate counter into the
	 *  candidates-per-probe ratio; judge any change by that ratio and the live-cell count
	 *  together. */
	static constexpr int32 ProbesPerCell = 128;
};

/** One in-flight dispatch. Owns its readback fence and its result.
 *
 *  ASYNCHRONOUS BY CONSTRUCTION. A tier callback cannot fill a particle buffer and
 *  return a count in the same call, because the
 *  answer does not exist until the GPU has run and the copy has landed. Poll
 *  IsReady() over subsequent frames and never block on it: a synchronous readback
 *  stalls the render thread for the entire pipeline depth, which is precisely the
 *  cost this migration is supposed to be buying us out of. */
class FGalaxyEntityGenRequest
{
public:
	/** Records in the shared entity buffer, and the number of cells requested.
	 *
	 *  EntityCapacity is a budget for the whole dispatch, not a per-cell run. Counts is
	 *  NumCells * 5 + 1 elements; the extra is the global entity append cursor. */
	int32 EntityCapacity = 0;
	int32 NumCells = 0;

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

	/** Set on the render thread once the copies are enqueued, read on the waiting
	 *  worker; and set on the game thread if the dispatch could not be issued at all.
	 *  Atomic because those are three different threads. */
	std::atomic<bool> bSubmitted{ false };
	std::atomic<bool> bAborted{ false };

	/** Set once the render thread has copied the staging buffers into the arrays
	 *  below. Lock() maps a staging buffer through the RHI and is render-thread only,
	 *  so the waiting worker cannot read the results itself -- it polls the fence,
	 *  then hands the copy to the render thread and waits again. */
	std::atomic<bool> bCopied{ false };

	/** Destinations for that copy. Owned by the request rather than by the caller
	 *  because the render thread writes them after the caller has stopped touching
	 *  anything. */
	TArray<FGalaxyEntityOut> Entities;
	TArray<uint32> Counts;

	// DO NOT ADD A CONSUMER THAT READS THE READBACKS DIRECTLY. Lock() maps through the
	// RHI and is render-thread only, so any such call crashes on a worker.
	// GenerateBatchBlocking marshals the copy to the render thread; a non-blocking
	// consumer has to do the same.

	TUniquePtr<FRHIGPUBufferReadback> Readback;

	/** Accepted count per cell. Separate from the entity buffer because the CPU has
	 *  to know how much of each cell's run is live, and reading a whole
	 *  SlotCapacity-wide run to find out would put back the traffic the compaction
	 *  removed. */
	TUniquePtr<FRHIGPUBufferReadback> CountReadback;
};

namespace GalaxyEntityGen
{
	/** Give up on the readback after this long.
	 *
	 *  A SAFETY VALVE, not a tuning knob, which is why it is a constant rather than a
	 *  UPROPERTY: if the render thread is blocked -- synchronous load, hitch, PIE
	 *  teardown -- the fence never lands, and a background worker spinning forever is a
	 *  hang with no stack pointing at the cause. Nothing about a particular galaxy
	 *  makes the right answer different, and exposing it invites tuning a deadlock
	 *  detector instead of fixing the deadlock.
	 *
	 *  Two seconds is generous for a dispatch measured in microseconds. If this is ever
	 *  hit on healthy hardware the timeout is not the problem. */
	inline constexpr double ReadbackTimeoutSeconds = 2.0;

	/** Weigh every cell of a tier's grid, so its placement constant can be solved.
	 *
	 *  Runs the probe pass and returns the PER-CELL masses, one per entry of InCells and
	 *  in the same order. It does not reduce them: which reduction the capacity divides
	 *  by depends on how the tier maps cells to slots, that is the caller's business, and
	 *  a max of per-parent sums is not something a single thread group can see.
	 *
	 *  ONCE PER TIER, not per batch. Dividing a target across whatever cells were in a
	 *  batch made a cell's yield depend on which neighbours streamed in with it: a void
	 *  neighbourhood came back as densely populated as an arm, and burned several times
	 *  the candidates doing it.
	 *
	 *  BACKGROUND THREAD ONLY, like GenerateBatchBlocking, and for the same reason. */
	bool CalibrateBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		TArray<float>& OutCellMass);

	/** Dispatch, wait, and return the placed entities.
	 *
	 *  CALL FROM A BACKGROUND THREAD ONLY. It blocks, which is safe here and nowhere
	 *  else: tier generation already runs on AnyBackgroundHiPriTask inside a ParallelFor,
	 *  so the wait stalls a worker rather than a frame. Calling it from the game or
	 *  render thread deadlocks against the very work it is waiting on.
	 *
	 *  ONE dispatch for the whole batch, never one per slot -- per-slot dispatches
	 *  serialise on a GPU round-trip each and the latency compounds across a
	 *  neighbourhood.
	 *
	 *  Returns false on timeout, leaving OutEntities untouched. There is no CPU path
	 *  behind this, so the caller blanks the affected slots rather than filling them
	 *  another way -- see GalaxyDataGenerator::GenerateTierBatchGPU. */
	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		float InBudgetScale,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts);

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
	 *  GalaxyEntityGen.usf indexes the same layout with literals and cannot share these.
	 *  Change one and change the other. */
	inline constexpr int32 CountersPerCell = 5;
	inline constexpr int32 CountGlobals = 1;

	/** The density the mass integral is expressed against: mass_i is the cell's mean of
	 *  (density / anchor)^exponent.
	 *
	 *  NOT A TUNING KNOB, AND NOT AUTHORED. It cancels out of the result entirely.
	 *  Calibration solves BudgetScale = capacity / sum(mass), so scaling the anchor
	 *  scales every mass by the same factor and BudgetScale by its inverse; the accepted
	 *  count per cell is unchanged. Only the logged scale moves.
	 *
	 *  It survives as a NUMERICAL NORMALISER. The field peaks in the high hundreds, and
	 *  the mass is a power of it, so at a large SpawnExponent an anchor of 1 would push
	 *  the sum toward the top of float range. Ten keeps it comfortable without mattering
	 *  to anything else. */
	inline constexpr float kBudgetAnchor = 10.0f;

	inline constexpr int32 CountElementsFor(int32 InNumCells)
	{
		return InNumCells * CountersPerCell + CountGlobals;
	}

	/** Total accepted, including anything the entity buffer had no room for. */
	inline constexpr int32 GlobalCursorIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell;
	}

	/** Enqueue the passes and a readback copy.
	 *
	 *  bInCalibrateOnly runs the probe pass alone -- the per-cell masses without any
	 *  entities, which is what CalibrateBlocking wants. Otherwise it runs probe then
	 *  generate, carrying the calibrated constant in as a uniform. */
	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest);
}
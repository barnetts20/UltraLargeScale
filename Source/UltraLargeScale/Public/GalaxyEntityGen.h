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

/** Mirrors FGalaxyEntityOut in GalaxyEntityGen.usf.
 *
 *  48 bytes with explicit padding. The .usf side pads to the same size deliberately:
 *  a structured buffer element whose members straddle a 16-byte boundary packs
 *  differently on some backends, and the failure is silent -- every field after the
 *  straddle reads shifted. Keeping both sides at a flat multiple of 16 with no
 *  implicit tail padding removes the question rather than documenting it. */
struct FGalaxyEntityOut
{
	FVector3f Pos = FVector3f::ZeroVector;
	float Extent = 0.0f;
	FVector3f Decor = FVector3f::ZeroVector;
	float Density = 0.0f;
	/** The candidate index that produced this entity, NOT its position in the buffer.
	 *  The dispatch compacts with a global atomic, so storage ORDER is scheduling
	 *  dependent while the set, and each entity's identity within it, is not.
	 *
	 *  Written as a float by the shader so the whole record is three float4s. */
	float Slot = 0.0f;

	/** Index into the batch's cell array. Required, not diagnostic: entities are
	 *  appended to one shared buffer, so position no longer says which cell -- and
	 *  therefore which SLOT -- an entity belongs to. */
	float CellIndex = 0.0f;

	float Pad[2] = { 0.0f, 0.0f };
};
static_assert(sizeof(FGalaxyEntityOut) == 48, "FGalaxyEntityOut must match the .usf layout");

/** Mirrors FGalaxyGenCell in GalaxyEntityGen.usf.
 *
 *  GEOMETRY ONLY. A centre, a half extent, and the grid coord that keys the cell.
 *  Nothing about the field: the shader probes each cell, derives its own rejection
 *  envelope and its own candidate budget, and the CPU never evaluates the density.
 *
 *  DensityReference and Candidates used to live here. Filling them was the last thing
 *  requiring GalaxyDensityCore.ush to compile as C++, which is why they are gone rather
 *  than merely unused. The padding that replaces them keeps the record at 48 bytes so
 *  the layout assert below still means what it did. */
struct FGalaxyGenCell
{
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
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float4>, OutEntities)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutCounts)
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FGalaxyGenCell>, InCells)

		SHADER_PARAMETER(uint32, NumCells)
		SHADER_PARAMETER(uint32, EntityCapacity)
		SHADER_PARAMETER(float, BudgetScale)
		SHADER_PARAMETER(uint32, ProbeRounds)
		SHADER_PARAMETER(int32, KeySeed)
		SHADER_PARAMETER(float, InvGalaxyExtent)
		SHADER_PARAMETER(float, BudgetAnchor)
		SHADER_PARAMETER(float, EnvelopePad)

		SHADER_PARAMETER(float, PlaceCompression)
		SHADER_PARAMETER(float, PlaceSpawnExponent)
		SHADER_PARAMETER(float, PlaceExtentMin)
		SHADER_PARAMETER(float, PlaceExtentMax)
		SHADER_PARAMETER(float, PlaceExtentExponent)

		SHADER_PARAMETER_TEXTURE(Texture3D, NoiseTex)
		SHADER_PARAMETER_SAMPLER(SamplerState, NoiseTexSampler)

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
		SHADER_PARAMETER(float, InNoiseOctaves)
		SHADER_PARAMETER(float, InNoiseRidged)
		SHADER_PARAMETER(float, InNoiseEnable)
		END_SHADER_PARAMETER_STRUCT()

		/** Which pass this permutation compiles.
		 *
		 *    0 PROBE    -- one group per cell: cull, envelope, and weigh it
		 *    1 REDUCE   -- one group total: total and largest of the weights
		 *    2 GENERATE -- one group per cell: draw and place candidates
		 *
		 *  TWO OF THE THREE RUN PER BATCH. Generation is probe then generate; reduce runs
		 *  only during CALIBRATION, which happens once per tier over its whole grid and
		 *  produces the single constant generation needs.
		 *
		 *  Calibrating per tier rather than per batch is the point. Accepted count per cell
		 *  is BudgetScale x mass_i, so dividing a target across whatever cells happened to be
		 *  in a batch made a cell's yield depend on its neighbours -- a void neighbourhood
		 *  came back as full as an arm, and cost more candidates to produce.
		 *
		 *  A permutation rather than separate shader classes: one parameter struct, one
		 *  IMPLEMENT_GLOBAL_SHADER, one entry point, and no chance of them drifting apart in
		 *  what they bind. */
		class FPassDim : SHADER_PERMUTATION_INT("GALAXY_ENTITYGEN_PASS", 3);

	using FPermutationDomain = TShaderPermutationDomain<FPassDim>;

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
	{
		return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5);
	}

	static constexpr uint32 ThreadGroupSize = 64;

	/** Pass indices, named so the dispatch site does not read as magic numbers. */
	static constexpr int32 PassProbe = 0;
	static constexpr int32 PassReduce = 1;
	static constexpr int32 PassGenerate = 2;
};

/** One in-flight dispatch. Owns its readback fence and its result.
 *
 *  ASYNCHRONOUS BY CONSTRUCTION. The tier callbacks used to fill a particle buffer
 *  and return a count in the same call; that is no longer possible, because the
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
	 *  NumCells * 5 + 3 elements; the extras are the global append cursor and the two
	 *  mass reductions the budget is calibrated from. */
	int32 EntityCapacity = 0;
	int32 NumCells = 0;

	/** True once there is something to consume, or once we know there never will be.
	 *
	 *  bSubmitted matters: the readbacks are allocated on the calling thread but the
	 *  copies are enqueued later, on the render thread. Polling only IsReady() on a
	 *  readback that has not been enqueued yet returns false forever -- which is what
	 *  made the first version fall back to the CPU on nearly every batch. */
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

	// Consume() is gone. It locked the staging buffers on whatever thread called it,
	// and Lock() maps through the RHI and is render-thread only -- so it was a
	// function that crashed if anyone used it. GenerateBatchBlocking marshals the
	// copy to the render thread instead; if a non-blocking consumer is wanted later
	// it has to do the same, not read the readbacks directly.

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

	/** Rounds of sixty-four probes per cell during CALIBRATION.
	 *
	 *  Generation always uses one, because it probes a subcell. Calibration probes the
	 *  whole streamed cell that subcell came from, and sixty-four samples of a cell whose
	 *  structure occupies a few percent of its volume under-reports the mean badly.
	 *
	 *  The error is BIAS, not noise, and always in the same direction: mass too low ->
	 *  scale too high -> generation over-delivers, by more the deeper the subdivision.
	 *  Eight rounds is five hundred and twelve samples per cell, and calibration runs
	 *  once per tier per galaxy, so the cost does not appear in any frame. */
	inline constexpr int32 CalibrationProbeRounds = 8;

	/** Enqueues the dispatch and a readback copy. Safe to call from the game thread;
	 *  the work is deferred to the render thread.
	 *
	 *  InCells carries the placement KEY per cell, not the dispatch index. The large
	 *  tier's active set is sparse after its cull prepass, so cells are supplied
	 *  explicitly rather than derived from the thread id, which would repeat the cull
	 *  on the GPU. It also carries the per-cell DensityReference, which is how the
	 *  large tier's local envelope survives the move. */
	 /** Dispatch, wait, and return the fixed-slot buffer.
	  *
	  *  CALL FROM A BACKGROUND THREAD ONLY. It blocks, which is safe here and nowhere
	  *  else: tier generation already runs on AnyBackgroundHiPriTask inside a
	  *  ParallelFor, so the wait stalls a worker rather than a frame, and the streaming
	  *  system already tolerates generation taking arbitrary time -- that is what the
	  *  cache-miss path exists for. Calling this from the game or render thread would
	  *  deadlock against the very work it is waiting on.
	  *
	  *  ONE dispatch for the whole batch, not one per slot. Per-slot dispatches would
	  *  serialise on a GPU round-trip each, and the latency compounds across a
	  *  neighbourhood.
	  *
	  *  Returns false on timeout, leaving OutEntities untouched. There is no CPU path
	  *  behind this any more, so the caller blanks the affected slots rather than
	  *  filling them another way -- see GalaxyDataGenerator::GenerateTierBatchGPU. */
	  /** Measure a tier's whole grid so its placement constant can be solved.
	   *
	   *  Runs the probe pass over every cell and reduces, returning the TOTAL mass and the
	   *  LARGEST single cell mass. Which one the caller divides capacity by depends on how
	   *  the tier maps cells to slots -- total when they share one, largest when each cell
	   *  owns its own. Generation itself never reduces anything.
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
		UTexture* InNoiseTexture,
		float& OutTotalMass,
		float& OutMaxCellMass);

	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		float InBudgetScale,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts);

	/** COUNTER BUFFER LAYOUT, in one place.
	 *
	 *  Five slots per cell -- accepted, candidates evaluated, envelope, largest candidate
	 *  density, cell mass -- then three globals past the end: the entity append cursor
	 *  and the two mass reductions calibration divides by.
	 *
	 *  Named rather than spelled out because it was spelled out in four places and one of
	 *  them did not move when the layout widened. The buffer stayed at four per cell
	 *  while the copy asked for five plus three, so the copy ran off the end and the
	 *  shader's global writes landed outside the view -- and RDG's buffer POOLING hid it
	 *  for weeks by handing back oversized allocations left over from bigger dispatches.
	 *  It surfaced as an out-of-bounds copy assert with no connection to the layout.
	 *
	 *  GalaxyEntityGen.usf indexes the same layout with literals and cannot share these.
	 *  Change one and change the other. */
	inline constexpr int32 CountersPerCell = 5;
	inline constexpr int32 CountGlobals = 3;

	inline constexpr int32 CountElementsFor(int32 InNumCells)
	{
		return InNumCells * CountersPerCell + CountGlobals;
	}

	/** Total accepted, including anything the entity buffer had no room for. */
	inline constexpr int32 GlobalCursorIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell;
	}

	inline constexpr int32 GlobalTotalMassIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell + 1;
	}

	inline constexpr int32 GlobalMaxMassIndex(int32 InNumCells)
	{
		return InNumCells * CountersPerCell + 2;
	}

	/** Enqueue the passes and a readback copy.
	 *
	 *  bInCalibrateOnly runs probe and reduce and skips generation -- the mass reductions
	 *  without any entities, which is what CalibrateBlocking wants. Otherwise it runs
	 *  probe and generate and skips the reduce, because generation carries its constant
	 *  as a uniform rather than solving for it. */
	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest);
}
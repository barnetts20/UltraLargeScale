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
	 *  The dispatch compacts per cell with an atomic, so the ORDER within a cell is
	 *  nondeterministic while the set, and each entity's identity within it, is not. */
	 /** Written as a float by the shader so the whole record is three float4s. */
	float Slot = 0.0f;
	float Pad[3] = { 0.0f, 0.0f, 0.0f };
};
static_assert(sizeof(FGalaxyEntityOut) == 48, "FGalaxyEntityOut must match the .usf layout");

/** Mirrors FGalaxyGenCell in GalaxyEntityGen.usf. 32 bytes. */
struct FGalaxyGenCell
{
	FVector3f Centre = FVector3f::ZeroVector;
	float HalfExtent = 0.0f;
	FIntVector3 Coord = FIntVector3(0, 0, 0);
	float DensityReference = 1.0f;
	uint32 Candidates = 0;
	uint32 Pad[3] = { 0, 0, 0 };
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
		SHADER_PARAMETER(uint32, CandidateBudget)
		SHADER_PARAMETER(uint32, SlotStride)
		SHADER_PARAMETER(int32, KeySeed)
		SHADER_PARAMETER(float, InvGalaxyExtent)

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

		static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
	{
		return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5);
	}

	static constexpr uint32 ThreadGroupSize = 64;
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
	/** Number of entries reserved per cell, and the number of cells requested. */
	int32 SlotStride = 0;
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
	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		int32 InMaxCandidates,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts);

	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		int32 InMaxCandidates,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest);
}
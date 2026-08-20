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
	uint32 bValid = 0;
	uint32 Pad[3] = { 0, 0, 0 };
};
static_assert(sizeof(FGalaxyEntityOut) == 48, "FGalaxyEntityOut must match the .usf layout");

/** Mirrors FGalaxyGenCell in GalaxyEntityGen.usf. 32 bytes. */
struct FGalaxyGenCell
{
	FVector3f Centre = FVector3f::ZeroVector;
	float HalfExtent = 0.0f;
	FIntVector3 Coord = FIntVector3(0, 0, 0);
	float DensityReference = 1.0f;
};
static_assert(sizeof(FGalaxyGenCell) == 32, "FGalaxyGenCell must match the .usf layout");

class FGalaxyEntityGenCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FGalaxyEntityGenCS);
	SHADER_USE_PARAMETER_STRUCT(FGalaxyEntityGenCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FGalaxyEntityOut>, OutEntities)
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

	bool IsReady() const { return Readback.IsValid() && Readback->IsReady(); }

	/** Compacts valid entries out of the fixed-slot buffer. Only call once IsReady().
	 *  Returns entries in (cell, slot) order, which is stable across visits -- the
	 *  reason the shader writes fixed slots rather than appending. */
	void Consume(TArray<FGalaxyEntityOut>& OutValid);

	TUniquePtr<FRHIGPUBufferReadback> Readback;
};

namespace GalaxyEntityGen
{
	/** Enqueues the dispatch and a readback copy. Safe to call from the game thread;
	 *  the work is deferred to the render thread.
	 *
	 *  InCells carries the placement KEY per cell, not the dispatch index. The large
	 *  tier's active set is sparse after its cull prepass, so cells are supplied
	 *  explicitly rather than derived from the thread id, which would repeat the cull
	 *  on the GPU. It also carries the per-cell DensityReference, which is how the
	 *  large tier's local envelope survives the move. */
	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest);
}
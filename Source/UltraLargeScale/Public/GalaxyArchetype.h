// GalaxyArchetype.h
// One galaxy morphology as an asset: this says what a spiral IS, FGalaxyArchetypeEntry says
// how common spirals are HERE.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Engine/VolumeTexture.h"
#include "GalaxyParams.h"
#include "GalaxyArchetype.generated.h"

/** One rolled parameter: which one, and the closed interval it rolls over. Sparse by design --
 *  the list IS the answer to what varies; anything absent is fixed at the exemplar. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyParamRange
{
	GENERATED_BODY()

	/** Property name on FGalaxyProceduralParams. Vector and colour members take a COMPONENT
	 *  SUFFIX -- "NoiseOffset.X". The dropdown is reflected off the struct itself. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range",
		meta = (GetOptions = "/Script/UltraLargeScale.GalaxyArchetype.GetRollableParameterNames"))
	FName Parameter;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Min = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Max = 0.0f;

	/** Distribution shape: the roll is Lerp(Min, Max, pow(FRand(), Bias)). 1 is uniform, above
	 *  1 clusters toward MIN (most galaxies ordinary, a few extreme), below 1 toward MAX. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range", meta = (ClampMin = "0.01"))
	float Bias = 1.0f;

	/** Snap the result to a multiple of this; 0 leaves it continuous. REQUIRED FOR COUNTS --
	 *  ArmCount is a float, so a roll over 2..6 yields 3.7 arms and is taken seriously. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range", meta = (ClampMin = "0.0"))
	float Quantize = 0.0f;
};

/** A galaxy morphology: one hand-authored exemplar plus what varies around it. Morphology is
 *  expressed by ZEROING LAYERS, which is free rather than cheap -- the field early-outs on
 *  each layer's density, so a globular skips the arm merge loop entirely.
 *
 *    Globular   bulge only, high BulgeConcentration, BulgeVerticalRatio ~ 1, no arms or disc
 *    Disc       disc + bulge, ArmDensity = 0
 *    Spiral     all four layers
 *
 *  A morphology that is only DIFFERENT NUMBERS needs no subclass: duplicate the asset and
 *  retune. Subclass when the rolls need logic -- see ApplyGenerationLogic. */
UCLASS(BlueprintType, Blueprintable)
class ULTRALARGESCALE_API UGalaxyArchetype : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	/** The hand-authored exemplar. Used verbatim in default mode, and the base every roll is
	 *  applied on top of: a parameter absent from Ranges keeps this value. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Exemplar")
	FGalaxyProceduralParams Default;

	/** What varies, and by how much. Everything not listed is fixed at its exemplar value. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rolls")
	TArray<FGalaxyParamRange> Ranges;

	/** Candidate field textures, one drawn per galaxy from each bag, ROLLED INDEPENDENTLY so
	 *  the four choices give their full product. The bags are not interchangeable: the warp
	 *  pair must be SIGNED vector volumes and the modulation pair UNORM multinoise, and ridged
	 *  bakes belong in the disc bag. EMPTY keeps the exemplar's own choice. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rolls")
	TArray<TObjectPtr<UVolumeTexture>> WarpDiscTextures;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rolls")
	TArray<TObjectPtr<UVolumeTexture>> WarpHaloTextures;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rolls")
	TArray<TObjectPtr<UVolumeTexture>> NoiseDiscTextures;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rolls")
	TArray<TObjectPtr<UVolumeTexture>> NoiseHaloTextures;

	/** THE PIPELINE, in order:
	 *    1. the exemplar
	 *    2. every listed range, each keyed to its own stream by parameter NAME
	 *    3. the texture draw, if the bag is non-empty
	 *    4. ApplyGenerationLogic
	 *
	 *  Not virtual -- the ORDER is the contract, and a subclass replacing this could silently
	 *  stop applying its own ranges. Override the hook. bInDefaultsOnly short-circuits at step
	 *  1 and returns the exemplar VERBATIM, hook included. */
	FGalaxyProceduralParams Resolve(int32 InSeed, bool bInDefaultsOnly) const;

	/** Every authored range checked once: name resolves, is rollable, Min <= Max, no duplicates.
	 *  Returns false and logs one line per problem. Inverted bounds otherwise produce SILENT
	 *  garbage, FRandRange returning a value outside the interval. */
	bool Validate() const;

	/** One line naming the archetype and the parameters that most define its silhouette. */
	void LogPreview(int32 InSeed, bool bInDefaultsOnly) const;

	/** A named stream off this galaxy's seed, for a subclass writing its own rolls. USE THIS
	 *  RATHER THAN FRandomStream(InSeed), which aliases against the archetype selection. */
	static FRandomStream ParameterStream(FName InChannel, int32 InSeed)
	{
		return ProcSeed::Stream(InSeed, ProcSeed::ChannelId(InChannel));
	}

	/** One range applied to one address, shaped by Bias and Quantize. Exposed so a subclass can
	 *  reuse the generic path's exact draw. */
	static float RollRange(const FGalaxyParamRange& InRange, int32 InSeed);

	/** Every rollable parameter name, reflected off FGalaxyProceduralParams: floats by name,
	 *  colour and vector members per component, object members omitted. */
	UFUNCTION()
	static TArray<FString> GetRollableParameterNames();

	/** Address of the float a parameter name designates, or nullptr if it does not resolve. THE
	 *  ONE PLACE the name grammar is interpreted: Resolve, Validate and the dropdown all go
	 *  through it. */
	static float* ResolveParameter(FGalaxyProceduralParams& InOutParams, FName InParameter);

protected:
	/** BESPOKE GENERATION, for rolls that ranges cannot express: a parameter derived from
	 *  another, a group that must move together, a rule like "if there is no disc the arms
	 *  cannot host". Runs LAST so rolled values are readable, never in default mode. */
	virtual void ApplyGenerationLogic(FGalaxyProceduralParams& InOutParams, int32 InSeed) const {}
};
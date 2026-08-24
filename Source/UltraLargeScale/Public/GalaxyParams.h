#pragma once



#include "Engine/VolumeTexture.h"

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "GalaxyParams.generated.h"
/** GALAXY-LAYER DRAW CHANNELS. Mechanism in ProcSeed (ProceduralSpaceActor.h).
 *
 *  Named to match the parameter CATEGORIES on FGalaxyDensityParams, so "I am editing
 *  the Disc group" means the same thing in the details panel, in the Make function
 *  that rolls it, and in what moves on the next spawn. Adding a roll to Arms leaves
 *  the disc, the bulge and the rotation exactly where they were.
 *
 *  ADD, NEVER REUSE. A channel is free; sharing one between two consumers reintroduces
 *  precisely the aliasing this exists to prevent. */
namespace GalaxySeed
{
	inline constexpr uint32 Archetype = ProcSeed::ChannelId("Galaxy.Archetype");
	inline constexpr uint32 Rotation = ProcSeed::ChannelId("Galaxy.Rotation");

	inline constexpr uint32 Arms = ProcSeed::ChannelId("Galaxy.Arms");
	inline constexpr uint32 Disc = ProcSeed::ChannelId("Galaxy.Disc");
	inline constexpr uint32 Bulge = ProcSeed::ChannelId("Galaxy.Bulge");
	inline constexpr uint32 Background = ProcSeed::ChannelId("Galaxy.Background");
	inline constexpr uint32 Noise = ProcSeed::ChannelId("Galaxy.Noise");
	inline constexpr uint32 Void = ProcSeed::ChannelId("Galaxy.Void");
	inline constexpr uint32 Master = ProcSeed::ChannelId("Galaxy.Master");

	/** The GPU placement key, INDEXED BY TIER (0 Large, 1 Mid, 2 Small).
	 *
	 *  Calibration and generation must pass the SAME tier index. The constant is solved
	 *  against the candidate stream generation then draws from, so two different keys
	 *  would calibrate one galaxy and generate a different one -- with no error, just
	 *  counts that miss their capacity. GalaxyDataGenerator::TierKeySeed is the single
	 *  place that mapping happens; do not spell it out at a call site. */
	inline constexpr uint32 Placement = ProcSeed::ChannelId("Galaxy.Placement");
}

// -----------------------------------------------------------------------------
// FGalaxyDensityParams -- THE AUTHORED FIELD INPUTS.
//
// One UPROPERTY per input of MakeGalaxyDensityParams, IN THE SAME ORDER. Nothing is
// combined or scaled here: correlations such as "arm width is a fraction of the disc
// scale height" are resolved inside the shared derivation, so this struct and the
// material's pin set are the same list of raw values.
//
// TWO ORGANISATIONS, DELIBERATELY DIFFERENT.
//
//   DECLARATION ORDER, and the pragma regions, follow the DERIVATION SIGNATURE: the
//   packed vectors first in packing order, then the loose scalars. ToDerived() and
//   FillShaderParameters both read straight down the struct, so a member in the wrong
//   place is visible by holding the three side by side. That eyeball check is the
//   ONLY check those marshalling sites have -- DO NOT REORDER MEMBERS to tidy the
//   file, and add new ones at the end of the block that feeds their vector.
//
//   CATEGORY follows the AUTHOR'S view: Arms / Disc / Bulge / Background, each split
//   into Scale, Density, Noise, Structure and Asymmetry, mirroring the parameter
//   groups on the volume material so the same knob is in the same place on both. The
//   details panel sorts by category and ignores declaration order, so the two views
//   cost nothing to keep apart.
//
// Flat rather than pre-packed into float4s for the same reason the categories exist:
// the panel is easier to drive one scalar at a time, and the packing into vectors
// happens in exactly one place, ToDerived().
// -----------------------------------------------------------------------------

// GalaxyHLSL::GalaxyDensityParams is the DERIVED field -- a different type entirely,
// declared in GalaxyDensityCore.ush, which GalaxyDataGenerator.cpp compiles INSIDE
// namespace GalaxyHLSL. The namespace is what keeps the shim's sqrt/abs/exp from
// colliding with the float overloads MSVC's <cmath> puts at global scope, so the type
// is namespace-qualified everywhere outside that file.
namespace GalaxyHLSL { struct GalaxyDensityParams; }

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyDensityParams
{
	GENERATED_BODY()

	/** Fills a compute shader parameter struct with the same raw values ToDerived
	 *  passes to MakeGalaxyDensityParams, in the same order.
	 *
	 *  THREE PLACES MARSHAL THESE: ToDerived, this, and the material's Custom node body.
	 *  Adding a parameter to MakeGalaxyDensityParams breaks the first two at compile
	 *  time. THE CUSTOM NODE WILL NOT -- a Custom node fails at shader compile rather
	 *  than at build, so a missed parameter shows up as a red material. Check it
	 *  whenever the derivation signature changes.
	 *
	 *  A TEMPLATE on purpose. Taking FGalaxyEntityGenCS::FParameters& directly would
	 *  make this header depend on GalaxyEntityGen.h, which depends on this one -- a
	 *  cycle, and one that drags RenderCore into every translation unit that only wanted
	 *  the authored struct.
	 *
	 *  Raw rather than derived on purpose. Uploading a packed GalaxyDensityParams
	 *  would require HLSL's constant buffer packing to agree with the C++ layout
	 *  member for member, and a single float3 straddling a 16-byte boundary shifts
	 *  every field after it with no diagnostic at all -- the galaxy would simply come
	 *  out wrong. Passing raw inputs and deriving in the shader costs sixteen arm
	 *  hashes and a tan per thread, against a full field evaluation per thread, and
	 *  it cannot drift.
	 *
	 *  InNoiseEnable is hard-wired to 1. Placement is always textured; the material's
	 *  EnableNoise static switch controls the RENDER only, and is a visualisation aid
	 *  rather than a field parameter. */
	template <typename TShaderParams>
	void FillShaderParameters(TShaderParams& Out) const
	{
		Out.InLateralScale = FVector4f(
			ArmRadius, DiscRadius, BulgeRadius, BackgroundRadius);

		Out.InVerticalScale = FVector4f(
			ArmVerticalRatio, DiscVerticalRatio,
			BulgeVerticalRatio, BackgroundVerticalRatio);

		Out.InLayerDensity = FVector4f(
			ArmDensity, DiscDensity, BulgeDensity, BackgroundDensity);

		Out.InNoiseAmount = FVector4f(
			ArmNoiseAmount, DiscNoiseAmount,
			BulgeNoiseAmount, BackgroundNoiseAmount);

		Out.InWarpAmount = FVector4f(
			WarpAmountArms, WarpAmountDisc,
			WarpAmountBulge, WarpAmountBackground);

		Out.InArmAsym = FVector4f(
			ArmAsymPitch, ArmAsymPhase, ArmAsymDensity, ArmAsymLength);

		Out.InSpiralTwist = FVector4f(
			ArmPitchAngle, ArmPitchTightening, ArmPhaseOffset, HaloTwistInherit);

		Out.InCentralVoid = FVector3f(
			CentralVoidRadius, CentralVoidAmount, CentralVoidExponent);

		Out.InNoiseScale = FVector4f(
			NoiseDiscLateralScale, NoiseDiscVerticalScale,
			NoiseHaloLateralScale, NoiseHaloVerticalScale);

		Out.InWarpScale = FVector4f(
			WarpDiscLateralScale, WarpDiscVerticalScale,
			WarpHaloLateralScale, WarpHaloVerticalScale);

		// FLinearColor is RGBA in memory but the shader reads it as xyzw against
		// NoiseChannelWeights, so the component order is spelled out rather than
		// relying on a reinterpret that would silently reorder if the type changed.
		Out.InNoiseChannelWeights = FVector4f(
			NoiseChannelWeights.R, NoiseChannelWeights.G,
			NoiseChannelWeights.B, NoiseChannelWeights.A);

		Out.InNoiseOffset = FVector3f(
			NoiseOffset.X, NoiseOffset.Y, NoiseOffset.Z);

		Out.InBoundsFadeStart = BoundsFadeStart;
		Out.InDiscScaleLengthRatio = DiscScaleRatio;
		Out.InDiscVerticalFalloff = DiscVerticalFalloff;
		Out.InDiscFlare = DiscFlare;
		Out.InDiscWarpAmplitude = DiscWarpAmplitude;
		Out.InDiscWarpPhase = DiscWarpPhase;
		Out.InDiscWarpTwist = DiscWarpTwist;
		Out.InDiscLopsidedAmount = DiscLopsidedAmount;
		Out.InDiscLopsidedPhase = DiscLopsidedPhase;
		Out.InArmCount = ArmCount;
		Out.InArmAsymSeed = ArmAsymSeed;
		Out.InArmProfileExponent = ArmProfileExponent;
		Out.InArmRadialGrowth = ArmRadialGrowth;
		Out.InArmHostFalloff = ArmHostFalloff;
		Out.InBulgeConcentration = BulgeConcentration;
		Out.InBackgroundConcentration = BackgroundConcentration;
		Out.InNoiseRidged = NoiseRidged;
		// ALWAYS TEXTURED. Placement samples the volume texture unconditionally --
		// that is the whole point of the GPU path, and a switch that turned it off
		// here would make the placed field differ from the drawn one with nothing
		// to say so.
		Out.InNoiseEnable = 1.0f;
	}

#pragma region Packed: InLateralScale
	/** Arm half-width as a FRACTION OF THE DISC SCALE HEIGHT. Arms are thinner than
	 *  the stellar disc they sit in, so this is a ratio rather than an absolute. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Scale", meta = (ClampMin = "0.0"))
	float ArmRadius = 0.75f;

	/** Disc radius in normalized space. Doubles as the arm system's radial
	 *  reference: arms live in the disc and have no radius of their own. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Scale", meta = (ClampMin = "0.0"))
	float DiscRadius = 0.7f;

	/** Bulge zero-density radius, as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Scale", meta = (ClampMin = "0.0"))
	float BulgeRadius = 0.33f;

	/** The background layer spans the galaxy by definition, so its lateral scale is
	 *  fixed at 1 rather than authored. BoundsFadeStart and BackgroundConcentration
	 *  shape the outer profile; a third knob over the same falloff only made the three
	 *  interact. It keeps its slot in InLateralScale so the packing is unchanged. */
	static constexpr float BackgroundRadius = 1.0f;
#pragma endregion

#pragma region Packed: InVerticalScale
	/** All four are axis ratios; smaller flattens. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Scale", meta = (ClampMin = "0.0"))
	float ArmVerticalRatio = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Scale", meta = (ClampMin = "0.0"))
	float DiscVerticalRatio = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Scale", meta = (ClampMin = "0.0"))
	float BulgeVerticalRatio = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Scale", meta = (ClampMin = "0.0"))
	float BackgroundVerticalRatio = 0.5f;
#pragma endregion

#pragma region Packed: InLayerDensity
	/** OPTICAL DEPTH per layer, not a volume density: what you would measure looking
	 *  through that layer along its natural axis. The derivation divides each by its
	 *  own path length, which is what makes these four comparable to each other and
	 *  independent of the march step count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Density", meta = (ClampMin = "0.0"))
	float ArmDensity = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Density", meta = (ClampMin = "0.0"))
	float DiscDensity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Density", meta = (ClampMin = "0.0"))
	float BulgeDensity = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Density", meta = (ClampMin = "0.0"))
	float BackgroundDensity = 0.1f;
#pragma endregion

#pragma region Packed: InNoiseAmount, InWarpAmount
	/** Multiplicative depth: density *= 1 + Amount * n. Above 1 the modulator goes
	 *  negative in places, clamps to zero, and breaks the layer apart.
	 *
	 *  Affects the RENDER AND PLACEMENT ALIKE: both sample the same textured field. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Noise")
	float ArmNoiseAmount = 0.333f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Noise")
	float DiscNoiseAmount = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Noise")
	float BulgeNoiseAmount = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Noise")
	float BackgroundNoiseAmount = 1.0f;

	/** Positional warp per layer. Signed: negative flips the displacement direction.
	 *  Affects render and placement alike, as the amounts above do. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Noise")
	float WarpAmountArms = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Noise")
	float WarpAmountDisc = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Noise")
	float WarpAmountBulge = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Noise")
	float WarpAmountBackground = 0.0f;
#pragma endregion

#pragma region Packed: InArmAsym (+ ArmAsymSeed scalar)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Asymmetry")
	float ArmAsymPitch = 0.175f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Asymmetry")
	float ArmAsymPhase = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Asymmetry")
	float ArmAsymDensity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Asymmetry")
	float ArmAsymLength = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Asymmetry")
	float ArmAsymSeed = 211.588882f;
#pragma endregion

#pragma region Packed: InSpiralTwist (+ arm shape scalars)
	/** Degrees at the disc rim; sign sets chirality. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "-89.0", ClampMax = "89.0"))
	float ArmPitchAngle = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "0.0"))
	float ArmPitchTightening = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure")
	float ArmPhaseOffset = 0.0f;

	/** How much of the spiral twist the bulge/background noise frame inherits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float HaloTwistInherit = 1.0f;

	/** CLAMPED TO GALAXY_MAX_ARMS (16) by the derivation. Values above that are
	 *  silently truncated, not honoured. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "1.0", ClampMax = "16.0"))
	float ArmCount = 16.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "0.0"))
	float ArmProfileExponent = 0.5f;

	/** Coefficient on DiscFlare: arms widen outward in step with the disc thickening.
	 *  1.0 means they taper together. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "0.0"))
	float ArmRadialGrowth = 2.0f;

	/** How strongly arm strength follows the disc's radial profile.
	 *  0 = independent, 1 = fades exactly with the disc, >1 faster. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arms|Structure", meta = (ClampMin = "0.0"))
	float ArmHostFalloff = 1.5f;
#pragma endregion

#pragma region Scalars: disc shape and asymmetry
	/** Radial scale length as a MULTIPLE OF THE BULGE RADIUS. The bulge-to-disc
	 *  scale ratio is a measured quantity; DiscRadius is only a truncation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Structure", meta = (ClampMin = "0.0"))
	float DiscScaleRatio = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Structure", meta = (ClampMin = "0.1"))
	float DiscVerticalFalloff = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Structure", meta = (ClampMin = "0.0"))
	float DiscFlare = 10.183177f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Asymmetry")
	float DiscWarpAmplitude = 0.112f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Asymmetry")
	float DiscWarpPhase = 0.0f;

	/** Radians of node-line precession across the disc. Past ~2.5 the warp folds
	 *  back on itself and reads as corrugation rather than an S-curve. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Asymmetry")
	float DiscWarpTwist = 2.276695f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Asymmetry")
	float DiscLopsidedAmount = 0.72f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Disc|Asymmetry")
	float DiscLopsidedPhase = 0.074667f;
#pragma endregion

#pragma region Scalars: profile exponents and bounds fade
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bulge|Structure", meta = (ClampMin = "0.0"))
	float BulgeConcentration = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Structure", meta = (ClampMin = "0.0"))
	float BackgroundConcentration = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Background|Structure", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundsFadeStart = 0.33f;
#pragma endregion

#pragma region Packed: InCentralVoid
	/** Radius as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0"))
	float CentralVoidRadius = 0.033f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float CentralVoidAmount = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0"))
	float CentralVoidExponent = 1.0f;
#pragma endregion

#pragma region Packed: InNoiseScale, InWarpScale, InNoiseChannelWeights, InNoiseOffset
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float NoiseDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float NoiseDiscVerticalScale = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float NoiseHaloLateralScale = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float NoiseHaloVerticalScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float WarpDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float WarpDiscVerticalScale = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float WarpHaloLateralScale = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise|Scale")
	float WarpHaloVerticalScale = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FLinearColor NoiseChannelWeights = FLinearColor(-1.0f, -0.7f, -0.4f, -0.4f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FVector3f NoiseOffset = FVector3f(1.0f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NoiseRidged = 0.0f;

#pragma endregion

#pragma region Render only -- NOT marshalled to the compute path
	/** Global sigma multiplier in the raymarch. RENDERER ONLY.
	 *
	 *  NOT A STAR COUNT CONTROL, and it cannot be made into one. Calibration solves
	 *  BudgetScale = capacity / sum(mass) with mass proportional to density^exponent,
	 *  so a global multiplier on the field scales every mass and BudgetScale by its
	 *  inverse -- accepted counts are identical. It also never reaches the compute
	 *  path: FillShaderParameters does not send it. Use StarDensityScale. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Master", meta = (ClampMin = "0.0"))
	float MasterDensityScale = 1.0f;

	/** Render-side shaping exponent, applied after Sample. No CPU counterpart, and
	 *  the same non-control as MasterDensityScale above. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Master", meta = (ClampMin = "0.0"))
	float MasterDensityPower = 1.0f;
#pragma endregion

	/** Pack into the shared derivation. Defined in GalaxyDataGenerator.cpp, the one
	 *  translation unit that compiles the shim and the .ush. */
	GalaxyHLSL::GalaxyDensityParams ToDerived() const;
};

/** Material-side parameters for the volumetric proxy.
 *
 *  PROVISIONAL. The galaxy raymarcher is a debug marcher until the paradigm has
 *  propagated to the other layers; this set will be rebuilt against the real one. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyMaterialParams
{
	GENERATED_BODY()

	/** THE MARCH CONTROLS. All four reach the material, so the baseline lives here
	 *  rather than only on the material asset.
	 *
	 *  Step length is max(MinStep, StepRatio x cameraDistance), capped so the chord
	 *  always takes at least MinSamples steps. THREE OF THE FOUR ARE STEP BUDGETS, in
	 *  steps, because that is what an author is actually choosing:
	 *
	 *    NearSteps    the ceiling, reached on close approach
	 *    MinSamples   the floor, reached in the far field
	 *    MaxSteps     the safety stop, which should never be reached
	 *
	 *  StepRatio is the remaining shape control: it decides WHERE between those two
	 *  ends the cost sits, not how high either end is.
	 *
	 *  These are PERFORMANCE controls rather than look controls: because LayerDensity is
	 *  an optical depth normalised by path length, changing any of them does not require
	 *  retuning a density. They are candidates for the game's quality settings. */

	 /** Steps across the chord on closest approach -- the CEILING on march cost.
	  *
	  *  MinStep is derived from this, as 2 / NearSteps, because the galaxy spans two
	  *  units in normalized space and a march floored at MinStep takes chord/MinStep
	  *  steps. Authoring the length instead hid that: 0.005 units reads as a small
	  *  number and means four hundred steps.
	  *
	  *  STEPRATIO CANNOT REDUCE THIS. The ratio only controls how close the camera has
	  *  to get before the floor takes over; once it does, the march is uniform at
	  *  MinStep and the adaptive term is irrelevant. A step count that will not respond
	  *  to StepRatio is this budget, not a bug. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "8.0", ClampMax = "1024.0"))
	float VolumeNearSteps = 160.0f;

	/** Fewest steps across the chord, which caps step length at chord/MinSamples -- the
	 *  FLOOR on march cost, and what stops a distant galaxy resolving to a handful of
	 *  samples. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "1.0", ClampMax = "512.0"))
	float VolumeMinSamples = 24.0f;

	/** Hard bound on iterations. A SAFETY STOP, not the step count -- the march should
	 *  reach NearSteps and stop there, so hitting this means the budget above is not
	 *  being respected, and raising it hides that. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "1.0", ClampMax = "2048.0"))
	float VolumeMaxSteps = 512.0f;

	/** Fraction of camera distance a step spans, before the two budgets clamp it.
	 *
	 *  The SHAPE of the cost curve between them. The floor takes over within
	 *  D* = 2 / (NearSteps x StepRatio) galaxy radii of the centre, so lowering the
	 *  ratio widens the expensive band outward rather than making the peak cheaper. At
	 *  the defaults that distance is 1.6 radii -- just outside the volume. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0001", ClampMax = "0.5"))
	float VolumeStepRatio = 0.005f;

	/** Shortest step, in normalized space where the galaxy spans -1 to 1. */
	float GetMinStep() const
	{
		return 2.0f / FMath::Max(VolumeNearSteps, 1.0f);
	}

	/** Camera distance, in galaxy radii from the centre, inside which the march runs at
	 *  MinStep and costs NearSteps. Diagnostic rather than plumbed anywhere. */
	float GetFloorDistance() const
	{
		return GetMinStep() / FMath::Max(VolumeStepRatio, 1e-6f);
	}

	/** The volume texture the material samples for modulation and positional warp.
	 *
	 *  Distinct from FGalaxyParams::NoiseTexture, which is the same asset reached by the
	 *  compute path. They must be the same texture, or placement and render disagree. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VolumeNoise = "/UltraLargeScale/VolumeTextures/VT_PerlinWorley_Balanced";
};


/** Galaxy-layer generation parameters; extends FBaseParams (mirrors
 *  FUniverseParams structure). */

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyParams : public FBaseParams
{
	GENERATED_BODY()

#pragma region GPU Generation
	/** The same volume texture the material samples. REQUIRED.
	 *
	 *  Placement is GPU-only, and the dispatch samples this. Without it the galaxy
	 *  generates NOTHING -- there is no analytic path behind it any more. The actor's
	 *  constructor assigns a default when this is left unset, so it is a requirement
	 *  rather than a chore; clearing it deliberately is the case that fails.
	 *
	 *  Set NEVER STREAM on the asset. GalaxyDensityCore.ush reads mip 0 on both paths, but
	 *  the material handles streaming residency and a compute dispatch does not: if
	 *  mip 0 is not resident when the dispatch runs it reads whatever is, and placement
	 *  silently stops matching the render.
	 *
	 *  The sampler here is Trilinear/Wrap. If the material's Texture Sample node uses
	 *  anything else the two paths read different values at the same coordinate, which
	 *  shows up as a small plausible-looking difference rather than an obvious break. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPU Generation")
	TObjectPtr<UVolumeTexture> NoiseTexture = nullptr;
#pragma endregion

	// TODO: SEE IF WE CAN BRIDGE THE GAP TO REAL WORLD SCALE HERE, I THINK WE HIT PRECISION ISSUES THOUGH... UNIT SCALE AND POTENTIALLY STAR SYSTEM SCALES/PARAMS MAY NEED TO SHIFT

#pragma region Tier Scale Derivation
	/** Fixed absolute largest star-system scale in world cm.
	 *  All galaxies generate star particles in the same physical size
	 *  range regardless of parent galaxy size. With the current value (4e16)
	 *  and the tier depth sequence (1/3/5, spacing 2, ratio 4, 64x total
	 *  spread) DeriveScaleRanges produces:
	 *
	 *    Large: 1e16    -> 4e16      (largest systems in the population)
	 *    Mid:   2.5e15  -> 1e16
	 *    Small: 6.25e14 -> 2.5e15    (compact systems)
	 *
	 *  Real references (for retuning context):
	 *    Solar system to Pluto orbit ~ 1.2e19 cm diameter
	 *    Compact M-dwarf habitable zone ~ 3e16 cm
	 *    Wide binary separation ~ 1e18 cm
	 *
	 *  The octree insert path converts these to octree-local extents using
	 *  the galaxy's UnitScale, so insert depth adapts to each galaxy's
	 *  coordinate system while the physical size stays constant. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 4e16;

#pragma endregion


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FGalaxyDensityParams DensityParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Material")
	FGalaxyMaterialParams MaterialParams;

#pragma region Encoded Noise Graph & Derivation
	/** Serialized FastNoise graph, kept for a future FastNoise swap-in. */
	static constexpr const char* EncodedTree = "DQAFAAAAAAAAQAgAAAAAAD8AAAAAAA==";

	/** Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	 *  depth sequence. Delegates to FTierParams::DeriveTierScaleRanges. */
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		FTierParams::DeriveTierScaleRanges(MaxEntityScale, Tiers);
	}

	/** Post-Generate overlay: fills parent-derived (context) fields only —
	 *  the randomizable ranged fields are Generate's job. Kept here so the
	 *  parent's acquire call stays a one-liner. Confirm the field list in step E. */
	FGalaxyParams& ApplyContext(const FOctreeNode& Node)
	{
		Seed = Node.Data.Seed;
		ParentColor = FLinearColor(Node.Data.Composition);
		return *this;
	}

	FGalaxyParams()
	{
		Seed = 666;
		// Galaxy-layer design constant (see FBaseParams::UnitScale).
		// Derived galaxy extent = particle real size / UnitScale. With the
		// current universe MaxEntityScale (1e22) and its 64x tier spread
		// (1.5625e20 .. 1e22 cm), derived extents span ~6.5e6 (smallest
		// small-tier galaxies) to ~4.2e8 (largest large-tier galaxies).
		// For reference, a Milky-Way-class galaxy (~1.26e23 cm) would map
		// to ~5.2e9 local units at this constant.
		UnitScale = 2.4e13;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1, 0);

		// Large tier: single cell covering the full galaxy extent.
		// NeighborhoodRadius = 0 -> 1x1x1 = 1 slot, exhaustive single-pass.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.GenerationSubdivision = 4;
		LargeTier.SlotCapacity = 10000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.GenerationSubdivision = 2;
		MidTier.SlotCapacity = 8000;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.GenerationSubdivision = 2;
		SmallTier.SlotCapacity = 6000;

		DeriveScaleRanges();
	}

#pragma endregion
};

/** Min/max bounds for randomized galaxy params; Generate() samples between
 *  them for a given seed. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyParamBounds {
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Param Bounds")
	FGalaxyParams Min;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Param Bounds")
	FGalaxyParams Max;

	static FGalaxyParams Generate(const FGalaxyParamBounds& Bounds, int64 Seed) {
		//TODO [E]: real Min..Max interpolation from Seed. Stub = uniform Max.
		return Bounds.Max;
	}
	static FGalaxyParams Minimal(const FGalaxyParamBounds& Bounds) { return Bounds.Min; }
	static FGalaxyParams Maximal(const FGalaxyParamBounds& Bounds) { return Bounds.Max; }
};
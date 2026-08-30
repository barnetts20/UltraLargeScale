#pragma once



#include "Engine/VolumeTexture.h"

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "GalaxyParams.generated.h"

class UGalaxyArchetype;
/** GALAXY-LAYER DRAW CHANNELS. Mechanism in ProcSeed (ProceduralSpaceActor.h).
 *
 *  Named to match the parameter CATEGORIES on FGalaxyProceduralParams, so "I am editing
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

	// THE LAYER CHANNELS BELOW ARE FOR HAND-WRITTEN ROLLS ONLY. The generic roller in
	// UGalaxyArchetype keys each parameter off its own NAME via ProcSeed::ChannelId(FName),
	// so a range list needs none of these. They exist for the escape hatch: a future
	// per-archetype function that rolls a correlated GROUP rather than independent
	// parameters, and therefore wants one stream for the group.
	inline constexpr uint32 Arms = ProcSeed::ChannelId("Galaxy.Arms");
	inline constexpr uint32 Disc = ProcSeed::ChannelId("Galaxy.Disc");
	inline constexpr uint32 Bulge = ProcSeed::ChannelId("Galaxy.Bulge");
	inline constexpr uint32 Background = ProcSeed::ChannelId("Galaxy.Background");
	inline constexpr uint32 Noise = ProcSeed::ChannelId("Galaxy.Noise");
	inline constexpr uint32 Void = ProcSeed::ChannelId("Galaxy.Void");
	inline constexpr uint32 Master = ProcSeed::ChannelId("Galaxy.Master");

	/** Which entry of the archetype's NoiseTextures bag this galaxy draws. Separate
	 *  from Noise so that adding a texture to the bag does not disturb noise
	 *  parameter rolls, and vice versa. */
	inline constexpr uint32 NoiseTexture = ProcSeed::ChannelId("Galaxy.NoiseTexture");

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
// FGalaxyProceduralParams -- THE AUTHORED FIELD INPUTS.
//
// One UPROPERTY per input of MakeGalaxyDensityParams. Nothing is combined or scaled
// here: correlations such as "arm width is a fraction of the disc scale height" are
// resolved inside the shared derivation, so this struct and the material's pin set are
// the same list of raw values. LEAF NAMES MATCH THE MATERIAL PIN NAMES EXACTLY --
// Arms.ArmRadius pushes to "ArmRadius" -- which is what keeps that correspondence
// checkable by eye rather than by convention.
//
// GROUPED INTO SUB-STRUCTS BECAUSE THE DETAILS PANEL IGNORES CATEGORY INSIDE A
// USTRUCT. Category metadata is applied by the details layout builder at the UObject
// level only; a nested struct's children are laid out FLAT, IN DECLARATION ORDER, and
// the category strings do nothing. This struct is always nested -- inside FGalaxyParams
// on the actor, inside Default on the archetype asset -- so nesting is the only
// grouping mechanism available without an editor module, and it is why these members
// live one level down instead of behind a Category string.
//
// The Category strings that remain inside each sub-struct are inert for the same
// reason. They are kept as labels of intent, and would become live if a sub-struct were
// ever hoisted onto a UObject.
//
// THE DERIVATION MIRROR IS GONE, deliberately. Declaration order used to follow
// MakeGalaxyDensityParams' argument list so a misplaced member was visible by reading
// the two side by side. Grouping by layer costs that, and buys something back: each
// float4 in ToDerived now reads one slot per layer --
// float4(Arms.ArmRadius, Disc.DiscRadius, Bulge.BulgeRadius, ...) -- so the packing's
// own structure is visible in the expression instead of having to be remembered. The
// check that remains is that every member appears exactly once in ToDerived.
// -----------------------------------------------------------------------------

/** GalaxyHLSL::GalaxyDensityParams is the DERIVED field -- a different type entirely,
 *  declared in GalaxyDensityCore.ush, which GalaxyDataGenerator.cpp compiles INSIDE
 *  namespace GalaxyHLSL. The namespace is what keeps the shim's sqrt/abs/exp from
 *  colliding with the float overloads MSVC's <cmath> puts at global scope, so the type
 *  is namespace-qualified everywhere outside that file. */
namespace GalaxyHLSL { struct GalaxyDensityParams; }

/** GLOBAL ORIENTATION of the whole field. Applied at the sample entry point, so every
 *  layer, the noise frame and the m=1 modes all turn together.
 *
 *  IN THE FIELD, NOT ON THE ACTOR TRANSFORM, and that is not the obvious choice. The
 *  scene graph would rotate the proxy and the Niagara components for free -- but it
 *  would also rotate the octree, the tier grids and VirtualTraversal, and the streaming
 *  scheme assumes galaxy-local and world axes coincide (AGalaxyActor::UpdateSpawnScan
 *  uses VirtualTraversal, a WORLD delta, directly as a LOCAL position). Rotating here
 *  costs three dot products per sample and leaves all of that untouched, because a
 *  rotation preserves dot(p,p) and every bounds test in the system is a radius.
 *
 *  It also means orientation reaches both paths through MakeGalaxyDensityParams like
 *  every other field parameter, so placement and render cannot disagree about it. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyOrientationParams
{
	GENERATED_BODY()

	/** The disc normal, in galaxy space. Normalized in the derivation, so magnitude is
	 *  free. (0,0,1) is disc-up.
	 *
	 *  A NORMAL, NOT EULER ANGLES, so the value the universe already computes per
	 *  particle -- a uniform unit vector, used for sprite orientation -- drops straight
	 *  in with no conversion. Euler would need a convention translation, and a rotation
	 *  convention guessed wrong renders plausibly rather than visibly.
	 *
	 *  NORMALLY INHERITED, NOT ROLLED. FGalaxySpawnConfig::bInheritParticleOrientation
	 *  overwrites this at spawn from the parent particle, which is what keeps a galaxy
	 *  facing the same way as the sprite that stood in for it a moment earlier. Ranging
	 *  the components while that flag is set does nothing -- turn it off first.
	 *
	 *  MARSHALLED WITH FieldSpin as one float4 -- xyz normal, w spin -- under the single
	 *  material parameter name "FieldNormal". Split here because the details panel and
	 *  the archetype range list want them apart: a normal rolls as three components with
	 *  a sampling caveat, a spin rolls as one clean scalar. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orientation")
	FVector3f FieldNormal = FVector3f(0.0f, 0.0f, 1.0f);

	/** Degrees about the normal. The one remaining freedom once the disc plane is
	 *  fixed, and free to roll -- a spin is uniform on a circle, so unlike a normal it
	 *  has no sampling subtlety. Never inherited; the particle sprite has no meaningful
	 *  spin about its own axis.
	 *
	 *  Rides in w of the FieldNormal vector on the material side. There is no FieldSpin
	 *  material parameter -- do not add one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orientation")
	float FieldSpin = 0.0f;
};

/** The spiral arms. Zeroing ArmDensity skips the entire arm merge loop
 *  including the sixteen arm hashes -- most of the field's cost -- which is what makes a
 *  globular archetype cheap rather than merely correct. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyArmParams
{
	GENERATED_BODY()

#pragma region Scale
	/** Arm half-width as a FRACTION OF THE DISC SCALE HEIGHT. Arms are thinner than
	 *  the stellar disc they sit in, so this is a ratio rather than an absolute. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float ArmRadius = 0.75f;

	/** All four are axis ratios; smaller flattens. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float ArmVerticalRatio = 0.75f;
#pragma endregion

#pragma region Density
	/** OPTICAL DEPTH per layer, not a volume density: what you would measure looking
	 *  through that layer along its natural axis. The derivation divides each by its
	 *  own path length, which is what makes these four comparable to each other and
	 *  independent of the march step count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density", meta = (ClampMin = "0.0"))
	float ArmDensity = 1.0f;
#pragma endregion

#pragma region Noise
	/** Multiplicative depth: density *= 1 + Amount * n. Above 1 the modulator goes
	 *  negative in places, clamps to zero, and breaks the layer apart.
	 *
	 *  Affects the RENDER AND PLACEMENT ALIKE: both sample the same textured field. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float ArmNoiseAmount = 0.333f;

	/** Positional warp per layer. Signed: negative flips the displacement direction.
	 *  Affects render and placement alike, as the amounts above do. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountArms = 0.05f;
#pragma endregion

#pragma region Structure
	/** CLAMPED TO GALAXY_MAX_ARMS (16) by the derivation. Values above that are
	 *  silently truncated, not honoured. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "1.0", ClampMax = "16.0"))
	float ArmCount = 16.0f;

	/** Degrees at the disc rim; sign sets chirality. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "-89.0", ClampMax = "89.0"))
	float ArmPitchAngle = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float ArmPitchTightening = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure")
	float ArmPhaseOffset = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float ArmProfileExponent = 0.5f;

	/** Coefficient on DiscFlare: arms widen outward in step with the disc thickening.
	 *  1.0 means they taper together. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float ArmRadialGrowth = 2.0f;

	/** How strongly arm strength follows the disc's radial profile.
	 *  0 = independent, 1 = fades exactly with the disc, >1 faster. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float ArmHostFalloff = 1.5f;
#pragma endregion

#pragma region Asymmetry
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float ArmAsymPitch = 0.175f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float ArmAsymPhase = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float ArmAsymDensity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float ArmAsymLength = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float ArmAsymSeed = 211.588882f;
#pragma endregion
};

/** The smooth disc the arms sit in. Zeroing DiscDensity leaves arms hosted
 *  on nothing; the two are usually zeroed together. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyDiscParams
{
	GENERATED_BODY()

#pragma region Scale
	/** Disc radius in normalized space. Doubles as the arm system's radial
	 *  reference: arms live in the disc and have no radius of their own. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float DiscRadius = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float DiscVerticalRatio = 0.01f;
#pragma endregion

#pragma region Density
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density", meta = (ClampMin = "0.0"))
	float DiscDensity = 0.5f;
#pragma endregion

#pragma region Noise
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float DiscNoiseAmount = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountDisc = 0.01f;
#pragma endregion

#pragma region Structure
	/** Radial scale length as a MULTIPLE OF THE BULGE RADIUS. The bulge-to-disc
	 *  scale ratio is a measured quantity; DiscRadius is only a truncation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float DiscScaleRatio = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.1"))
	float DiscVerticalFalloff = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float DiscFlare = 10.183177f;
#pragma endregion

#pragma region Asymmetry
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float DiscWarpAmplitude = 0.112f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float DiscWarpPhase = 0.0f;

	/** Radians of node-line precession across the disc. Past ~2.5 the warp folds
	 *  back on itself and reads as corrugation rather than an S-curve. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float DiscWarpTwist = 2.276695f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float DiscLopsidedAmount = 0.72f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Asymmetry")
	float DiscLopsidedPhase = 0.074667f;
#pragma endregion
};

/** The central spheroid. A globular is this layer alone, with high
 *  BulgeConcentration and BulgeVerticalRatio near 1. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyBulgeParams
{
	GENERATED_BODY()

#pragma region Scale
	/** Bulge zero-density radius, as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float BulgeRadius = 0.33f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float BulgeVerticalRatio = 0.6f;
#pragma endregion

#pragma region Density
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density", meta = (ClampMin = "0.0"))
	float BulgeDensity = 3.0f;
#pragma endregion

#pragma region Noise
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float BulgeNoiseAmount = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountBulge = 0.05f;
#pragma endregion

#pragma region Structure
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float BulgeConcentration = 1.0f;
#pragma endregion
};

/** The halo the whole galaxy fades into. Its radius is fixed rather than
 *  authored -- see BackgroundRadius. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyBackgroundParams
{
	GENERATED_BODY()

	/** The background layer spans the galaxy by definition, so its lateral scale is
	 *  fixed at 1 rather than authored. BoundsFadeStart and BackgroundConcentration
	 *  shape the outer profile; a third knob over the same falloff only made the three
	 *  interact. It keeps its slot in InLateralScale so the packing is unchanged.
	 *
	 *  Referenced as FGalaxyBackgroundParams::BackgroundRadius. */
	static constexpr float BackgroundRadius = 1.0f;

#pragma region Scale
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale", meta = (ClampMin = "0.0"))
	float BackgroundVerticalRatio = 0.5f;
#pragma endregion

#pragma region Density
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density", meta = (ClampMin = "0.0"))
	float BackgroundDensity = 0.1f;
#pragma endregion

#pragma region Noise
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float BackgroundNoiseAmount = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountBackground = 0.0f;
#pragma endregion

#pragma region Structure
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0"))
	float BackgroundConcentration = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Structure", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundsFadeStart = 0.33f;
#pragma endregion
};

/** The central clearing carved out of every layer. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyVoidParams
{
	GENERATED_BODY()

	/** Radius as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0"))
	float CentralVoidRadius = 0.033f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float CentralVoidAmount = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0"))
	float CentralVoidExponent = 1.0f;
};

/** The texture frame: which channels are read, at what scale, and how the
 *  lookup is warped. Shared by every layer rather than owned by one. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyNoiseFieldParams
{
	GENERATED_BODY()

#pragma region Field
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	FLinearColor NoiseChannelWeights = FLinearColor(-1.0f, -0.7f, -0.4f, -0.4f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	FVector3f NoiseOffset = FVector3f(1.0f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NoiseRidged = 0.0f;

	/** How much of the spiral twist the bulge/background noise frame inherits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	float HaloTwistInherit = 1.0f;
#pragma endregion

#pragma region Scale
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float NoiseDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float NoiseDiscVerticalScale = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float NoiseHaloLateralScale = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float NoiseHaloVerticalScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float WarpDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float WarpDiscVerticalScale = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float WarpHaloLateralScale = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	float WarpHaloVerticalScale = 0.5f;
#pragma endregion
};

/** Render-only shaping. NOT marshalled to the compute path. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyMasterParams
{
	GENERATED_BODY()

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
};

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyProceduralParams
{
	GENERATED_BODY()

#pragma region Ungrouped -- the two knobs reached for most often, kept at the top
	/** The volume texture BOTH the dispatch and the material sample. REQUIRED.
	 *
	 *  PROCEDURAL, NOT CONFIG. It is an input to the field evaluation, so swapping it
	 *  changes the look and the density the placement pass measures -- it is exactly
	 *  as much a morphology decision as BulgeConcentration. It sits here so a future
	 *  bag of packed noise assets can be drawn from per galaxy.
	 *
	 *  CATEGORICAL, NOT CONTINUOUS. A Min/Max range means nothing for a texture, so it
	 *  is drawn from UGalaxyArchetype::NoiseTextures -- a candidate ARRAY -- and this
	 *  member holds the resolved choice.
	 *
	 *  Placement is GPU-only, and the dispatch samples this. Without it the galaxy
	 *  generates NOTHING -- there is no analytic path behind it any more.
	 *  AGalaxyActor::InitializeData substitutes DefaultNoiseTexture when this is unset,
	 *  which is what keeps an unauthored archetype from silently producing an empty
	 *  galaxy. NOT the constructor: ReInit assigns Params wholesale, so anything the
	 *  constructor wrote into Params is gone by the time a pooled galaxy generates.
	 *
	 *  Set NEVER STREAM on the asset. GalaxyDensityCore.ush reads mip 0 on both paths, but
	 *  the material handles streaming residency and a compute dispatch does not: if
	 *  mip 0 is not resident when the dispatch runs it reads whatever is, and placement
	 *  silently stops matching the render.
	 *
	 *  The sampler here is Trilinear/Wrap. If the material's Texture Sample node uses
	 *  anything else the two paths read different values at the same coordinate, which
	 *  shows up as a small plausible-looking difference rather than an obvious break. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	TObjectPtr<UVolumeTexture> NoiseTexture = nullptr;

	/** FILL FRACTION on every tier's capacity TARGET. The ONLY handle on star count.
	 *
	 *  NO DENSITY PARAMETER CAN CHANGE STAR COUNT. Calibration solves
	 *  BudgetScale = capacity / sum(mass) with mass proportional to density^exponent,
	 *  so scaling the whole field by c scales every mass by c^exponent and BudgetScale
	 *  by its inverse: accepted counts come out identical. After calibration the
	 *  density field is a SHAPE and its absolute magnitude is unobservable in
	 *  placement. (Same property that makes kBudgetAnchor a free parameter.)
	 *
	 *  NOR CAN SlotCapacity. That sizes the Niagara buffers, allocated once per pooled
	 *  actor, so rolling it per galaxy means reallocating on every spawn -- the exact
	 *  cost pooling exists to avoid.
	 *
	 *  So it lands here, applied where BudgetScale is computed:
	 *  capacity * StarDensityScale / divisor. Buffer sizes never move, and a sparse
	 *  galaxy genuinely GENERATES fewer entities rather than being thinned afterwards.
	 *
	 *  THINNING ONLY -- clamped to 1. SlotCapacity both sizes the buffer and is the
	 *  calibration target, so above 1 the tier aims past what the buffer holds and the
	 *  overflow is clamped silently (the ExceededCells path). Denser than capacity
	 *  requires raising SlotCapacity, which is the thing that cannot roll. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Placement", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float StarDensityScale = 1.0f;
#pragma endregion

	/** Applies to everything below. Rolled like any other procedural member -- but see
	 *  the uniformity caveat on FGalaxyOrientationParams before ranging all three. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyOrientationParams Orientation;

#pragma region Layers
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyArmParams Arms;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyDiscParams Disc;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyBulgeParams Bulge;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyBackgroundParams Background;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyVoidParams Void;
#pragma endregion

#pragma region Shared
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyNoiseFieldParams Noise;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyMasterParams Master;
#pragma endregion

	/** Pack into the shared derivation. Defined in GalaxyDataGenerator.cpp, the one
	 *  translation unit that compiles the shim and the .ush. */
	GalaxyHLSL::GalaxyDensityParams ToDerived() const;

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
	 *  Leaf names still match the material pin names exactly -- Arms.ArmRadius feeds
	 *  "ArmRadius" -- so grouping the members did not weaken the three-way check above.
	 *
	 *  InNoiseEnable is hard-wired to 1. Placement is always textured; the material's
	 *  EnableNoise static switch controls the RENDER only, and is a visualisation aid
	 *  rather than a field parameter. */
	template <typename TShaderParams>
	void FillShaderParameters(TShaderParams& Out) const
	{
		Out.InLateralScale = FVector4f(
			Arms.ArmRadius, Disc.DiscRadius, Bulge.BulgeRadius, FGalaxyBackgroundParams::BackgroundRadius);

		Out.InVerticalScale = FVector4f(
			Arms.ArmVerticalRatio, Disc.DiscVerticalRatio,
			Bulge.BulgeVerticalRatio, Background.BackgroundVerticalRatio);

		Out.InLayerDensity = FVector4f(
			Arms.ArmDensity, Disc.DiscDensity, Bulge.BulgeDensity, Background.BackgroundDensity);

		Out.InNoiseAmount = FVector4f(
			Arms.ArmNoiseAmount, Disc.DiscNoiseAmount,
			Bulge.BulgeNoiseAmount, Background.BackgroundNoiseAmount);

		Out.InWarpAmount = FVector4f(
			Arms.WarpAmountArms, Disc.WarpAmountDisc,
			Bulge.WarpAmountBulge, Background.WarpAmountBackground);

		Out.InArmAsym = FVector4f(
			Arms.ArmAsymPitch, Arms.ArmAsymPhase, Arms.ArmAsymDensity, Arms.ArmAsymLength);

		Out.InSpiralTwist = FVector4f(
			Arms.ArmPitchAngle, Arms.ArmPitchTightening, Arms.ArmPhaseOffset, Noise.HaloTwistInherit);

		Out.InCentralVoid = FVector3f(
			Void.CentralVoidRadius, Void.CentralVoidAmount, Void.CentralVoidExponent);

		Out.InNoiseScale = FVector4f(
			Noise.NoiseDiscLateralScale, Noise.NoiseDiscVerticalScale,
			Noise.NoiseHaloLateralScale, Noise.NoiseHaloVerticalScale);

		Out.InWarpScale = FVector4f(
			Noise.WarpDiscLateralScale, Noise.WarpDiscVerticalScale,
			Noise.WarpHaloLateralScale, Noise.WarpHaloVerticalScale);

		// FLinearColor is RGBA in memory but the shader reads it as xyzw against
		// Noise.NoiseChannelWeights, so the component order is spelled out rather than
		// relying on a reinterpret that would silently reorder if the type changed.
		Out.InNoiseChannelWeights = FVector4f(
			Noise.NoiseChannelWeights.R, Noise.NoiseChannelWeights.G,
			Noise.NoiseChannelWeights.B, Noise.NoiseChannelWeights.A);

		Out.InNoiseOffset = FVector3f(
			Noise.NoiseOffset.X, Noise.NoiseOffset.Y, Noise.NoiseOffset.Z);

		Out.InBoundsFadeStart = Background.BoundsFadeStart;
		Out.InDiscScaleLengthRatio = Disc.DiscScaleRatio;
		Out.InDiscVerticalFalloff = Disc.DiscVerticalFalloff;
		Out.InDiscFlare = Disc.DiscFlare;
		Out.InDiscWarpAmplitude = Disc.DiscWarpAmplitude;
		Out.InDiscWarpPhase = Disc.DiscWarpPhase;
		Out.InDiscWarpTwist = Disc.DiscWarpTwist;
		Out.InDiscLopsidedAmount = Disc.DiscLopsidedAmount;
		Out.InDiscLopsidedPhase = Disc.DiscLopsidedPhase;
		Out.InArmCount = Arms.ArmCount;
		Out.InArmAsymSeed = Arms.ArmAsymSeed;
		Out.InArmProfileExponent = Arms.ArmProfileExponent;
		Out.InArmRadialGrowth = Arms.ArmRadialGrowth;
		Out.InArmHostFalloff = Arms.ArmHostFalloff;
		Out.InBulgeConcentration = Bulge.BulgeConcentration;
		Out.InBackgroundConcentration = Background.BackgroundConcentration;
		Out.InNoiseRidged = Noise.NoiseRidged;
		// ALWAYS TEXTURED. Placement samples the volume texture unconditionally --
		// that is the whole point of the GPU path, and a switch that turned it off
		// here would make the placed field differ from the drawn one with nothing
		// to say so.
		Out.InNoiseEnable = 1.0f;

		Out.InFieldOrientation = FVector4f(
			Orientation.FieldNormal.X, Orientation.FieldNormal.Y,
			Orientation.FieldNormal.Z, Orientation.FieldSpin);
	}
};

/** Material-side parameters for the volumetric proxy.
 *
 *  PROVISIONAL. The galaxy raymarcher is a debug marcher until the paradigm has
 *  propagated to the other layers; this set will be rebuilt against the real one. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyMaterialParams
{
	GENERATED_BODY()

	/** THE MARCH CONTROLS. BOTH of them reach the material, so the baseline lives here
	 *  rather than only on the material asset.
	 *
	 *  THE STEPPING RULE IS THE UNIVERSE LAYER'S, and this set replaced four properties
	 *  that described the old one -- NearSteps, MinSamples, StepRatio and MaxSteps. Step
	 *  length is now
	 *
	 *      baseStep = span / StepBudget
	 *      h        = baseStep * (1 + StepGrowth * t01)
	 *
	 *  with t01 the progress along the marched span rather than the distance from the
	 *  camera. The old rule kept a step a constant size ON SCREEN, which is a genuine
	 *  property and worth remembering was given up deliberately. What it could not do is
	 *  give a PREDICTABLE COST: the count depended on where the camera was and collapsed
	 *  toward chord/MinStep on close approach, and with four density marchers able to be
	 *  on screen at once, per-layer cost has to be something set rather than discovered.
	 *
	 *  These are PERFORMANCE controls rather than look controls: because LayerDensity is
	 *  an optical depth normalised by path length, changing either does not require
	 *  retuning a density. They are candidates for the game's quality settings. */

	 /** A CEILING ON THE STEP COUNT, NOT A FLOOR, which is what the old MinSamples name
	  *  had backwards. Growth only ever lengthens steps, so every step is at least
	  *  span / StepBudget and the actual count is StepBudget * ln(1+g)/g -- equal to the
	  *  budget only at growth 0, 0.69 of it at growth 1, 0.55 at growth 2. See
	  *  GetEffectiveStepCount.
	  *
	  *  THE DEFAULT IS NOT THE OLD NEAR-FIELD BUDGET. NearSteps was 256 and MinSamples 32,
	  *  and the floor took over just outside the volume, so a close pass ran 256 uniform
	  *  steps and a distant galaxy ran 32. 192 at growth 2 resolves to about 105 steps and
	  *  pays that whatever the viewer does. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "8.0", ClampMax = "1024.0"))
	float VolumeStepBudget = 192.0f;

	/** How fast steps lengthen along the span; 0 is uniform stepping. The cheapest quality
	 *  lever here, since it trades far-side detail for step count directly.
	 *
	 *  LOWER THAN THE UNIVERSE'S 4, and for a structural reason rather than a tuning one.
	 *  That layer's proxy is centred on the camera, so its span always begins AT the
	 *  viewer and growth along it tracks perspective exactly. This proxy is usually viewed
	 *  from OUTSIDE, where the whole chord sits at roughly one distance and the far side
	 *  is barely more forgiving than the near one -- so aggressive growth coarsens the back
	 *  half for no perceptual return. It earns its keep again on approach and from
	 *  inside. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0", ClampMax = "8.0"))
	float VolumeStepGrowth = 2.0f;

	// NO STEP FLOOR AND NO ITERATION BOUND HERE, and both were removed rather than
	// forgotten. The march's step floor is P.MinFeatureStep -- half the narrowest of the
	// arm cross-section and the disc scale height, derived in the core where both live --
	// and an authored absolute cannot compete with a floor that knows the field's own
	// thicknesses. The loop bound is derived in the Custom node as int(StepBudget) + 8,
	// because the budget already bounds the count and an authored bound could therefore
	// only truncate: a truncated march looks identical to one that finished, minus the far
	// half of the volume.

	/** Steps the march will actually take, as opposed to the budget it was given. The two
	 *  diverge fast: at growth 4 a budget of 32 resolves to about 13. Diagnostic rather
	 *  than plumbed anywhere, but it is the number that describes the cost. */
	float GetEffectiveStepCount() const
	{
		const float G = FMath::Max(VolumeStepGrowth, 0.0f);
		if (G < UE_SMALL_NUMBER)
		{
			return VolumeStepBudget;
		}
		return VolumeStepBudget * FMath::Loge(1.0f + G) / G;
	}

	// VolumeNoise REMOVED. It named the material's noise texture by path while the
	// compute path held an object reference, so "they must be the same texture" was an
	// invariant nothing enforced -- and became unholdable once NoiseTexture turned
	// procedural and rollable. Both paths now call AGalaxyActor::ResolveNoiseTexture.
};


// -----------------------------------------------------------------------------
// FGalaxyConfigParams -- NEVER ROLLED.
//
// The other half of the split. Everything here is a machine or asset decision: which
// texture to sample, how many slots to allocate, how many march steps to spend. None
// of it says anything about what a galaxy LOOKS like, and none of it may vary per
// galaxy -- SlotCapacity in particular sizes the Niagara buffers, which are allocated
// once per pooled actor.
//
// Authored ONCE on FGalaxySpawnConfig and copied wholesale into every galaxy's
// resolved params. If a member here ever wants to differ between two galaxies, it is
// in the wrong struct.
// -----------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyConfigParams
{
	GENERATED_BODY()

	// TODO: SEE IF WE CAN BRIDGE THE GAP TO REAL WORLD SCALE HERE, I THINK WE HIT PRECISION ISSUES THOUGH... UNIT SCALE AND POTENTIALLY STAR SYSTEM SCALES/PARAMS MAY NEED TO SHIFT

#pragma region Tier Scale Derivation
	/** Fixed absolute largest star-system scale in world cm.
	 *  All galaxies generate star particles in the same physical size
	 *  range regardless of parent galaxy size. With the current value (1e16)
	 *  and the tier depth sequence (1/3/5, spacing 2, ratio 4, 64x total
	 *  spread) DeriveScaleRanges produces:
	 *
	 *    Large: 2.5e15   -> 1e16      (largest systems in the population)
	 *    Mid:   6.25e14  -> 2.5e15
	 *    Small: 1.5625e14 -> 6.25e14  (compact systems)
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
	double MaxEntityScale = 1e16;
#pragma endregion

#pragma region Tiers
	/** SlotCapacity is ALSO the calibration target, not just an allocation size: the
	 *  tier's placement constant is solved so a full cell lands within about 1% of it.
	 *  Changing one changes both the buffer and the star count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;
#pragma endregion

	/** The four march budgets. Entirely config after the polish pass: performance
	 *  decisions, not morphology. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Material")
	FGalaxyMaterialParams MaterialParams;

	/** Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	 *  depth sequence. Delegates to FTierParams::DeriveTierScaleRanges. */
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		FTierParams::DeriveTierScaleRanges(MaxEntityScale, Tiers);
	}

	FGalaxyConfigParams()
	{
		// Large tier: single cell covering the full galaxy extent.
		// NeighborhoodRadius = 0 -> 1x1x1 = 1 slot, exhaustive single-pass.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.GenerationSubdivision = 3;
		LargeTier.SlotCapacity = 6000;

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
};

// -----------------------------------------------------------------------------
// FGalaxyParams -- RESOLVED. What one actor runs on.
//
//   FBaseParams              context:   Seed, ParentColor, UnitScale, Rotation, Extent
//   FGalaxyProceduralParams  rolled, or an archetype's authored Default set
//   FGalaxyConfigParams      never rolled, copied from FGalaxySpawnConfig
//
// OUTPUT, NOT INPUT, once spawning is config-driven. Editing AGalaxyActor::Params in
// the details panel does nothing -- FGalaxySpawnConfig::Generate overwrites the whole
// struct at spawn. That is why it is VisibleAnywhere on the actor: it is the natural
// place to reach for and it would otherwise waste an afternoon.
// -----------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyParams : public FBaseParams
{
	GENERATED_BODY()

	/** Rolled per galaxy, or taken from an archetype's Default set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyProceduralParams Procedural;

	/** Copied wholesale from FGalaxySpawnConfig::Config. Never rolled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy")
	FGalaxyConfigParams Config;

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
	}
};

// -----------------------------------------------------------------------------
// FGalaxyArchetypeEntry -- one archetype's place in THIS universe.
//
// SPLIT OF OWNERSHIP. UGalaxyArchetype says what a spiral IS -- exemplar, ranges,
// texture bag. This says how common spirals are HERE. The same asset can be referenced
// by several universes at different weights, and neither file has to know about the
// other's decision.
//
// An ARRAY rather than named members, so adding "Barred" later is an entry rather than
// a struct edit plus every call site.
// -----------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyArchetypeEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Archetype")
	TObjectPtr<UGalaxyArchetype> Archetype = nullptr;

	/** Off means never selected. Distinct from Weight 0 only in intent, but that is
	 *  worth a separate control: zeroing a weight loses the value you had. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Archetype")
	bool bEnabled = true;

	/** RELATIVE spawn frequency, not a probability -- weights are normalised across
	 *  the enabled entries, so they need not sum to anything. This is what a flag
	 *  cannot give: spirals common, globulars rare. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Archetype", meta = (ClampMin = "0.0"))
	float Weight = 1.0f;
};

// -----------------------------------------------------------------------------
// FGalaxySpawnConfig -- the authored side. Lives on AUniverseActor.
//
// The ONE place galaxy generation is configured. Generate() is the only producer of a
// galaxy's params.
// -----------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxySpawnConfig
{
	GENERATED_BODY()

	/** USE THE AUTHORED EXEMPLAR instead of rolling. Resolves to DefaultArchetype's
	 *  Default set, verbatim, with no ranges applied.
	 *
	 *  The comparison mode: spawn the exemplar, turn this off, and the only thing that
	 *  changed is the roll. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	bool bUseDefaults = false;

	/** Which entry bUseDefaults resolves to. Index into Archetypes. Ignored when
	 *  rolling, which selects by weight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	int32 DefaultArchetype = 0;

	/** Take each galaxy's disc normal from its PARENT PARTICLE's rotation.
	 *
	 *  The universe already generates a uniform unit vector per particle and feeds it to
	 *  Niagara for sprite orientation. Inheriting it means the galaxy faces the way the
	 *  sprite that stood in for it was facing -- so flying in does not snap the disc to
	 *  a new angle -- and it comes correctly distributed for free, which three
	 *  independent Euler ranges never would.
	 *
	 *  WHILE THIS IS SET, an archetype's FieldNormal and any range on it are OVERWRITTEN
	 *  at spawn. That is a deliberate, greppable override rather than a silent one: turn
	 *  it off to author orientation per archetype. FieldSpin is never inherited and
	 *  rolls normally either way. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	bool bInheritParticleOrientation = true;

	/** One line per spawned galaxy naming the archetype and its silhouette
	 *  parameters. The alternative is spawning and eyeballing, which is the whole cost
	 *  of the authoring phase. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	bool bLogGeneration = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	TArray<FGalaxyArchetypeEntry> Archetypes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Spawn")
	FGalaxyConfigParams Config;

	/** Selects an archetype and returns fully resolved params -- including Seed and
	 *  ParentColor, which come from the arguments rather than a separate overlay.
	 *
	 *  An EMPTY Archetypes array, or one whose entries have no asset assigned, yields
	 *  FGalaxyProceduralParams struct defaults. That is the galaxy this system
	 *  generated before proceduralization, and therefore the regression baseline. */
	static FGalaxyParams Generate(const FGalaxySpawnConfig& InConfig,
		FLinearColor InParentColor, int32 InSeed);

	/** Node adapter. THE ONE PLACE that says which octree fields become context, which
	 *  is why it lives here beside the fields rather than at the spawn call site.
	 *
	 *  ParentColor has NO CONSUMER yet: every colour parameter was removed as vestigial
	 *  during the polish pass, and it stays inert until the raymarching pass builds the
	 *  tint chain. It is threaded through so the signatures are right. Do not add
	 *  colour parameters back speculatively to give it somewhere to land. */
	static FGalaxyParams Generate(const FGalaxySpawnConfig& InConfig, const FOctreeNode& InNode)
	{
		return Generate(InConfig, FLinearColor(InNode.Data.Composition), InNode.Data.Seed);
	}

	/** Every referenced archetype validated, once. Returns false if any failed; each
	 *  problem is logged by the archetype itself. */
	static bool Validate(const FGalaxySpawnConfig& InConfig);
};
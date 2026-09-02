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

	/** Which entry of each archetype texture bag this galaxy draws. Separate from Noise so
	 *  that adding a texture to a bag does not disturb noise parameter rolls, and vice
	 *  versa.
	 *
	 *  FOUR CHANNELS, ONE PER BAG, rather than one channel indexing all four. A shared
	 *  channel would lock the four choices together -- every galaxy that drew ridged gas
	 *  would draw the same halo asset alongside it, so the bags would multiply out to as
	 *  many distinct galaxies as the SHORTEST bag has entries. Independent channels give
	 *  the full product, and adding an entry to one bag leaves the other three rolls
	 *  undisturbed.
	 *
	 *  ADD A CHANNEL, NEVER REPURPOSE ONE. A ChannelId carries its stream with it, so
	 *  pointing an existing name at a new meaning leaves every galaxy holding its old index
	 *  for that one draw while everything around it rerolls -- a population that is
	 *  half-old and half-new with nothing saying so. A fresh name rerolls cleanly. */
	inline constexpr uint32 WarpTexDisc = ProcSeed::ChannelId("Galaxy.WarpTexDisc");
	inline constexpr uint32 WarpTexHalo = ProcSeed::ChannelId("Galaxy.WarpTexHalo");
	inline constexpr uint32 NoiseTexDisc = ProcSeed::ChannelId("Galaxy.NoiseTexDisc");
	inline constexpr uint32 NoiseTexHalo = ProcSeed::ChannelId("Galaxy.NoiseTexHalo");

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
// DECLARATION ORDER DOES NOT MIRROR THE DERIVATION'S ARGUMENT LIST, and that is a trade.
// Mirroring it would make a misplaced member visible by reading the two side by side;
// grouping by layer instead makes each float4 in Pack() read one slot per layer --
// (Arms.ArmRadius, Disc.DiscRadius, Bulge.BulgeRadius, ...) -- so the packing's own
// structure is visible in the expression rather than having to be remembered.
//
// THE CHECK THAT REPLACES IT is that every member appears exactly once in Pack(), which is
// the single marshal site. See FGalaxyDensityArgs.
// -----------------------------------------------------------------------------

/** HLSLShim::GalaxyDensityParams is the DERIVED field -- a different type entirely,
 *  declared in GalaxyDensityCore.ush, which GalaxyDataGenerator.cpp compiles INSIDE
 *  namespace HLSLShim. The namespace is what keeps the shim's sqrt/abs/exp from
 *  colliding with the float overloads MSVC's <cmath> puts at global scope, so the type
 *  is namespace-qualified everywhere outside that file. */
namespace HLSLShim { struct GalaxyDensityParams; }

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
	 *  Affects render and placement alike, as the amounts above do.
	 *
	 *  DENOMINATED IN THE ASSET'S VALUE SCALE. The warp fetch reads a SIGNED vector volume
	 *  raw, so the decoded displacement spans [-1,+1]; a UNORM volume decoded as (v - 0.5)
	 *  spans half that and needs twice the amount for the same bend. The defaults here are
	 *  tuned for the signed assets.
	 *
	 *  AN ARCHETYPE CARRIES ITS OWN AMOUNTS and is not retuned by this file. One authored
	 *  against a half-scale convention warps twice as hard until its amounts are halved. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountArms = 0.025f;
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

	/** Halved with the signed warp assets; see WarpAmountArms. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	float WarpAmountDisc = 0.005f;
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
	float WarpAmountBulge = 0.025f;
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

	// NoiseRidged REMOVED. It lerped every centred channel of every modulation fetch
	// toward the 1-2|c| fold, at four lerps and four abs per fetch on the hottest path in
	// the stack, to produce something a ridged noise volume simply IS.
	//
	// IT WOULD ALSO BE ONE VALUE FOR TWO FAMILIES. Gas lanes want a hard ridge and the halo
	// wants none, so any single setting suits one and spoils the other. Gas and halo read
	// separate assets, so ridging is chosen by WHICH ASSET IS BOUND, independently per
	// family, at no per-sample cost.
	//
	// AN ARCHETYPE WANTING RIDGED GAS LANES points NoiseDiscTextures at a ridged bake.
	// Nothing to set here.

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

// NOT A USTRUCT, and it must stay ABOVE the USTRUCT() macro below rather than between it
// and the struct it decorates. UHT binds a USTRUCT() to the very next struct it sees, so a
// plain struct slipped in between silently steals the macro and fails with "Expected a
// GENERATED_BODY() at the start of the struct" -- pointing at this struct rather than at the
// declaration that actually lost its macro.
//
// It is deliberately not reflected: it holds raw non-owning pointers for transport, so
// exposing it to Blueprint or to the GC would imply a lifetime it does not have.
/** The field's four volume textures, RESOLVED. One bundle rather than four loose pointers,
 *  because this set crosses the same boundaries the universe layer's does -- actor to
 *  material, actor to data generator, generator to dispatch, across a thread hop and into a
 *  render command -- and four parameters at each is four chances to pass three of them.
 *
 *  ALL FOUR OR NONE, enforced by IsComplete at every consumer. There is no partial mode: a
 *  missing modulation volume takes its family's noise term to zero and a missing warp
 *  volume straightens that family's displacement, and both change the FIELD rather than
 *  degrading it, so placement against them would not approximate what the material draws.
 *
 *  RAW POINTERS, not UPROPERTY. Transport only; FGalaxyProceduralParams owns the
 *  TObjectPtr references that keep these alive. */
 // =============================================================================
 // THE PACKED ARGUMENT LIST
 // =============================================================================

 /** The arguments of MakeGalaxyDensityParams, in order, packed.
  *
  *  ONE MARSHAL SITE, and that is why it exists. ToDerived and FillShaderParameters each
  *  listed every parameter independently -- the same thirty values written out twice, kept
  *  in step by nothing but care. Both read this struct instead, so adding a parameter to the
  *  derivation breaks Pack() at compile time, and Pack() is the only place the packing
  *  decisions live.
  *
  *  THE MATERIAL'S CUSTOM NODE IS STILL A THIRD LIST, and no C++ change reaches it: a Custom
  *  node fails at SHADER compile rather than at build, so a parameter missed there surfaces
  *  as a red material with nothing pointing at the cause. Check it whenever the derivation
  *  signature changes. What this removes is the two lists that could drift SILENTLY; what
  *  remains is the one that fails loudly, if late.
  *
  *  NOT UPLOADED AS A CONSTANT BUFFER. That would require HLSL's cbuffer packing to agree
  *  with the C++ layout member for member, and one float3 straddling a 16-byte boundary
  *  shifts every field after it with no diagnostic at all -- the galaxy would simply come
  *  out wrong. These are raw inputs and the derivation runs in the shader, costing sixteen
  *  arm hashes and a tan per thread against a full field evaluation per thread.
  *
  *  LEAF NAMES STILL MATCH THE MATERIAL PIN NAMES -- Arms.ArmRadius feeds "ArmRadius" -- so
  *  grouping the members did not weaken the three-way check. */
struct FGalaxyDensityArgs
{
	// --- LAYER GEOMETRY --- (arms, disc, bulge, background)
	FVector4f LateralScale = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f VerticalScale = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f LayerDensity = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f NoiseAmount = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f WarpAmount = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);

	// --- ARMS ---
	FVector4f ArmAsym = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f SpiralTwist = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);

	// --- VOID ---
	FVector3f CentralVoid = FVector3f::ZeroVector;

	// --- NOISE FRAME ---
	FVector4f NoiseScale = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f WarpScale = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector4f NoiseChannelWeights = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
	FVector3f NoiseOffset = FVector3f::ZeroVector;

	// --- SCALARS, in derivation order ---
	float BoundsFadeStart = 0.0f;
	float DiscScaleLengthRatio = 0.0f;
	float DiscVerticalFalloff = 0.0f;
	float DiscFlare = 0.0f;
	float DiscWarpAmplitude = 0.0f;
	float DiscWarpPhase = 0.0f;
	float DiscWarpTwist = 0.0f;
	float DiscLopsidedAmount = 0.0f;
	float DiscLopsidedPhase = 0.0f;
	float ArmCount = 0.0f;
	float ArmAsymSeed = 0.0f;
	float ArmProfileExponent = 0.0f;
	float ArmRadialGrowth = 0.0f;
	float ArmHostFalloff = 0.0f;
	float BulgeConcentration = 0.0f;
	float BackgroundConcentration = 0.0f;

	/** ALWAYS 1. Placement samples the volume textures unconditionally -- that is the whole
	 *  point of the GPU path -- and a switch that turned it off here would make the placed
	 *  field differ from the drawn one with nothing to say so. The material's EnableNoise
	 *  static switch controls the RENDER only and is a visualisation aid. */
	float NoiseEnable = 1.0f;

	// --- ORIENTATION --- (normal xyz, spin w)
	FVector4f FieldOrientation = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
};


struct FGalaxyFieldTextures
{
	UTexture* WarpDisc = nullptr;
	UTexture* WarpHalo = nullptr;
	UTexture* NoiseDisc = nullptr;
	UTexture* NoiseHalo = nullptr;

	bool IsComplete() const
	{
		return WarpDisc != nullptr && WarpHalo != nullptr
			&& NoiseDisc != nullptr && NoiseHalo != nullptr;
	}

	/** Which ones are missing, for a log line that says something actionable. Empty when
	 *  the set is complete. */
	FString DescribeMissing() const
	{
		TArray<FString> Missing;
		if (!WarpDisc) { Missing.Add(TEXT("WarpTexDisc")); }
		if (!WarpHalo) { Missing.Add(TEXT("WarpTexHalo")); }
		if (!NoiseDisc) { Missing.Add(TEXT("NoiseTexDisc")); }
		if (!NoiseHalo) { Missing.Add(TEXT("NoiseTexHalo")); }
		return FString::Join(Missing, TEXT(", "));
	}
};

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyProceduralParams
{
	GENERATED_BODY()

#pragma region Ungrouped -- the two knobs reached for most often, kept at the top
	/** THE FOUR VOLUME TEXTURES BOTH THE DISPATCH AND THE MATERIAL SAMPLE. ALL REQUIRED.
	 *
	 *  One texture served every fetch until the split; the field now takes one per fetch,
	 *  divided by ROLE and by FAMILY at once:
	 *
	 *    WarpTexDisc     disc warp       signed vector volume   displaces arms and disc
	 *    WarpTexHalo    halo warp       signed vector volume   displaces bulge, background
	 *    NoiseTexDisc    disc modulation UNORM multinoise       lanes and filament
	 *    NoiseTexHalo   halo modulation UNORM multinoise       grain in bulge and halo
	 *
	 *  PROCEDURAL, NOT CONFIG. They are inputs to the field evaluation, so swapping one
	 *  changes the look and the density the placement pass measures -- exactly as much a
	 *  morphology decision as BulgeConcentration. They sit here so the archetype's bags can
	 *  be drawn from per galaxy.
	 *
	 *  CATEGORICAL, NOT CONTINUOUS. A Min/Max range means nothing for a texture, so each is
	 *  drawn from its own candidate ARRAY on UGalaxyArchetype and these members hold the
	 *  resolved choices. FOUR BAGS, FOUR INDEPENDENT ROLLS, so a galaxy can pair ridged gas
	 *  lanes with a smooth halo without either bag having to enumerate the combinations.
	 *
	 *  WHICH IS WHERE RIDGING LIVES. A uniform lerping every channel toward a fold at sample
	 *  time would be one value shared by gas and halo, and no setting suits both. Point
	 *  NoiseDiscTextures at ridged bakes and NoiseHaloTextures at smooth ones and each
	 *  family gets what it wants, for free.
	 *
	 *  THE WARP PAIR MUST BE SIGNED, values in [-1,1]. The shader applies their channels
	 *  directly as displacements with no centring step at all. A UNORM volume cannot carry
	 *  that: read as signed it has a mean of +0.5, and the family is bodily translated by
	 *  half its warp amount rather than locally displaced -- the disc looks like a disc,
	 *  just not concentric with the bulge. Check the asset's format.
	 *
	 *  Placement is GPU-only and the dispatch samples all four. With any one unset the
	 *  galaxy generates NOTHING; there is no analytic path behind it.
	 *  AGalaxyActor::InitializeData substitutes its defaults for whichever are unset, which
	 *  is what keeps an unauthored archetype from silently producing an empty galaxy. NOT
	 *  the constructor: ReInit assigns Params wholesale, so anything the constructor wrote
	 *  into Params is gone by the time a pooled galaxy generates.
	 *
	 *  Set NEVER STREAM on all four assets. GalaxyDensityCore.ush reads mip 0 on both paths,
	 *  but the material handles streaming residency and a compute dispatch does not: if
	 *  mip 0 is not resident when the dispatch runs it reads whatever is, and placement
	 *  silently stops matching the render.
	 *
	 *  The samplers here are Trilinear/Wrap. If the material's Custom node pins carry
	 *  anything else -- and they inherit whatever addressing each ASSET is saved with -- the
	 *  two paths read different values at the same coordinate, which shows up as a small
	 *  plausible-looking difference rather than an obvious break. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	TObjectPtr<UVolumeTexture> WarpTexDisc = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	TObjectPtr<UVolumeTexture> WarpTexHalo = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	TObjectPtr<UVolumeTexture> NoiseTexDisc = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field")
	TObjectPtr<UVolumeTexture> NoiseTexHalo = nullptr;

	/** The four as a transport bundle, for the data generator and the dispatch.
	 *
	 *  BUILT ON DEMAND rather than cached, so it cannot go stale against the members above
	 *  after a roll or a pooled ReInit rewrites them. It is four pointer copies. */
	FGalaxyFieldTextures GetFieldTextures() const
	{
		FGalaxyFieldTextures Out;
		Out.WarpDisc = WarpTexDisc;
		Out.WarpHalo = WarpTexHalo;
		Out.NoiseDisc = NoiseTexDisc;
		Out.NoiseHalo = NoiseTexHalo;
		return Out;
	}

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
	 *  So it lands where BudgetScale is computed, as capacity * StarDensityScale / divisor
	 *  -- see GalaxyDataGenerator::GetTierBudgetScale. Buffer sizes never move, and a sparse
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

	/** THE ONLY PLACE THE PACKING DECISIONS LIVE. Both consumers -- the CPU derivation and
	 *  the compute parameter fill -- read this rather than packing again. */
	FGalaxyDensityArgs Pack() const
	{
		FGalaxyDensityArgs Out;

		Out.LateralScale = FVector4f(
			Arms.ArmRadius, Disc.DiscRadius, Bulge.BulgeRadius,
			FGalaxyBackgroundParams::BackgroundRadius);

		Out.VerticalScale = FVector4f(
			Arms.ArmVerticalRatio, Disc.DiscVerticalRatio,
			Bulge.BulgeVerticalRatio, Background.BackgroundVerticalRatio);

		Out.LayerDensity = FVector4f(
			Arms.ArmDensity, Disc.DiscDensity, Bulge.BulgeDensity, Background.BackgroundDensity);

		Out.NoiseAmount = FVector4f(
			Arms.ArmNoiseAmount, Disc.DiscNoiseAmount,
			Bulge.BulgeNoiseAmount, Background.BackgroundNoiseAmount);

		Out.WarpAmount = FVector4f(
			Arms.WarpAmountArms, Disc.WarpAmountDisc,
			Bulge.WarpAmountBulge, Background.WarpAmountBackground);

		Out.ArmAsym = FVector4f(
			Arms.ArmAsymPitch, Arms.ArmAsymPhase, Arms.ArmAsymDensity, Arms.ArmAsymLength);

		Out.SpiralTwist = FVector4f(
			Arms.ArmPitchAngle, Arms.ArmPitchTightening, Arms.ArmPhaseOffset,
			Noise.HaloTwistInherit);

		Out.CentralVoid = FVector3f(
			Void.CentralVoidRadius, Void.CentralVoidAmount, Void.CentralVoidExponent);

		Out.NoiseScale = FVector4f(
			Noise.NoiseDiscLateralScale, Noise.NoiseDiscVerticalScale,
			Noise.NoiseHaloLateralScale, Noise.NoiseHaloVerticalScale);

		Out.WarpScale = FVector4f(
			Noise.WarpDiscLateralScale, Noise.WarpDiscVerticalScale,
			Noise.WarpHaloLateralScale, Noise.WarpHaloVerticalScale);

		// FLinearColor is RGBA in memory but the shader reads it as xyzw against
		// Noise.NoiseChannelWeights, so the component order is spelled out rather than
		// relying on a reinterpret that would silently reorder if the type changed.
		Out.NoiseChannelWeights = FVector4f(
			Noise.NoiseChannelWeights.R, Noise.NoiseChannelWeights.G,
			Noise.NoiseChannelWeights.B, Noise.NoiseChannelWeights.A);

		Out.NoiseOffset = FVector3f(
			Noise.NoiseOffset.X, Noise.NoiseOffset.Y, Noise.NoiseOffset.Z);

		Out.BoundsFadeStart = Background.BoundsFadeStart;
		Out.DiscScaleLengthRatio = Disc.DiscScaleRatio;
		Out.DiscVerticalFalloff = Disc.DiscVerticalFalloff;
		Out.DiscFlare = Disc.DiscFlare;
		Out.DiscWarpAmplitude = Disc.DiscWarpAmplitude;
		Out.DiscWarpPhase = Disc.DiscWarpPhase;
		Out.DiscWarpTwist = Disc.DiscWarpTwist;
		Out.DiscLopsidedAmount = Disc.DiscLopsidedAmount;
		Out.DiscLopsidedPhase = Disc.DiscLopsidedPhase;
		Out.ArmCount = Arms.ArmCount;
		Out.ArmAsymSeed = Arms.ArmAsymSeed;
		Out.ArmProfileExponent = Arms.ArmProfileExponent;
		Out.ArmRadialGrowth = Arms.ArmRadialGrowth;
		Out.ArmHostFalloff = Arms.ArmHostFalloff;
		Out.BulgeConcentration = Bulge.BulgeConcentration;
		Out.BackgroundConcentration = Background.BackgroundConcentration;

		Out.FieldOrientation = FVector4f(
			Orientation.FieldNormal.X, Orientation.FieldNormal.Y,
			Orientation.FieldNormal.Z, Orientation.FieldSpin);

		return Out;
	}

	/** The CPU derivation, from the packed arguments. Defined in GalaxyDataGenerator.cpp,
	 *  the one translation unit that compiles the shim and the .ush. */
	HLSLShim::GalaxyDensityParams ToDerived() const;

	/** Fills a compute shader parameter struct from the packed arguments, member for
	 *  member. The shader-side names match MakeGalaxyDensityParams's parameter names, so a
	 *  parameter added to the derivation breaks this at compile time on both sides.
	 *
	 *  A TEMPLATE on purpose. Taking FGalaxyEntityGenCS::FParameters& directly would make
	 *  this header depend on GalaxyEntityGen.h, which depends on this one -- a cycle, and
	 *  one that drags RenderCore into every translation unit that only wanted the authored
	 *  struct. */
	template <typename TShaderParams>
	void FillShaderParameters(TShaderParams& Out) const
	{
		const FGalaxyDensityArgs A = Pack();

		Out.InLateralScale = A.LateralScale;
		Out.InVerticalScale = A.VerticalScale;
		Out.InLayerDensity = A.LayerDensity;
		Out.InNoiseAmount = A.NoiseAmount;
		Out.InWarpAmount = A.WarpAmount;
		Out.InArmAsym = A.ArmAsym;
		Out.InSpiralTwist = A.SpiralTwist;
		Out.InCentralVoid = A.CentralVoid;
		Out.InNoiseScale = A.NoiseScale;
		Out.InWarpScale = A.WarpScale;
		Out.InNoiseChannelWeights = A.NoiseChannelWeights;
		Out.InNoiseOffset = A.NoiseOffset;
		Out.InBoundsFadeStart = A.BoundsFadeStart;
		Out.InDiscScaleLengthRatio = A.DiscScaleLengthRatio;
		Out.InDiscVerticalFalloff = A.DiscVerticalFalloff;
		Out.InDiscFlare = A.DiscFlare;
		Out.InDiscWarpAmplitude = A.DiscWarpAmplitude;
		Out.InDiscWarpPhase = A.DiscWarpPhase;
		Out.InDiscWarpTwist = A.DiscWarpTwist;
		Out.InDiscLopsidedAmount = A.DiscLopsidedAmount;
		Out.InDiscLopsidedPhase = A.DiscLopsidedPhase;
		Out.InArmCount = A.ArmCount;
		Out.InArmAsymSeed = A.ArmAsymSeed;
		Out.InArmProfileExponent = A.ArmProfileExponent;
		Out.InArmRadialGrowth = A.ArmRadialGrowth;
		Out.InArmHostFalloff = A.ArmHostFalloff;
		Out.InBulgeConcentration = A.BulgeConcentration;
		Out.InBackgroundConcentration = A.BackgroundConcentration;
		Out.InNoiseEnable = A.NoiseEnable;
		Out.InFieldOrientation = A.FieldOrientation;
	}
};

/** Material-side parameters for the volumetric proxy.
 *
 *  PROVISIONAL. The galaxy raymarcher is a debug marcher; this set is sized for it and
 *  wants rebuilding against a shipping marcher. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyMaterialParams
{
	GENERATED_BODY()

	/** THE MARCH CONTROLS. BOTH of them reach the material, so the baseline lives here
	 *  rather than only on the material asset.
	 *
	 *  THE STEPPING RULE IS THE UNIVERSE LAYER'S:
	 *
	 *      baseStep = span / StepBudget
	 *      h        = baseStep * (1 + StepGrowth * t01)
	 *
	 *  with t01 the progress along the marched span rather than the distance from the
	 *  camera. A camera-relative step keeps every step a constant size ON SCREEN, which is
	 *  a genuine property -- but it makes the count depend on where the camera is,
	 *  collapsing toward chord/floor on close approach. With four density marchers able to
	 *  be on screen at once, per-layer cost has to be SET rather than discovered.
	 *
	 *  PERFORMANCE CONTROLS, NOT LOOK CONTROLS. LayerDensity is an optical depth normalised
	 *  by path length, so changing either of these does not require retuning a density.
	 *  Candidates for the game's quality settings. */

	 /** A CEILING ON THE STEP COUNT, NOT A FLOOR. Growth only ever lengthens steps, so every
	  *  step is at least span / StepBudget and the actual count is StepBudget * ln(1+g)/g --
	  *  equal to the budget only at growth 0, 0.69 of it at growth 1, 0.55 at growth 2. See
	  *  GetEffectiveStepCount.
	  *
	  *  READ IT AS THE RESOLVED COUNT, NOT THE BUDGET. 192 at growth 2 resolves to about 105
	  *  steps, and pays that whatever the viewer is doing -- which is the property a
	  *  camera-relative rule cannot offer, and the reason this number looks larger than a
	  *  step count would. */
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

	// NO STEP FLOOR AND NO ITERATION BOUND HERE, deliberately. The step floor is
	// P.MinFeatureStep -- half the narrowest of the arm cross-section and the disc scale
	// height, derived in the core where both live -- and an authored absolute cannot compete
	// with a floor that knows the field's own thicknesses. The loop bound is derived in the
	// Custom node as int(StepBudget) + 8, since the budget already bounds the count and an
	// authored bound could therefore only truncate: a truncated march looks identical to one
	// that finished, minus the far half of the volume.

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

	// NO TEXTURE PATHS HERE. The material and the compute dispatch must sample the
	// IDENTICAL assets, and a path on this struct beside an object reference on the other
	// path is an invariant nothing can enforce -- doubly so once the textures became
	// rollable per archetype. Both paths take the bundle
	// AGalaxyActor::ResolveFieldTextures returns.
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

	// TODO: the star-system scale range below is roughly three orders under real scale --
	// see the references on MaxEntityScale. Closing that gap is not a matter of raising the
	// number: UnitScale and the star-system layer's own extents move with it, and the
	// coordinate chain has to be re-checked at the new magnitudes before anything is
	// authored. Trigger to close: the star-system layer getting its own entity-gen pass,
	// which is where the precision question has to be answered anyway.

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
		LargeTier.SlotCapacity = 6000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 8000;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 6000;

		// GENERATION SUBDIVISION, READ OFF THE C/P RATIO IN THE BATCH LOG. Probes and
		// candidates are both full field evaluations, so their ratio is the whole cost
		// argument: below about 1 the probes have overtaken placement and the tier wants one
		// level fewer, above about 9 it wants one more.
		//
		// THE TIERS PULL IN OPPOSITE DIRECTIONS because their grids already differ by four
		// per depth step before any subdivision. The Large tier's parent grid collapses to a
		// SINGLE cell -- GridDepth 1 against GridExtentMultiplier 4 puts every neighbour
		// entirely outside the unit sphere -- so its subdivision is the whole generation
		// grid, and at level 2 that is 64 cells across an entire galaxy, four per axis.
		// Mid and Small subdivide a grid that is already fine, and over-subdividing them
		// spends the dispatch on probes: measured C/P 0.02 on the Mid tier at level 2, which
		// is fifty probes per candidate.
		//
		// A CELL TOO LARGE FOR ITS FIELD FAILS QUIETLY. The envelope is a max over the
		// cell's probes, so the more density range a cell contains the further short that
		// max falls -- and the acceptance ratio then saturates, which is the one place the
		// per-cell normalisation stops cancelling. It costs the cell candidates and caps its
		// acceptance at once, so the brightest cells under-deliver and the arms and core
		// come out holed while the raymarch still draws structure through them.
		//
		// Level 3 is FTierParams::MaxGenerationSubdivision, so the Large tier is at the cap;
		// a finer grid there needs a deeper GridDepth instead.
		LargeTier.GenerationSubdivision = 3;
		MidTier.GenerationSubdivision = 1;
		SmallTier.GenerationSubdivision = 1;

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
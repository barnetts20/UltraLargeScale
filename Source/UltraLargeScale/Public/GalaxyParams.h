#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "GalaxyParams.generated.h"
// GalaxyDensityParams.h  (replaces FGalaxyDensityParams in GalaxyParams.h)
//
// One UPROPERTY per input of MakeGalaxyDensityParams, in the same order. Nothing is
// combined or scaled here: correlations such as "arm width is a fraction of the disc
// scale height" are resolved inside the shared derivation, so this struct and the
// material's pin set are the same list of raw values.
//
// Keeping it flat rather than packing float4s is deliberate: the details panel is
// easier to drive, and the packing into vectors happens in one place, ToDerived().
// Declared in GalaxyDensityCore.ush, which GalaxyDataGenerator.cpp compiles INSIDE
// namespace GalaxyHLSL. The namespace is what keeps the shim's sqrt/abs/exp from
// colliding with the float overloads MSVC's <cmath> puts at global scope, so the type
// is namespace-qualified everywhere outside that file.
namespace GalaxyHLSL { struct GalaxyDensityParams; }

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyDensityParams
{
	GENERATED_BODY()

#pragma region Lateral scales
	/** Arm half-width as a FRACTION OF THE DISC SCALE HEIGHT. Arms are thinner than
	 *  the stellar disc they sit in, so this is a ratio rather than an absolute. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float ArmRadius = 0.75f;

	/** Disc radius in normalized space. Doubles as the arm system's radial
	 *  reference: arms live in the disc and have no radius of their own. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float DiscRadius = 0.7f;

	/** Bulge zero-density radius, as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float BulgeRadius = 0.33f;

	/** Background zero-density radius, absolute in normalized space. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float BackgroundRadius = 1.0f;
#pragma endregion

#pragma region Vertical ratios
	/** All four are axis ratios; smaller flattens. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float ArmVerticalRatio = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float DiscVerticalRatio = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float BulgeVerticalRatio = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Scale", meta = (ClampMin = "0.0"))
	float BackgroundVerticalRatio = 0.5f;
#pragma endregion

#pragma region Layer densities
	/** OPTICAL DEPTH per layer, not a volume density: what you would measure looking
	 *  through that layer along its natural axis. The derivation divides each by its
	 *  own path length, which is what makes these four comparable to each other and
	 *  independent of the march step count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Layers", meta = (ClampMin = "0.0"))
	float ArmDensity = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Layers", meta = (ClampMin = "0.0"))
	float DiscDensity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Layers", meta = (ClampMin = "0.0"))
	float BulgeDensity = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Layers", meta = (ClampMin = "0.0"))
	float BackgroundDensity = 0.1f;
#pragma endregion

#pragma region Noise response
	/** Multiplicative depth: density *= 1 + Amount * n. GPU only -- the CPU path
	 *  evaluates the analytic field, so these do not affect star placement. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float ArmNoiseAmount = 0.333f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float DiscNoiseAmount = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float BulgeNoiseAmount = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float BackgroundNoiseAmount = 1.0f;

	/** Positional warp per layer. Signed: negative flips the displacement direction.
	 *  GPU only, for the same reason as the amounts above. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpAmountArms = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpAmountDisc = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpAmountBulge = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpAmountBackground = 0.0f;
#pragma endregion

#pragma region Arm asymmetry
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmAsymPitch = 0.175f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmAsymPhase = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmAsymDensity = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmAsymLength = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmAsymSeed = 211.588882f;
#pragma endregion

#pragma region Spiral
	/** Degrees at the disc rim; sign sets chirality. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "-89.0", ClampMax = "89.0"))
	float ArmPitchAngle = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "0.0"))
	float ArmPitchTightening = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmPhaseOffset = 0.0f;

	/** How much of the spiral twist the bulge/background noise frame inherits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float HaloTwistInherit = 1.0f;

	/** CLAMPED TO GALAXY_MAX_ARMS (16) by the derivation. Values above that are
	 *  silently truncated, not honoured. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "1.0", ClampMax = "16.0"))
	float ArmCount = 16.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "0.0"))
	float ArmProfileExponent = 0.5f;

	/** Coefficient on DiscFlare: arms widen outward in step with the disc thickening.
	 *  1.0 means they taper together. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "0.0"))
	float ArmRadialGrowth = 2.0f;

	/** How strongly arm strength follows the disc's radial profile.
	 *  0 = independent, 1 = fades exactly with the disc, >1 faster. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms", meta = (ClampMin = "0.0"))
	float ArmHostFalloff = 1.5f;
#pragma endregion

#pragma region Disc shape and asymmetry
	/** Radial scale length as a MULTIPLE OF THE BULGE RADIUS. The bulge-to-disc
	 *  scale ratio is a measured quantity; DiscRadius is only a truncation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc", meta = (ClampMin = "0.0"))
	float DiscScaleRatio = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc", meta = (ClampMin = "0.1"))
	float DiscVerticalFalloff = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc", meta = (ClampMin = "0.0"))
	float DiscFlare = 10.183177f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscWarpAmplitude = 0.112f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscWarpPhase = 0.0f;

	/** Radians of node-line precession across the disc. Past ~2.5 the warp folds
	 *  back on itself and reads as corrugation rather than an S-curve. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscWarpTwist = 2.276695f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscLopsidedAmount = 0.72f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscLopsidedPhase = 0.074667f;
#pragma endregion

#pragma region Profile exponents
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge", meta = (ClampMin = "0.0"))
	float BulgeConcentration = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background", meta = (ClampMin = "0.0"))
	float BackgroundConcentration = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BoundsFadeStart = 0.33f;
#pragma endregion

#pragma region Central void
	/** Radius as a fraction of DiscRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Void", meta = (ClampMin = "0.0"))
	float CentralVoidRadius = 0.033f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Void", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float CentralVoidAmount = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Void", meta = (ClampMin = "0.0"))
	float CentralVoidExponent = 1.0f;
#pragma endregion

#pragma region Noise field (GPU only)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float NoiseDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float NoiseDiscVerticalScale = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float NoiseHaloLateralScale = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float NoiseHaloVerticalScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpDiscLateralScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpDiscVerticalScale = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpHaloLateralScale = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	float WarpHaloVerticalScale = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	FLinearColor NoiseChannelWeights = FLinearColor(-1.0f, -0.7f, -0.4f, -0.4f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	FVector3f NoiseOffset = FVector3f(1.0f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise", meta = (ClampMin = "1.0", ClampMax = "2.0"))
	float NoiseOctaves = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float NoiseRidged = 0.0f;

	/** Renderer only. The CPU path always evaluates the analytic field, so leaving
	 *  this on does not make star placement disagree with itself -- it makes the
	 *  RENDER disagree with the placement by the amount the noise displaces things. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Noise")
	bool bEnableNoise = true;
#pragma endregion

#pragma region Render and spawn mapping
	/** Global sigma multiplier in the raymarch. Renderer only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Render", meta = (ClampMin = "0.0"))
	float MasterDensityScale = 1.0f;

	/** Render-side shaping exponent, applied after Sample. No CPU counterpart. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Render", meta = (ClampMin = "0.0"))
	float MasterDensityPower = 1.0f;

	/** THE DENSITY AT WHICH A CANDIDATE ALWAYS SPAWNS.
	 *
	 *  The field is an optical depth and is deliberately unbounded -- it peaks near
	 *  260 at the default tuning while 83% of the volume sits under 0.01. Feeding it
	 *  to the rejection test directly would accept every candidate above 1.0, which
	 *  is the arms, the inner disc and the whole bulge, erasing exactly the structure
	 *  the field describes.
	 *
	 *  Dividing by this reference restores a probability. Star count scales roughly
	 *  as 1/reference, so retune tier capacities alongside it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Spawn", meta = (ClampMin = "0.001"))
	float SpawnDensityReference = 10.0f;

	/** Compresses the spawn probability rather than scaling it linearly:
	 *      0 = linear,  d / reference        star density tracks gas density
	 *      1 = Beer,    1 - exp(-d / ref)    tracks apparent brightness
	 *  Linear is the physically faithful choice; compression fills dense cores more
	 *  evenly and stops the faint halo from spawning almost nothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Spawn", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SpawnCompression = 0.0f;

	/** Normaliser applied before quantising to the volume texture, which is a byte
	 *  channel and would otherwise saturate to white everywhere the field exceeds 1.
	 *  Only used by the bake path. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bake", meta = (ClampMin = "0.001"))
	float BakeDensityReference = 250.0f;
#pragma endregion

	/** Pack into the shared derivation. Defined in GalaxyDataGenerator.cpp, the one
	 *  translation unit that compiles the shim and the .ush. */
	GalaxyHLSL::GalaxyDensityParams ToDerived() const;

	/** Map a raw field value to a spawn probability in [0,1]. */
	float ToSpawnProbability(float InDensity) const;
};

//TODO: DESCRIPTION COMMENT - THIS FORMS THE PARAMETER INTERFACE WITH THE RAYMARCH MATERIAL OUR RAYMARCHER IS NOT MATURE AT THIS POINT SO THIS WILL CHANGE, MANY OF THESE VALUES CURRENTLY HAVE NO EFFECT - THATS FINE, IT WILL NEED REFACTORING WHEN WE DO A GALAXY RAYMARCHER DEEP DIVE ANYWAY
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyMaterialParams
{
	GENERATED_BODY()
	/** Bake the density field into a pseudovolume texture.
	 *
	 *  Only the OLD raymarch material reads it; the analytic material evaluates the
	 *  field directly. Off by default: baking costs a full 256^3 evaluation plus a
	 *  4096^2 upscale and upload on every spawn, and ~64 MB resident per galaxy.
	 *
	 *  Kept as an INDEPENDENT cross-check of the CPU implementation rather than as a
	 *  render path -- the CPU evaluates and bakes, the old material renders the
	 *  texture, and that should agree with the analytic material evaluating the same
	 *  parameters live. Set BakeDensityReference on the density params first: the
	 *  field is an unbounded optical depth and this channel is a byte. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	bool bBakeDensityVolume = false;

	/** March steps across the chord. Step length adapts to chord length, so this is
	 *  quality, not scale -- and because LayerDensity is an optical depth normalised
	 *  by path, changing it no longer requires retuning any density. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	float VolumeMaxSteps = 64.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	int32 DensityVolumeResolution = 256;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeAmbientColor = FLinearColor(1, 1, 1, 1);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeCoolShift = FLinearColor(.2, .5, .8);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeHotShift = FLinearColor(.5, 1.5, 3);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeHueVariance = .1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeHueVarianceScale = .5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeSaturationVariance = .1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeTemperatureInfluence = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeTemperatureScale = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeDensity = .5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeWarpAmount = .00;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeWarpScale = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VolumeNoise = "/UltraLargeScale/VolumeTextures/VT_PerlinWorley_Balanced";
};


/** Galaxy-layer generation parameters; extends FBaseParams (mirrors
 *  FUniverseParams structure). */

USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FGalaxyParams : public FBaseParams
{
	GENERATED_BODY()
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

#pragma region Large Tier SDF Culling Grid

	/** Grid depth used to subdivide the galaxy volume for SDF-based cell
	 *  culling during large tier generation. Cells whose every corner has
	 *  zero composite density are skipped entirely, concentrating candidate
	 *  sampling on arms/disc/bulge.
	 *
	 *  Depth N produces (2^N)^3 cells over the GridExtentMultiplier-scaled
	 *  volume. Depth 3 = 8^3 = 512 cells. Higher values give finer culling
	 *  at the cost of more corner evaluations (8 * CellCount SDF samples).
	 *  Values of 2-4 are recommended; 5+ rarely improves acceptance rate
	 *  enough to justify the overhead. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Large Tier")
	int32 LargeTierCullDepth = 4;

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
		LargeTier.SlotCapacity = 1000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 250;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 250;

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
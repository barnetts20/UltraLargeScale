#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "GalaxyParams.generated.h"

/** Analytic density-field parameters for galaxy generation: bounds fade, noise
 *  power, and the bulge/disc/arms/background spiral density layers. */
USTRUCT(BlueprintType)
struct SVO_API FGalaxyDensityParams
{
	GENERATED_BODY()

	/** Exponent applied to noise values during volume texture sampling in the raymarch material.
	 *  Higher = sharper contrast between noise dense and empty regions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	float NoisePower = 2.0f;

#pragma region Bounds Fade
	/** A global multiplier applied to the entire composite density to prevent
	 *  hard transitions at the volume bounds. The fade is spherical, applied in
	 *  normalized space based on distance from the origin. */
	 /** Fraction of the normalized extent [0, 1] at which the bounds fade begins.
	  *  Below this distance, density is unmodified. Above it, density fades
	  *  to zero via smoothstep reaching zero at the cube edge (distance = 1).
	  *  0.67 = fade starts at 2/3 of the way from center to edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	float BoundsFadeStart = 0.7f;

#pragma endregion

#pragma region Spiral Density Field Parameters
	/** Analytic density function driving both particle rejection sampling and
	 *  volume texture baking. The galaxy shape is composed of four additive
	 *  layers: bulge, disc, arms, and background. All coordinates are in
	 *  normalized space [-1, 1] where 1 = Extent.
	 *
	 *  The arm structure works by "un-twisting" the query point back to a
	 *  straight-arm reference frame, then measuring angular distance to the
	 *  nearest arm. */

#pragma region Bulge
	 /** The bulge uses a Hernquist density profile evaluated in oblate
	  *  (vertically squashed) coordinates. */

	  /** Scale radius for the Hernquist profile, in normalized [0,1] space.
	   *  Smaller = sharper core concentration. 0.1 = tight core, 0.3 = diffuse. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeScaleRadius = 1.0f;

	/** Hard radial cutoff for the bulge, in normalized space.
	 *  Beyond this the bulge contributes zero density. Prevents the
	 *  Hernquist 1/r^4 tail from polluting the disc/arm region. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeCutoffRadius = 1.0f;

	/** Peak density of the bulge at the center (r approaching 0) [0, 1].
	 *  Zeroed for arm/disc iteration - set to 0.8-1.0 when compositing. */
	 // TODO: THIS VALUE IS VERY LOW DUE TO HAVING AN OUTSIZED IMPACT ON THE DENSITY COMPOSITE... SAFE TO SAY THIS IS NOT THE ACTUAL PEAK DENSITY BUT SEEMS TO HAVE MORE OF A GEOMETRIC EFFECT
	 // ANALISE AND SEE IF WE CAN COME UP WITH A MORE STRAIGHTFORWARD USER FACING PARAM FOR THIS
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgePeakDensity = 0.04f;

	/** Vertical squash factor for the bulge. 1.0 = sphere, < 1.0 = oblate.
	 *  Applied to Z before computing the Hernquist radius.
	 *  Maps to legacy BulgeAxisScale.Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeVerticalSquash = 0.6f;

#pragma endregion

#pragma region Disc
	/** The disc uses a separable analytic profile: exponential radial decay
	 *  multiplied by an exp(-|z/h|^falloff) vertical profile. */

	 /** Radial scale of the disc, in normalized space. 1.0 = extends to Extent.
	  *  Also used as the hard radial cutoff for the disc cylinder. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscRadius = 1.0f;

	/** Vertical scale height of the disc, as a fraction of DiscRadius.
	 *  Acts as the sech^2/exp scale height: ~76% of disc mass lies within
	 *  1x this height above/below the plane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscHeightRatio = 0.3f;

	/** Peak density of the disc at the center (r=0, z=0) [0, 1]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscBaseDensity = 0.5f;

	/** Exponential radial scale length, as a fraction of DiscRadius.
	 *  Controls how quickly density drops with radius.
	 *  0.2 = tight nucleus-concentrated disc, 0.5 = very diffuse. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscRadialScaleLength = 1.0f;

	/** Vertical profile exponent. Applied as exp(-(|z|/h)^DiscVerticalFalloff).
	 *  1.0 = exponential / isothermal sheet (sharp equatorial peak).
	 *  2.0 = Gaussian (softer, better for a thick stellar disc). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscVerticalFalloff = 1.0f;

#pragma endregion

#pragma region Arms
	/** SampleArmSDF returns unsigned distance from the arm centerline in
	 *  normalized space; SampleDensity remaps it to [0, 1] via the core/envelope
	 *  thresholds. */

	 /** Number of spiral arms. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	int32 ArmCount = 2;

	/** Twist strength in radians at the disc edge (r = DiscRadius).
	 *  Higher = more wound spirals. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmTwistStrength = 4.0f;

	/** Core twist boost - extra winding near the center that falls off
	 *  exponentially. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreTwistStrength = 8.0f;

	/** Core twist radius - controls how quickly the core twist boost decays. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreTwistRadius = 0.2f;

	/** Radial start of the arms, as a fraction of DiscRadius.
	 *  Below this radius, arms fade out (merge into bulge). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmStartRadius = 0.05f;

	/** Width of the blend zone where arms fade in from ArmStartRadius,
	 *  in normalized space. Controls how sharp the inner arm boundary is. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmStartBlendWidth = 0.15f;

	/** Radial Growth: as distance along the arm increases (inner -> outer edge),
	 *  three properties evolve together:
	 *    1. Envelope grows (arm widens) by ArmRadialGrowth factor
	 *    2. Peak density drops inversely proportional to growth (mass conservation)
	 *    3. Vertical squash relaxes toward ArmVerticalSquashOuter
	 *  All parameterized by t = (rXY - armStart) / (discR - armStart), 0 at the
	 *  inner edge to 1 at the disc rim. */

	 /** Vertical squash coefficient for the arm distance calculation.
	  *  Multiplied into Z before computing distance from the arm centerline.
	  *  This is the squash at the INNER edge (ArmStartRadius).
	  *  Values > 1 compress the arm vertically (thinner), < 1 expand it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmVerticalSquash = 4.0f;

	/** Vertical squash at the OUTER edge (disc rim). Lerped from
	 *  ArmVerticalSquash at the inner edge to this value at the outer edge.
	 *  Should typically be less than ArmVerticalSquash (arms get vertically
	 *  thicker as they widen outward). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmVerticalSquashOuter = 2.0f;

	/** Factor by which core/envelope thickness grows from inner to outer edge.
	 *  At the inner edge, thicknesses are as specified. At the outer edge,
	 *  they are multiplied by this value. 1.0 = no growth, 3.0 = 3x wider. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmRadialGrowth = 6.0f;

	/** Controls how aggressively peak density drops as the arm widens.
	 *  Peak density = SDFPeakDensity / pow(growthFactor, this exponent).
	 *  1.0 = full inverse (3x wider = 1/3 density, 3D volume conservation)
	 *  0.5 = square root (3x wider = ~0.58 density, 2D area conservation)
	 *  0.0 = no density drop at all (constant peak everywhere) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmDensityFalloffExponent = 0.5f;

	/** SDF to Density Remapping: the arm SDF returns distance from the arm
	 *  centerline, remapped on absolute distance. Two thresholds control the
	 *  profile:
	 *    ArmCoreThickness: distance within which density = peak (solid arm)
	 *    ArmEnvelopeThickness: distance at which density = 0 (outer bound)
	 *  Between core and envelope density fades smoothly; beyond envelope it is
	 *  zero and cells can be culled entirely. */

	 /** Distance from arm centerline within which density is at peak.
	  *  This defines the solid core of the arm. In normalized space. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreThickness = 0.05f;

	/** Distance from arm centerline at which density reaches zero.
	 *  Must be >= ArmCoreThickness. The zone between core and envelope
	 *  is the falloff gradient. Also used for cell culling: cells whose
	 *  nearest possible SDF distance exceeds this are skipped entirely. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmEnvelopeThickness = 0.4f;

	/** Peak density at the arm core. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmPeakDensity = 1.5f;

#pragma endregion

#pragma region Background / Halo

	/** Low-level background density that fills the full galaxy volume.
	 *  Provides scattered stars outside the disc plane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundDensity = 0.01f;

	/** Vertical squash of the background halo. 1.0 = spherical, 0.5 = oblate.
	 *  Maps to legacy BackgroundHeightRatio. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundVerticalSquash = 0.8f;

	/** Radial falloff for the background. Uses smoothstep fade. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundCutoffRadius = 1.0f;

	/** Radius at which the background begins fading to zero,
	 *  as a fraction of BackgroundCutoffRadius. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundFadeStart = 0.7f;

#pragma endregion

#pragma endregion
};

//TODO: DESCRIPTION COMMENT - THIS FORMS THE PARAMETER INTERFACE WITH THE RAYMARCH MATERIAL OUR RAYMARCHER IS NOT MATURE AT THIS POINT SO THIS WILL CHANGE, MANY OF THESE VALUES CURRENTLY HAVE NO EFFECT - THATS FINE, IT WILL NEED REFACTORING WHEN WE DO A GALAXY RAYMARCHER DEEP DIVE ANYWAY
USTRUCT(BlueprintType)
struct SVO_API FGalaxyMaterialParams
{
	GENERATED_BODY()

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
	double VolumeWarpAmount = .05;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeWarpScale = .13;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VolumeNoise = "/svo/VolumeTextures/VT_PerlinWorley_Balanced";
};


/** Galaxy-layer generation parameters; extends FBaseParams (mirrors
 *  FUniverseParams structure). */

USTRUCT(BlueprintType)
struct SVO_API FGalaxyParams : public FBaseParams
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
	int32 LargeTierCullDepth = 2;

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
struct SVO_API FGalaxyParamBounds {
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Param Bounds")
	FGalaxyParams Min;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Param Bounds")
	FGalaxyParams Max;

	static FGalaxyParams Generate(FGalaxyParamBounds Bounds, int Seed) {
		//TODO: Randomize
		return Bounds.Max;
	}
};
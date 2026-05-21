// GalaxyDataGenerator.h
// Galaxy density field (spiral SDF + bulge/disc/background), tier generation,
// and volume texture sampling.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"  // FNoiseParams, FTierParams
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"
#include "GalaxyDataGenerator.generated.h"

// ============================================================================
// FGalaxyParams � extends FBaseParams (mirrors FUniverseParams structure)
// ============================================================================

USTRUCT(BlueprintType)
struct SVO_API FGalaxyParams : public FBaseParams
{
	GENERATED_BODY()

	// --- Density volume ---

	/// Voxel resolution per axis for the density pseudo-volume texture.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density Volume")
	int32 DensityVolumeResolution = 256;

	// --- Tier scale derivation ---

	/// Fixed absolute largest star-system scale in world cm.
	/// All galaxies generate star particles in the same physical size
	/// range regardless of parent galaxy size. The tier depth sequence
	/// (1/3/5, spacing 2, ratio 4) gives a 64x total spread:
	///
	///   Large: 3e18 → 7.5e17   (bright giants, wide binaries)
	///   Mid:   7.5e17 → 1.875e17 (solar-type systems)
	///   Small: 1.875e17 → ~4.7e16 (compact red dwarf systems)
	///
	/// Real references:
	///   Solar system to Pluto orbit ≈ 1.2e19 cm diameter
	///   Compact M-dwarf habitable zone ≈ 3e16 cm
	///   Wide binary separation ≈ 1e18 cm
	///
	/// MakePointDataFromWorldScale converts these to octree-local extents
	/// using the galaxy's UnitScale, so the octree depth adapts to each
	/// galaxy's coordinate system while the physical size stays constant.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 3e18;

	// --- Per-tier streaming configs ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	// --- Large tier SDF culling grid ---

	/// Grid depth used to subdivide the galaxy volume for SDF-based cell
	/// culling during large tier generation. Cells whose every corner has
	/// zero composite density are skipped entirely, concentrating candidate
	/// sampling on arms/disc/bulge.
	///
	/// Depth N produces (2^N)^3 cells over the GridExtentMultiplier-scaled
	/// volume. Depth 3 = 8^3 = 512 cells. Higher values give finer culling
	/// at the cost of more corner evaluations (8 * CellCount SDF samples).
	/// Values of 2–4 are recommended; 5+ rarely improves acceptance rate
	/// enough to justify the overhead.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Large Tier")
	int32 LargeTierCullDepth = 2;

	// --- Volume material params (carried over from legacy for volumetric setup) ---

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

	// --- Encoded noise graph (kept for future FastNoise swap-in) ---
	static constexpr const char* EncodedTree = "DQAFAAAAAAAAQAgAAAAAAD8AAAAAAA==";

	// --- Noise power for volume sampling ---

	/// Exponent applied to noise values during volume texture sampling.
	/// Higher = sharper contrast between dense and empty regions.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	float NoisePower = 2.0f;

	// =====================================================================
	//  Spiral Density Field Parameters
	// =====================================================================
	// These control the analytic density function that drives both particle
	// rejection sampling and volume texture baking. The galaxy shape is
	// composed of four additive layers: bulge, disc, arms, and background.
	// All coordinates are in normalized space [-1, 1] where 1 = Extent.
	//
	// The arm structure works by "un-twisting" the query point back to a
	// straight-arm reference frame, then measuring angular distance to the
	// nearest arm. This reverses the legacy GenerateArms + ApplyTwist flow
	// into a single analytic evaluation.
	// =====================================================================

	// --- Bulge ---
	// The bulge uses a Hernquist density profile evaluated in oblate
	// (vertically squashed) coordinates. No SDF remap needed — spherical
	// symmetry means the profile IS the density directly, identical
	// reasoning to the disc.
	//
	// Hernquist: density(r) proportional to a / (r * (1 + r/a)^3)
	//   - 1/r cusp at center, 1/r^4 tail (very concentrated)
	//   - a = BulgeScaleRadius controls how quickly density falls off
	//   - Hard cut at BulgeCutoffRadius prevents the long tail from
	//     polluting the disc/arm region

	/// Scale radius for the Hernquist profile, in normalized [0,1] space.
	/// Smaller = sharper core concentration. 0.1 = tight core, 0.3 = diffuse.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeScaleRadius = 1.0f;

	/// Peak density of the bulge at the center (r approaching 0) [0, 1].
	/// Zeroed for arm/disc iteration — set to 0.8-1.0 when compositing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgePeakDensity = 0.03f;

	/// Vertical squash factor for the bulge. 1.0 = sphere, < 1.0 = oblate.
	/// Applied to Z before computing the Hernquist radius.
	/// Maps to legacy BulgeAxisScale.Z.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeVerticalSquash = 0.6f;

	/// Hard radial cutoff for the bulge, in normalized space.
	/// Beyond this the bulge contributes zero density. Prevents the
	/// Hernquist 1/r^4 tail from polluting the disc/arm region.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bulge")
	float BulgeCutoffRadius = 1.0f;

	// --- Disc ---
	// The disc uses a separable analytic profile: exponential radial decay
	// multiplied by an exp(-|z/h|^falloff) vertical profile. No SDF remap
	// needed — rotational symmetry means there is no "nearest surface" to
	// measure distance to; the profile IS the density directly.

	/// Radial scale of the disc, in normalized space. 1.0 = extends to Extent.
	/// Also used as the hard radial cutoff for the disc cylinder.
	/// Maps to legacy GalaxyRatio (was 0.3 * Extent).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscRadius = 1.0f;

	/// Vertical scale height of the disc, as a fraction of DiscRadius.
	/// Acts as the sech²/exp scale height: ~76% of disc mass lies within
	/// 1× this height above/below the plane. Maps to legacy DiscHeightRatio.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscHeightRatio = 0.3f;

	/// Peak density of the disc at the center (r=0, z=0) [0, 1].
	/// Zeroed for arm iteration — set to 0.3–0.5 when compositing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscBaseDensity = 0.5f;

	/// Exponential radial scale length, as a fraction of DiscRadius.
	/// Controls how quickly density drops with radius.
	/// 0.2 = tight nucleus-concentrated disc, 0.5 = very diffuse.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscRadialScaleLength = 0.5f;

	/// Vertical profile exponent. Applied as exp(-(|z|/h)^DiscVerticalFalloff).
	/// 1.0 = exponential / isothermal sheet (sharp equatorial peak).
	/// 2.0 = Gaussian (softer, better for a thick stellar disc).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Disc")
	float DiscVerticalFalloff = 3.0f;

#pragma region Arm Params
	// --- Arms (SDF-based) ---
	// The arm density is derived from a signed distance field.
	// SampleArmSDF returns unsigned distance from the arm centerline in
	// normalized space. SampleDensity remaps it to [0, 1] via the
	// core/envelope thresholds.

	/// Number of spiral arms. Maps to legacy ArmNumArms.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	int32 ArmCount = 4;

	/// Twist strength in radians at the disc edge (r = DiscRadius).
	/// Higher = more wound spirals. Maps to legacy TwistStrength.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmTwistStrength = 6.0f;

	/// Core twist boost — extra winding near the center that falls off
	/// exponentially. Maps to legacy TwistCoreStrength. Set to 0 to disable.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreTwistStrength = 8.0f;

	/// Core twist radius — controls how quickly the core boost decays.
	/// Maps to legacy TwistCoreRadius. Smaller = tighter core winding.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreTwistRadius = 0.1f;

	/// Radial start of the arms, as a fraction of DiscRadius.
	/// Below this radius, arms fade out (merge into bulge).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmStartRadius = 0.05f;

	/// Width of the blend zone where arms fade in from ArmStartRadius,
	/// in normalized space. Controls how sharp the inner arm boundary is.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmStartBlendWidth = 0.15f;

	/// Vertical squash coefficient for the arm distance calculation.
	/// Multiplied into Z before computing distance from the arm centerline.
	/// This is the squash at the INNER edge (ArmStartRadius).
	/// Values > 1 compress the arm vertically (thinner), < 1 expand it.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmVerticalSquash = 6.0f;

	// --- Radial Growth ---
	// As distance along the arm increases (inner → outer edge), three
	// properties evolve together:
	//   1. Envelope grows (arm widens) by ArmRadialGrowth factor
	//   2. Peak density drops inversely proportional to growth (mass conservation)
	//   3. Vertical squash relaxes toward ArmVerticalSquashOuter
	//
	// All three are parameterized by t = (rXY - armStart) / (discR - armStart)
	// which goes from 0 at the inner edge to 1 at the disc rim.

	/// Factor by which core/envelope thickness grows from inner to outer edge.
	/// At the inner edge, thicknesses are as specified. At the outer edge,
	/// they are multiplied by this value. 1.0 = no growth, 3.0 = 3x wider.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmRadialGrowth = 4.0f;

	/// Controls how aggressively peak density drops as the arm widens.
	/// Peak density = SDFPeakDensity / pow(growthFactor, this exponent).
	/// 1.0 = full inverse (3x wider = 1/3 density, 3D volume conservation)
	/// 0.5 = square root (3x wider = ~0.58 density, 2D area conservation)
	/// 0.0 = no density drop at all (constant peak everywhere)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmDensityFalloffExponent = 0.333f;

	/// Vertical squash at the OUTER edge (disc rim). Lerped from
	/// ArmVerticalSquash at the inner edge to this value at the outer edge.
	/// Should typically be less than ArmVerticalSquash (arms get vertically
	/// thicker as they widen outward).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmVerticalSquashOuter = 3.0f;

	// --- SDF → Density Remapping ---
	// The arm SDF returns distance from the arm centerline (positive = inside
	// arm cross-section, but we remap based on absolute distance).
	// Two thresholds control the density profile:
	//
	//   ArmCoreThickness: distance within which density = peak (solid arm)
	//   ArmEnvelopeThickness: distance at which density = 0 (outer bound)
	//
	// Between core and envelope, density fades smoothly.
	// Beyond envelope, density is zero and cells can be culled entirely.

	/// Distance from arm centerline within which density is at peak.
	/// This defines the solid core of the arm. In normalized space.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmCoreThickness = 0.0f;

	/// Distance from arm centerline at which density reaches zero.
	/// Must be >= ArmCoreThickness. The zone between core and envelope
	/// is the falloff gradient. Also used for cell culling: cells whose
	/// nearest possible SDF distance exceeds this are skipped entirely.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmEnvelopeThickness = 0.6f;

	/// Peak density at the arm core.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Arms")
	float ArmPeakDensity = 1.0f;
#pragma endregion

	// --- Background / Halo ---

	/// Low-level background density that fills the full galaxy volume.
	/// Provides scattered stars outside the disc plane.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundDensity = 0.01f;

	/// Vertical squash of the background halo. 1.0 = spherical, 0.5 = oblate.
	/// Maps to legacy BackgroundHeightRatio.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundVerticalSquash = 0.8f;

	/// Radial falloff for the background. Uses smoothstep fade.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundCutoffRadius = 1.0f;

	/// Radius at which the background begins fading to zero,
	/// as a fraction of BackgroundCutoffRadius.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Background")
	float BackgroundFadeStart = 0.7f;

	// --- Bounds Fade ---
	// A global multiplier applied to the entire composite density to prevent
	// hard transitions at the volume bounds. The fade is spherical, applied
	// in normalized space based on distance from the origin.

	/// Fraction of the normalized extent [0, 1] at which the bounds fade begins.
	/// Below this distance, density is unmodified. Above it, density fades
	/// to zero via smoothstep reaching zero at the cube edge (distance = 1).
	/// 0.67 = fade starts at 2/3 of the way from center to edge.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bounds")
	float BoundsFadeStart = 0.67f;

	/// Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	/// depth sequence. Delegates to FTierParams::DeriveTierScaleRanges.
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		FTierParams::DeriveTierScaleRanges(MaxEntityScale, Tiers);
	}

	FGalaxyParams()
	{
		Seed = 666;
		Extent = 274877906944;
		UnitScale = 1e11;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1, 0);

		// Large tier: single cell covering the full galaxy extent.
		// NeighborhoodRadius = 0 -> 1x1x1 = 1 slot, exhaustive single-pass.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.MaxParticlesPerSlot = 16000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 8000;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 4000;

		DeriveScaleRanges();
	}
};


// ============================================================================
// GalaxyDataGenerator � owns noise composition and tier generation callbacks
// ============================================================================
//
// Mirrors UniverseDataGenerator structure. The galaxy actor wires tier
// callbacks that delegate here; this class has no knowledge of actors,
// Niagara, octrees, or the streaming pipeline.

class SVO_API GalaxyDataGenerator
{
public:
	GalaxyDataGenerator() {};
	GalaxyDataGenerator(FGalaxyParams InParams) : Params(InParams) {};

	FGalaxyParams Params;
	FastNoise::SmartNode<> DensityNoise;

	// -----------------------------------------------------------------------
	// Math Helpers
	// -----------------------------------------------------------------------

	/// Smooth maximum of three values using the log-sum-exp (LSE) formulation.
	///
	/// SmoothMax(a, b, c, k) ≈ max(a, b, c) with a differentiable blend
	/// zone of radius ~1/k around each crossing. Higher k = sharper
	/// (approaches hard max); lower k = softer blend.
	///
	/// LSE form:  (1/k) * log( exp(k*a) + exp(k*b) + exp(k*c) )
	///
	/// Numerically stabilized by subtracting the running max before
	/// exponentiation so the result never overflows regardless of k.
	///
	/// @param A, B, C  Input values (any range).
	/// @param K        Sharpness (k > 0). Typical range 2–16.
	/// @return         Smooth maximum of the three inputs.
	static FORCEINLINE float SmoothMax(float A, float B, float C, float K = 8.0f)
	{
		const float M = FMath::Max3(A, B, C);
		const float ExpA = FMath::Exp(K * (A - M));
		const float ExpB = FMath::Exp(K * (B - M));
		const float ExpC = FMath::Exp(K * (C - M));
		return M + FMath::Loge(ExpA + ExpB + ExpC) / K;
	}

	// -----------------------------------------------------------------------
	// Signed Distance Fields + Analytic Density Helpers
	// -----------------------------------------------------------------------
	// Arms use a two-phase approach: SampleArmSDF returns unsigned distance
	// from the centerline, then SampleDensity remaps through core/envelope
	// thresholds. The disc and bulge are rotationally symmetric so they use
	// direct analytic profiles instead of SDF remapping.

	/// Unsigned distance from the nearest arm centerline at the query radius.
	/// @param InNormPos  Position in [-1, 1] normalized galaxy space.
	/// @param rXY        Pre-computed cylindrical radius.
	float SampleArmSDF(const FVector& InNormPos, double rXY) const;

	/// Signed distance to the bulge ellipsoid. Positive inside.
	/// Used as a hard boundary guard; actual bulge density uses a Hernquist profile.
	float SampleBulgeSDF(const FVector& InNormPos) const;

	/// Analytic bulge density using a Hernquist profile in oblate coordinates.
	/// Returns density in [0, 1]; hard zero outside BulgeCutoffRadius.
	float SampleBulgeDensity(const FVector& InNormPos) const;

	/// Signed distance to the disc cylinder. Positive inside.
	/// Used as a hard boundary guard in SampleDiscDensity.
	float SampleDiscSDF(const FVector& InNormPos, double rXY, double absZ) const;

	/// Analytic disc density at the given cylindrical coordinates.
	/// Separable exponential radial × exp(-|z/h|^falloff) vertical profile.
	/// Returns density in [0, 1]; hard zero outside the disc cylinder.
	float SampleDiscDensity(double rXY, double absZ) const;

	// -----------------------------------------------------------------------
	// Density Sampling
	// -----------------------------------------------------------------------
	// Composites all active layers into a single [0, 1] density value.
	// Arms, disc, and bulge are max-blended (union). Background is additive.
	//   Arms:  SDF-based, core/envelope remap + radial growth.
	//   Disc:  Analytic exponential radial x vertical profile.
	//   Bulge: Hernquist profile in oblate coordinates.
	//   BG:    Additive halo (zeroed until compositing phase).

	/// Sample density at a single normalized position.
	/// InNormPos is in [-1, 1] noise space (position / Extent).
	/// Returns density in [0, 1].
	float SampleDensity(const FVector& InNormPos) const;

	/// Batch-evaluate the density field for an array of positions.
	void SampleDensityBatch(
		float* OutDensity,
		int32 InCount,
		const float* InX,
		const float* InY,
		const float* InZ) const;

	// -----------------------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------------------

	/// Build the encoded noise graph (kept for future use) and mark ready.
	void Initialize();

	/// Build noise from encoded tree. Kept for future FastNoise swap-in.
	FastNoise::SmartNode<> BuildNoise() const;

	/// Sample the density field into a CPU-side BGRA8 volume texture buffer.
	/// Uses the C++ SampleDensity path directly rather than going through
	/// FVolumeTextureUtils::SampleNoiseToVolume.
	TArray<uint8> SampleNoiseVolume(int InNoiseResolution) const;

	// -----------------------------------------------------------------------
	// Tier Generation Callbacks
	// -----------------------------------------------------------------------
	// Self-contained generation functions that write directly into particle
	// buffers. The galaxy actor's tier system calls them via
	// FParticleTierConfig::GenerateCallback lambdas.

	/// Large tier: generates particles across the full galaxy extent using
	/// batched noise rejection sampling. Particle count is derived from
	// -----------------------------------------------------------------------
	// Tier Generation — Unified
	// -----------------------------------------------------------------------
	// Single generation function used by all tiers. The only differences
	// between Large/Mid/Small are: the candidate volume (full extent vs
	// cell-local), the tier params (scale range, density curve), and the
	// seed offset for stream isolation.

	/// Generates particles for a single tier cell via batched noise rejection
	/// sampling. Candidates are distributed uniformly within InCellExtent
	/// around InNodeCenter, density-gated by SampleDensity, and written
	/// into InBuffer at the slot region for InSlotIndex.
	///
	/// For the Large tier (full galaxy), pass InNodeCenter = ZeroVector and
	/// InCellExtent = Params.Extent to cover the entire volume.
	///
	/// @param InCoord        Grid coordinate of the cell.
	/// @param InSlotIndex    Flat slot index within the particle buffer.
	/// @param InBuffer       Target buffer to write accepted particles into.
	/// @param InNodeCenter   Center of the cell in galaxy-local space.
	/// @param InCellExtent   Half-extent of the candidate volume.
	/// @param InTierParams   Tier config (scale range, density curve, etc).
	/// @param InSeedOffset   Added to Params.Seed for stream isolation between tiers.
	/// @param OutSlotCount   Receives the number of accepted particles.
	void GenerateTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		int32& OutSlotCount) const;

	// -----------------------------------------------------------------------
	// Large Tier — SDF-culled grid generation
	// -----------------------------------------------------------------------

	/// One active cell in the large tier culling grid.
	struct FActiveLargeTierCell
	{
		FVector    Center;    // Galaxy-local center (not normalized)
		double     HalfExt;  // Half-extent of the cell (same on all axes)
		FIntVector GridCoord; // Integer grid coordinate at LargeTierCullDepth
	};

	/// Subdivide the galaxy volume into a uniform grid at Params.LargeTierCullDepth
	/// and return only cells where at least one corner has non-zero composite
	/// density. Cells whose all 8 corners evaluate to SampleDensity == 0 are
	/// entirely outside all SDF envelopes and can never produce accepted candidates.
	///
	/// Grid covers [-Extent, +Extent] on each axis (galaxy-local). Corner
	/// positions are converted to normalized [-1, 1] space before testing.
	TArray<FActiveLargeTierCell> CollectActiveLargeTierCells() const;

	/// Generate the full large tier slot by iterating over SDF-active cells.
	/// Candidates are distributed proportionally: each active cell receives
	/// ceil(SlotCapacity / ActiveCellCount) candidates, capped at SlotCapacity
	/// total. This concentrates sampling on arms/disc/bulge and avoids wasting
	/// rejection attempts on empty inter-arm space.
	///
	/// Writes into InBuffer at the slot region for InSlotIndex. Pads remaining
	/// entries dead. Sets OutSlotCount to the accepted particle count.
	void GenerateLargeTierSlot(
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		int32& OutSlotCount) const;

};
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "UniverseParams.generated.h"

/** Seed channels for the universe layer, mirroring GalaxySeed.
 *
 *  ADD, NEVER REUSE. A channel is free; sharing one between two consumers reintroduces
 *  exactly the aliasing ProcSeed exists to prevent. */
namespace UniverseSeed
{
	/** The GPU placement key. One channel for all three tiers -- the tier index enters
	 *  through MixSeed's index argument, so the streams stay distinct without a channel
	 *  each, and adding a fourth tier needs no new constant. */
	inline constexpr uint32 Placement = ProcSeed::ChannelId("Universe.Placement");
}


// FUniverseNoiseGraphParams REMOVED. It authored the legacy FastNoise graph that the
// three tiers rejection-sampled for cluster placement, which the cosmic-web field
// replaced -- and it named a second, unrelated "density params" alongside
// FUniverseDensityParams, which is exactly the arrangement that lets the render and
// placement quietly read different fields. It came out with EncodedTree, BuildNoise and
// DensityNoise; they were each other's only consumers.


// =============================================================================
// FUniverseVarianceRange -- the packing convention, as a type
// =============================================================================

/** One regionally-varying quantity: two authored extremes and the curve that decides
 *  where between them a given sample falls.
 *
 *  THE CORE PACKS THESE AS (min, max, bias, free) AND SO DOES THIS. Making the convention
 *  a type rather than a naming discipline is most of the point: eight parameters use it,
 *  and as eight sets of loose scalars it was eight chances to pack .z into the wrong slot
 *  or forget which of a pair was the bias. Pack() is written once.
 *
 *  THERE IS NO BASE VALUE. These are not a default plus a deviation -- the region channel
 *  picks a point between Min and Max and that IS the value. A range with Min == Max
 *  reproduces a constant exactly, which is also how the core decides whether to pay for
 *  the fetch at all: see IsDegenerate.
 *
 *  MIN > MAX IS VALID and simply runs the interpolation backwards. There is no
 *  requirement that Min be the smaller number, and no clamping anywhere. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseVarianceRange
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Min = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Max = 0.0f;

	/** Exponent applied to the raw region channel before the lerp: t = pow(channel, Bias).
	 *
	 *    1     the channel straight through
	 *    > 1   pushes most of the field toward MIN, so the max end is the rare case
	 *    < 1   pushes it toward MAX
	 *
	 *  AT OR BELOW ZERO IS LEGAL BUT ALMOST NEVER MEANT. The core's PowSafe guards the
	 *  zero base, and the result reads as "always at Max" once the zero-power-of-zero
	 *  case is accounted for. Worth an eye if a bias of exactly 0 is ever authored on
	 *  purpose. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Bias = 1.0f;

	FUniverseVarianceRange() = default;
	FUniverseVarianceRange(float InMin, float InMax, float InBias = 1.0f)
		: Min(InMin), Max(InMax), Bias(InBias) {}

	/** Packs into the float4 the derivation expects. InW is the .w passenger -- unclaimed
	 *  on most ranges, and carrying a partner setting on four of them. */
	FVector4f Pack(float InW = 0.0f) const
	{
		return FVector4f(Min, Max, Bias, InW);
	}

	/** Whether this range is a constant, which is exactly the test the core applies to
	 *  decide whether its region fetch is worth paying for.
	 *
	 *  ONLY MIN AND MAX ARE TESTED, and that matters because four of these carry an
	 *  unrelated scalar in .w. A range whose min equals its max is a constant however its
	 *  passenger is set, so changing a warp scale must never make the layer think its
	 *  amplitude varies. */
	bool IsDegenerate() const { return Min == Max; }

	/** The value with no region signal at all: the plain midpoint, NOT run through the
	 *  bias. This mirrors the core's fallback when a fetch is skipped, and it is exact
	 *  rather than approximate -- a fetch is only skipped when every range feeding it is
	 *  degenerate, in which case the lerp returns that constant whatever t is. */
	float Midpoint() const { return FMath::Lerp(Min, Max, 0.5f); }
};


// =============================================================================
// Field groups
// =============================================================================

/** THE TWO LATTICES. The small one is the reference for everything else in the field:
 *  warp amplitudes and scales, both region scales, feature width and void spread are all
 *  per SMALL cell, and the large lattice inherits every one of them scaled by the ratio.
 *  A region that switches lattice therefore switches scale in every property at once,
 *  rather than growing voids with unchanged wall widths. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseLatticeParams
{
	GENERATED_BODY()

	/** Small cell size in normalized units. Larger values give a coarser web. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice", meta = (ClampMin = "0.000001"))
	float CellSizeSmall = 0.3f;

	/** Large cell size. QUANTIZED BY THE DERIVATION to a whole multiple of CellSizeSmall,
	 *  and that is precision rather than convenience. The field offset arrives as an exact
	 *  integer count of small cells and has to be re-split into whole large cells plus a
	 *  remainder: exact under integer division at any magnitude, and not exact as a float
	 *  multiply by 1/ratio. At 1e11 cells one ulp is hundreds of cells and the error
	 *  CHANGES as the offset crosses a cell plane, so an arbitrary ratio would make the
	 *  coarse lattice shift bodily and discontinuously as the player moves. A tear, not a
	 *  drift.
	 *
	 *  KEEP THE RATIO OFF A HALF. 1.4 against a small cell of 0.4 was exactly 3.5, sitting
	 *  precisely on the rounding boundary the derivation quantizes across -- and HLSL rounds
	 *  halves away from zero while the C++ shim's round defers to nearbyint, which rounds to
	 *  even. Both happen to land on 4 today, but the value is one ulp of tuning away from
	 *  the two sides disagreeing, and the failure is the coarse lattice coming out a
	 *  different size in the render than in placement. 0.9 against 0.3 gives an unambiguous
	 *  ratio of 3, with the nearest half boundary a long way off in either direction.
	 *
	 *  At or below CellSizeSmall this collapses to a single lattice and skips the second
	 *  neighbourhood walk entirely, which is roughly half the cost of a sample. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice", meta = (ClampMin = "0.0"))
	float CellSizeLarge = 0.9f;

	/** Exponent shaping where between the two lattices a region sits. NOT a range: the two
	 *  extremes it interpolates are the lattices themselves, so it has no min/max of its
	 *  own and never passes through the variance remap.
	 *
	 *  THE BLEND CANNOT BE PINNED OR CLAMPED at present. Every value in [0,1] is reachable
	 *  somewhere and there is no way to author "30% coarse everywhere" or "never enter the
	 *  mixed band". The useful band is compressed -- 0.4 to 0.8 is where both lattices
	 *  compete, and mean density peaks around 0.30 there against 0.06 at either end -- so
	 *  this will want a real override. CellSizeRange.w is free and is the natural host for
	 *  one; it is packed as 0 and the core ignores it, so claiming it is a change in the
	 *  core first and here second. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice")
	float CellSizeBias = 1.0f;
};


/** WALLS: two nodes participating, the sheet between two voids. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseWallParams
{
	GENERATED_BODY()

	/** WHAT THE MARCH ACCUMULATES IS DENSITY TIMES VOLUME FRACTION, not peak density, and
	 *  a wall covers an order of magnitude more volume than a filament. Authoring the two
	 *  at similar numbers is what makes the field read as foam; the filament wants to be
	 *  several times this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wall")
	FUniverseVarianceRange Density = FUniverseVarianceRange(0.0f, 1.0f, 6.0f);

	/** Exponent on the wall term, saturate(N - 1). 1 is the bare saturate; up is the
	 *  direction that clears voids.
	 *
	 *  THIS IS THE VOID-FRACTION LEVER, not the densities. The participation weight has no
	 *  compact support, so the aggregate tail of the twenty-five candidates not in
	 *  contention lifts N off its floor everywhere -- and once that tail alone carries N
	 *  past 2, the saturate pins the wall term at full and nothing else touches it. That
	 *  is the symptom to recognise: a wall control that appears dead while the filament
	 *  one still works, because the filament threshold at 3 sits above the tail rather
	 *  than in it.
	 *
	 *  The exponent leaves the on-wall value at exactly 1, since 1^p is 1, and crushes the
	 *  tail: 0.4 goes to 0.064 at p = 3. It shrinks exponentially in the exponent, so
	 *  narrowing here clears voids far faster than lowering densities does.
	 *
	 *  Below zero is a division-by-zero shape that lights the void up rather than clearing
	 *  it; exactly 0 makes every non-zero base read as 1 and floods the field. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wall")
	FUniverseVarianceRange Falloff = FUniverseVarianceRange(2.0f, 12.0f, 1.0f);
};


/** FILAMENTS: three nodes participating, the edge where three sheets meet. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseFilamentParams
{
	GENERATED_BODY()

	/** Covers far less volume than a wall, so it carries a larger number for the same
	 *  apparent brightness. See the volume-fraction note on FUniverseWallParams::Density.
	 *
	 *  There is no filament falloff to match the wall's. The filament term is a bare
	 *  saturate(N - 2), and its threshold sits above the aggregate tail rather than inside
	 *  it, so it never needed the correction the wall term does. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Filament")
	FUniverseVarianceRange Density = FUniverseVarianceRange(0.5f, 1.5f, 0.5f);
};


/** VOIDS: one node dominating, the interior of a cell. Both the ambient floor that keeps
 *  them from being empty and the per-node offset that keeps them from being all one
 *  size. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseVoidParams
{
	GENERATED_BODY()

	/** Ambient density everywhere.
	 *
	 *  BELOW ZERO IS NOT A DARKER VOID. The march's exp(-density) turns negative density
	 *  into unbounded gain rather than opacity. Both ends matter now that it is a range: a
	 *  range spanning zero puts the negative end somewhere, and a noise channel decides
	 *  where. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	FUniverseVarianceRange Floor = FUniverseVarianceRange(0.0f, 0.5f, 8.0f);

	/** Per-node power-diagram offset, so voids differ in size rather than tiling at one
	 *  scale.
	 *
	 *  SQUARED CELL UNITS, not cells. It is subtracted from the squared distance, which is
	 *  the space the weight wanted anyway and is what removed 27 square roots from the
	 *  walk. A node out-reaches an unoffset neighbour by this much in squared distance --
	 *  about the same in linear distance at a typical half-cell separation, shrinking as
	 *  the separation grows. Values carried over from the old additive form do not mean
	 *  what they used to.
	 *
	 *  THIS IS WHAT ERODES THE 27-CELL SEARCH. The window is sound while the offset stays
	 *  well under 1.0 minus the dominant candidate's own power distance, and the bound
	 *  degrades with WIDE features as well as with large offsets. The 0.80 that shipped
	 *  against a form with no bound at all is very likely too large here -- start an order
	 *  lower and come up. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	FUniverseVarianceRange SizeSpread = FUniverseVarianceRange(0.0f, 0.5f, 2.0f);

	/** Rides SizeSpread's own .w. Above 1 makes large voids rare and small ones common.
	 *
	 *  IT RIDES THAT SLOT BECAUSE IT IS A FUNCTION OF THAT RANGE. Skew already used the
	 *  spread's bias-shaped interpolant as a derived multiplier, so the two were coupled
	 *  in the field before they were coupled in the packing.
	 *
	 *  INERT AT THE SHIPPED SETTING. UNIVERSE_SKEW_FIXED is 4, so the core's SkewPow
	 *  ignores its exponent argument and this pin, the per-candidate product it feeds and
	 *  the .w slot it rides are all dead. Exposed anyway because a claimed slot with a
	 *  hidden pin is worse than an inert one -- but do not tune against it expecting a
	 *  result. Restoring the general path costs 54 pow calls per sample, which is a bad
	 *  trade; the cheap version is a lerp between the fixed exponents 2 and 4. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void", meta = (ClampMin = "0.0"))
	float SizeSkew = 1.0f;
};


/** ONE OCTAVE OF THE DOMAIN WARP. Two of these exist and they are not symmetric: only the
 *  large octave uses LatticeFollow, and their usable amplitudes differ by two orders at
 *  the default scales. See the fold ceiling on FUniverseDensityParams. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseWarpOctaveParams
{
	GENERATED_BODY()

	/** Peak displacement in SMALL cells.
	 *
	 *  NOT THE CONVENTION THE RETIRED LATTICE PATH USED -- that one centred and doubled,
	 *  so a value carried over from it must be doubled to displace the same distance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	FUniverseVarianceRange Amount = FUniverseVarianceRange(0.333f, 0.666f, 1.0f);

	/** Texture repeats per small cell, riding Amount's .w.
	 *
	 *  THE PAIR IS ONE SETTING. What actually curves the web is SHEAR -- amount times
	 *  scale times the asset's gradient -- so neither half means anything alone, which is
	 *  what makes this a passenger rather than a pin of its own.
	 *
	 *  QUANTIZED BY THE DERIVATION to a multiple of 1/4096, and that is correctness rather
	 *  than taste. The warp UV wraps every 4096 cells by masking the cell index, and the
	 *  two sides of that wrap are the same point in the texture only if 4096 * scale is a
	 *  whole number. Otherwise there is a hard seam every 4096 cells that no amount of
	 *  amplitude tuning will hide.
	 *
	 *  FIXED ACROSS THE DRAW, not ranged. That was tried and rolled back. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp", meta = (ClampMin = "0.0"))
	float Scale = 0.099854f;

	/** Per-axis gain on the decoded displacement. Elementwise, not a dot product -- a warp
	 *  needs all three axes, unlike a scalar site's weights. 0 flattens the warp along
	 *  that axis, negative inverts it, magnitude above 1 is gain. Deliberately
	 *  unclamped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	FVector Weights = FVector(1.0, 1.0, 1.0);

	/** LARGE OCTAVE ONLY. Rides the weights vector's .w, where there was no fourth axis to
	 *  steer, and is saturated by the core.
	 *
	 *  How much of the cell size ratio this octave's amplitude follows on the coarse
	 *  lattice: 0 gives the same physical displacement on both tiers, 1 gives the same
	 *  displacement measured in each tier's own cells. The small octave is pinned at full
	 *  follow -- that is what "scales its amount" means for it -- so this is ignored
	 *  there. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float LatticeFollow = 1.0f;
};


/** THE TWO REGION FETCHES. These are the archetype system for this layer: rather than
 *  rolling a variant per instance the way the galaxy layer does, one parameter set is read
 *  at two spatial frequencies, so a province of wide soft walls and a province of tight
 *  bright filaments are the same authored numbers sampled in different places.
 *
 *  NEITHER CAN BE RANGED, and that is structural rather than a matter of degree. These
 *  DEFINE where a region is, so putting them on the variance path would make the
 *  definition self-referential -- the field deciding how big its own regions are, at a
 *  position it can only know by having already fetched itself. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseRegionParams
{
	GENERATED_BODY()

	/** Repeats per small cell for the STRUCTURE field, the coarser of the two. Drives the
	 *  lattice blend, void size spread and skew, feature width, and the large warp octave
	 *  -- everything that decides what SHAPE the web has, which changes over provinces
	 *  rather than patches.
	 *
	 *  Default 19/4096, nudged off the authored 0.005 which quantizes to 20 and shares a
	 *  factor of 10 with the large warp octave's 410 -- those two re-align every 409 cells,
	 *  and at five cells per proxy that is eighty proxy widths, reachable in one session.
	 *
	 *  ODD IS NOT SUFFICIENT, which is what 21 got wrong: 21 is 3x7 and the small warp
	 *  octave's 3072 is 2^10x3, so they still share a 3 and repeat every 1365 cells. 19 is
	 *  prime and divides none of the others. The visual difference between 0.005 and
	 *  0.004639 is nothing; the difference between a repeating field and one that does not
	 *  repeat is not. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	float ScaleStructure = 0.004639f;

	/** Repeats per small cell for the APPEARANCE field. Drives wall density and falloff,
	 *  filament density, void floor, and the small warp octave.
	 *
	 *  Default 41/4096, which is what the authored 0.010 already quantizes to, and odd, so
	 *  it stays. Note the pairing it was in: 41 against the large warp octave's 410 is a
	 *  shared factor of 41, re-aligning every NINETY-NINE CELLS -- twenty proxy widths, the
	 *  worst repeat in the set, and the reason that warp scale moved to 409/4096.
	 *
	 *  IT CURRENTLY REUSES THE STRUCTURE FIELD'S FOUR ARCHETYPES at a different frequency.
	 *  A second asset with four different ones would remove the last correlation the
	 *  channel reallocation could not. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	float ScaleAppearance = 0.010010f;
};


// =============================================================================
// THE FIELD OFFSET
// =============================================================================

/** The player's position in the field, split the way UniverseDensityCore.ush wants it:
 *  an exact integer cell count plus a fraction in [0,1). Runtime state rather than
 *  authored, which is why it is a Pack() argument instead of a member -- it changes every
 *  frame and nothing else in the set does.
 *
 *  THE CELL COUNT IS int32 AND THE PIN IS float3, and neither is the magnitude the
 *  coordinate design was built for. MakeUniverseDensityParams casts InOffsetCell through
 *  (int), so the core's own ceiling is int32, about 2.1e9 small cells; marshalling through
 *  a float3 loses exactness above 2^24, about 1.7e7 cells, because a float32 has no unit
 *  ulp past that. The cell/frac formulation in the core survives 1e11. This hand-off to it
 *  does not. When traversal reaches those magnitudes the pin has to become a high and low
 *  part per axis, or the offset has to be folded into a cell index the caller supplies
 *  directly. Noted here rather than in the core because the core is not where it breaks. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseFieldOffset
{
	GENERATED_BODY()

	/** Whole small cells. */
	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	FIntVector Cell = FIntVector::ZeroValue;

	/** Fractional remainder per axis, in [0,1). */
	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	FVector Frac = FVector::ZeroVector;

	/** Splits a small-cell position into the exact cell/frac pair the core wants.
	 *
	 *  FLOOR, NOT TRUNCATION, for the same reason SampleAtPosition uses floor: truncation
	 *  folds the cell boundary at zero and mirrors the field through the origin on every
	 *  axis. */
	static FUniverseFieldOffset FromCellPosition(const FVector& InCellPos)
	{
		FUniverseFieldOffset Out;
		const FVector Floored(
			FMath::Floor(InCellPos.X),
			FMath::Floor(InCellPos.Y),
			FMath::Floor(InCellPos.Z));

		Out.Cell = FIntVector(
			static_cast<int32>(Floored.X),
			static_cast<int32>(Floored.Y),
			static_cast<int32>(Floored.Z));
		Out.Frac = InCellPos - Floored;
		return Out;
	}
};


// =============================================================================
// THE PACKED ARGUMENT LIST
// =============================================================================

/** The sixteen arguments of MakeUniverseDensityParams, in order, packed.
 *
 *  ONE MARSHAL SITE, and that is why it exists. The galaxy layer packs its equivalent in
 *  three independent places -- the CPU derivation, the compute parameter fill, and the
 *  material's Custom node body -- and its own header warns that the third fails at SHADER
 *  compile rather than at build, so a missed parameter surfaces as a red material with no
 *  build error. Here everything downstream reads this struct instead: the compute fill
 *  copies it member for member, the material push sets it member for member, and a future
 *  CPU derivation hands it straight over. Adding a parameter to the derivation breaks
 *  Pack() at compile time, which is the only place the packing decisions live.
 *
 *  NOT UPLOADED AS A CONSTANT BUFFER. That would require HLSL's cbuffer packing to agree
 *  with the C++ layout member for member, and one float3 straddling a 16-byte boundary
 *  shifts every field after it with no diagnostic at all -- the universe would simply come
 *  out wrong. These are raw inputs and the derivation runs in the shader. */
struct FUniverseDensityArgs
{
	// --- WEB GEOMETRY --- (small, large, bias, free)
	FVector4f CellSizeRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	float Seed = 0.0f;

	// --- FIELD OFFSET ---
	FVector3f OffsetCell = FVector3f::ZeroVector;
	FVector3f OffsetFrac = FVector3f::ZeroVector;

	// --- WEB DENSITY, WIDTH AND FALLOFF --- (min, max, bias, free)
	FVector4f WallDensityRange = FVector4f(0.0f, 0.1f, 1.0f, 0.0f);
	FVector4f WallFalloffRange = FVector4f(2.0f, 12.0f, 1.0f, 0.0f);
	FVector4f FilamentDensityRange = FVector4f(0.25f, 1.25f, 1.0f, 0.0f);
	FVector4f FeatureWidthRange = FVector4f(0.05f, 0.4f, 2.0f, 0.0f);
	FVector4f VoidFloorRange = FVector4f(0.0f, 0.05f, 3.0f, 0.0f);

	// --- ORGANICS --- .w carries a passenger on all but the small octave's weights
	FVector4f VoidSizeSpreadRange = FVector4f(0.0f, 0.0f, 1.0f, 1.0f);
	FVector4f WarpAmountLargeRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	FVector4f WarpAmountSmallRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	FVector4f WarpLargeWeights = FVector4f(1.0f, 1.0f, 1.0f, 0.0f);
	FVector4f WarpSmallWeights = FVector4f(1.0f, 1.0f, 1.0f, 0.0f);

	// --- REGION FETCH SCALES --- (structure, appearance, unclaimed, unclaimed)
	FVector4f RegionScales = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);

	// --- BOUNDS ---
	float BoundsFadeStart = 1.0f;
};


// =============================================================================
// THE AUTHORED FIELD
// =============================================================================

/** The cosmic-web density field: every input of MakeUniverseDensityParams, grouped by the
 *  feature it describes.
 *
 *  GROUPED RATHER THAN FLAT, following FGalaxyParams. Each group's comment block carries
 *  the argument for that group once instead of repeating it per scalar, and the eight
 *  regionally-varying quantities share one range type rather than twenty-four loose
 *  scalars whose .z slots all had to be remembered separately.
 *
 *  NOTHING IS COMBINED OR CORRELATED HERE. The lattice ratio rounding, the four scale
 *  quantizations, the offset re-split, the lambda derivation and the two region-fetch
 *  enables are all resolved inside MakeUniverseDensityParams, which the material and the
 *  compute shader both call. That is what lets this struct and the material's pin set be
 *  the same list of raw values, so neither side can drift from the other.
 *
 *  ONE SET FOR THE WHOLE LAYER, AND NO ARCHETYPES. The galaxy layer needs UGalaxyArchetype
 *  and a per-seed roll because each galaxy is a distinct object and the variation between
 *  them has to come from somewhere. This field is omnipresent and carries its own
 *  variation: the two region fetches ARE the archetype system, and they resolve per sample
 *  rather than per instance. Direct configuration plus a seed is the whole input.
 *
 *  That simplifies the compute path as much as it does this struct -- entity generation
 *  needs one parameter set for the layer rather than a per-cell archetype lookup, and the
 *  constant buffer is the same sixteen values for every dispatch.
 *
 *  NO CLAMPING beyond the UPROPERTY meta on obvious division guards. Every value passes
 *  through at whatever it is given so that pushing one past its sensible range is visible
 *  rather than silently absorbed. Validate() reports the constraints that are not visible
 *  -- the fold ceiling, the scale coprimality -- but it reports rather than corrects, for
 *  the same reason. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseDensityParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseLatticeParams Lattice;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseWallParams Wall;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseFilamentParams Filament;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseVoidParams Void;

	/** THE ONE WIDTH, in small cells, serving walls and filaments alike -- which is why it
	 *  sits here rather than inside either group. Both features read the same participation
	 *  count off the same weights, so a second width would need a second set of
	 *  exponentials over the whole neighbourhood, 27 more exp2 and 54 more madds, to shift
	 *  a threshold the wall term reaches through its falloff exponent for two instructions.
	 *
	 *  IT IS A FALLOFF RATE, NOT A BAND EDGE. It is the distance at which a competing
	 *  node's weight has halved. There is no cutoff at all -- the tail is exponential
	 *  rather than absent, which is what the wall falloff exponent exists to control.
	 *
	 *  It also floors the march's step size through MinFeatureStep, half the narrowest wall
	 *  the parameters can produce, derived from this and the small cell size together.
	 *  Widening it makes the march cheaper as well as softer. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseVarianceRange FeatureWidth = FUniverseVarianceRange(0.1f, 0.4f, 2.0f);

	/** The octave that BENDS the web. Large scale with small amplitude is grain rather
	 *  than curvature; small scale with large amplitude translates whole regions rigidly
	 *  and the straight bisectors arrive straight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseWarpOctaveParams WarpLarge;

	/** The octave that BREAKS UP what the large one bent -- and the one that binds the
	 *  fold ceiling, by a wide margin at the default scales.
	 *
	 *  On the coarse lattice the small octave's shear term is multiplied by the cell size
	 *  ratio while the large octave's is divided by it, because the small octave scales its
	 *  amplitude against a fixed wavelength and the large octave does the reverse. At ratio
	 *  4 and scale 0.5 this octave therefore contributes roughly 160 times the shear per
	 *  unit amplitude that the large one does. Expect its usable amplitude to be two orders
	 *  below the large octave's, and check PredictedFoldShear before raising it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseWarpOctaveParams WarpSmall = FUniverseWarpOctaveParams();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseRegionParams Region;

	/** Radius in normalized space at which the ray march begins fading the field toward the
	 *  proxy edge. At or above 1 disables it.
	 *
	 *  APPLIED BY THE MARCH, NOT BY THE FIELD. SampleAtPosition returns the field unmodified
	 *  and UniverseRayMarch applies the fade after sampling, which is what makes the entity
	 *  path viable: a fade in placement would make where an entity lands depend on where the
	 *  player was standing when the cell was generated. It still travels through this
	 *  derivation because the march reads it off the params struct.
	 *
	 *  ZERO IS NOT "NO FADE" -- it is the MOST fade. The span is 1 - start, so at 0 the
	 *  attenuation runs across the whole volume and the field is a soft ball centred on the
	 *  camera rather than a window with a horizon. That is what the instance was tuned at
	 *  and it is very likely part of why the march is not saturating: it is thinning
	 *  everything past the near field. Kept as the default so the tuned look survives the
	 *  push, but it is a look decision standing in for a bounds control, and raising it
	 *  will make the distance dense again. 1 and above is the disable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bounds", meta = (ClampMin = "0.0"))
	float BoundsFadeStart = 0.0f;

	/** The largest |dTex/dUV| the filtered noise volume reaches. A property of the ASSET,
	 *  measured rather than chosen, and NOT one of the sixteen derivation inputs -- it is
	 *  used only to predict the fold ceiling. A 64^3 three-octave smooth volume measures
	 *  14.09.
	 *
	 *  WRONG IN BOTH DIRECTIONS. If the amplitudes look right but the predicted shear is
	 *  far above one, this is the number that disagrees, and the ratio between where the
	 *  web actually tears and the prediction is the factor to correct it by. Becomes 1 and
	 *  drops out entirely once volumes are gradient-normalized at bake time. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Asset", meta = (ClampMin = "0.0"))
	float WarpTexGradient = 14.09f;

	FUniverseDensityParams()
	{
		// THE SMALL OCTAVE IS NOT A COPY OF THE LARGE ONE, so its defaults are set here
		// rather than left to FUniverseWarpOctaveParams's own. Amplitude two orders lower
		// and a scale ten times higher: fine grain against coarse bend. See the fold
		// ceiling note on the member.
		WarpSmall.Amount = FUniverseVarianceRange(0.04f, 0.06f, 2.0f);
		WarpSmall.Scale = 0.75f;
		WarpSmall.LatticeFollow = 0.0f; // ignored; the small octave is pinned at full follow
	}

#pragma region Derivation
	/** THE ONLY PLACE THE PACKING DECISIONS LIVE. Every downstream consumer -- the material
	 *  push, the compute parameter fill, and a future CPU derivation -- reads this rather
	 *  than packing again.
	 *
	 *  Pure packing plus the four .w passengers. Each passenger belongs to the SAME FIELD
	 *  as its host: skew already rode the spread's interpolant, each warp scale multiplies
	 *  its own octave's amount in the shear product, and the lattice-follow steers the same
	 *  octave the weights vector gains. A pin carrying both is one setting rather than two
	 *  unrelated ones sharing a wire, and that is the test the next claim should pass.
	 *
	 *  THE SEED IS AN ARGUMENT, not a member, because FUniverseParams::Seed is the
	 *  authoritative one and a second copy here would be free to disagree with it. The
	 *  offset is an argument because it is per-frame runtime state. */
	FUniverseDensityArgs Pack(int32 InSeed, const FUniverseFieldOffset& InOffset) const
	{
		FUniverseDensityArgs Out;

		Out.CellSizeRange = FVector4f(
			Lattice.CellSizeSmall, Lattice.CellSizeLarge, Lattice.CellSizeBias,
			// UNCLAIMED. The natural host for a lattice blend override or a weight between
			// the noise value and a constant; the core ignores it today.
			0.0f);

		Out.Seed = static_cast<float>(InSeed);

		Out.OffsetCell = FVector3f(
			static_cast<float>(InOffset.Cell.X),
			static_cast<float>(InOffset.Cell.Y),
			static_cast<float>(InOffset.Cell.Z));
		Out.OffsetFrac = FVector3f(
			static_cast<float>(InOffset.Frac.X),
			static_cast<float>(InOffset.Frac.Y),
			static_cast<float>(InOffset.Frac.Z));

		Out.WallDensityRange = Wall.Density.Pack();
		Out.WallFalloffRange = Wall.Falloff.Pack();
		Out.FilamentDensityRange = Filament.Density.Pack();
		Out.FeatureWidthRange = FeatureWidth.Pack();
		Out.VoidFloorRange = Void.Floor.Pack();

		// --- THE FOUR PASSENGERS, loaded in one place ---
		Out.VoidSizeSpreadRange = Void.SizeSpread.Pack(Void.SizeSkew);
		Out.WarpAmountLargeRange = WarpLarge.Amount.Pack(WarpLarge.Scale);
		Out.WarpAmountSmallRange = WarpSmall.Amount.Pack(WarpSmall.Scale);

		Out.WarpLargeWeights = FVector4f(
			static_cast<float>(WarpLarge.Weights.X),
			static_cast<float>(WarpLarge.Weights.Y),
			static_cast<float>(WarpLarge.Weights.Z),
			WarpLarge.LatticeFollow);

		Out.WarpSmallWeights = FVector4f(
			static_cast<float>(WarpSmall.Weights.X),
			static_cast<float>(WarpSmall.Weights.Y),
			static_cast<float>(WarpSmall.Weights.Z),
			0.0f); // no passenger; the small octave is pinned at full lattice follow

		Out.RegionScales = FVector4f(Region.ScaleStructure, Region.ScaleAppearance, 0.0f, 0.0f);

		Out.BoundsFadeStart = BoundsFadeStart;

		return Out;
	}

	/** Fills a compute shader parameter struct from the packed arguments, member for
	 *  member. The shader-side names match MakeUniverseDensityParams's parameter names, so
	 *  a parameter added to the derivation breaks this at compile time on both sides.
	 *
	 *  A TEMPLATE on purpose: naming the concrete FParameters type would make this header
	 *  depend on the entity-gen header, which depends on this one, and would drag
	 *  RenderCore into every translation unit that only wanted the authored struct. */
	template <typename TShaderParams>
	void FillShaderParameters(TShaderParams& Out, int32 InSeed, const FUniverseFieldOffset& InOffset) const
	{
		const FUniverseDensityArgs A = Pack(InSeed, InOffset);

		Out.InCellSizeRange = A.CellSizeRange;
		Out.InSeed = A.Seed;
		Out.InOffsetCell = A.OffsetCell;
		Out.InOffsetFrac = A.OffsetFrac;
		Out.InWallDensityRange = A.WallDensityRange;
		Out.InWallFalloffRange = A.WallFalloffRange;
		Out.InFilamentDensityRange = A.FilamentDensityRange;
		Out.InFeatureWidthRange = A.FeatureWidthRange;
		Out.InVoidFloorRange = A.VoidFloorRange;
		Out.InVoidSizeSpreadRange = A.VoidSizeSpreadRange;
		Out.InWarpAmountLargeRange = A.WarpAmountLargeRange;
		Out.InWarpAmountSmallRange = A.WarpAmountSmallRange;
		Out.InWarpLargeWeights = A.WarpLargeWeights;
		Out.InWarpSmallWeights = A.WarpSmallWeights;
		Out.InRegionScales = A.RegionScales;
		Out.InBoundsFadeStart = A.BoundsFadeStart;
	}

	/** NO CPU DERIVATION YET, and the omission is deliberate rather than pending.
	 *
	 *  The galaxy layer compiles its core into C++ behind GalaxyHLSLShim.h and returns a
	 *  constructed params struct. The universe core does not currently compile that way:
	 *  WeightedVector3 returns c.xyz and the shim's float4 has no .xyz accessor, and the
	 *  weighted-scalar path needs float4 - float and float4 * float4 which the shim also
	 *  lacks. Those gaps are pre-existing and latent, since nothing consumes this field
	 *  from C++ yet.
	 *
	 *  CLOSING THEM WOULD NOT MAKE A CPU PLACEMENT PATH CORRECT. The shim stubs Texture3D
	 *  fetches to a neutral 0.5, so the C++ field reduces to the unwarped analytic web --
	 *  and this field is more texture-dependent than the galaxy's, not less: two region
	 *  fetches and three warp fetches, and turning them all neutral changes the GEOMETRY,
	 *  not just the shading. GalaxyHLSLShim.h already says in bold that the shim path must
	 *  never place entities for exactly that reason.
	 *
	 *  So entity generation lands on the compute harness, which samples the same volume the
	 *  material does, and FillShaderParameters is how it gets its inputs. If a CPU probe is
	 *  ever wanted for debugging rather than placement, close the shim gaps then and add a
	 *  derivation beside Pack -- it is three lines against this struct. */
#pragma endregion

#pragma region Validation
	 /** Predicted worst-case warp shear on the COARSE lattice, where the ceiling binds:
	  *
	  *      (AmountLarge * ScaleLarge / ratio + AmountSmall * ScaleSmall * ratio) * Gradient
	  *
	  *  At or below 1 the web bends. Above it the displacement folds, neighbouring samples
	  *  cross over, and the web TEARS -- and only in the regions the blend has handed to the
	  *  coarse lattice, so region-dependent tearing reads as a lot of other things before it
	  *  reads as a warp amplitude problem.
	  *
	  *  Each amount is that octave's MAX, since that is what a region can actually resolve
	  *  to. Neither octave is divergence-free, so there is no headroom above 1 the way curl
	  *  noise had: this sits close to the real tear point rather than being a conservative
	  *  bound under it. Practical tuning is unchanged -- fix the scales, raise one amplitude
	  *  until the web visibly tears, back off about thirty percent. */
	float PredictedFoldShear() const
	{
		const float Ratio = FMath::Max(FMath::RoundToFloat(
			FMath::Max(Lattice.CellSizeLarge, 0.0f) / FMath::Max(Lattice.CellSizeSmall, 1e-6f)), 1.0f);

		const float LargeTerm = WarpLarge.Amount.Max * WarpLarge.Scale / Ratio;
		const float SmallTerm = WarpSmall.Amount.Max * WarpSmall.Scale * Ratio;

		return (LargeTerm + SmallTerm) * WarpTexGradient;
	}

	/** Cells before two scales re-align, given both are whole numbers of 1/4096ths. Two
	 *  scales with numerators p and q repeat every 4096/gcd(p,q) cells; coprime numerators
	 *  give the full 4096, which is the precision wrap itself, so nothing repeats before
	 *  the field wraps anyway. Odd numerators are coprime to any power-of-two scale for
	 *  free. */
	static int32 ScaleRepeatPeriod(float InScaleA, float InScaleB)
	{
		const int32 Wrap = 4096;
		const int32 P = FMath::Max(FMath::RoundToInt(InScaleA * Wrap), 1);
		const int32 Q = FMath::Max(FMath::RoundToInt(InScaleB * Wrap), 1);
		return Wrap / FMath::GreatestCommonDivisor(P, Q);
	}

	/** Logs the constraints that are invisible in the details panel: the fold ceiling, the
	 *  pairwise repeat period of the four texture scales, and the handful of values whose
	 *  failure mode is a lit void or an unbounded march rather than an odd-looking field.
	 *
	 *  REPORTS RATHER THAN CORRECTS, matching the core's own no-clamping policy. Returns
	 *  false if anything is out of bounds. */
	bool Validate(const TCHAR* InContext = TEXT("FUniverseDensityParams")) const;
#pragma endregion
};


// =============================================================================
// THE MARCH
// =============================================================================

/** Controls for UniverseRayMarch. PERFORMANCE controls rather than look controls, and
 *  candidates for the game's quality settings.
 *
 *  THE STEPPING RULE IS NOT THE GALAXY'S. The galaxy sizes each step as a fraction of
 *  camera distance, floored and capped by two budgets. This march sizes the first step
 *  from the chord and grows every step after it with progress along that chord:
 *
 *      baseStep = chord / StepCount
 *      h        = baseStep * (1 + StepGrowth * t01)
 *
 *  The reason is the near field. A camera-distance step goes to ZERO at the camera, and
 *  the camera is normally inside this volume since the proxy is centred on it, so
 *  successive samples landed inside a single cell -- each paying a full fifty-four
 *  candidate neighbourhood walk to re-derive a field that had not changed between them. A
 *  step derived from the chord cannot collapse that way, because it does not know where
 *  the camera is. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseMaterialParams
{
	GENERATED_BODY()

	/** A CEILING ON THE STEP COUNT, NOT A FLOOR, which is what the material's old
	 *  MinSamples name had backwards. Growth only ever lengthens steps, so every step is
	 *  at least span / StepBudget and the actual count is StepBudget * ln(1+g)/g -- equal
	 *  to the budget only at growth 0, 0.69 of it at growth 1, 0.46 at growth 3. See
	 *  GetEffectiveStepCount. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "8.0", ClampMax = "1024.0"))
	float VolumeStepBudget = 128.0f;

	/** How fast steps lengthen along the chord; 0 is uniform stepping. The cheapest quality
	 *  lever here, since it trades far-field detail for step count directly and the far
	 *  field is where the bounds fade is taking the field out anyway. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0", ClampMax = "8.0"))
	float VolumeStepGrowth = 4.0f;

	// NO STEP CEILING AND NO ITERATION BOUND HERE, and both were removed rather than
	// forgotten. The march's step floor is P.MinFeatureStep -- half the narrowest wall the
	// parameters can produce, derived in the core from FeatureWidth and CellSizeSmall
	// together -- and an authored absolute cannot compete with a floor that knows the cell
	// size. The loop bound is derived in the Custom node from StepBudget, because the
	// budget already bounds the count and an authored bound could therefore only truncate:
	// a truncated march looks identical to one that finished, minus the far half of the
	// volume.

	/** Multiplier on density before the march's exp(-density * scale * h) -- how opaque the
	 *  result is, and nothing else. The field's own densities set the RATIO between walls,
	 *  filaments and the floor.
	 *
	 *  KEEP IT AT 1 UNLESS THERE IS A REASON. It is tempting as a brightness control,
	 *  because it is the last multiplier before the accumulation, but driving the look from
	 *  here scales the apparent field several times over its authored range -- and then
	 *  every judgement made against the picture is made against a field that does not exist.
	 *  The exponents stop reading as falloff curves because the values they shape are off
	 *  their intended scale, and the void floor stops looking like a floor.
	 *
	 *  Output brightness belongs after the march, on the material's own intensity
	 *  multiplier, where it cannot be mistaken for a property of the field. Note that this
	 *  never reached placement either way: acceptance normalises against each cell's own
	 *  probed envelope, so nothing here has ever changed where an entity lands -- only what
	 *  the field looked like while it was being tuned. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0"))
	float VolumeDensityScale = 1.0f;

	/** Exponent applied to each sample before accumulation. Exactly 1 skips the pow -- the
	 *  march tests for it. Above 1 deepens voids and thins sheets; below 1 flattens toward
	 *  uniform. Prefer the wall falloff for clearing voids: it does the same job inside the
	 *  field and costs its two SFU ops once rather than per step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0"))
	float VolumeNoisePower = 1.0f;

	// NO DITHER PIN. It is computed per pixel inside the Custom node, from SvPosition and
	// the frame index, because Parameters and View exist only inside the generated material
	// function -- and there is no case for disabling it in a shipped material. A debug
	// toggle would be lerp(1, jitter, k) in the node with a new pin, not a value pushed
	// from here.

	/** WHO OWNS THE FIELD PARAMETERS: FUniverseDensityParams, or the material instance.
	 *
	 *  FALSE for as long as look development is happening in the material.
	 *  MT_UniverseRaymarchAnalytic_Inst already carries a tuned set, and pushing would
	 *  overwrite every one of them with defaults reasoned from the core's commentary rather
	 *  than measured against a picture. The field offset is pushed either way -- it is
	 *  runtime state with no authored counterpart, so there is nothing for it to clobber.
	 *
	 *  IT MUST BE TRUE BEFORE ENTITY GENERATION LANDS. The compute path reads
	 *  FUniverseDensityParams, so from the moment anything is placed against the field, a
	 *  material tuned independently of that struct means the render and the placement are
	 *  two different universes -- and the failure is silent, because both look plausible.
	 *  Transcribe the instance's tuned values into this struct's defaults and set this, in
	 *  that order, as one step.
	 *
	 *  A MID SILENTLY IGNORES A NAME THE MATERIAL DOES NOT HAVE, which is the other reason
	 *  to flip this deliberately: the first push is where a mismatched pin name surfaces,
	 *  and it surfaces as a parameter that keeps its authored value with no warning
	 *  anywhere. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	bool bPushDensityParams = true;

	/** DEBUG: write each entity's SAMPLED DENSITY into its colour instead of its decoratives.
	 *
	 *  THE ONE MEASUREMENT THAT SEPARATES THE TWO FAILURES. Entities landing in the wrong
	 *  place can mean the placement path sampled the field correctly and the population was
	 *  distributed badly, or it can mean the two paths disagree about WHERE a given point of
	 *  the field is. Those look identical from outside and have nothing in common as fixes.
	 *
	 *  Each entity carries the density the dispatch actually read at its own position. Paint
	 *  that and compare against the raymarch:
	 *
	 *    bright entities in bright nebula, dark in dark   the paths agree; the fault is in
	 *                                                     the budget or the distribution
	 *    bright entities sitting in dark nebula           the paths disagree about position:
	 *                                                     one of them samples a shifted or
	 *                                                     differently scaled field
	 *    entities uniformly mid-grey                      the dispatch is not reading the
	 *                                                     texture at all
	 *
	 *  Normalised by the analytic ceiling -- the sum of the three resolved maxima -- so the
	 *  scale is comparable between tiers and does not move when the ranges are retuned. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material|Debug")
	bool bDebugColorByDensity = false;

	/** The packed noise volume the field fetches for both region variances and both warp
	 *  octaves.
	 *
	 *  ITS REQUIREMENTS, for when the bake utility lands: periodic, so repeat addressing
	 *  closes the 4096-cell wrap; per-channel normalized to 0.5 with comparable spread; a
	 *  distinct archetype per channel. Ideally gradient-normalized too, which makes
	 *  WarpTexGradient 1 and drops it out of the fold ceiling entirely.
	 *
	 *  THE SAME ASSET MUST REACH THE COMPUTE PATH. Placement and render sample one texture
	 *  or they are not one field. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VolumeNoise = "/UltraLargeScale/VolumeTextures/VT_PerlinWorley_Balanced";

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
};


/** Universe-layer generation parameters: the cosmic-web density field, the march that
 *  draws it, per-tier streaming configs, and scale derivation.
 *
 *  ONE FIELD, ONE PLACE. The legacy noise graph that used to sit beside DensityParams is
 *  gone; there is no longer a second set of parameters describing a second universe.
 *
 *  THE GAS LAYER IS GONE. GasExtentMinMultiplier and GasExtentMaxMultiplier sized a
 *  nebula sprite that shared the Large tier's positions; the raymarch replaced it. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

#pragma region Density Field

	/** The cosmic-web field: what UniverseDensityCore.ush evaluates, what the ray march
	 *  draws, and what entity generation will place against. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseDensityParams DensityParams;

	/** How the field is MARCHED, as opposed to what it contains. Kept apart from
	 *  DensityParams because none of it reaches MakeUniverseDensityParams: these are the
	 *  march's own arguments plus the noise asset, and only the render consumes them.
	 *  Entity generation samples the same field with none of this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FUniverseMaterialParams MaterialParams;

#pragma endregion

#pragma region Tier Scale Derivation

	/** Absolute largest entity scale (world units) the sector supports. All tier
	 *  scale ranges cascade downward from this single value:
	 *    Tier[0].MaxScale = MaxEntityScale
	 *    Tier[0].MinScale = MaxEntityScale / 2^(depth[1] - depth[0])
	 *    Tier[1].MaxScale = Tier[0].MinScale
	 *    ... and so on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 1e22;

#pragma endregion

#pragma region Tier Debug

	/** DEBUG: which tiers stream at all.
	 *
	 *  FOR ISOLATING ONE TIER AGAINST THE RENDER, which is otherwise hard to do by eye:
	 *  the three tiers reach very different distances -- Large spans the whole marched
	 *  proxy, Mid a quarter of it and Small a sixteenth -- so what you see at any given
	 *  depth is a different mixture, and a placement fault in one reads as a fault
	 *  everywhere.
	 *
	 *  A tier switched off here is never initialized and never updated, so it costs nothing
	 *  rather than being generated and hidden. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Debug")
	bool bEnableLargeTier = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Debug")
	bool bEnableMidTier = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Debug")
	bool bEnableSmallTier = true;

#pragma endregion

#pragma region Per-Tier Streaming Configs

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

#pragma endregion

#pragma region Defaults & Derivation

	/** Derives MinScale/MaxScale for each tier from MaxEntityScale and the depth
	 *  sequence. Delegates to FTierParams::DeriveTierScaleRanges. */
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		FTierParams::DeriveTierScaleRanges(MaxEntityScale, Tiers);
	}

	FUniverseParams() {
		Seed = 69;
		UnitScale = 1.6e17;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1);

		// Tier streaming params: depths evenly spaced by 2 (ratio 4 per tier).
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.SlotCapacity = 1000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 500;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 500;

		// GENERATION SUBDIVISION, sized against the FIELD'S cell rather than left at the
		// zero default -- which gave nine thread groups per transition batch and left the
		// GPU essentially idle.
		//
		// A field cell is ProxyExtent * CellSizeSmall = 0.9 sector extents at the shipped
		// values, and a tier cell at depth d is 4/2^d of one. So the Large tier's cell spans
		// about 2.2 field cells undivided, which is far too coarse for its probes to
		// describe; two levels bring it to 0.56. Mid and Small are already at or below that
		// undivided, so they take one level for OCCUPANCY rather than for resolution --
		// nine groups is not a dispatch.
		//
		// These are a starting point, not a tuning. Read the C/P ratio in the batch log:
		// below about 1 the probes have overtaken placement and a tier wants one level
		// fewer, above about 9 it wants one more.
		LargeTier.GenerationSubdivision = 2;
		MidTier.GenerationSubdivision = 1;
		SmallTier.GenerationSubdivision = 1;

		// Scale ranges derived from MaxEntityScale (1e22) + depth spacing (2).
		// 2^2 = 4, so each tier covers two octaves of scale (64x total spread):
		//   Large: 2.5e21    -> 1e22
		//   Mid:   6.25e20   -> 2.5e21
		//   Small: 1.5625e20 -> 6.25e20
		DeriveScaleRanges();
	}

#pragma endregion
};

// FUniverseParamBounds IS GONE, and its removal is the point rather than a tidy-up.
//
// It held a Min and a Max, both whole FUniverseParams, and AUniverseActor's CONSTRUCTOR
// assigned UniverseParams = Generate(Bounds, 666) -- whose stub returned Bounds.Max. So the
// authored data lived at UniverseParamBounds.Max, the struct it fed was not itself a
// UPROPERTY and therefore not editable, and the copy happened at construction rather than
// at spawn.
//
// EVERY PART OF THAT WAS A TRAP. Editing Bounds.Min did nothing. Editing Bounds.Max on a
// placed instance did nothing until the actor was constructed again. And nothing anywhere
// said so -- the symptom was a parameter that appeared to have no effect, which reads as a
// broken parameter rather than a value that never arrived. It cost two rounds of debugging
// here: GenerationSubdivision read 0 after being set to 2, and SpawnExponent showed no
// difference between 0.01 and 16 because both were the default.
//
// THE GALAXY LAYER ALREADY MADE THIS CALL, retiring FGalaxyParamBounds for the same reason
// its own comment gives: Min and Max were whole param structs and so carried a config block
// each, leaving two config sources in one panel with one of them doing nothing.
//
// A RANDOMISED UNIVERSE NEEDS NO BOUNDS PAIR ANYWAY. There is one universe per sector and
// its variation is regional rather than per-instance -- the two region fetches ARE the
// archetype system, resolving per sample. If per-sector variation is ever wanted, it wants
// the shape UGalaxyArchetype uses -- a named parameter, a range and a bias, rolled from the
// sector seed -- not a second copy of every field.
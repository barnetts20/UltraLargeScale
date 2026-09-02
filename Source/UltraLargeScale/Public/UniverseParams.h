// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "UniverseParams.generated.h"

class UTexture;

/** The field's four volume textures, RESOLVED. One bundle rather than four loose pointers,
 *  and that is the whole reason it exists: this set travels from the actor to the material,
 *  to the data generator, to the entity-gen dispatch, across two thread hops and into a
 *  render command. Four parameters at each of those boundaries is four chances to pass
 *  three of them, and a dispatch bound with a stale fourth texture places entities against
 *  a field nobody renders -- which looks exactly like a plausible cosmic web with the
 *  entities somewhere else.
 *
 *  ALL FOUR OR NONE, enforced by IsComplete at every consumer. There is deliberately no
 *  partial mode: with one texture missing the field does not degrade, it becomes a
 *  different field. The variance fetches fall back to a neutral 0.5 -- the midpoint of
 *  every authored range, so all regional structure vanishes -- and a missing warp volume
 *  straightens the bisectors outright. Both are changes in GEOMETRY, not shading, so
 *  placement against them is not an approximation of the rendered field but an unrelated
 *  one.
 *
 *  RAW POINTERS, not UPROPERTY. This is a transport struct; the actor owns the references
 *  that keep the objects alive, and every consumer here outlives nothing. Anything holding
 *  one of these across a frame boundary should hold the actor's pointers instead. */
struct FUniverseFieldTextures
{
	/** Region axes, coarse scale -- the structure field. UNORM multinoise. */
	UTexture* VarianceA = nullptr;

	/** Region axes, finer scale -- the appearance field. A DIFFERENT multinoise variant;
	 *  see the authored paths in FUniverseMaterialParams for why it must not be the same
	 *  asset as VarianceA. */
	UTexture* VarianceB = nullptr;

	/** Large warp octave. Signed vector field. Read TWICE per sample -- once on the fine
	 *  lattice and once on the coarse one -- and those two fetches must stay on one asset
	 *  or the two tiers stop bending like one field. */
	UTexture* WarpLarge = nullptr;

	/** Small warp octave. Signed vector field. */
	UTexture* WarpSmall = nullptr;

	bool IsComplete() const
	{
		return VarianceA != nullptr && VarianceB != nullptr
			&& WarpLarge != nullptr && WarpSmall != nullptr;
	}

	/** Which ones are missing, for a log line that says something actionable. Returns an
	 *  empty string when the set is complete. */
	FString DescribeMissing() const
	{
		TArray<FString> Missing;
		if (!VarianceA) { Missing.Add(TEXT("VarianceA")); }
		if (!VarianceB) { Missing.Add(TEXT("VarianceB")); }
		if (!WarpLarge) { Missing.Add(TEXT("WarpLarge")); }
		if (!WarpSmall) { Missing.Add(TEXT("WarpSmall")); }
		return FString::Join(Missing, TEXT(", "));
	}
};

/** Seed channels for the universe layer, mirroring GalaxySeed.
 *
 *  ADD, NEVER REUSE. Channels are free; sharing one between two consumers reintroduces
 *  exactly the aliasing ProcSeed exists to prevent. */
namespace UniverseSeed
{
	/** The GPU placement key. One channel for all three tiers -- the tier index enters
	 *  through MixSeed's index argument, so the streams stay distinct without a channel
	 *  each, and adding a fourth tier needs no new constant. */
	inline constexpr uint32 Placement = ProcSeed::ChannelId("Universe.Placement");
}


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
	 *  KEEP THE RATIO CLEAR OF A HALF. The derivation rounds CellSizeLarge / CellSizeSmall,
	 *  and HLSL rounds halves away from zero while the C++ shim's round defers to nearbyint,
	 *  which rounds to even. A ratio landing on x.5 is therefore free to quantize
	 *  differently on the two sides, and the failure is the coarse lattice coming out a
	 *  different size in the render than in placement. 0.9 against 0.3 gives 3 exactly, with
	 *  the nearest half boundary far off in both directions; 1.4 against 0.4 gives 3.5 and
	 *  sits on it.
	 *
	 *  At or below CellSizeSmall this collapses to a single lattice and skips the second
	 *  neighbourhood walk entirely, which is roughly half the cost of a sample. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice", meta = (ClampMin = "0.0"))
	float CellSizeLarge = 0.9f;

	/** Exponent shaping where between the two lattices a region sits. NOT a range: the two
	 *  extremes it interpolates are the lattices themselves, so it has no min/max of its
	 *  own and never passes through the variance remap.
	 *
	 *  THERE IS NO WAY TO PIN OR CLAMP THE BLEND. Every value in [0,1] is reachable
	 *  somewhere, so "30% coarse everywhere" and "never enter the mixed band" are both
	 *  unauthorable. The useful band is narrow -- 0.4 to 0.8 is where both lattices compete,
	 *  and mean density peaks near 0.30 there against 0.06 at either end. CellSizeRange.w is
	 *  the free slot an override would ride; it is packed as 0 and the core ignores it, so
	 *  claiming it is a change in the core first and here second. */
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
	 *  into unbounded gain rather than opacity. BOTH ENDS OF THE RANGE MATTER: a range
	 *  spanning zero puts the negative end somewhere, and a noise channel decides where. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	FUniverseVarianceRange Floor = FUniverseVarianceRange(0.0f, 0.5f, 8.0f);

	/** Per-node power-diagram offset, so voids differ in size rather than tiling at one
	 *  scale.
	 *
	 *  SQUARED CELL UNITS, not cells. It is subtracted from the squared distance, which is
	 *  the space the weight works in and is what keeps 27 square roots out of the walk. A
	 *  node out-reaches an unoffset neighbour by this much in squared distance -- about the
	 *  same in linear distance at a typical half-cell separation, shrinking as the
	 *  separation grows.
	 *
	 *  THIS IS WHAT ERODES THE 27-CELL SEARCH. The window holds while the offset stays well
	 *  under 1.0 minus the dominant candidate's own power distance, and the bound degrades
	 *  with WIDE features as well as with large offsets -- so the safe value depends on
	 *  FeatureWidth, not on this pin alone. Past it, a node outside the searched
	 *  neighbourhood wins and the cell it should have claimed is drawn by the wrong one.
	 *  Tune upward from an order below the default rather than downward. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	FUniverseVarianceRange SizeSpread = FUniverseVarianceRange(0.0f, 0.5f, 2.0f);

	/** Rides SizeSpread's own .w. Above 1 makes large voids rare and small ones common.
	 *
	 *  IT RIDES THAT SLOT BECAUSE IT IS A FUNCTION OF THAT RANGE: skew multiplies the
	 *  spread's own bias-shaped interpolant, so the two are one setting in the field before
	 *  they are one pin in the packing.
	 *
	 *  INERT. UNIVERSE_SKEW_FIXED is 4, so the core's SkewPow ignores its exponent argument
	 *  and this pin, the per-candidate product it feeds and the .w slot it rides do nothing.
	 *  Tuning against it produces no change of any kind. The general path costs 54 pow calls
	 *  per sample and is not worth restoring; a lerp between the fixed exponents 2 and 4 is
	 *  the affordable version. Retire the pin or implement the lerp -- an authored control
	 *  that cannot move anything is worse than no control. */
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
	 *  DENOMINATED IN THE ASSET'S VALUE SCALE, not an absolute one. The core decodes a
	 *  displacement from the fetched channels and multiplies by this, so a volume whose
	 *  channels swing half as far needs twice the amount for the same bend. SWAPPING AN
	 *  ASSET SILENTLY RETUNES THE GEOMETRY: re-tune this and the octave's WarpTexGradient
	 *  together whenever the volume changes, and judge it by PredictedFoldShear rather than
	 *  by the picture. Each octave carries its own asset and its own gradient, so the two
	 *  octaves re-tune independently.
	 *
	 *  THE RANGE IS A 4x SPREAD around a linear bias, so a region can resolve anywhere from
	 *  a barely-bent web to a strongly-bent one. Wider than the small octave's in ratio
	 *  terms and much narrower in shear terms, because the coarse lattice divides this
	 *  octave's contribution by the lattice ratio. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	FUniverseVarianceRange Amount = FUniverseVarianceRange(0.333f, 1.333f, 1.0f);

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
	 *  FIXED ACROSS THE DRAW, never ranged. A scale that varies by region puts a different
	 *  texture frequency on each side of a province boundary, which the wrap cannot close.
	 *
	 *  THE NUMERATOR MUST BE COPRIME WITH THE OTHER THREE SCALES. 409 is prime. A round 0.1
	 *  quantizes to 410, which shares a factor of 41 with Region.ScaleAppearance and
	 *  re-aligns every ninety-nine cells -- twenty proxy widths, and visible as a grid.
	 *  Author 0.1 in the details panel and that grid appears.
	 *
	 *  ANYTHING IN [0.09984, 0.10008] IS THIS SAME NUMBER. The quantum is 1/4096, about
	 *  0.000244, so a smaller edit lands back on 409 and changes nothing: the details panel
	 *  shows the authored value while the field uses the quantized one. To move this scale
	 *  at all, move by at least one quantum, and check the new numerator against the
	 *  coprimality report in Validate(). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp", meta = (ClampMin = "0.0"))
	float Scale = 0.099854f;

	/** Per-axis gain on the decoded displacement. Elementwise, not a dot product -- a warp
	 *  needs all three axes, unlike a scalar site's weights. 0 flattens the warp along
	 *  that axis, negative inverts it, magnitude above 1 is gain. Deliberately
	 *  unclamped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	FVector Weights = FVector(1.0, 1.0, 1.0);

	/** LARGE OCTAVE ONLY. Rides the weights vector's .w -- there is no fourth axis to steer
	 *  -- and is saturated by the core.
	 *
	 *  How much of the cell size ratio this octave's amplitude follows on the coarse
	 *  lattice: 0 gives the same physical displacement on both tiers, 1 gives the same
	 *  displacement measured in each tier's own cells. THE SMALL OCTAVE IGNORES THIS; it is
	 *  pinned at full follow, which is what scaling its amount against a fixed wavelength
	 *  amounts to. */
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
	 *  19/4096, AND PRIME FOR A REASON. A round 0.005 quantizes to 20, which shares a factor
	 *  of 10 with a large warp octave at 410 and re-aligns every 409 cells -- eighty proxy
	 *  widths, reachable in one session.
	 *
	 *  ODD IS NOT SUFFICIENT. 21 is 3x7 against a small warp octave of 3072 = 2^10x3, still
	 *  sharing a 3 and repeating every 1365 cells. Only a prime numerator is coprime with
	 *  the rest by construction. The visual difference between 0.005 and 0.004639 is
	 *  nothing; the difference between a repeating field and one that does not is not. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	float ScaleStructure = 0.004639f;

	/** Repeats per small cell for the APPEARANCE field. Drives wall density and falloff,
	 *  filament density, void floor, and the small warp octave.
	 *
	 *  41/4096, prime, which is what a round 0.010 quantizes to anyway. WATCH IT AGAINST THE
	 *  WARP SCALES: a large warp octave at 410 shares a factor of 41 with this and re-aligns
	 *  every ninety-nine cells, twenty proxy widths, the tightest repeat the set can
	 *  produce. See the coprimality note on FUniverseWarpOctaveParams::Scale.
	 *
	 *  IT READS THE SAME FOUR ARCHETYPES AS THE STRUCTURE FIELD, at a different frequency. A
	 *  second asset with four different ones removes the last correlation between the two
	 *  region fetches. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	float ScaleAppearance = 0.010010f;
};


// =============================================================================
// THE FIELD OFFSET
// =============================================================================

/** THE FIELD PERIOD, MIRRORED FROM THE CORE. MakeUniverseDensityParams holds the
 *  authoritative derivation; this is the CPU copy, needed to reduce a cell index BEFORE it
 *  narrows to a float3 pin.
 *
 *  A MIRROR IS NOT THE PREFERENCE. UniverseDensityCore.ush compiles for C++ behind
 *  GalaxyHLSLShim.h, so this could in principle call the real thing -- but no universe
 *  translation unit includes the core, and pulling the whole field surface into
 *  UniverseActor.cpp to share four lines of integer arithmetic is the worse trade.
 *
 *  WHAT A DISAGREEMENT COSTS IS BOUNDED, which is why the mirror is tolerable at all: the
 *  core reduces AGAIN with its own period, so a mismatch here cannot seam the field. It can
 *  only leave the index above 2^24 when it reaches the pin, costing exactness and nothing
 *  else. Keeping the core's reduction decomposes the failure into "less precise" rather
 *  than "wrong".
 *
 *  THE RATIO MUST COME FROM THE SAME CELL SIZES THE CORE RECEIVES. Both reach the core
 *  through FUniverseDensityParams::Pack, so FieldCellPeriod below takes the lattice group
 *  whole and every consumer calls it. */
namespace UniverseCellWrap
{
	/** Mirror of UNIVERSE_CELL_EXACT_MAX and UNIVERSE_CELL_WRAP_ALIGN in the core. The
	 *  second is the texture wrap (UNIVERSE_WARP_WRAP_SHIFT), which the period must
	 *  preserve or the warp UV steps where the density does not. */
	constexpr int32 ExactMax = 1 << 24;
	constexpr int32 WrapAlign = 1 << 12;

	/** The lattice ratio, rounded and floored exactly as MakeUniverseDensityParams rounds
	 *  it. Any drift between the two roundings changes the period, so this expression is
	 *  deliberately the same shape rather than the same intent. */
	inline int32 DeriveRatio(double InCellSizeSmall, double InCellSizeLarge)
	{
		const double Small = FMath::Max(InCellSizeSmall, 1e-6);
		const double Ratio = FMath::FloorToDouble(FMath::Max(InCellSizeLarge, 0.0) / Small + 0.5);
		return FMath::Max(static_cast<int32>(Ratio), 1);
	}

	/** The largest multiple of (WrapAlign * ratio) below ExactMax. Both divisors are
	 *  required; see THE FIELD PERIOD in the core. */
	inline int32 DerivePeriod(int32 InRatio)
	{
		const int32 Ratio = FMath::Max(InRatio, 1);
		const int32 Block = WrapAlign * Ratio;
		return Block * FMath::Max(ExactMax / Block, 1);
	}

	/** THE ONE CALL EVERY CONSUMER MAKES. Both sides of the field reduce a cell index by
	 *  this period before it crosses to the shader -- the actor for the material offset,
	 *  the data generator for each gen cell's centre -- and reducing by two different
	 *  periods puts placement and the render on wraps that disagree, which shows up only a
	 *  long way out and logs nothing.
	 *
	 *  Taking the lattice group rather than two floats is what makes that hard to get
	 *  wrong: there is no call shape that passes a small cell size from one source and a
	 *  large one from another. */
	inline int32 FieldCellPeriod(const FUniverseLatticeParams& InLattice)
	{
		return DerivePeriod(DeriveRatio(
			static_cast<double>(InLattice.CellSizeSmall),
			static_cast<double>(InLattice.CellSizeLarge)));
	}

	/** Reduces into [0, InPeriod). FLOORED, not truncated: C++ and HLSL both truncate
	 *  toward zero, so a negative index would come back negative and hash as a different
	 *  cell from its counterpart one period away. */
	inline int32 Wrap(int32 InValue, int32 InPeriod)
	{
		if (InPeriod <= 0) return InValue;
		const int32 R = InValue % InPeriod;
		return (R < 0) ? (R + InPeriod) : R;
	}

	/** THE PRESENTATION HALF OF Wrap, and the ONLY thing it is for.
	 *
	 *  Every consumer of a cell index -- the hash, StepCell's bounds check, the & 4095
	 *  texture wrap, the gen cells -- wants the unsigned [0, period) form Wrap produces, and
	 *  none of them may be handed this one. What that form costs is legibility: the index is
	 *  reduced with a FLOORED modulo, so one cell left of the origin reports period-1. That
	 *  is not a rollover, it is the torus closure -- cell -1 and cell period-1 are the same
	 *  cell, and that identity is exactly what makes the wrap seamless -- but nobody reads
	 *  sixteen million as 'minus one', least of all while debugging something else.
	 *
	 *  This folds to [-period/2, period/2), which moves the displayed discontinuity half a
	 *  period away in each direction instead of leaving it at spawn. It lives here rather
	 *  than in the readout so the two mappings sit together and a second reader cannot
	 *  invent a third convention.
	 *
	 *  DISPLAY ONLY. Feeding this back into anything that hashes gives a different cell.
	 *  Period must be even for the halves to meet, which the derivation guarantees -- it is
	 *  a multiple of 4096. */
	inline int32 ToSigned(int32 InWrapped, int32 InPeriod)
	{
		if (InPeriod <= 0) return InWrapped;
		return (InWrapped >= InPeriod / 2) ? (InWrapped - InPeriod) : InWrapped;
	}
}


/** The player's position in the field, split the way UniverseDensityCore.ush wants it: an
 *  exact integer cell count plus a fraction in [0,1). Runtime state rather than authored,
 *  which is why it is a Pack() argument instead of a member -- it changes every frame and
 *  nothing else in the set does.
 *
 *  THE CELL COUNT IS REDUCED MODULO THE FIELD PERIOD, which is what keeps the float3 pin
 *  exact rather than merely distant. A float32 has no unit ulp above 2^24, so an unreduced
 *  index quantizes there: the field stops advancing smoothly and then stalls on whichever
 *  axis saturated first, with both paths reading the same wrong value so nothing disagrees
 *  and nothing complains. The period sits just under 2^24, so a reduced index always
 *  round-trips.
 *
 *  The cost is that the field repeats, at about 3.7 million Gly -- four thousand times
 *  further out than the warp and region textures repeat. See THE FIELD PERIOD in
 *  UniverseDensityCore.ush for the two divisors the period carries, and for why this is not
 *  the situation ScaleStructure's coprime scales exist to avoid.
 *
 *  THE REDUCTION IS THE CALLER'S, because the period is not known here. A caller that has
 *  one uses FromCellPositionWrapped, which reduces in double BEFORE narrowing to int32;
 *  splitting first and wrapping the int afterwards holds only to 2.1e9 cells, past which
 *  the cast is undefined and the offset collapses toward zero.
 *
 *  An unreduced offset arriving here is not an error -- the core reduces again with its own
 *  period -- it just wastes the exactness the reduction exists to preserve. */
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

	/** WHICH REPEAT OF THE FIELD Cell was reduced out of: floor(cellPos / period), per
	 *  axis. Zero in the period containing the origin, -1 one period negative, and so on.
	 *
	 *  NOT MARSHALLED. Pack() ignores it and no pin carries it, so the shader cannot tell
	 *  one period from another -- which is the definition of the field repeating. It is
	 *  produced here because FromCellPositionWrapped computes it anyway to do the reduction,
	 *  and thrown away it would have to be recomputed by anything that wanted it.
	 *
	 *  WHAT IT IS FOR is reading the wrap. Cross the origin on an axis and Cell jumps from 0
	 *  to period-1 with nothing saying why; this is the term that says why, and it moves by
	 *  exactly one when it happens. */
	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	FIntVector Period = FIntVector::ZeroValue;

	/** Splits a small-cell position into the exact cell/frac pair the core wants.
	 *
	 *  UNGUARDED ABOVE 2.1e9 CELLS -- the cast is undefined there. Prefer
	 *  FromCellPositionWrapped anywhere a period is available; this is the fallback it
	 *  degrades to when one is not.
	 *
	 *  FLOOR, NOT TRUNCATION, matching SampleAtPosition: truncation folds the cell boundary
	 *  at zero and mirrors the field through the origin on every axis. */
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

	/** The split and the wrap in one, with the reduction taken IN DOUBLE before the cast to
	 *  int32 -- the only ordering that survives an arbitrary traversal.
	 *
	 *  FromCellPosition alone casts a floored double straight to int32, so a cell position
	 *  past 2.1e9 is undefined behaviour. It wraps in practice, any reduction applied after
	 *  it operates on a number that means nothing, and the field offset collapses toward
	 *  zero -- reachable in seconds at a high compression factor, and looking exactly like
	 *  the field snapping back to the origin.
	 *
	 *  FLOOR FIRST, REDUCE SECOND, and that order matters too. Flooring is exact in double
	 *  at any magnitude, so Frac comes out with the full precision the position had;
	 *  reducing first would take the cancellation on the fractional part as well and lose
	 *  it for nothing. Only the integer part is reduced, and only then narrowed.
	 *
	 *  What is left as a ceiling is double precision in the position itself, which decays
	 *  gracefully rather than wrapping: Frac keeps ulp(cellPos) of resolution, and that
	 *  stays far below one cell until well past any bound this universe is likely to have.
	 *
	 *  InPeriod <= 0 means 'no wrap' and falls through to the plain split, overflow and
	 *  all -- a caller that has no period has no business pretending to one. */
	static FUniverseFieldOffset FromCellPositionWrapped(const FVector& InCellPos, int32 InPeriod)
	{
		if (InPeriod <= 0)
		{
			return FromCellPosition(InCellPos);
		}

		const double Period = static_cast<double>(InPeriod);

		const FVector Floored(
			FMath::Floor(InCellPos.X),
			FMath::Floor(InCellPos.Y),
			FMath::Floor(InCellPos.Z));

		// Floored modulo, matching WrapCell in the core and Wrap in UniverseCellWrap: the
		// result is in [0, Period) for negative inputs too, so a cell left of the origin
		// hashes as the counterpart it should rather than as its own negative mirror.
		//
		// THE QUOTIENT IS KEPT. It is the repeat index, it is free here, and recovering it
		// afterwards would mean redoing this division against a reduced cell index, which
		// does not carry the magnitude the quotient was taken from.
		auto Split = [Period](double InValue, int32& OutRepeat) -> int32
			{
				const double Q = FMath::Floor(InValue / Period);
				const double R = InValue - Q * Period;
				OutRepeat = static_cast<int32>(FMath::Clamp(Q, -2147483648.0, 2147483647.0));
				return static_cast<int32>(FMath::Clamp(R, 0.0, Period - 1.0));
			};

		FUniverseFieldOffset Out;
		int32 Rx = 0, Ry = 0, Rz = 0;
		Out.Cell = FIntVector(
			Split(Floored.X, Rx), Split(Floored.Y, Ry), Split(Floored.Z, Rz));
		Out.Period = FIntVector(Rx, Ry, Rz);
		Out.Frac = InCellPos - Floored;
		return Out;
	}
};


// =============================================================================
// THE PACKED ARGUMENT LIST
// =============================================================================

/** The sixteen arguments of MakeUniverseDensityParams, in order, packed.
 *
 *  ONE MARSHAL SITE, and that is why it exists. Everything downstream reads this struct:
 *  the compute fill copies it member for member, the material push sets it member for
 *  member, and a CPU derivation would hand it straight over. Adding a parameter to the
 *  derivation breaks Pack() at compile time, and Pack() is the only place the packing
 *  decisions live.
 *
 *  THE ALTERNATIVE IS THE GALAXY LAYER'S, which packs its equivalent in three independent
 *  places -- CPU derivation, compute parameter fill, and the material's Custom node body.
 *  The third fails at SHADER compile rather than at build, so a parameter missed there
 *  surfaces as a red material with no build error and nothing pointing at the cause.
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
 *  regionally-varying quantities share one range type rather than being twenty-four loose
 *  scalars with a .z slot each to remember.
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
 *  That simplifies the compute path as much as it does this struct: entity generation needs
 *  one parameter set for the layer rather than a per-cell archetype lookup, and the constant
 *  buffer is the same sixteen values for every dispatch.
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
	 *  count off the same weights, so a second width would cost a second set of exponentials
	 *  over the whole neighbourhood -- 27 more exp2 and 54 more madds -- to shift a threshold
	 *  the wall term reaches through its falloff exponent for two instructions.
	 *
	 *  A FALLOFF RATE, NOT A BAND EDGE: the distance at which a competing node's weight has
	 *  halved. There is no cutoff anywhere -- the tail is exponential rather than absent,
	 *  which is what the wall falloff exponent controls.
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
	 *  ZERO IS NOT "NO FADE" -- it is the MOST fade, and 1 and above is the disable. The
	 *  span is 1 - start, so at 0 the attenuation runs across the whole volume and the field
	 *  is a soft ball centred on the camera rather than a window with a horizon.
	 *
	 *  A LOOK DECISION STANDING IN FOR A BOUNDS CONTROL at the default. Sitting at 0 thins
	 *  everything past the near field, which is the first thing to suspect when the march
	 *  will not saturate; raising it makes the distance dense again. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bounds", meta = (ClampMin = "0.0"))
	float BoundsFadeStart = 0.0f;

	/** The largest |dTex/dUV| each filtered WARP volume reaches. A property of the ASSET,
	 *  measured rather than chosen, and NOT a derivation input -- it only predicts the fold
	 *  ceiling.
	 *
	 *  ONE PER OCTAVE, because the octaves read separate assets. A single shared pin would
	 *  make a bake that sharpens one octave move the other's predicted term as well, so the
	 *  two halves of the ceiling sum have to carry their own constants.
	 *
	 *  THE VARIANCE VOLUMES NEED NO EQUIVALENT. Their channels feed VarianceT as lerp
	 *  factors, never as displacements, so their gradient cannot fold anything.
	 *
	 *  MEASUREMENT CONDITIONS: 14.09 holds for VT_PerlinWorley_Balanced at 256^3 with a
	 *  centring decode. A value is only valid for the asset, resolution and decode it was
	 *  taken on -- gradient is per UV, so halving a volume's resolution roughly halves the
	 *  largest |dTex/dUV| the same content reaches.
	 *
	 *  TODO: NEITHER PIN MATCHES ITS ASSET. The octaves read purpose-baked signed vector
	 *  volumes at 128^3 with a non-centring decode -- three conditions different from the
	 *  ones above. Until each is re-measured on its own bake, PredictedFoldShear and every
	 *  fold-ceiling line Validate() emits are arithmetic on constants that do not describe
	 *  the assets, in an unknown direction. Read a ceiling warning as unproven rather than
	 *  as a reading. Trigger to close: a re-measure, or a gradient-normalized bake of both
	 *  volumes, which sets each pin to 1 and drops the term out entirely. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Asset", meta = (ClampMin = "0.0"))
	float WarpTexGradientLarge = 14.09f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Asset", meta = (ClampMin = "0.0"))
	float WarpTexGradientSmall = 14.09f;

	FUniverseDensityParams()
	{
		// THE SMALL OCTAVE IS NOT A COPY OF THE LARGE ONE, so its defaults are set here
		// rather than left to FUniverseWarpOctaveParams's own. Lower amplitude and a scale
		// seven times higher: fine grain against coarse bend.
		//
		// The spread is wide and biased hard to the low end because this octave is what the
		// appearance fetch varies most visibly -- the difference between a province of clean
		// sheets and one of broken filament.
		//
		// THIS MAX IS ESSENTIALLY THE WHOLE FOLD CEILING. The coarse lattice multiplies this
		// octave by the lattice ratio and divides the large one by it, so this term carries
		// about 94% of predicted shear at these defaults. Raising it is the most dangerous
		// edit in the warp block, and it is why the curl volume belongs on this octave -- see
		// the asset paths in FUniverseMaterialParams.
		WarpSmall.Amount = FUniverseVarianceRange(0.05f, 0.3f, 2.0f);

		// 3079/4096, AND PRIME. A round 0.75 quantizes to 3072 = 2^10 * 3, the one composite
		// numerator the set can produce; it happens to stay coprime with 19, 41 and 409, but
		// only by luck, and one badly-chosen neighbouring scale would re-align against it. A
		// prime numerator is coprime with any other by construction. The difference between
		// the two values is 0.23%, well below what the eye reads as a frequency change.
		WarpSmall.Scale = 0.751730f;
		WarpSmall.LatticeFollow = 0.0f; // ignored; the small octave is pinned at full follow
	}

#pragma region Derivation
	/** THE ONLY PLACE THE PACKING DECISIONS LIVE. Every downstream consumer -- the material
	 *  push, the compute parameter fill, a CPU derivation -- reads this rather than packing
	 *  again.
	 *
	 *  Pure packing plus the four .w passengers. EACH PASSENGER BELONGS TO THE SAME FIELD AS
	 *  ITS HOST: skew multiplies the spread's own interpolant, each warp scale multiplies its
	 *  own octave's amount in the shear product, and the lattice-follow steers the octave the
	 *  weights vector gains. A pin carrying both is one setting, not two unrelated ones
	 *  sharing a wire -- the test any further claim on a free slot has to pass.
	 *
	 *  THE SEED IS AN ARGUMENT, not a member: FUniverseParams::Seed is the authoritative one
	 *  and a second copy here would be free to disagree with it. The offset is an argument
	 *  because it is per-frame runtime state. */
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

	/** THERE IS NO CPU DERIVATION, and adding one would not give this layer a CPU placement
	 *  path. The galaxy layer has one -- it compiles its core into C++ behind
	 *  GalaxyHLSLShim.h and returns a constructed params struct -- and the same construction
	 *  here would be three lines against this struct.
	 *
	 *  IT WOULD SAMPLE A DIFFERENT FIELD. The shim stubs every Texture3D fetch to a neutral
	 *  0.5, so a C++ evaluation reduces to the unwarped analytic web. This field is more
	 *  texture-dependent than the galaxy's, not less -- two region fetches and three warp
	 *  fetches -- and turning them all neutral changes the GEOMETRY, not the shading. See
	 *  the same warning in GalaxyHLSLShim.h: the shim path must never place entities.
	 *
	 *  So placement lands on the compute harness, which samples the same volumes the
	 *  material does, and FillShaderParameters is how it gets its inputs. A CPU probe for
	 *  DEBUGGING is a reasonable thing to want; expect a first compile against the shim to
	 *  surface missing operators, since only the decode and the texture neutrals have been
	 *  built on that side. */
#pragma endregion

#pragma region Validation
	 /** Predicted worst-case warp shear on the COARSE lattice, where the ceiling binds:
	  *
	  *      AmountLarge * ScaleLarge / ratio * GradientLarge
	  *    + AmountSmall * ScaleSmall * ratio * GradientSmall
	  *
	  *  At or below 1 the web bends. Above it the displacement folds, neighbouring samples
	  *  cross over, and the web TEARS -- only in the regions the blend has handed to the
	  *  coarse lattice, so region-dependent tearing reads as a lot of other things before it
	  *  reads as a warp amplitude problem.
	  *
	  *  Each amount is that octave's MAX, since that is what a region can resolve to, and each
	  *  octave carries its OWN gradient because the two read different assets.
	  *
	  *  A DIVERGENCE-FREE OCTAVE FOLDS ONLY AT SECOND ORDER, so a curl volume leaves real
	  *  headroom above 1 rather than putting the bound on the tear point -- a prediction
	  *  slightly over 1 is not an emergency while both octaves read curl. Tune by fixing the
	  *  scales, raising one amplitude until the web visibly tears, then backing off about
	  *  thirty percent; this number is the check, not the target. */
	float PredictedFoldShear() const
	{
		const float Ratio = FMath::Max(FMath::RoundToFloat(
			FMath::Max(Lattice.CellSizeLarge, 0.0f) / FMath::Max(Lattice.CellSizeSmall, 1e-6f)), 1.0f);

		// EACH TERM CARRIES ITS OWN ASSET'S GRADIENT. Factoring one gradient out of the sum
		// is valid only if both octaves read the same volume; they read separate ones.
		const float LargeTerm =
			WarpLarge.Amount.Max * WarpLarge.Scale / Ratio * WarpTexGradientLarge;
		const float SmallTerm =
			WarpSmall.Amount.Max * WarpSmall.Scale * Ratio * WarpTexGradientSmall;

		return LargeTerm + SmallTerm;
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
 *  THE NEAR FIELD IS WHY. A camera-distance step goes to ZERO at the camera, and the camera
 *  is normally inside this volume since the proxy is centred on it -- so successive samples
 *  land inside a single cell, each paying a full fifty-four candidate neighbourhood walk to
 *  re-derive a field that has not changed between them. A step derived from the chord cannot
 *  collapse that way, because it does not know where the camera is. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseMaterialParams
{
	GENERATED_BODY()

	/** A CEILING ON THE STEP COUNT, NOT A FLOOR. Growth only ever lengthens steps, so every
	 *  step is at least span / StepBudget and the actual count is StepBudget * ln(1+g)/g --
	 *  equal to the budget only at growth 0, 0.69 of it at growth 1, 0.46 at growth 3. See
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

	// NO STEP CEILING AND NO ITERATION BOUND HERE, deliberately. The step floor is
	// P.MinFeatureStep -- half the narrowest wall the parameters can produce, derived in the
	// core from FeatureWidth and CellSizeSmall together -- and an authored absolute cannot
	// compete with a floor that knows the cell size. The loop bound is derived in the Custom
	// node from StepBudget, since the budget already bounds the count and an authored bound
	// could therefore only truncate: a truncated march looks identical to one that finished,
	// minus the far half of the volume.

	/** Multiplier on density before the march's exp(-density * scale * h) -- how opaque the
	 *  result is, and nothing else. The field's own densities set the RATIO between walls,
	 *  filaments and the floor.
	 *
	 *  KEEP IT AT 1 UNLESS THERE IS A REASON. It is tempting as a brightness control, being
	 *  the last multiplier before the accumulation, but driving the look from here scales the
	 *  apparent field several times over its authored range -- and every judgement made
	 *  against the picture is then made against a field that does not exist. The exponents
	 *  stop reading as falloff curves because the values they shape are off their intended
	 *  scale, and the void floor stops looking like a floor.
	 *
	 *  Output brightness belongs after the march, on the material's own intensity multiplier,
	 *  where it cannot be mistaken for a property of the field. IT NEVER REACHES PLACEMENT
	 *  either way: acceptance normalises against each cell's own probed envelope, so this
	 *  changes what the field looks like and not where an entity lands. */
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
	// function -- and there is no case for disabling it in a release build. A debug
	// toggle would be lerp(1, jitter, k) in the node with a new pin, not a value pushed
	// from here.

	// NO OWNERSHIP TOGGLE. FUniverseDensityParams is the field, and the push is
	// unconditional. The compute path reads this struct, so a material instance tuned
	// independently of it would put the render and placement on two different universes --
	// silently, since both look plausible. Look development belongs in this struct's
	// values, not in a mode that lets the asset win.
	//
	// A MID SILENTLY IGNORES A NAME THE MATERIAL DOES NOT HAVE, so a mismatched pin name
	// surfaces as a parameter sitting at its authored value with no warning anywhere. The
	// push is where that would be caught; skipping it means it never is.

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

	/** THE FOUR FIELD VOLUMES, one purpose-baked per fetch site: the variance family and the
	 *  warp family want opposite things from a texture, and a shared asset can only be a
	 *  compromise between them. The mapping is in the fetch table at the top of
	 *  UniverseDensityCore.ush.
	 *
	 *  ALL FOUR MUST REACH THE COMPUTE PATH. Placement and render sample one field or they
	 *  are not one field, which is four correspondences to keep. The actor resolves all four
	 *  in LoadRuntimeAssets and hands the same pointers to the material instance and to the
	 *  dispatch, so the correspondence is structural rather than four things to remember.
	 *
	 *  VARIANCE VOLUMES -- the two region-axis fetches. Periodic, so repeat addressing
	 *  closes the 4096-cell wrap. UNORM, per-channel normalized to 0.5 with comparable
	 *  spread, since a channel feeds VarianceT as a raw [0,1] lerp factor and a channel with
	 *  a different histogram makes a Range's min and max land somewhere other than where
	 *  they were authored. A distinct archetype per channel.
	 *
	 *  THE TWO MUST BE DIFFERENT MULTINOISE VARIANTS, not the same asset twice. A is the
	 *  structure field and B is the appearance field, and separating them is what lets a
	 *  province's shape and its painting change in different places. Two fetches into one
	 *  asset share all four generators and put the boundaries back together at a scale
	 *  offset. LoadRuntimeAssets warns if they resolve equal.
	 *
	 *  WARP VOLUMES -- the two domain-warp octaves. Periodic for the same reason. SIGNED,
	 *  values in [-1,1], because the shader applies them as displacements with NO centring
	 *  step at all -- see WeightedVector3, where the failure mode for an unsigned asset is
	 *  written out. And a coherent VECTOR field across xyz rather than three unrelated
	 *  scalar archetypes, or the warp is axis-coupled and the swizzle test shows it.
	 *
	 *  CURL FOR PREFERENCE. Divergence-free warp folds only at second order, worth a factor
	 *  of several in usable amplitude, and it hands void-size variation back to VoidSizeSpread
	 *  alone. Gradient fields work and are not divergence-free.
	 *
	 *  GRADIENT-NORMALIZED AT BAKE IF POSSIBLE, for both warp volumes: that sets the
	 *  matching WarpTexGradient pin to 1 and reduces the fold ceiling to amounts and scales
	 *  alone, which is the only form of it that can be held in the head while tuning.
	 *
	 *  THEY LIVE IN ANOTHER PLUGIN'S CONTENT, /UniverseNoisePack rather than
	 *  /UltraLargeScale, and that carries two conditions these strings cannot enforce.
	 *  UniverseNoisePack must be ENABLED, or the mount point does not exist and LoadObject
	 *  returns null -- which surfaces as a plausible-looking field rather than an error, and
	 *  fails for all four at once. And because these are STRINGS rather than hard
	 *  TObjectPtrs, the cooker has no edge to follow: each asset needs to be reachable from
	 *  a cooked reference or listed in the project's additional asset directories, or a
	 *  packaged build loses structure the editor keeps.
	 *
	 *  FULL Package.Object FORM, not the package path alone. LoadObject can often infer the
	 *  object name from the package, and "often" is the problem: inference is least reliable
	 *  in a packaged build, which is where a failed load is hardest to see. The mesh and
	 *  material loads elsewhere in this actor spell the object out for the same reason.
	 *
	 *  THE WARP AMOUNTS AND BOTH WarpTexGradient PINS BELONG TO WHATEVER THESE NAME.
	 *  Changing an asset without re-tuning its octave's pair is a silent retune of the
	 *  field's geometry.
	 *
	 *  CURL GOES ON THE SMALL OCTAVE, THE PLAIN VECTOR FIELD ON THE LARGE ONE. Only one of
	 *  the two warp assets is divergence-free, so this is a real choice, and the two
	 *  arguments point opposite ways:
	 *
	 *    - the FOLD BUDGET wants curl on the SMALL octave. On the coarse lattice the small
	 *      term carries a factor of ratio while the large term is divided by it, so the small
	 *      octave is essentially the entire ceiling -- around 94% of predicted shear at the
	 *      defaults. Divergence-free warp folds only at second order, so curl buys headroom
	 *      exactly where the risk is concentrated.
	 *    - VOID SIZE wants curl on the LARGE octave. A divergence-free warp cannot vary void
	 *      size and a generic one can, so a generic large octave contributes size variation
	 *      that reads as VoidSizeSpread's and is not.
	 *
	 *  THE FOLD BUDGET WINS. 94% of the ceiling on one octave makes that octave the only one
	 *  worth protecting; void-size contamination on the large octave is a second-order
	 *  authoring annoyance beside it. Revisit only if void size needs tuning cleanly against
	 *  VoidSizeSpread -- not because of tearing.
	 *
	 *  BOTH WARP ASSETS ARE S4 and both variance assets are S8. The warps are what the eye
	 *  reads as structure, so they benefit most from the larger features a lower tiling
	 *  interval gives at the same resolution. The variance fetches run two orders of
	 *  magnitude coarser -- 19 and 41 against 409 and 3079 per 4096 cells -- so their
	 *  features are enormous in cell terms either way and the higher interval costs nothing
	 *  visible.
	 *
	 *  THE RESOLUTION SPLIT IS THE /256 AND /128 IN THESE PATHS, and it follows the same
	 *  logic one level down. The variance volumes take 256^3 and the warp volumes 128^3,
	 *  which is the right way round for what each fetch does with its result:
	 *
	 *    - a VARIANCE fetch resolves four region axes from four channels and those axes are
	 *      read as VALUES, straight into VarianceT as lerp factors. Quantization there lands
	 *      directly on the parameter -- a coarse volume makes provinces step between
	 *      discrete-looking settings rather than drift, and it is a fetch per sample either
	 *      way, so the resolution buys smoothness in the thing most visible per byte.
	 *    - a WARP fetch is read as a DISPLACEMENT and then filtered by everything downstream:
	 *      the trilinear tap, the amplitude, the lattice blend, and the density falloff the
	 *      displaced position lands in. Detail below the feature width cannot survive that
	 *      chain, so 128^3 spends nothing on structure the field cannot express.
	 *
	 *  IT ALSO COSTS LESS THAN THE OTHER WAY ROUND. A 256^3 volume is eight times the texels
	 *  of a 128^3 one, so at RGBA8 the two variance volumes are 64 MB each against 8 MB for
	 *  each warp volume -- around 144 MB resident for the set, and around 160 MB if the warp
	 *  volumes need a 16-bit float format to carry their sign. The high resolution on the
	 *  warp pair instead would cost the same and buy less.
	 *
	 *  A RESOLUTION CHANGE INVALIDATES THAT OCTAVE'S WarpTexGradient. Gradient is measured
	 *  per UV, so halving a volume's resolution roughly halves the largest |dTex/dUV| the
	 *  same content reaches. See the measurement conditions on those pins. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VarianceVolumeA = "/UniverseNoisePack/256/VT_MultiNoise_1_S8.VT_MultiNoise_1_S8";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VarianceVolumeB = "/UniverseNoisePack/256/VT_MultiNoise_2_S8.VT_MultiNoise_2_S8";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString WarpVolumeLarge = "/UniverseNoisePack/128/VT_PerlinVector_S4.VT_PerlinVector_S4";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString WarpVolumeSmall = "/UniverseNoisePack/128/VT_PerlinCurl_S4.VT_PerlinCurl_S4";

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


/** Universe-layer generation parameters: the cosmic-web density field, the march that draws
 *  it, per-tier streaming configs, and scale derivation. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

#pragma region Density Field

	/** The cosmic-web field: what UniverseDensityCore.ush evaluates, what the ray march
	 *  draws, and what entity generation places against. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseDensityParams DensityParams;

	/** How the field is MARCHED, as opposed to what it contains. Kept apart from
	 *  DensityParams because none of it reaches MakeUniverseDensityParams: these are the
	 *  march's own arguments plus the volume assets, and only the render consumes them.
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

		// GENERATION SUBDIVISION, sized against the FIELD'S cell rather than the tier's. At
		// zero a transition batch is nine thread groups, which leaves the GPU idle.
		//
		// A field cell is ProxyExtent * CellSizeSmall = 0.9 sector extents at these values,
		// and a tier cell at depth d is 4/2^d of one. The Large tier's cell therefore spans
		// about 2.2 field cells undivided, too coarse for its probes to describe; two levels
		// bring it to 0.56. Mid and Small sit at or below that undivided, so their one level
		// buys OCCUPANCY rather than resolution -- nine groups is not a dispatch.
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
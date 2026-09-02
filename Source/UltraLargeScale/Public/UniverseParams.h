// UniverseParams.h
// Authored parameters for the universe layer: the cosmic-web field, the march that draws it,
// the field offset, and per-tier streaming config.
//
// NOTHING IS DERIVED HERE. The lattice ratio, the four scale quantizations, the offset
// re-split, lambda and the region-fetch enables all resolve inside MakeUniverseDensityParams,
// which the material and the compute dispatch both call -- which is what lets this struct and
// the material's pin set be the same list of raw values.
//
// NO CLAMPING beyond UPROPERTY meta on division guards. Validate() reports the constraints the
// details panel cannot show and reports rather than corrects.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "UniverseParams.generated.h"

class UTexture;

/** The field's four volume textures, resolved. One bundle rather than four loose pointers,
 *  because this set crosses the actor, the material, the data generator and the dispatch --
 *  four parameters at each boundary is four chances to pass three of them.
 *
 *  ALL FOUR OR NONE, enforced by IsComplete at every consumer. With one missing the field does
 *  not degrade, it becomes a different field: a missing variance volume takes every regional
 *  axis to its midpoint, a missing warp volume straightens the bisectors. Both are changes in
 *  GEOMETRY, so placement against them is unrelated to what is drawn.
 *
 *  RAW POINTERS: transport only. The actor owns the references that keep these alive. */
struct FUniverseFieldTextures
{
	/** Region axes, coarse -- the structure field. UNORM multinoise. */
	UTexture* VarianceA = nullptr;

	/** Region axes, finer -- the appearance field. Must be a DIFFERENT multinoise variant from
	 *  VarianceA; see the asset paths in FUniverseMaterialParams. */
	UTexture* VarianceB = nullptr;

	/** Large warp octave, signed vector field. Read TWICE per sample, once per lattice, and
	 *  those two fetches must stay on one asset or the tiers stop bending like one field. */
	UTexture* WarpLarge = nullptr;

	/** Small warp octave, signed vector field. */
	UTexture* WarpSmall = nullptr;

	bool IsComplete() const
	{
		return VarianceA != nullptr && VarianceB != nullptr
			&& WarpLarge != nullptr && WarpSmall != nullptr;
	}

	/** Which ones are missing, for an actionable log line. Empty when the set is complete. */
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

/** Seed channels for the universe layer. ADD, NEVER REUSE -- channels are free, and sharing
 *  one reintroduces the aliasing ProcSeed exists to prevent. */
namespace UniverseSeed
{

	/** The GPU placement key, one channel for all three tiers -- the tier index enters through
	 *  MixSeed, so a fourth tier needs no new constant. */
	inline constexpr uint32 Placement = ProcSeed::ChannelId("Universe.Placement");
}

USTRUCT(BlueprintType)
/** One regionally-varying quantity: two authored extremes and the curve deciding where between
 *  them a sample falls. Packs as (Min, Max, Bias, passenger), the convention the core reads.
 *
 *  THERE IS NO BASE VALUE -- the region channel picks a point between Min and Max and that IS
 *  the value. Min == Max reproduces a constant exactly, which is also how the core decides
 *  whether to pay for the fetch; see IsDegenerate. MIN > MAX IS VALID and runs the
 *  interpolation backwards. Nothing is clamped. */
	struct ULTRALARGESCALE_API FUniverseVarianceRange
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Min = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	float Max = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Range")
	/** t = pow(channel, Bias) before the lerp. 1 passes the channel through, > 1 pushes the
	 *  field toward MIN so the max end is rare, < 1 toward MAX. At or below zero is legal and
	 *  almost never meant: PowSafe guards the zero base and it reads as "always at Max". */
	float Bias = 1.0f;

	FUniverseVarianceRange() = default;
	FUniverseVarianceRange(float InMin, float InMax, float InBias = 1.0f)
		: Min(InMin), Max(InMax), Bias(InBias) {}

	/** InW is the .w passenger -- unclaimed on most ranges, a partner setting on three. */
	FVector4f Pack(float InW = 0.0f) const
	{
		return FVector4f(Min, Max, Bias, InW);
	}

	/** True when the range carries no region signal, which is what lets the core skip a fetch.
	 *  THE SKIP IS EXACT: a fetch is skipped only when EVERY range feeding it is degenerate, in
	 *  which case the lerp returns that constant whatever t is. ONLY MIN AND MAX ARE TESTED, so
	 *  setting a .w passenger never makes a constant range look variable. */
	bool IsDegenerate() const { return Min == Max; }
};

USTRUCT(BlueprintType)
/** THE TWO LATTICES. The small one is the reference for everything else: warp amplitudes and
 *  scales, both region scales, feature width and void spread are all per SMALL cell, and the
 *  large lattice inherits each scaled by the ratio. A region that switches lattice switches
 *  scale in every property at once rather than growing voids with unchanged walls. */
	struct ULTRALARGESCALE_API FUniverseLatticeParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice", meta = (ClampMin = "0.000001"))
	/** Small cell size in normalized units; larger gives a coarser web. */
	float CellSizeSmall = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice", meta = (ClampMin = "0.0"))
	/** QUANTIZED BY THE DERIVATION to a whole multiple of CellSizeSmall. The field offset is an
	 *  exact integer count of small cells re-split by integer division, exact at any magnitude
	 *  where a float multiply by 1/ratio is not -- at 1e11 cells one ulp is hundreds of cells
	 *  and the error changes as the offset crosses a cell plane, so an arbitrary ratio would
	 *  shift the coarse lattice bodily as the player moves.
	 *
	 *  KEEP THE RATIO CLEAR OF A HALF: the two sides round halves differently, so x.5 can
	 *  quantize to different coarse cell sizes in render and placement. 0.9 against 0.3 gives 3
	 *  exactly; 1.4 against 0.4 gives 3.5 and sits on the boundary.
	 *
	 *  At or below CellSizeSmall this collapses to one lattice and skips the second walk. */
	float CellSizeLarge = 0.9f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lattice")
	/** Exponent shaping where between the two lattices a region sits. NOT a range: the extremes
	 *  it interpolates are the lattices themselves.
	 *
	 *  THE BLEND CANNOT BE PINNED OR CLAMPED -- every value in [0,1] is reachable somewhere, so
	 *  "30% coarse everywhere" is unauthorable. The useful band is 0.4 to 0.8, where both
	 *  lattices compete; mean density peaks near 0.30 there against 0.06 at either end. */
	float CellSizeBias = 1.0f;
};

USTRUCT(BlueprintType)
/** WALLS: two nodes participating, the sheet between two voids. */
struct ULTRALARGESCALE_API FUniverseWallParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wall")
	/** THE MARCH ACCUMULATES DENSITY TIMES VOLUME FRACTION, not peak density, and a wall covers
	 *  an order of magnitude more volume than a filament. Authoring the two at similar numbers
	 *  is what makes the field read as foam. */
	FUniverseVarianceRange Density = FUniverseVarianceRange(0.0f, 1.0f, 6.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wall")
	/** Exponent on the wall term, saturate(N - 1). 1 is the bare saturate; up clears voids.
	 *
	 *  THIS IS THE VOID-FRACTION LEVER, not the densities. The weight has no compact support, so
	 *  the aggregate tail of the candidates not in contention lifts N off its floor everywhere,
	 *  and once the tail alone carries N past 2 the saturate pins the wall term at full. The
	 *  symptom is a wall control that appears dead while the filament one works.
	 *
	 *  The exponent leaves the on-wall value at exactly 1 and crushes the tail (0.4 to 0.064 at
	 *  p = 3), far faster than lowering densities. Below zero lights the void up rather than
	 *  clearing it; exactly 0 floods the field. */
	FUniverseVarianceRange Falloff = FUniverseVarianceRange(2.0f, 12.0f, 1.0f);
};

USTRUCT(BlueprintType)
/** FILAMENTS: three nodes participating, the edge where three sheets meet. */
struct ULTRALARGESCALE_API FUniverseFilamentParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Filament")
	/** Covers far less volume than a wall, so it carries a larger number for the same apparent
	 *  brightness. There is no filament falloff to match the wall's -- its threshold sits above
	 *  the aggregate tail rather than inside it. */
	FUniverseVarianceRange Density = FUniverseVarianceRange(0.5f, 1.5f, 0.5f);
};

USTRUCT(BlueprintType)
/** VOIDS: one node dominating, the interior of a cell. The ambient floor that keeps them from
 *  being empty and the per-node offset that keeps them from being one size.
 *
 *  THERE IS NO SIZE SKEW PIN. The offset is the hash channel's FOURTH POWER times the spread,
 *  fixed in the core. An authored exponent would buy tail rather than spread: dispersion of
 *  void volume peaks near 2 and moves by under a tenth across 1 to 8, while the largest void
 *  grows monotonically. SizeSpread is the lever for how varied voids are. */
	struct ULTRALARGESCALE_API FUniverseVoidParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	/** Ambient density everywhere. BELOW ZERO IS NOT A DARKER VOID -- the march's exp(-density)
	 *  turns negative density into unbounded gain. Both ends matter: a range spanning zero puts
	 *  the negative end somewhere a noise channel decides. */
	FUniverseVarianceRange Floor = FUniverseVarianceRange(0.0f, 0.5f, 8.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Void")
	/** Per-node power-diagram offset, so voids differ in size rather than tiling at one scale.
	 *  SQUARED CELL UNITS -- subtracted from the squared distance, which is what keeps 27 square
	 *  roots out of the walk.
	 *
	 *  THIS IS WHAT ERODES THE 27-CELL SEARCH. The window holds while the offset stays well
	 *  under 1.0 minus the dominant candidate's own power distance, and the bound degrades with
	 *  WIDE features too, so the safe value depends on FeatureWidth rather than on this alone.
	 *  Past it a node outside the neighbourhood wins and the wrong cell draws it. */
	FUniverseVarianceRange SizeSpread = FUniverseVarianceRange(0.0f, 0.5f, 2.0f);

};

USTRUCT(BlueprintType)
/** ONE OCTAVE OF THE DOMAIN WARP. Two exist and they are not symmetric: only the large octave
 *  uses LatticeFollow, and their usable amplitudes differ by two orders at the default
 *  scales. See PredictedFoldShear. */
	struct ULTRALARGESCALE_API FUniverseWarpOctaveParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	/** Peak displacement in SMALL cells, DENOMINATED IN THE ASSET'S VALUE SCALE. The core
	 *  multiplies the decoded channels by this, so a volume whose channels swing half as far
	 *  needs twice the amount for the same bend. SWAPPING AN ASSET SILENTLY RETUNES THE
	 *  GEOMETRY: re-tune this and the octave's WarpTexGradient together, and judge by
	 *  PredictedFoldShear rather than by the picture. */
	FUniverseVarianceRange Amount = FUniverseVarianceRange(0.333f, 1.333f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp", meta = (ClampMin = "0.0"))
	/** Texture repeats per small cell, riding Amount's .w. THE PAIR IS ONE SETTING -- shear is
	 *  amount times scale times the asset's gradient, so neither half means anything alone.
	 *
	 *  QUANTIZED to a multiple of 1/4096, which is correctness: the warp UV wraps every 4096
	 *  cells by masking the cell index, and the two sides are the same texel only if
	 *  4096 * scale is whole. FIXED ACROSS THE DRAW, never ranged -- a region-varying scale puts
	 *  a different frequency either side of a province boundary, which the wrap cannot close.
	 *
	 *  THE NUMERATOR MUST BE COPRIME WITH THE OTHER THREE. 409 is prime; a round 0.1 quantizes
	 *  to 410, sharing 41 with ScaleAppearance and re-aligning every ninety-nine cells, visible
	 *  as a grid. The quantum is 1/4096, so anything in [0.09984, 0.10008] is this same number:
	 *  move by at least one quantum and check Validate()'s coprimality report. */
	float Scale = 0.099854f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp")
	/** Per-axis gain on the decoded displacement, elementwise -- a warp needs all three axes. 0
	 *  flattens that axis, negative inverts, magnitude above 1 gains. Unclamped. */
	FVector Weights = FVector(1.0, 1.0, 1.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Warp", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	/** LARGE OCTAVE ONLY, riding the weights vector's .w. How much of the cell size ratio this
	 *  octave's amplitude follows on the coarse lattice: 0 gives the same physical displacement
	 *  on both tiers, 1 the same displacement in each tier's own cells. The small octave ignores
	 *  it and is pinned at full follow. */
	float LatticeFollow = 1.0f;
};

USTRUCT(BlueprintType)
/** THE TWO REGION FETCHES, which are this layer's archetype system. Rather than rolling a
 *  variant per instance the way the galaxy layer does, one parameter set is read at two
 *  spatial frequencies -- so a province of wide soft walls and one of tight bright filaments
 *  are the same authored numbers sampled in different places.
 *
 *  NEITHER CAN BE RANGED, structurally: these DEFINE where a region is, so putting them on the
 *  variance path would make the definition self-referential. */
	struct ULTRALARGESCALE_API FUniverseRegionParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	/** Repeats per small cell for the STRUCTURE field, the coarser. Drives the lattice blend,
	 *  void size spread, feature width and the large warp octave -- what SHAPE the web has.
	 *
	 *  19/4096, PRIME. A round 0.005 quantizes to 20, sharing a factor of 10 with a large warp
	 *  octave at 410 and re-aligning every 409 cells. ODD IS NOT SUFFICIENT: 21 is 3x7 against a
	 *  small octave of 3072 and still repeats every 1365 cells. Only a prime numerator is
	 *  coprime with the rest by construction. */
	float ScaleStructure = 0.004639f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Region", meta = (ClampMin = "0.0"))
	/** Repeats per small cell for the APPEARANCE field. Drives wall density and falloff, filament
	 *  density, void floor and the small warp octave.
	 *
	 *  41/4096, prime. WATCH IT AGAINST THE WARP SCALES: a large octave at 410 shares 41 with
	 *  this and re-aligns every ninety-nine cells, the tightest repeat the set can produce. */
	float ScaleAppearance = 0.010010f;
};

/** THE FIELD PERIOD, MIRRORED FROM THE CORE. MakeUniverseDensityParams holds the authoritative
 *  derivation; this is the CPU copy, needed to reduce a cell index BEFORE it narrows to a
 *  float3 pin. A mirror because nothing compiles the core as C++, and pulling the whole field
 *  surface in to share four lines of integer arithmetic is the worse trade.
 *
 *  A DISAGREEMENT IS BOUNDED: the core reduces AGAIN with its own period, so a mismatch cannot
 *  seam the field -- it can only leave the index above 2^24 at the pin.
 *
 *  WHAT HAS TO STAY IN STEP: ExactMax = 1 << 24, WrapAlign = 1 << WARP_WRAP_SHIFT,
 *  DeriveRatio = floor(large / max(small, 1e-6) + 0.5) floored at 1, and DerivePeriod =
 *  block * max(ExactMax / block, 1) for block = WrapAlign * ratio. The one difference is
 *  arithmetic type -- the core rounds in float, this in double -- which can disagree only on a
 *  half boundary, which CellSizeLarge already warns to avoid. */
namespace UniverseCellWrap
{

	/** WrapAlign is the texture wrap, which the period must preserve or the warp UV steps where
	 *  the density does not. */
	constexpr int32 ExactMax = 1 << 24;
	constexpr int32 WrapAlign = 1 << 12;

	/** Deliberately the same SHAPE as the core's rounding, not the same intent: drift changes the period. */
	inline int32 DeriveRatio(double InCellSizeSmall, double InCellSizeLarge)
	{
		const double Small = FMath::Max(InCellSizeSmall, 1e-6);
		const double Ratio = FMath::FloorToDouble(FMath::Max(InCellSizeLarge, 0.0) / Small + 0.5);
		return FMath::Max(static_cast<int32>(Ratio), 1);
	}

	/** Largest multiple of (WrapAlign * ratio) below ExactMax. Both divisors are required --
	 *  without ratio the lattices wrap in different places, without WrapAlign the warp UV steps
	 *  where the density does not. */
	inline int32 DerivePeriod(int32 InRatio)
	{
		const int32 Ratio = FMath::Max(InRatio, 1);
		const int32 Block = WrapAlign * Ratio;
		return Block * FMath::Max(ExactMax / Block, 1);
	}

	/** THE ONE CALL EVERY CONSUMER MAKES. Both sides reduce a cell index by this before it
	 *  crosses to the shader, and two different periods put placement and render on wraps that
	 *  disagree -- visible only a long way out, logging nothing. Takes the lattice group whole
	 *  so there is no call shape that mixes cell sizes from two sources. */
	inline int32 FieldCellPeriod(const FUniverseLatticeParams& InLattice)
	{
		return DerivePeriod(DeriveRatio(
			static_cast<double>(InLattice.CellSizeSmall),
			static_cast<double>(InLattice.CellSizeLarge)));
	}

	/** FOLDS A WRAPPED CELL INDEX BACK TO SIGNED, FOR DISPLAY ONLY.
	 *
	 *  Every consumer of an index wants the unsigned [0, period) form and must not be handed
	 *  this one. What that form costs is legibility: the reduction is a floored modulo, so one
	 *  cell left of the origin reports period-1 -- the torus closure rather than a rollover, but
	 *  nobody reads sixteen million as minus one while debugging something else. This folds to
	 *  [-period/2, period/2), moving the displayed discontinuity half a period from spawn. */
	inline int32 ToSigned(int32 InWrapped, int32 InPeriod)
	{
		if (InPeriod <= 0) return InWrapped;
		return (InWrapped >= InPeriod / 2) ? (InWrapped - InPeriod) : InWrapped;
	}
}

USTRUCT(BlueprintType)
/** The player's position in the field, split as the core wants it: an exact integer cell count
 *  plus a fraction in [0,1). Runtime state, which is why it is a Pack() argument.
 *
 *  THE CELL COUNT IS REDUCED MODULO THE FIELD PERIOD, which keeps the float3 pin exact rather
 *  than merely distant: a float32 has no unit ulp above 2^24, so an unreduced index quantizes
 *  there and the field stalls on whichever axis saturated first, with both paths reading the
 *  same wrong value so nothing complains.
 *
 *  THE REDUCTION IS THE CALLER'S, since the period is not known here -- use
 *  FromCellPositionWrapped. An unreduced offset is not an error, just a waste. */
	struct ULTRALARGESCALE_API FUniverseFieldOffset
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	/** Whole small cells. */
	FIntVector Cell = FIntVector::ZeroValue;

	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	/** Fractional remainder per axis, in [0,1). */
	FVector Frac = FVector::ZeroVector;

	UPROPERTY(BlueprintReadWrite, Category = "Field Offset")
	/** Which repeat of the field Cell was reduced out of: floor(cellPos / period) per axis.
	 *  NOT MARSHALLED -- no pin carries it, which is the definition of the field repeating. WHAT
	 *  IT IS FOR is reading the wrap: cross the origin and Cell jumps from 0 to period-1 with
	 *  nothing saying why; this moves by exactly one when it happens. */
	FIntVector Period = FIntVector::ZeroValue;

	/** Splits a small-cell position into the cell/frac pair the core wants.
	 *
	 *  SATURATING, NOT WRAPPING, past 2.1e9 cells: an unclamped cast of an out-of-range double is
	 *  undefined behaviour and yields INT32_MIN on x86, collapsing every far position onto one
	 *  cell. Saturation makes the far behaviour "stops advancing" instead -- a defined failure,
	 *  not a correct answer. Where a period is available, use the wrapped form.
	 *
	 *  FLOOR, NOT TRUNCATION, matching the core: truncation mirrors the field through the
	 *  origin on every axis. */
	static FUniverseFieldOffset FromCellPosition(const FVector& InCellPos)
	{
		FUniverseFieldOffset Out;
		const FVector Floored(
			FMath::Floor(InCellPos.X),
			FMath::Floor(InCellPos.Y),
			FMath::Floor(InCellPos.Z));

		auto Narrow = [](double InValue) -> int32
			{
				return static_cast<int32>(
					FMath::Clamp(InValue, -2147483648.0, 2147483647.0));
			};

		Out.Cell = FIntVector(Narrow(Floored.X), Narrow(Floored.Y), Narrow(Floored.Z));
		Out.Frac = InCellPos - Floored;
		return Out;
	}

	/** The split and the wrap in one, with the reduction taken IN DOUBLE before the cast -- the
	 *  only ordering that survives an arbitrary traversal.
	 *
	 *  FLOOR FIRST, REDUCE SECOND. Flooring is exact in double at any magnitude, so Frac keeps
	 *  the full precision the position had; reducing first would take the cancellation on the
	 *  fraction too. InPeriod <= 0 means no wrap and falls through to the plain split. */
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

/** The sixteen arguments of MakeUniverseDensityParams, in order, packed.
 *
 *  ONE MARSHAL SITE. The compute fill copies it member for member and the material push sets it
 *  member for member, so adding a parameter to the derivation breaks Pack() at compile time.
 *
 *  NOT UPLOADED AS A CONSTANT BUFFER -- that would need HLSL's cbuffer packing to agree with the
 *  C++ layout member for member, and one float3 straddling a 16-byte boundary shifts every
 *  field after it with no diagnostic. These are raw inputs; the derivation runs in the
 *  shader. */
struct FUniverseDensityArgs
{
	FVector4f CellSizeRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	float Seed = 0.0f;

	FVector3f OffsetCell = FVector3f::ZeroVector;
	FVector3f OffsetFrac = FVector3f::ZeroVector;

	FVector4f WallDensityRange = FVector4f(0.0f, 0.1f, 1.0f, 0.0f);
	FVector4f WallFalloffRange = FVector4f(2.0f, 12.0f, 1.0f, 0.0f);
	FVector4f FilamentDensityRange = FVector4f(0.25f, 1.25f, 1.0f, 0.0f);
	FVector4f FeatureWidthRange = FVector4f(0.05f, 0.4f, 2.0f, 0.0f);
	FVector4f VoidFloorRange = FVector4f(0.0f, 0.05f, 3.0f, 0.0f);

	FVector4f VoidSizeSpreadRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	FVector4f WarpAmountLargeRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	FVector4f WarpAmountSmallRange = FVector4f(0.0f, 0.0f, 1.0f, 0.0f);
	FVector4f WarpLargeWeights = FVector4f(1.0f, 1.0f, 1.0f, 0.0f);
	FVector4f WarpSmallWeights = FVector4f(1.0f, 1.0f, 1.0f, 0.0f);

	FVector4f RegionScales = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);

	float BoundsFadeStart = 1.0f;
};

USTRUCT(BlueprintType)
/** The cosmic-web density field: every input of MakeUniverseDensityParams, grouped by feature.
 *
 *  ONE SET FOR THE WHOLE LAYER, AND NO ARCHETYPES. The galaxy layer needs a per-seed roll
 *  because each galaxy is a distinct object. This field is omnipresent and carries its own
 *  variation -- the two region fetches ARE the archetype system, resolving per sample rather
 *  than per instance. Entity generation therefore needs one parameter set for the layer rather
 *  than a per-cell archetype lookup. */
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	/** THE ONE WIDTH, in small cells, serving walls and filaments alike -- which is why it sits
	 *  here rather than in either group. Both read the same participation count off the same
	 *  weights, so a second width would cost a second set of exponentials over the whole
	 *  neighbourhood to shift a threshold the wall falloff reaches for two instructions.
	 *
	 *  A FALLOFF RATE, NOT A BAND EDGE: the distance at which a competing node's weight has
	 *  halved. It also floors the march's step through MinFeatureStep, so widening it makes the
	 *  march cheaper as well as softer. */
	FUniverseVarianceRange FeatureWidth = FUniverseVarianceRange(0.1f, 0.4f, 2.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	/** The octave that BENDS the web. Large scale with small amplitude is grain rather than
	 *  curvature; small scale with large amplitude translates whole regions rigidly. */
	FUniverseWarpOctaveParams WarpLarge;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	/** The octave that BREAKS UP what the large one bent, and the one that binds the fold ceiling
	 *  by a wide margin. On the coarse lattice its shear term is multiplied by the cell ratio
	 *  while the large octave's is divided by it, so at ratio 4 and scale 0.5 it contributes
	 *  roughly 160 times the shear per unit amplitude. Expect its usable amplitude two orders
	 *  below the large octave's, and check PredictedFoldShear before raising it. */
	FUniverseWarpOctaveParams WarpSmall = FUniverseWarpOctaveParams();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	FUniverseRegionParams Region;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Bounds", meta = (ClampMin = "0.0"))
	/** Radius in normalized space where the march begins fading the field toward the proxy edge.
	 *  APPLIED BY THE MARCH, NOT BY THE FIELD -- a fade in placement would make where an entity
	 *  lands depend on where the player stood when the cell generated.
	 *
	 *  ZERO IS NOT "NO FADE", it is the MOST fade; 1 and above disables. The span is 1 - start,
	 *  so at 0 the attenuation runs across the whole volume and the field is a soft ball centred
	 *  on the camera -- the first thing to suspect when the march will not saturate. */
	float BoundsFadeStart = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Asset", meta = (ClampMin = "0.0"))
	/** The largest |dTex/dUV| each filtered WARP volume reaches. A property of the ASSET,
	 *  measured rather than chosen, and NOT a derivation input -- it only predicts the fold
	 *  ceiling. One per octave, since the octaves read separate assets. The variance volumes
	 *  need no equivalent: their channels feed VarianceT as lerp factors, never displacements.
	 *
	 *  TODO: NEITHER PIN MATCHES ITS ASSET. 14.09 was measured on VT_PerlinWorley_Balanced at
	 *  256^3 with a centring decode; the octaves now read signed vector volumes at 128^3 without
	 *  one, and gradient is per UV so resolution alone moves it. Until each is re-measured,
	 *  PredictedFoldShear and every fold-ceiling line Validate() emits are arithmetic on
	 *  constants that do not describe the assets, in an unknown direction. */
	float WarpTexGradientLarge = 14.09f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density|Asset", meta = (ClampMin = "0.0"))
	float WarpTexGradientSmall = 14.09f;

	FUniverseDensityParams()
	{
		// THE SMALL OCTAVE IS NOT A COPY OF THE LARGE ONE: lower amplitude, a scale seven times
		// higher -- fine grain against coarse bend. The spread is biased hard low because this
		// octave is what the appearance fetch varies most visibly. THIS MAX IS ESSENTIALLY THE
		// WHOLE FOLD CEILING, ~94% of predicted shear at these defaults.
		WarpSmall.Amount = FUniverseVarianceRange(0.05f, 0.3f, 2.0f);

		// 3079/4096, PRIME. A round 0.75 quantizes to 3072 = 2^10 * 3, the one composite numerator
		// the set can produce; it stays coprime with 19, 41 and 409 only by luck. The two values
		// differ by 0.23%, below what the eye reads as a frequency change.
		WarpSmall.Scale = 0.751730f;
		WarpSmall.LatticeFollow = 0.0f;
	}

#pragma region Derivation

	/** THE ONLY PLACE THE PACKING DECISIONS LIVE. Pure packing plus the three .w passengers, each
	 *  of which belongs to the same field as its host -- a warp scale multiplies its own octave's
	 *  amount, the lattice-follow steers the octave the weights vector gains. THAT TEST IS
	 *  NECESSARY AND NOT SUFFICIENT: a retired fourth passenger belonged to its host on its face
	 *  and still moved nothing, because the exponent it fed was a compile-time constant. A claim
	 *  must reach a LIVE consumer.
	 *
	 *  Seed and offset are arguments rather than members: the seed is authoritative on
	 *  FUniverseParams, the offset is per-frame state. */
	FUniverseDensityArgs Pack(int32 InSeed, const FUniverseFieldOffset& InOffset) const
	{
		FUniverseDensityArgs Out;

		Out.CellSizeRange = FVector4f(
			Lattice.CellSizeSmall, Lattice.CellSizeLarge, Lattice.CellSizeBias, 0.0f);

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

		Out.VoidSizeSpreadRange = Void.SizeSpread.Pack();
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
			0.0f);
		Out.RegionScales = FVector4f(Region.ScaleStructure, Region.ScaleAppearance, 0.0f, 0.0f);

		Out.BoundsFadeStart = BoundsFadeStart;

		return Out;
	}

	/** Fills a compute shader parameter struct from the packed arguments, member for member. The
	 *  shader-side names match MakeUniverseDensityParams's parameter names, so a parameter added
	 *  to the derivation breaks this at compile time on both sides.
	 *
	 *  A TEMPLATE on purpose: naming the concrete FParameters type would make this header depend
	 *  on the entity-gen header, which depends on this one. */
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

#pragma endregion

#pragma region Validation

	/** Predicted worst-case warp shear on the COARSE lattice, where the ceiling binds:
	 *
	 *      AmountLarge * ScaleLarge / ratio * GradientLarge
	 *    + AmountSmall * ScaleSmall * ratio * GradientSmall
	 *
	 *  At or below 1 the web bends; above it the displacement folds and the web TEARS -- only in
	 *  the regions the blend handed to the coarse lattice, so region-dependent tearing reads as
	 *  several other things first. Each amount is that octave's MAX, and each octave carries its
	 *  own gradient. THE CHECK, NOT THE TARGET. */
	float PredictedFoldShear() const
	{
		const float Ratio = FMath::Max(FMath::RoundToFloat(
			FMath::Max(Lattice.CellSizeLarge, 0.0f) / FMath::Max(Lattice.CellSizeSmall, 1e-6f)), 1.0f);

		const float LargeTerm =
			WarpLarge.Amount.Max * WarpLarge.Scale / Ratio * WarpTexGradientLarge;
		const float SmallTerm =
			WarpSmall.Amount.Max * WarpSmall.Scale * Ratio * WarpTexGradientSmall;

		return LargeTerm + SmallTerm;
	}

	/** Cells before two scales re-align: numerators p and q over 4096 repeat every 4096/gcd(p,q)
	 *  cells, so coprime numerators give the full 4096 -- the precision wrap itself. */
	static int32 ScaleRepeatPeriod(float InScaleA, float InScaleB)
	{
		const int32 Wrap = 4096;
		const int32 P = FMath::Max(FMath::RoundToInt(InScaleA * Wrap), 1);
		const int32 Q = FMath::Max(FMath::RoundToInt(InScaleB * Wrap), 1);
		return Wrap / FMath::GreatestCommonDivisor(P, Q);
	}

	/** Logs the constraints invisible in the details panel: the fold ceiling, the pairwise repeat
	 *  period of the four texture scales, and the values whose failure mode is a lit void or an
	 *  unbounded march. REPORTS RATHER THAN CORRECTS. False if anything is out of bounds. */
	bool Validate(const TCHAR* InContext = TEXT("FUniverseDensityParams")) const;
#pragma endregion
};

USTRUCT(BlueprintType)
/** Controls for UniverseRayMarch. PERFORMANCE controls rather than look controls.
 *
 *  THE STEPPING RULE IS NOT THE GALAXY'S. That layer sizes each step as a fraction of camera
 *  distance; this one sizes the first step from the chord and grows from there:
 *  h = chord / StepCount * (1 + StepGrowth * t01).
 *
 *  THE NEAR FIELD IS WHY. A camera-distance step goes to ZERO at the camera, and the camera is
 *  normally inside this volume, so successive samples land in one cell each paying a full
 *  fifty-four candidate walk for a field that has not changed. */
	struct ULTRALARGESCALE_API FUniverseMaterialParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "8.0", ClampMax = "1024.0"))
	/** A CEILING ON THE STEP COUNT, NOT A FLOOR. Growth only lengthens steps, so the actual count
	 *  is StepBudget * ln(1+g)/g -- equal to the budget only at growth 0, 0.46 of it at 3. */
	float VolumeStepBudget = 128.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0", ClampMax = "8.0"))
	/** How fast steps lengthen along the chord; 0 is uniform. The cheapest quality lever, since
	 *  the far field is where the bounds fade is taking the field out anyway.
	 *
	 *  NO STEP CEILING AND NO ITERATION BOUND HERE. The step floor is P.MinFeatureStep, derived
	 *  in the core from FeatureWidth and CellSizeSmall, and an authored absolute cannot compete
	 *  with a floor that knows the cell size. The loop bound is derived in the Custom node from
	 *  StepBudget, so an authored bound could only truncate -- and a truncated march looks
	 *  identical to one that finished, minus the far half of the volume. */
	float VolumeStepGrowth = 4.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0"))
	/** Multiplier on density before the march's exp(-density * scale * h) -- how opaque the result
	 *  is, and nothing else. The field's own densities set the RATIO between walls, filaments and
	 *  the floor.
	 *
	 *  KEEP IT AT 1 UNLESS THERE IS A REASON. Driving the look from here scales the apparent field
	 *  several times over its authored range, so every judgement made against the picture is made
	 *  against a field that does not exist. Output brightness belongs after the march. IT NEVER
	 *  REACHES PLACEMENT: acceptance normalises against each cell's own probed envelope. */
	float VolumeDensityScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material",
		meta = (ClampMin = "0.0"))
	/** Exponent applied to each sample before accumulation. Exactly 1 skips the pow. Above 1
	 *  deepens voids and thins sheets. Prefer the wall falloff for clearing voids -- same job
	 *  inside the field, and it costs its two SFU ops once rather than per step.
	 *
	 *  NO DITHER PIN and NO OWNERSHIP TOGGLE. The dither is per pixel inside the Custom node,
	 *  since Parameters and View exist only there. The push is unconditional because the compute
	 *  path reads this struct, and a material instance tuned independently would put render and
	 *  placement on two different universes -- silently, since both look plausible. A MID ignores
	 *  a name the material does not have, so a mismatched pin surfaces as a parameter sitting at
	 *  its authored value; the push is where that gets caught. */
	float VolumeNoisePower = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material|Debug")
	/** DEBUG: write each entity's SAMPLED DENSITY into its colour instead of its decoratives.
	 *
	 *  SEPARATES TWO FAILURES THAT LOOK IDENTICAL from outside -- a bad distribution, against the
	 *  two paths disagreeing about WHERE a point of the field is:
	 *
	 *    bright in bright, dark in dark    the paths agree; look at the budget
	 *    bright entities in dark nebula    the paths disagree about position
	 *    uniformly mid-grey                the dispatch is not reading the texture at all
	 *
	 *  Normalised by the analytic ceiling, so the scale is comparable between tiers. */
	bool bDebugColorByDensity = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	/** THE FOUR FIELD VOLUMES, one purpose-baked per fetch site. ALL FOUR MUST REACH THE COMPUTE
	 *  PATH -- placement and render sample one field or they are not one field. The actor
	 *  resolves all four in LoadRuntimeAssets and hands the same pointers to the material and the
	 *  dispatch. ALL FOUR PERIODIC, or repeat addressing cannot close the 4096-cell wrap.
	 *
	 *  VARIANCE, the region-axis fetches: UNORM, per-channel normalized to 0.5 with comparable
	 *  spread, a distinct archetype per channel -- a channel feeds VarianceT as a raw [0,1] lerp
	 *  factor, so a different histogram makes a Range's extremes land somewhere else. THE TWO
	 *  MUST BE DIFFERENT VARIANTS; two fetches into one asset share all four generators, and
	 *  LoadRuntimeAssets warns if they resolve equal.
	 *
	 *  WARP, the two octaves: SIGNED in [-1,1], since the shader applies them as displacements
	 *  with NO centring step, and a coherent VECTOR field across xyz. CURL GOES ON THE SMALL
	 *  OCTAVE -- it carries ~94% of predicted shear and curl folds only at second order. The cost
	 *  is a generic large octave contributing void-size variation that reads as VoidSizeSpread's
	 *  and is not. Gradient-normalize at bake if possible; that sets WarpTexGradient to 1.
	 *
	 *  RESOLUTION: variance 256^3, warp 128^3, and that way round. A variance fetch is a VALUE
	 *  straight into a lerp, so quantization lands on the parameter; a warp fetch is a
	 *  DISPLACEMENT filtered by the tap, the amplitude, the blend and the falloff it lands in. A
	 *  RESOLUTION CHANGE INVALIDATES THAT OCTAVE'S WarpTexGradient.
	 *
	 *  THEY LIVE IN /UniverseNoisePack, which must be ENABLED or all four return null -- which
	 *  surfaces as a plausible-looking field rather than an error. STRINGS, so the cooker has no
	 *  edge to follow: each needs a cooked reference or a listing in the project's additional
	 *  asset directories. FULL Package.Object FORM, since LoadObject's inference is least
	 *  reliable in a packaged build. */
	FString VarianceVolumeA = "/UniverseNoisePack/256/VT_MultiNoise_1_S8.VT_MultiNoise_1_S8";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VarianceVolumeB = "/UniverseNoisePack/256/VT_MultiNoise_2_S8.VT_MultiNoise_2_S8";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString WarpVolumeLarge = "/UniverseNoisePack/128/VT_PerlinVector_S4.VT_PerlinVector_S4";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString WarpVolumeSmall = "/UniverseNoisePack/128/VT_PerlinCurl_S4.VT_PerlinCurl_S4";

	/** Steps the march will actually take, as opposed to the budget. The two diverge fast: at
	 *  growth 4 a budget of 32 resolves to about 13. Diagnostic only. */
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

USTRUCT(BlueprintType)
/** Universe-layer generation parameters: the field, the march that draws it, per-tier streaming
 *  configs, and scale derivation. */
	struct ULTRALARGESCALE_API FUniverseParams : public FBaseParams {
	GENERATED_BODY()

#pragma region Density Field

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density")
	/** What UniverseDensityCore.ush evaluates, the ray march draws, and entity generation places
	 *  against. */
	FUniverseDensityParams DensityParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	/** How the field is MARCHED, as opposed to what it contains. Kept apart because none of it
	 *  reaches MakeUniverseDensityParams -- entity generation samples the same field without
	 *  any of it. */
	FUniverseMaterialParams MaterialParams;

#pragma endregion

#pragma region Tier Scale Derivation

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	/** Absolute largest entity scale in world units. Every tier's range cascades from it:
	 *  Tier[0].Max = this, Tier[0].Min = this / 2^(depth[1] - depth[0]), Tier[1].Max =
	 *  Tier[0].Min, and so on. */
	double MaxEntityScale = 1e22;

#pragma endregion

#pragma region Tier Debug

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Debug")
	/** DEBUG: which tiers stream at all. FOR ISOLATING ONE TIER AGAINST THE RENDER, hard by eye
	 *  otherwise -- the three reach very different distances, so what is visible at any depth is
	 *  a different mixture and a fault in one reads as a fault everywhere. A tier switched off is
	 *  never initialized and never updated. */
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

	/** Derives MinScale/MaxScale per tier from MaxEntityScale and the depth sequence. */
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

		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 1;
		LargeTier.SlotCapacity = 1000;

		MidTier.GridDepth = 3;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 500;

		SmallTier.GridDepth = 5;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 500;

		// GENERATION SUBDIVISION, sized against the FIELD'S cell rather than the tier's. At zero a
		// transition batch is nine thread groups, which leaves the GPU idle. A field cell is 0.9
		// sector extents here and a tier cell at depth d is 4/2^d of one, so Large spans ~2.2
		// field cells undivided -- too coarse for its probes; two levels bring it to 0.56. Mid and
		// Small sit at or below that already, so their one level buys OCCUPANCY.
		//
		// A STARTING POINT, NOT A TUNING. Read the C/P ratio in the batch log: below ~1 a tier
		// wants one level fewer, above ~9 one more.
		LargeTier.GenerationSubdivision = 2;
		MidTier.GenerationSubdivision = 1;
		SmallTier.GenerationSubdivision = 1;

		// Depth spacing of 2 gives 4x per tier, so each covers two octaves of scale:
		//   Large 2.5e21 -> 1e22, Mid 6.25e20 -> 2.5e21, Small 1.5625e20 -> 6.25e20
		DeriveScaleRanges();
	}

#pragma endregion
};
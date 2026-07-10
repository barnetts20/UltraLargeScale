#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "FTierStreamingSystem.h"
#include "StarSystemParams.generated.h"


// ---------------------------------------------------------------------------
// FStarSystemParams
//
// Tiers use the shared FTierParams (same struct as Universe/Galaxy). Star
// systems lay planets out analytically, so the tier curve/scale fields go
// unused here — only GridDepth / NeighborhoodRadius / SlotCapacity
// feed the streaming pipeline.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct SVO_API FStarSystemParams : public FBaseParams
{
	GENERATED_BODY()

	/** How much larger the star system's real span is than the star
	 *  particle's real size. NOTE: MaxEntityScale already authors a full
	 *  system DIAMETER (orbits included — Pluto-orbit class at the top of
	 *  the range), so this multiplier only needs to clear the star glyph
	 *  plus margin. Over-provisioning here directly burns the system's
	 *  integer-lattice and float-precision budget: span/smallest-feature
	 *  must stay within ~2^41, which with the current planet scale ranges
	 *  caps this near 4 (30 left NO valid StarSystemUnitScale window). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star")
	double BoundsScaleMultiplier = 1.0;

	/** Maximum number of planet sprites to generate (line layout). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	int32 MaxPlanets = 8;

	/** Absolute world-cm scale ranges for planet generation.
	 *  Inner rocky planets are drawn from TerrestrialMinScale..TerrestrialMaxScale.
	 *  Outer gas giants are drawn from GasGiantMinScale..GasGiantMaxScale.
	 *  MakePointDataFromWorldScale converts these to octree-local extents
	 *  using UnitScale, so planets have consistent physical sizes regardless
	 *  of which galaxy or star spawned them.
	 *
	 *  Real references (diameters in cm):
	 *    Mercury ≈ 4.9e8, Mars ≈ 6.8e8, Earth ≈ 1.27e9, Venus ≈ 1.2e9
	 *    Neptune ≈ 4.95e9, Saturn ≈ 1.16e10, Jupiter ≈ 1.4e10 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Terrestrial")
	double TerrestrialMinScale = 1e8;   // Small rocky (Ceres-to-Mercury class)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Terrestrial")
	double TerrestrialMaxScale = 1e9;   // Large super-Earth

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Gas Giant")
	double GasGiantMinScale = 3e9;      // Sub-Neptune / ice giant

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Gas Giant")
	double GasGiantMaxScale = 2e10;     // Hot Jupiter class

	/** Fraction of Extent used for innermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double InnerOrbitFraction = 0.08;

	/** Fraction of Extent used for outermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double OuterOrbitFraction = 0.85;

	// --- Tier configs (shared FTierParams; Mid/Small unused in first pass) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	FStarSystemParams()
	{
		UnitScale = 1.2e7;
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.SlotCapacity = 64;

		MidTier.GridDepth = 4;
		MidTier.NeighborhoodRadius = 1;
		MidTier.SlotCapacity = 0; // Zero — unused for now

		SmallTier.GridDepth = 7;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.SlotCapacity = 0; // Zero — unused for now
	}
};
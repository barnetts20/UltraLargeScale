// Fill out your copyright notice in the Description page of Project Settings.

// UniverseParams.h is otherwise all USTRUCTs and inline packing. Validate() is the one
// thing that wants to be out of line: it logs, so it would drag the format machinery into
// every translation unit that only wanted the authored struct.

#include "UniverseParams.h"

bool FUniverseDensityParams::Validate(const TCHAR* InContext) const
{
	bool bOk = true;

	// --- THE FOLD CEILING ---
	// First, because past it the coarse web tears rather than bends, and it does so only in
	// the regions the lattice blend has handed to the coarse tier. Region-dependent tearing
	// reads as a lattice problem, a region scale problem or a hash problem long before it
	// reads as a warp amplitude problem.
	const float Shear = PredictedFoldShear();
	if (Shear > 1.0f)
	{
		bOk = false;
		// BOTH GRADIENTS REPORTED, one per octave, because the octaves read separate assets
		// now and each term carries its own. A single number here would be an average of two
		// and would send the reader to the wrong pin half the time.
		UE_LOG(LogTemp, Warning,
			TEXT("%s: predicted warp shear on the coarse lattice is %.3f, above the fold ")
			TEXT("ceiling of 1. The web will tear rather than bend, and only in coarse ")
			TEXT("regions. The small octave almost always binds: amount %.4f x scale %.4f ")
			TEXT("x ratio x gradient %.2f, against the large octave's %.4f x %.4f / ratio ")
			TEXT("x gradient %.2f."),
			InContext, Shear,
			WarpSmall.Amount.Max, WarpSmall.Scale, WarpTexGradientSmall,
			WarpLarge.Amount.Max, WarpLarge.Scale, WarpTexGradientLarge);
	}
	else if (Shear > 0.7f)
	{
		UE_LOG(LogTemp, Log,
			TEXT("%s: predicted warp shear %.3f. Under the ceiling but inside the ~30%% ")
			TEXT("margin the tuning procedure asks for; little headroom left for raising ")
			TEXT("either octave."),
			InContext, Shear);
	}

	// --- SCALE COPRIMALITY ---
	// Four texture scales, each a whole number of 1/4096ths. Any pair re-aligns every
	// 4096/gcd cells, so a shared factor puts a visible repeat inside the precision wrap.
	// ADDING A FIFTH FETCH MEANS ADDING IT TO THIS TABLE and checking it against all four.
	//
	// WEAKER THAN IT WAS, AND WORTH KNOWING WHICH WAY. When every fetch read one packed
	// volume, a shared factor between two scales meant those two fetches literally repeated
	// each other's CONTENT at the re-alignment period -- the same texels, twice. All four
	// of these now read different assets, so that failure is gone outright: two fetches into
	// two unrelated volumes cannot repeat content whatever their periods do.
	//
	// What survives is the weaker structural version: two fields whose periods share a
	// factor have their region BOUNDARIES co-occur, so provinces start and stop in the same
	// places even though they are made of different noise. That is still worth avoiding and
	// still what this check finds, but it is a subtlety rather than a visible tiling seam,
	// so treat a warning here as advisory rather than as a defect.
	struct FNamedScale { const TCHAR* Name; float Scale; };
	const FNamedScale Scales[] = {
		{ TEXT("Region.ScaleStructure"),  Region.ScaleStructure },
		{ TEXT("Region.ScaleAppearance"), Region.ScaleAppearance },
		{ TEXT("WarpLarge.Scale"),        WarpLarge.Scale },
		{ TEXT("WarpSmall.Scale"),        WarpSmall.Scale },
	};
	const int32 NumScales = UE_ARRAY_COUNT(Scales);

	for (int32 i = 0; i < NumScales; ++i)
	{
		for (int32 j = i + 1; j < NumScales; ++j)
		{
			const int32 Period = ScaleRepeatPeriod(Scales[i].Scale, Scales[j].Scale);

			// 4096 is the precision wrap itself, so a coprime pair never repeats before the
			// field wraps anyway and there is nothing to report.
			if (Period >= 4096)
			{
				continue;
			}

			// Anything reachable inside a single session is the case that matters. A pair
			// repeating every few hundred cells is a visible grid; one repeating every
			// couple of thousand is a note rather than a fault.
			const bool bSevere = (Period < 512);
			bOk = bOk && !bSevere;

			UE_LOG(LogTemp, Warning,
				TEXT("%s: %s (%.6f) and %s (%.6f) share a factor and re-align every %d ")
				TEXT("cells%s. Odd numerators over 4096 are coprime to any power-of-two ")
				TEXT("scale for free."),
				InContext,
				Scales[i].Name, Scales[i].Scale,
				Scales[j].Name, Scales[j].Scale,
				Period,
				bSevere ? TEXT(", which is reachable in one session") : TEXT(""));
		}
	}

	// --- NEGATIVE VOID FLOOR ---
	// The march's exp(-density) turns negative density into unbounded gain rather than
	// opacity. Both ends, since a range spanning zero puts the negative end somewhere and a
	// noise channel decides where.
	if (FMath::Min(Void.Floor.Min, Void.Floor.Max) < 0.0f)
	{
		bOk = false;
		UE_LOG(LogTemp, Warning,
			TEXT("%s: Void.Floor reaches below zero (%.4f..%.4f). Negative density is ")
			TEXT("unbounded gain in the march, not a darker void."),
			InContext, Void.Floor.Min, Void.Floor.Max);
	}

	// --- WALL FALLOFF AT OR BELOW ZERO ---
	// 0 makes every non-zero base read as 1 and floods the field; below zero is a
	// division-by-zero shape that lights the void up rather than clearing it.
	if (FMath::Min(Wall.Falloff.Min, Wall.Falloff.Max) <= 0.0f)
	{
		bOk = false;
		UE_LOG(LogTemp, Warning,
			TEXT("%s: Wall.Falloff reaches zero or below (%.4f..%.4f). At 0 every non-zero ")
			TEXT("base reads as 1 and the field floods; below 0 the void lights up rather ")
			TEXT("than clearing. 1 is the bare saturate."),
			InContext, Wall.Falloff.Min, Wall.Falloff.Max);
	}

	// --- DEGENERATE FEATURE WIDTH ---
	// Not a range limit; a division. The core guards it, but a zero width also drives
	// MinFeatureStep to zero, which hands the march's step floor back to the authored pin
	// alone -- an absolute distance with no knowledge of the cell size.
	if (FMath::Max(FeatureWidth.Min, FeatureWidth.Max) <= 0.0f)
	{
		bOk = false;
		UE_LOG(LogTemp, Warning,
			TEXT("%s: FeatureWidth is degenerate at or below zero. The lambda guard holds ")
			TEXT("the field together, but the march loses its field-resolution step floor."),
			InContext);
	}

	// --- VOID SIZE SPREAD AGAINST THE SEARCH WINDOW ---
	// The power-diagram offset is what erodes the 27-cell neighbourhood: a node out-reaches
	// an unoffset neighbour by this much in squared distance, and once that approaches the
	// cell separation the true nearest node can sit outside the window. The bound degrades
	// with wide features as well as with large offsets, which is why the width is in the
	// test rather than a fixed threshold.
	const float MaxSpread = FMath::Max(Void.SizeSpread.Min, Void.SizeSpread.Max);
	const float MaxWidth = FMath::Max(FeatureWidth.Min, FeatureWidth.Max);
	if (MaxSpread > 0.5f || (MaxSpread > 0.25f && MaxWidth > 0.5f))
	{
		UE_LOG(LogTemp, Log,
			TEXT("%s: Void.SizeSpread reaches %.3f in SQUARED cell units with feature width ")
			TEXT("up to %.3f. This is the pin that erodes the 27-cell search, and a value ")
			TEXT("carried over from the old additive form does not mean what it used to. ")
			TEXT("Start an order lower and come up."),
			InContext, MaxSpread, MaxWidth);
	}

	// --- REGION FETCHES THAT WILL BE SKIPPED ---
	// The core pays for a region fetch only if at least one consumer of that fetch is
	// non-degenerate, so flattening a whole group silently turns its variance off. That is
	// legal and occasionally wanted, but it is not visible from the details panel, and the
	// symptom -- a field with no provincial variation at all -- reads as a broken texture
	// long before it reads as an authored constant.
	//
	// EVERY RANGE MUST APPEAR IN ITS FETCH'S LIST, matching the core's own bRegionEnable
	// tests. A range that moves between fields has to move between these lists with it.
	const bool bStructureLive =
		!Void.SizeSpread.IsDegenerate() ||
		!FeatureWidth.IsDegenerate() ||
		!WarpLarge.Amount.IsDegenerate() ||
		(FMath::RoundToFloat(FMath::Max(Lattice.CellSizeLarge, 0.0f)
			/ FMath::Max(Lattice.CellSizeSmall, 1e-6f)) > 1.5f);

	const bool bAppearanceLive =
		!Wall.Density.IsDegenerate() ||
		!Wall.Falloff.IsDegenerate() ||
		!Filament.Density.IsDegenerate() ||
		!Void.Floor.IsDegenerate() ||
		!WarpSmall.Amount.IsDegenerate();

	if (!bStructureLive)
	{
		UE_LOG(LogTemp, Log,
			TEXT("%s: the STRUCTURE region fetch will be skipped -- void spread, feature ")
			TEXT("width and large warp amount are all constant and there is only one ")
			TEXT("lattice. Every consumer falls back to its plain midpoint."),
			InContext);
	}
	if (!bAppearanceLive)
	{
		UE_LOG(LogTemp, Log,
			TEXT("%s: the APPEARANCE region fetch will be skipped -- wall density and ")
			TEXT("falloff, filament density, void floor and small warp amount are all ")
			TEXT("constant. Every consumer falls back to its plain midpoint."),
			InContext);
	}

	return bOk;
}
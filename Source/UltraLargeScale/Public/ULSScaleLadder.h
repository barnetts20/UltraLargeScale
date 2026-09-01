// ULSScaleLadder.h
#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"

/** THE UNIT LADDER.
 *
 *  Everything in this plugin that has a length is ultimately a count of real centimetres,
 *  because UnitScale is defined as "real cm per one local unit of this layer" and that is
 *  the only bridge between the layers. The ladder therefore takes cm and nothing else --
 *  a caller that has local units multiplies by its layer's UnitScale FIRST. Accepting
 *  local units here would mean the formatter had to know which layer it was looking at,
 *  and a universe-local 1.0 and a star-system-local 1.0 differ by ten orders of
 *  magnitude, so the mistake would print a confident, well-formatted, wrong number.
 *
 *  WHY A LADDER AND NOT %g. Scientific notation is correct at every magnitude and legible
 *  at none of them: "3.412e+21 cm" tells you nothing about whether you are inside a
 *  galaxy. The ladder trades exactness for a unit whose name carries the scale, which is
 *  the entire point of a readout you glance at while flying.
 *
 *  THE WIDEST RUNG IS AU -> ly (4.8 orders), so an AU-band value can reach five integer
 *  digits ("63241 AU") before it promotes. That was accepted deliberately: the candidates
 *  for filling the gap are the light-minute and the light-hour, and neither is a unit
 *  anyone reads as a distance without doing arithmetic first. Five digits of AU is worse
 *  formatting and better communication. Every other rung is 2-3 orders.
 *
 *  FAILURE MODE. If a value arrives already divided by UnitScale, or multiplied by it
 *  twice, the output is still a perfectly formatted distance in a plausible unit -- it
 *  just describes somewhere else. There is no in-band signal for that. The guard is that
 *  every call site in ULSNavReadout.cpp does the conversion on one line, next to the
 *  UnitScale it used.
 */
namespace ULSScale
{
	/** Speed of light in cm/s. Used only to annotate speeds; nothing simulates it. */
	constexpr double CmPerSecondLight = 2.99792458e10;

	/** cm per metre / kilometre, for callers that want the raw constant. */
	constexpr double CmPerMetre = 1.0e2;
	constexpr double CmPerKilometre = 1.0e5;

	/** cm per astronomical unit (IAU 2012 exact definition, 1.495978707e11 m). */
	constexpr double CmPerAstronomicalUnit = 1.495978707e13;

	/** cm per light year (Julian year x c, exact by construction). */
	constexpr double CmPerLightYear = 9.4607304725808e17;

	/** One rung: the suffix that gets printed and how many cm it is worth. */
	struct FRung
	{
		const TCHAR* Suffix;
		double       Cm;
	};

	/** ASCENDING, and the order is load-bearing -- SelectRung walks from the top down and
	 *  takes the first rung the value clears, so an out-of-order entry silently makes an
	 *  entire band unreachable rather than producing an error. */
	inline TArrayView<const FRung> GetLadder()
	{
		static const FRung Rungs[] =
		{
			{ TEXT("cm"),  1.0                      },
			{ TEXT("m"),   CmPerMetre               },
			{ TEXT("km"),  CmPerKilometre           },
			{ TEXT("Mm"),  1.0e8                    },  // 1e3 km
			{ TEXT("Gm"),  1.0e11                   },  // 1e6 km
			{ TEXT("AU"),  CmPerAstronomicalUnit    },
			{ TEXT("ly"),  CmPerLightYear           },
			{ TEXT("kly"), CmPerLightYear * 1.0e3   },
			{ TEXT("Mly"), CmPerLightYear * 1.0e6   },
			{ TEXT("Gly"), CmPerLightYear * 1.0e9   },
		};
		return MakeArrayView(Rungs, UE_ARRAY_COUNT(Rungs));
	}

	/** Index of the rung a magnitude should print in: the largest rung it is at least one
	 *  of. Zero and denormal-scale values pin to METRES rather than to the bottom of the
	 *  ladder, because a stationary readout should say "0.00 m/s" and not "0.00 cm/s" --
	 *  the latter reads as a measurement precise to a hundredth of a centimetre, which it
	 *  is not. */
	inline int32 SelectRung(double InAbsCm)
	{
		const TArrayView<const FRung> Ladder = GetLadder();

		if (!FMath::IsFinite(InAbsCm) || InAbsCm < 1.0e-6)
		{
			return 1; // metres
		}

		for (int32 Index = Ladder.Num() - 1; Index >= 0; --Index)
		{
			if (InAbsCm >= Ladder[Index].Cm)
			{
				return Index;
			}
		}
		return 0;
	}

	/** Decimal places that keep the printed number near InSigFigs significant digits.
	 *  Capped at three: past that the digits are noise from a double that has already
	 *  been divided by ~1e18, and the extra width costs a column. */
	inline int32 DecimalsFor(double InAbsValue, int32 InSigFigs)
	{
		const int32 IntegerDigits = (InAbsValue >= 1.0)
			? static_cast<int32>(FMath::FloorToDouble(FMath::LogX(10.0, InAbsValue))) + 1
			: 1;
		return FMath::Clamp(InSigFigs - IntegerDigits, 0, 3);
	}

	/** Formats a cm magnitude in a CALLER-CHOSEN rung. Exists so the three axes of a
	 *  position can share one unit: formatting each axis independently gives
	 *  "1.20 Mly / 431 ly / 88.2 kly", three numbers that cannot be compared by eye,
	 *  which defeats the purpose of showing a vector at all. */
	inline FString FormatCmInRung(double InCm, int32 InRungIndex, int32 InSigFigs = 4)
	{
		const TArrayView<const FRung> Ladder = GetLadder();
		if (!FMath::IsFinite(InCm))
		{
			return TEXT("---");
		}

		const int32  Index = FMath::Clamp(InRungIndex, 0, Ladder.Num() - 1);
		const double Value = InCm / Ladder[Index].Cm;
		const double Abs = FMath::Abs(Value);

		// PAST THE TOP RUNG the ladder has nothing left to promote to, so it falls back to
		// exponent form rather than printing a seven-digit Gly count. Reaching this is not
		// an error -- the octree covers +/-128 sector extents -- but it is the point where
		// the readout stops being glanceable, which is worth seeing.
		if (Abs >= 1.0e5)
		{
			return FString::Printf(TEXT("%.3e %s"), Value, Ladder[Index].Suffix);
		}

		return FString::Printf(TEXT("%.*f %s"), DecimalsFor(Abs, InSigFigs), Value, Ladder[Index].Suffix);
	}

	/** Formats a single cm magnitude, choosing its own rung. */
	inline FString FormatCm(double InCm, int32 InSigFigs = 4)
	{
		return FormatCmInRung(InCm, SelectRung(FMath::Abs(InCm)), InSigFigs);
	}

	/** Formats a cm/s magnitude. Same ladder, "/s" appended -- deliberately NOT a second
	 *  ladder of speed units. A speed ladder would need its own rungs (c, ly/day, ...)
	 *  and then a distance and a speed at the same magnitude would print in unrelated
	 *  units, so "how long to cross that" stops being mental arithmetic. */
	inline FString FormatSpeed(double InCmPerSecond, int32 InSigFigs = 4)
	{
		return FormatCm(InCmPerSecond, InSigFigs) + TEXT("/s");
	}

	/** Multiples of c, for annotating a speed. Returned as a bare number; the caller
	 *  decides how to label it. */
	inline double AsLightSpeedMultiple(double InCmPerSecond)
	{
		return InCmPerSecond / CmPerSecondLight;
	}

	/** Formats a c-multiple compactly: fixed below 1000, exponent above. */
	inline FString FormatLightSpeedMultiple(double InCmPerSecond)
	{
		const double C = AsLightSpeedMultiple(InCmPerSecond);
		const double Abs = FMath::Abs(C);

		if (!FMath::IsFinite(C))       return TEXT("---");
		if (Abs < 1.0e-3)              return TEXT("<0.001c");
		if (Abs < 1.0e3)               return FString::Printf(TEXT("%.*fc"), DecimalsFor(Abs, 4), C);
		return FString::Printf(TEXT("%.3ec"), C);
	}

	/** Formats a dimensionless multiplier (the compression factor, the implied ratio).
	 *  Fixed-point while it is small enough to read as a number, exponent after --
	 *  SpeedScale spans 1 to ~1e17, and "160000000000000000" is not a readout. */
	inline FString FormatMultiplier(double InValue)
	{
		if (!FMath::IsFinite(InValue)) return TEXT("---");

		const double Abs = FMath::Abs(InValue);
		if (Abs < 1.0e4)
		{
			return FString::Printf(TEXT("x%.*f"), DecimalsFor(Abs, 5), InValue);
		}
		return FString::Printf(TEXT("x%.4e"), InValue);
	}

	/** Formats a position as three axes sharing ONE rung, chosen from the largest
	 *  component. See FormatCmInRung for why they share. */
	inline FString FormatVectorCm(const FVector& InCm, int32 InSigFigs = 4)
	{
		const double Largest = FMath::Max3(FMath::Abs(InCm.X), FMath::Abs(InCm.Y), FMath::Abs(InCm.Z));
		const int32  Rung = SelectRung(Largest);
		const TArrayView<const FRung> Ladder = GetLadder();

		// Suffix printed ONCE at the end rather than per axis: three repeats of "Mly" is
		// most of the line's width and none of its information.
		const double Scale = Ladder[Rung].Cm;
		const double X = InCm.X / Scale;
		const double Y = InCm.Y / Scale;
		const double Z = InCm.Z / Scale;
		const double Abs = FMath::Max3(FMath::Abs(X), FMath::Abs(Y), FMath::Abs(Z));

		if (Abs >= 1.0e5)
		{
			return FString::Printf(TEXT("%.3e  %.3e  %.3e  %s"), X, Y, Z, Ladder[Rung].Suffix);
		}

		const int32 Decimals = DecimalsFor(Abs, InSigFigs);
		return FString::Printf(TEXT("%+.*f  %+.*f  %+.*f  %s"),
			Decimals, X, Decimals, Y, Decimals, Z, Ladder[Rung].Suffix);
	}
}

// GalaxyArchetype.cpp

#include "GalaxyArchetype.h"

namespace
{
	/** Component letters for the two non-scalar members, in memory order. Both are
	 *  plain float aggregates, so a component index is an offset into the struct. */
	const TCHAR* const kColorComponents[] = { TEXT("R"), TEXT("G"), TEXT("B"), TEXT("A") };
	const TCHAR* const kVectorComponents[] = { TEXT("X"), TEXT("Y"), TEXT("Z") };

	/** Index of a component letter within a struct's letter table, or INDEX_NONE. */
	int32 FindComponentIndex(const FString& InComponent, const TCHAR* const* InTable, int32 InCount)
	{
		for (int32 Index = 0; Index < InCount; ++Index)
		{
			if (InComponent.Equals(InTable[Index], ESearchCase::IgnoreCase))
			{
				return Index;
			}
		}
		return INDEX_NONE;
	}

	/** The two float-aggregate leaf types, and their component letters in memory order.
	 *  Returns nullptr for anything else. */
	const TCHAR* const* GetComponentTable(const UScriptStruct* InStruct, int32& OutCount)
	{
		if (InStruct == TBaseStructure<FLinearColor>::Get())
		{
			OutCount = UE_ARRAY_COUNT(kColorComponents);
			return kColorComponents;
		}
		// NOTE: if TVariantStructure<FVector3f> fails to compile against the engine
		// version, comparing InStruct->GetFName() to "Vector3f" is the equivalent.
		if (InStruct == TVariantStructure<FVector3f>::Get())
		{
			OutCount = UE_ARRAY_COUNT(kVectorComponents);
			return kVectorComponents;
		}
		OutCount = 0;
		return nullptr;
	}
}

float* UGalaxyArchetype::ResolveParameter(FGalaxyProceduralParams& InOutParams, FName InParameter)
{
	// A PATH, NOT A NAME, since the members were grouped into sub-structs: "Arms.ArmRadius",
	// and "Noise.NoiseOffset.X" for a component of an aggregate leaf. Walked segment by
	// segment rather than special-cased by depth, so nesting another level later needs
	// nothing here.
	TArray<FString> Segments;
	InParameter.ToString().ParseIntoArray(Segments, TEXT("."), true);
	if (Segments.Num() == 0)
	{
		return nullptr;
	}

	const UStruct* CurrentStruct = FGalaxyProceduralParams::StaticStruct();
	void* CurrentContainer = &InOutParams;

	for (int32 Depth = 0; Depth < Segments.Num(); ++Depth)
	{
		FProperty* Property = CurrentStruct->FindPropertyByName(*Segments[Depth]);
		if (!Property)
		{
			return nullptr;
		}

		const bool bIsLast = (Depth == Segments.Num() - 1);

		if (FFloatProperty* FloatProperty = CastField<FFloatProperty>(Property))
		{
			// A scalar has to end the path. A trailing component on one is an authoring
			// error, not something to ignore -- returning nullptr routes it to the
			// validator's message.
			return bIsLast ? FloatProperty->ContainerPtrToValuePtr<float>(CurrentContainer) : nullptr;
		}

		FStructProperty* StructProperty = CastField<FStructProperty>(Property);
		if (!StructProperty)
		{
			// Object properties (NoiseTexture) land here: categorical, drawn from
			// NoiseTextures rather than rolled over an interval.
			return nullptr;
		}

		void* Inner = StructProperty->ContainerPtrToValuePtr<void>(CurrentContainer);

		int32 ComponentCount = 0;
		if (const TCHAR* const* Table = GetComponentTable(StructProperty->Struct, ComponentCount))
		{
			// An aggregate leaf: exactly one component segment must follow. Without one
			// the path designates three or four floats rather than a single value --
			// rejected rather than defaulted to the first, which would roll one axis and
			// look like the others simply had no effect.
			if (bIsLast || Depth + 2 != Segments.Num())
			{
				return nullptr;
			}

			const int32 ComponentIndex = FindComponentIndex(Segments[Depth + 1], Table, ComponentCount);
			return ComponentIndex != INDEX_NONE
				? reinterpret_cast<float*>(Inner) + ComponentIndex
				: nullptr;
		}

		// A group. Descend and keep walking; a path that stops on one designates the
		// whole group, which is not a rollable value.
		if (bIsLast)
		{
			return nullptr;
		}

		CurrentStruct = StructProperty->Struct;
		CurrentContainer = Inner;
	}

	return nullptr;
}

namespace
{
	/** Depth-first walk emitting a dotted path for every rollable leaf. */
	void CollectRollableNames(const UStruct* InStruct, const FString& InPrefix, TArray<FString>& OutNames)
	{
		for (TFieldIterator<FProperty> It(InStruct); It; ++It)
		{
			FProperty* Property = *It;
			const FString Path = InPrefix.IsEmpty()
				? Property->GetName()
				: FString::Printf(TEXT("%s.%s"), *InPrefix, *Property->GetName());

			if (CastField<FFloatProperty>(Property))
			{
				OutNames.Add(Path);
				continue;
			}

			if (FStructProperty* StructProperty = CastField<FStructProperty>(Property))
			{
				int32 ComponentCount = 0;
				if (const TCHAR* const* Table = GetComponentTable(StructProperty->Struct, ComponentCount))
				{
					for (int32 Index = 0; Index < ComponentCount; ++Index)
					{
						OutNames.Add(FString::Printf(TEXT("%s.%s"), *Path, Table[Index]));
					}
				}
				else
				{
					CollectRollableNames(StructProperty->Struct, Path, OutNames);
				}
			}
		}
	}
}

TArray<FString> UGalaxyArchetype::GetRollableParameterNames()
{
	TArray<FString> Names;
	CollectRollableNames(FGalaxyProceduralParams::StaticStruct(), FString(), Names);

	// Alphabetical, which now also clusters by group -- every Arms.* entry lands
	// together. Declaration order would scatter them, and is in any case the wrong
	// organisation for finding a name in a dropdown.
	Names.Sort();
	return Names;
}

float UGalaxyArchetype::RollRange(const FGalaxyParamRange& InRange, int32 InSeed)
{
	// A STREAM PER PARAMETER, KEYED BY NAME. Drawing these sequentially from one stream
	// would identify each parameter by its POSITION in Ranges, so adding a row while
	// authoring would reshuffle every parameter below it -- and rows get added
	// constantly during exactly the phase where "what did my change do" has to stay
	// answerable. Keyed by name, adding a row moves one parameter.
	FRandomStream Stream = ProcSeed::Stream(InSeed, ProcSeed::ChannelId(InRange.Parameter));

	// Shaped by Bias BEFORE the lerp, so the exponent acts on the 0..1 position rather
	// than on the value -- which keeps it meaningful for ranges that do not start at 0.
	const float Alpha = FMath::Pow(Stream.FRand(), FMath::Max(InRange.Bias, 0.01f));
	float Value = FMath::Lerp(InRange.Min, InRange.Max, Alpha);

	if (InRange.Quantize > 0.0f)
	{
		// Snapped to multiples of Quantize in ABSOLUTE terms, not relative to Min, so
		// ArmCount lands on whole numbers rather than on Min plus whole numbers.
		// Re-clamped because snapping can step outside the authored interval.
		Value = FMath::GridSnap(Value, InRange.Quantize);
		Value = FMath::Clamp(Value, FMath::Min(InRange.Min, InRange.Max), FMath::Max(InRange.Min, InRange.Max));
	}

	return Value;
}

FGalaxyProceduralParams UGalaxyArchetype::Resolve(int32 InSeed, bool bInDefaultsOnly) const
{
	FGalaxyProceduralParams Out = Default;

	if (bInDefaultsOnly)
	{
		return Out;
	}

	for (const FGalaxyParamRange& Range : Ranges)
	{
		float* const Address = ResolveParameter(Out, Range.Parameter);
		if (!Address)
		{
			// Already reported by Validate at config load. Skipped silently here
			// because this runs per galaxy spawn and would otherwise flood the log
			// with one line per galaxy for a problem that is reported once.
			continue;
		}

		*Address = RollRange(Range, InSeed);
	}

	if (NoiseTextures.Num() > 0)
	{
		FRandomStream Stream = ProcSeed::Stream(InSeed, GalaxySeed::NoiseTexture);
		const int32 Index = Stream.RandRange(0, NoiseTextures.Num() - 1);
		if (NoiseTextures[Index])
		{
			Out.NoiseTexture = NoiseTextures[Index];
		}
	}

	// LAST, so it can read every rolled value and derive, correlate or override from
	// it. Empty on the base class.
	ApplyGenerationLogic(Out, InSeed);

	return Out;
}

bool UGalaxyArchetype::Validate() const
{
	bool bValid = true;

	// Resolution needs a mutable instance to compute addresses against; nothing is read
	// from it. Cheaper than a parallel const path that would have to stay in agreement
	// with ResolveParameter.
	FGalaxyProceduralParams Scratch = Default;
	TSet<FName> Seen;

	for (int32 Index = 0; Index < Ranges.Num(); ++Index)
	{
		const FGalaxyParamRange& Range = Ranges[Index];

		if (Range.Parameter.IsNone())
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d has no parameter set."),
				*GetName(), Index);
			bValid = false;
			continue;
		}

		if (!ResolveParameter(Scratch, Range.Parameter))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d names '%s', which is not a ")
				TEXT("rollable parameter. Names are dotted PATHS now that the members are ")
				TEXT("grouped -- \"Arms.ArmRadius\", or \"Noise.NoiseOffset.X\" for one ")
				TEXT("component of a vector. A path that stops on a group designates the ")
				TEXT("whole group and is not rollable, and NoiseTexture is categorical: it ")
				TEXT("belongs in NoiseTextures instead."),
				*GetName(), Index, *Range.Parameter.ToString());
			bValid = false;
			continue;
		}

		if (Range.Min > Range.Max)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d ('%s') has Min %.4f above Max ")
				TEXT("%.4f. Nothing downstream catches this -- the galaxy simply comes ")
				TEXT("out wrong."),
				*GetName(), Index, *Range.Parameter.ToString(), Range.Min, Range.Max);
			bValid = false;
		}

		if (Range.Bias <= 0.0f)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d ('%s') has Bias %.4f. Bias is an ")
				TEXT("exponent and must be above zero; 1 is uniform."),
				*GetName(), Index, *Range.Parameter.ToString(), Range.Bias);
			bValid = false;
		}

		// A quantum wider than the interval collapses the roll to a single value, or to
		// whichever endpoint the clamp lands on. Legal, and occasionally even intended,
		// but far more often it means the interval and the step disagree about units.
		if (Range.Quantize > 0.0f && Range.Quantize > (Range.Max - Range.Min))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d ('%s') quantizes to %.4f over an ")
				TEXT("interval of %.4f, so the roll can only produce one value."),
				*GetName(), Index, *Range.Parameter.ToString(),
				Range.Quantize, Range.Max - Range.Min);
			bValid = false;
		}

		// A duplicate is not merely redundant: both rows key the same stream off the
		// same name, so they draw the IDENTICAL value and the later one silently wins.
		// Whichever interval was meant, one of them is doing nothing.
		bool bAlreadySeen = false;
		Seen.Add(Range.Parameter, &bAlreadySeen);
		if (bAlreadySeen)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("UGalaxyArchetype '%s': range %d duplicates '%s'. The last entry ")
				TEXT("wins and the other has no effect."),
				*GetName(), Index, *Range.Parameter.ToString());
			bValid = false;
		}
	}

	return bValid;
}

void UGalaxyArchetype::LogPreview(int32 InSeed, bool bInDefaultsOnly) const
{
	const FGalaxyProceduralParams Resolved = Resolve(InSeed, bInDefaultsOnly);

	// The silhouette parameters: which layers are present, and the shape of the ones
	// that are. Enough to recognise a galaxy without spawning it.
	UE_LOG(LogTemp, Display,
		TEXT("GalaxyArchetype '%s' seed %d%s -> densities arm/disc/bulge/bg ")
		TEXT("%.3f/%.3f/%.3f/%.3f, arms %.1f pitch %.3f, bulge conc %.2f vert %.2f, ")
		TEXT("radii arm/disc/bulge %.3f/%.3f/%.3f, stars x%.2f"),
		*GetName(), InSeed, bInDefaultsOnly ? TEXT(" (defaults)") : TEXT(""),
		Resolved.Arms.ArmDensity, Resolved.Disc.DiscDensity, Resolved.Bulge.BulgeDensity, Resolved.Background.BackgroundDensity,
		Resolved.Arms.ArmCount, Resolved.Arms.ArmPitchAngle,
		Resolved.Bulge.BulgeConcentration, Resolved.Bulge.BulgeVerticalRatio,
		Resolved.Arms.ArmRadius, Resolved.Disc.DiscRadius, Resolved.Bulge.BulgeRadius,
		Resolved.StarDensityScale);
}
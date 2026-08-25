// Fill out your copyright notice in the Description page of Project Settings.

#include "GalaxyParams.h"
#include "GalaxyArchetype.h"

// -----------------------------------------------------------------------------
// FGalaxySpawnConfig::Generate
//
// The ONE producer of a galaxy's params. Out of line rather than in the header because
// it needs UGalaxyArchetype's definition, and that asset includes GalaxyParams.h.
// -----------------------------------------------------------------------------

namespace
{
	/** Weighted pick over the entries that are enabled, assigned, and carry weight.
	 *  Returns INDEX_NONE when nothing is selectable. */
	int32 SelectArchetype(const FGalaxySpawnConfig& InConfig, int32 InSeed)
	{
		float TotalWeight = 0.0f;
		for (const FGalaxyArchetypeEntry& Entry : InConfig.Archetypes)
		{
			if (Entry.bEnabled && Entry.Archetype && Entry.Weight > 0.0f)
			{
				TotalWeight += Entry.Weight;
			}
		}

		if (TotalWeight <= 0.0f)
		{
			return INDEX_NONE;
		}

		// ITS OWN CHANNEL. Adding an archetype or retuning a weight must not disturb
		// any parameter roll -- those key off their own property names.
		FRandomStream Stream = ProcSeed::Stream(InSeed, GalaxySeed::Archetype);
		float Pick = Stream.FRandRange(0.0f, TotalWeight);

		int32 LastSelectable = INDEX_NONE;
		for (int32 Index = 0; Index < InConfig.Archetypes.Num(); ++Index)
		{
			const FGalaxyArchetypeEntry& Entry = InConfig.Archetypes[Index];
			if (!Entry.bEnabled || !Entry.Archetype || Entry.Weight <= 0.0f)
			{
				continue;
			}

			LastSelectable = Index;
			Pick -= Entry.Weight;
			if (Pick <= 0.0f)
			{
				return Index;
			}
		}

		// Reached only when Pick lands exactly on TotalWeight through float rounding.
		// Falls through to the last selectable entry rather than INDEX_NONE, because
		// "no archetype" is a far louder outcome than "the last one, once too often".
		return LastSelectable;
	}
}

FGalaxyParams FGalaxySpawnConfig::Generate(const FGalaxySpawnConfig& InConfig,
	FLinearColor InParentColor, int32 InSeed)
{
	FGalaxyParams Out;
	Out.Seed = InSeed;
	Out.ParentColor = InParentColor;

	// Config is copied WHOLESALE and never rolled. Copied before any early return so an
	// unconfigured archetype array still yields a usable actor.
	Out.Config = InConfig.Config;

	// DEFAULT MODE picks by INDEX; roll mode picks by WEIGHT. Deliberately different:
	// default mode exists to look at ONE chosen archetype, and having it obey the
	// weights would make it a slower way of doing what roll mode already does.
	int32 Index = INDEX_NONE;
	if (InConfig.bUseDefaults)
	{
		Index = InConfig.Archetypes.IsValidIndex(InConfig.DefaultArchetype)
			? InConfig.DefaultArchetype
			: INDEX_NONE;

		if (Index == INDEX_NONE && InConfig.Archetypes.Num() > 0)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FGalaxySpawnConfig: DefaultArchetype %d is out of range for %d ")
				TEXT("entries; falling back to entry 0."),
				InConfig.DefaultArchetype, InConfig.Archetypes.Num());
			Index = 0;
		}
	}
	else
	{
		Index = SelectArchetype(InConfig, InSeed);
	}

	// NO ARCHETYPE IS A SUPPORTED STATE, not a misconfiguration to warn about here --
	// an empty array, every entry disabled, or no asset assigned. It yields
	// FGalaxyProceduralParams struct defaults, which is precisely the galaxy this
	// system generated before proceduralization and therefore the regression baseline
	// for the whole refactor. Validate() is where a suspicious version of this state
	// gets reported, once, rather than once per spawn.
	const UGalaxyArchetype* Archetype = InConfig.Archetypes.IsValidIndex(Index)
		? InConfig.Archetypes[Index].Archetype.Get()
		: nullptr;

	if (!Archetype)
	{
		return Out;
	}

	Out.Procedural = Archetype->Resolve(InSeed, InConfig.bUseDefaults);

	if (InConfig.bLogGeneration)
	{
		Archetype->LogPreview(InSeed, InConfig.bUseDefaults);
	}

	return Out;
}

bool FGalaxySpawnConfig::Validate(const FGalaxySpawnConfig& InConfig)
{
	bool bValid = true;

	// Two entries pointing at the same asset is checked rather than assumed harmless:
	// it is a legitimate way to express a doubled weight, but far more often it is a
	// reference that was meant to be changed and was not.
	TSet<const UGalaxyArchetype*> Seen;

	for (int32 Index = 0; Index < InConfig.Archetypes.Num(); ++Index)
	{
		const FGalaxyArchetypeEntry& Entry = InConfig.Archetypes[Index];

		if (!Entry.Archetype)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FGalaxySpawnConfig: entry %d has no archetype asset assigned; it ")
				TEXT("can never be selected."), Index);
			bValid = false;
			continue;
		}

		bool bAlreadySeen = false;
		Seen.Add(Entry.Archetype.Get(), &bAlreadySeen);
		if (bAlreadySeen)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FGalaxySpawnConfig: entry %d references '%s', which an earlier ")
				TEXT("entry already references."), Index, *Entry.Archetype->GetName());
			bValid = false;
		}

		if (!Entry.Archetype->Validate())
		{
			bValid = false;
		}
	}

	// Archetypes authored but none reachable: legal, since it falls back to struct
	// defaults, but almost certainly not what was meant once assets exist.
	if (InConfig.Archetypes.Num() > 0 && !InConfig.bUseDefaults)
	{
		bool bAnySelectable = false;
		for (const FGalaxyArchetypeEntry& Entry : InConfig.Archetypes)
		{
			bAnySelectable |= (Entry.bEnabled && Entry.Archetype && Entry.Weight > 0.0f);
		}

		if (!bAnySelectable)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FGalaxySpawnConfig: %d archetypes authored but none is selectable ")
				TEXT("(enabled, assigned, weight above zero). Every galaxy will use ")
				TEXT("struct defaults."), InConfig.Archetypes.Num());
			bValid = false;
		}
	}

	return bValid;
}
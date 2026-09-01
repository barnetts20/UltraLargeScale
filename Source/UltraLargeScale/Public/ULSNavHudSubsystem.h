// ULSNavHudSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "ULSNavReadout.h"
#include "ULSNavHudSubsystem.generated.h"

class UCanvas;
class UFont;
class APlayerController;
class AUniverseActor;

/** THE DIAGNOSTIC NAV READOUT. Replaces reading SpeedScale and traversal changes out of
 *  the log, which is a workflow that only works when exactly one thing is being tuned and
 *  which scrolls away the moment anything else logs.
 *
 *  WHY A WORLD SUBSYSTEM AND NOT AHUD. Three reasons, in order of how much they matter:
 *
 *  1. AHUD is a scarce slot. There is exactly one HUD class per GameMode, and the
 *     player-facing HUD -- when it exists -- will want it, most likely with a UMG widget
 *     tree a designer authors. A diagnostic layer that occupies that slot has to be
 *     un-occupied later, and in the meantime it forces the player HUD to be a child of
 *     the debug HUD or vice versa. The handoff's instinct that the two are different
 *     projects sharing a name is right; this is the half that should not own the slot.
 *
 *  2. Zero project setup. A subsystem instantiates itself in every game world. An AHUD
 *     subclass only draws if someone remembers to set HUDClass on the GameMode, and the
 *     failure mode of forgetting is a HUD that is silently absent, which during a tuning
 *     session reads as "the plugin is broken" rather than "the class is unset".
 *
 *  3. It outlives the pawn. Binding to a specific pawn class would break the moment the
 *     debug pawn is replaced by a ship. This resolves the possessed pawn per frame
 *     through the player controller, which is the same resolution
 *     AProceduralSpaceActor::GetPlayerLocation does, so the HUD and the field are always
 *     looking at the same pawn -- including the frame a repossession happens.
 *
 *  DRAWING GOES THROUGH UDebugDrawService, the same mechanism the engine's own gameplay
 *  debugger uses. It hands us a UCanvas after the scene is composited and is gated by the
 *  "Game" show flag, so it costs nothing in a shipping build with that flag off and
 *  nothing here when the cvar is off.
 *
 *  EVERYTHING ON SCREEN IS A READ. Positions come from AUniverseActor via
 *  ULSSampleUniverseNav; speeds are time derivatives of two values in that sample. This
 *  class computes no field quantity and must not start to -- see the note on
 *  FULSNavSample. */
UCLASS()
class ULTRALARGESCALE_API UULSNavHudSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	/** UDebugDrawService callback. Once per rendered viewport per frame. */
	void OnDebugDraw(UCanvas* InCanvas, APlayerController* InPC);

	/** Finds (and caches) the universe actor in this world. Re-searches only when the
	 *  cached pointer has gone stale, so the common case is a weak-pointer deref. */
	AUniverseActor* ResolveUniverse();

	/** Differences the sample against last frame and folds the result into the smoothed
	 *  speeds. Frame-guarded by the caller; see LastSampledFrame. */
	void UpdateTelemetry(const FULSNavSample& InSample, double InDeltaSeconds);

	/** Drops accumulated history. Called on a discontinuity (teleport, repossession, the
	 *  universe becoming Ready) so a one-frame jump does not smear across the next
	 *  second of readout. */
	void ResetTelemetry();

	TWeakObjectPtr<AUniverseActor> CachedUniverse;

	/** UDebugDrawService fires once per viewport, and a differencing readout that runs
	 *  twice in one frame halves its own dt. Guarded on the frame counter rather than on
	 *  viewport identity because the extra viewports want the same numbers, not their
	 *  own. */
	uint64 LastSampledFrame = 0;

	bool    bHasHistory = false;
	FVector PrevVirtualTraversal = FVector::ZeroVector;
	FVector PrevFrameOfReferenceWorld = FVector::ZeroVector;

	/** Smoothed magnitudes, cm/s. Display-only smoothing; see the tau note in the .cpp. */
	double SmoothedRealCmPerSec = 0.0;
	double SmoothedCompressedCmPerSec = 0.0;

	FDelegateHandle DebugDrawHandle;
};

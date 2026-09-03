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

/** Draws the diagnostic navigation readout: position, speed and streaming state, sampled
 *  once per frame via ULSSampleUniverseNav.
 *
 *  A world subsystem rather than an AHUD, so it needs no project setup, leaves the single
 *  per-GameMode HUD slot free for the player-facing HUD, and resolves the possessed pawn per
 *  frame instead of binding to a pawn class.
 *
 *  Drawing goes through UDebugDrawService, which supplies a UCanvas after the scene is
 *  composited and is gated by the "Game" show flag. Everything drawn is a read from
 *  FULSNavSample; this class computes no field quantity. */
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

	/** Finds and caches the universe actor in this world, re-searching only when the cached
	 *  pointer has gone stale. */
	AUniverseActor* ResolveUniverse();

	/** Differences the sample against last frame and folds the result into the smoothed
	 *  speeds. Frame-guarded by the caller; see LastSampledFrame. */
	void UpdateTelemetry(const FULSNavSample& InSample, double InDeltaSeconds);

	/** Drops accumulated history. Call on a discontinuity -- teleport, repossession, the
	 *  universe becoming Ready -- so a one-frame jump does not smear across the readout. */
	void ResetTelemetry();

	TWeakObjectPtr<AUniverseActor> CachedUniverse;

	/** Guards UpdateTelemetry to one call per frame. UDebugDrawService fires once per
	 *  viewport, and a differencing readout that runs twice halves its own dt. */
	uint64 LastSampledFrame = 0;

	bool    bHasHistory = false;
	FVector PrevVirtualTraversal = FVector::ZeroVector;
	FVector PrevFrameOfReferenceWorld = FVector::ZeroVector;

	/** Smoothed magnitudes, cm/s. Display-only smoothing; see the tau note in the .cpp. */
	double SmoothedRealCmPerSec = 0.0;
	double SmoothedCompressedCmPerSec = 0.0;

	FDelegateHandle DebugDrawHandle;
};
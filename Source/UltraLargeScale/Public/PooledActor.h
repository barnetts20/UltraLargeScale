// PooledActor.h
#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "PooledActor.generated.h"

/** Reflection stub for IPooledActor. MinimalAPI: only the type info is exported;
 *  the behavior lives on the C++ IPooledActor below (plain virtual, no UFUNCTIONs,
 *  so calls go through Cast<IPooledActor>() rather than Execute_*). */
UINTERFACE(MinimalAPI)
class UPooledActor : public UInterface
{
    GENERATED_BODY()
};

/**
 * Thin per-type runtime contract for actors managed by UActorPoolManager.
 * The manager itself is fully generic (it only ever touches AActor*); the two
 * genuinely type-specific operations — waking an instance and tearing it down
 * with its child cascade — live here so the manager never needs a class per type.
 *
 * Contract with UActorPoolManager:
 *   AcquireByClass()  pops an inert instance, calls OnAcquired(),  hands it back.
 *   Release()         calls OnReturnToPool(), applies generic inert, re-pools it.
 *
 * Placement/visibility is deliberately NOT part of this contract: the async-init
 * space layers defer world placement to their parent's FinalizeXxxPlacement (it
 * needs the frame's resolved VirtualTraversal, which isn't known at acquire time),
 * so OnAcquired must not unhide or position the instance for those layers.
 */
class ULTRALARGESCALE_API IPooledActor
{
    GENERATED_BODY()

public:
    /** Generic wake. Called after the instance is popped and before it is returned
     *  to the acquiring parent. Re-arm lifecycle/per-frame state and RE-STAMP any
     *  per-acquire traits (backdrop-capture flag, scale space) so a recycled
     *  instance never inherits a previous occupant's flags. Do NOT unhide/position
     *  here — the parent's deferred finalize owns that. */
    virtual void OnAcquired() {}

    /** Wake + reconfigure for a proxy-CARRIED body that needs its world radius
     *  (e.g. a voxel planet sizing/rebuilding itself). Called by the WRAPPING PROXY,
     *  never by the manager; default no-op for the manager-pooled layers that aren't
     *  carried. Bodies should skip a rebuild when the built radius already matches.
     *  Made non-pure (with OnAcquired() above) so each implementer defines only the
     *  one entry point it uses -- no dead stub on either side. */
    virtual void OnAcquired(double WorldRadius) {}

    /** Teardown + cascade-release. Called before the manager's generic inert step.
     *  Release this instance's own live children (via its SpawnedX map) and clear
     *  generator/tier state, so no live association leaks into an inert instance. */
    virtual void OnReturnToPool() = 0;
};

/** Reflection stub for IStarLit. */
UINTERFACE(MinimalAPI)
class UStarLit : public UInterface
{
    GENERATED_BODY()
};

/**
 * A proxy-carried body (e.g. a voxel planet) that orients its lighting toward its
 * system's star. The star system pushes the star's parallax-resolved world position
 * each frame; the body aims its atmosphere's directional light + raymarch at it.
 *
 * NOTE: this interface exists only so the star system (ULTRALARGESCALE) can call into
 * a planet (VOXELPLUGIN) across the module boundary. Once the actor modules are merged
 * it can be deleted and replaced with a direct Cast<APlanetActor>. Parked here rather
 * than in its own file precisely because it's temporary.
 */
class ULTRALARGESCALE_API IStarLit
{
    GENERATED_BODY()

public:
    /** Current world-space position of the system's star, already parallax-resolved. */
    virtual void SetStarWorldPosition(const FVector& StarWorldPos) = 0;
};
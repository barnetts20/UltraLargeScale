// PooledActor.h
#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "PooledActor.generated.h"

/** Reflection stub for IPooledActor. MinimalAPI: type info only, so calls go through
 *  Cast<IPooledActor>() rather than Execute_*. */
UINTERFACE(MinimalAPI)
class UPooledActor : public UInterface
{
    GENERATED_BODY()
};

/**
 * Per-type runtime contract for actors managed by UActorPoolManager, which is otherwise
 * generic and only ever touches AActor*.
 *
 *   AcquireByClass()  pops an inert instance, calls OnAcquired(), hands it back.
 *   Release()         calls OnReturnToPool(), applies generic inert, re-pools it.
 */
class ULTRALARGESCALE_API IPooledActor
{
    GENERATED_BODY()

public:
    /** Generic wake, called after the instance is popped and before it reaches the acquiring
     *  parent. Re-arm lifecycle state and re-stamp per-acquire traits (backdrop-capture flag,
     *  scale space) so a recycled instance never inherits a previous occupant's. MUST NOT
     *  unhide or position: the parent's deferred finalize owns that. */
    virtual void OnAcquired() {}

    /** Wake for a proxy-CARRIED body that needs its world radius, such as a voxel planet
     *  sizing itself. Called by the wrapping proxy, never by the manager; skip a rebuild when
     *  the built radius already matches. MUST UNHIDE, unlike the overload above -- the proxy
     *  has already positioned the body, so nothing else will. */
    virtual void OnAcquired(double WorldRadius) {}

    /** Teardown and cascade-release, called before the manager's generic inert step. Release
     *  this instance's live children and clear generator and tier state, so no live
     *  association leaks into an inert instance. */
    virtual void OnReturnToPool() = 0;
};

/** Reflection stub for IStarLit. */
UINTERFACE(MinimalAPI)
class UStarLit : public UInterface
{
    GENERATED_BODY()
};

/**
 * A proxy-carried body that orients its lighting toward its system's star. The star system
 * pushes the star's parallax-resolved world position each frame; the body aims its
 * atmosphere's directional light and raymarch at it.
 */
class ULTRALARGESCALE_API IStarLit
{
    GENERATED_BODY()

public:
    /** Current world-space position of the system's star, already parallax-resolved. */
    virtual void SetStarWorldPosition(const FVector& StarWorldPos) = 0;
};
// ActorPoolManager.h
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Templates/SubclassOf.h"
#include "ActorPoolManager.generated.h"

/** One inert sub-pool, keyed in UActorPoolManager::SubPools by the spawn UClass.
 *  USTRUCT so the Inert array can be a UPROPERTY (GC-visible) inside the TMap. */
USTRUCT()
struct FActorSubPool
{
    GENERATED_BODY()

    /** Parked, inert instances ready to acquire. Managed as a stack. */
    UPROPERTY()
    TArray<TObjectPtr<AActor>> Inert;

    /** Target prewarm count for this type; set by RegisterType, consumed by PrewarmAll.
     *  Non-reflected: it's plain config, never serialized. */
    int32 Prewarm = 0;
};

/**
 * Central, generic actor pool. One UActorPoolManager instance is owned by the
 * single permanent AUniverseActor; every layer reaches it by resolving up the
 * parent chain (GetPoolManager()), exactly like GetEffectiveSpeedScale.
 *
 * The manager is fully generic — it only ever touches AActor* and the UClass
 * identity. All per-type behavior (seed->config, typed ReInit, wake/teardown)
 * lives elsewhere: proc-gen on the F*ParamBounds structs, ReInit on the typed
 * parent's call, wake/teardown on IPooledActor. Adding a new poolable type is a
 * UClass + RegisterType + a ReInit/bounds — no manager change.
 */
UCLASS()
class ULTRALARGESCALE_API UActorPoolManager : public UObject
{
    GENERATED_BODY()

public:
    /** Register a spawnable type and its prewarm target. Idempotent: re-registering
     *  updates the prewarm count without discarding already-pooled instances.
     *  Register the CONCRETE spawn class (the same token later passed to Acquire). */
    void RegisterType(TSubclassOf<AActor> Class, int32 Prewarm);

    /** Fill every registered sub-pool up to its prewarm target with inert blanks.
     *  Call from Universe BeginPlay, before any child activates. Caches World for
     *  grow-on-exhaust. Top-up semantics: safe to call again (won't double-spawn). */
    void PrewarmAll(UWorld* World);

    /** Pop an inert instance of Class (grow + warn if the sub-pool is empty), wake it
     *  via IPooledActor::OnAcquired, and return it. Association (parent pointer,
     *  Params via ReInit) is the caller's job. */
    AActor* AcquireByClass(TSubclassOf<AActor> Class);

    /** Tear down an instance (IPooledActor::OnReturnToPool -> generic inert) and push
     *  it back onto its sub-pool. Keyed on Actor->GetClass(). */
    void Release(AActor* Actor);

    /** Re-pool an actor whose teardown the caller has ALREADY performed (a layer
     *  with an async pre-teardown the generic Release can't own): applies generic
     *  inert + re-pools, WITHOUT calling OnReturnToPool again.
     *  Release() == OnReturnToPool() + ReturnPrepared(). */
    void ReturnPrepared(AActor* Actor);

    /** Typed convenience wrapper. T::StaticClass() must have been registered. */
    template<typename T>
    T* Acquire() { return Cast<T>(AcquireByClass(T::StaticClass())); }

    /** Live/inert diagnostics for the dev HUD. Inert count for a registered type. */
    int32 NumInert(TSubclassOf<AActor> Class) const;

    /** Destroy every inert instance and clear the pools. Call from Universe EndPlay. */
    void Shutdown();

    /** UObject world hook so engine utilities resolve a world off the manager. */
    virtual UWorld* GetWorld() const override { return CachedWorld.Get(); }

private:
    /** Spawn one blank of Class and drop it straight into the inert state. */
    AActor* SpawnInert(TSubclassOf<AActor> Class);

    /** Generic quiet: hide, collision off, tick off, park at origin, deactivate any
     *  Niagara components. The typed OnReturnToPool has already run before this. */
    static void ApplyInert(AActor* Actor);

    UPROPERTY()
    TMap<TSubclassOf<AActor>, FActorSubPool> SubPools;

    /** World captured in PrewarmAll, used to spawn grow-on-exhaust blanks. */
    TWeakObjectPtr<UWorld> CachedWorld;
};
// ActorPoolManager.h
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Templates/SubclassOf.h"
#include "ActorPoolManager.generated.h"

/** One inert sub-pool, keyed in UActorPoolManager::SubPools by the spawn UClass. USTRUCT so
 *  the Inert array can be a GC-visible UPROPERTY inside the TMap. */
USTRUCT()
struct FActorSubPool
{
    GENERATED_BODY()

    /** Parked, inert instances ready to acquire. Managed as a stack. */
    UPROPERTY()
    TArray<TObjectPtr<AActor>> Inert;

    /** Target prewarm count for this type; set by RegisterType, consumed by PrewarmAll. */
    int32 Prewarm = 0;
};

/**
 * Central, generic actor pool. One instance is owned by the permanent AUniverseActor; every
 * layer reaches it by resolving up the parent chain via GetPoolManager(). The manager only
 * ever touches AActor* and the UClass identity -- per-type behaviour lives on the
 * F*ParamBounds structs, the typed parent's ReInit, and IPooledActor.
 */
UCLASS()
class ULTRALARGESCALE_API UActorPoolManager : public UObject
{
    GENERATED_BODY()

public:
    /** Register a spawnable type and its prewarm target. Idempotent: re-registering updates
     *  the count without discarding pooled instances. Pass the CONCRETE spawn class. */
    void RegisterType(TSubclassOf<AActor> Class, int32 Prewarm);

    /** Fill every registered sub-pool up to its prewarm target with inert blanks, and cache
     *  World for grow-on-exhaust. Call from Universe BeginPlay, before any child activates;
     *  tops up rather than refilling, so it is safe to call again. */
    void PrewarmAll(UWorld* World);

    /** Pop an inert instance of Class (grow and warn if the sub-pool is empty), wake it via
     *  IPooledActor::OnAcquired, and return it. Association is the caller's job. */
    AActor* AcquireByClass(TSubclassOf<AActor> Class);

    /** Tear down an instance (OnReturnToPool, then generic inert) and push it back onto its
     *  sub-pool. Keyed on Actor->GetClass(). */
    void Release(AActor* Actor);

    /** Re-pool an actor whose teardown the caller has ALREADY performed: applies generic
     *  inert and re-pools WITHOUT calling OnReturnToPool again, for layers with an async
     *  pre-teardown Release cannot own. Release() == OnReturnToPool() + ReturnPrepared(). */
    void ReturnPrepared(AActor* Actor);

    /** Typed convenience wrapper; T::StaticClass() must have been registered. Keys on the
     *  C++ class, so it matches AcquireByClass only when the registered token IS
     *  T::StaticClass() -- registering a Blueprint subclass instead splits the two keys. */
    template<typename T>
    T* Acquire() { return Cast<T>(AcquireByClass(T::StaticClass())); }

    /** Inert count for a registered type. Diagnostics for the dev HUD. */
    int32 NumInert(TSubclassOf<AActor> Class) const;

    /** Destroy every INERT instance and clear the pools. Call from Universe EndPlay. Live
     *  instances are neither tracked nor destroyed: an acquired actor belongs to whoever
     *  acquired it until released. */
    void Shutdown();

    /** UObject world hook so engine utilities resolve a world off the manager. */
    virtual UWorld* GetWorld() const override { return CachedWorld.Get(); }

private:
    /** Spawn one blank of Class and drop it straight into the inert state. */
    AActor* SpawnInert(TSubclassOf<AActor> Class);

    /** Generic quiet: hide, collision off, tick off, park at origin, deactivate Niagara
     *  components. The typed OnReturnToPool runs before this. */
    static void ApplyInert(AActor* Actor);

    UPROPERTY()
    TMap<TSubclassOf<AActor>, FActorSubPool> SubPools;

    /** World captured in PrewarmAll, used to spawn grow-on-exhaust blanks. */
    TWeakObjectPtr<UWorld> CachedWorld;
};
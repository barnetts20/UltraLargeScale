// ActorPoolManager.cpp
#include "ActorPoolManager.h"
#include "PooledActor.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "NiagaraComponent.h"

void UActorPoolManager::RegisterType(TSubclassOf<AActor> Class, int32 Prewarm)
{
    if (!Class)
    {
        UE_LOG(LogTemp, Warning, TEXT("[Pool] RegisterType called with null class"));
        return;
    }
    // FindOrAdd so re-registration only updates the target and never discards
    // instances already pooled under this key.
    FActorSubPool& Pool = SubPools.FindOrAdd(Class);
    Pool.Prewarm = FMath::Max(0, Prewarm);
}

AActor* UActorPoolManager::SpawnInert(TSubclassOf<AActor> Class)
{
    UWorld* World = CachedWorld.Get();
    if (!World || !Class) return nullptr;

    // Blank spawn only — no association, no Params, no Initialize(). The manager
    // stays generic; the acquiring parent stamps identity via ReInit.
    AActor* Actor = World->SpawnActor<AActor>(Class, FVector::ZeroVector, FRotator::ZeroRotator);
    if (!Actor) return nullptr;

    ApplyInert(Actor);
    return Actor;
}

void UActorPoolManager::PrewarmAll(UWorld* World)
{
    CachedWorld = World;
    if (!World) return;

    for (TPair<TSubclassOf<AActor>, FActorSubPool>& Entry : SubPools)
    {
        FActorSubPool& Pool = Entry.Value;
        // Top-up rather than blind-fill: PrewarmAll may be called more than once
        // and deferred returns can transiently leave a pool above target.
        const int32 ToSpawn = Pool.Prewarm - Pool.Inert.Num();
        for (int32 i = 0; i < ToSpawn; ++i)
        {
            if (AActor* Blank = SpawnInert(Entry.Key))
                Pool.Inert.Add(Blank);
        }
    }
}

AActor* UActorPoolManager::AcquireByClass(TSubclassOf<AActor> Class)
{
    if (!Class) return nullptr;

    FActorSubPool* Pool = SubPools.Find(Class);
    if (!Pool)
    {
        // Never fail silently: an unregistered acquire is a wiring bug. Register
        // it on the fly (prewarm 0) so the pool still functions, and warn.
        UE_LOG(LogTemp, Warning,
            TEXT("[Pool] AcquireByClass on unregistered type %s; registering with prewarm 0"),
            *Class->GetName());
        Pool = &SubPools.Add(Class);
    }

    AActor* Actor = nullptr;
    if (Pool->Inert.Num() > 0)
    {
        Actor = Pool->Inert.Pop(EAllowShrinking::No);
    }
    else
    {
        // Empty-pool policy: grow + warn, never fail silently (design invariant).
        UE_LOG(LogTemp, Warning,
            TEXT("[Pool] %s exhausted; growing by 1 (raise its prewarm count)"),
            *Class->GetName());
        Actor = SpawnInert(Class);
    }
    if (!Actor) return nullptr;

    // Generic wake. Placement/visibility stays with the parent's deferred finalize.
    if (IPooledActor* Pooled = Cast<IPooledActor>(Actor))
        Pooled->OnAcquired();

    return Actor;
}

void UActorPoolManager::Release(AActor* Actor)
{
    if (!Actor) return;

    // Typed teardown first: cascade-release children + clear generator/tier state
    // BEFORE the instance is parked, or live children leak into an inert parent.
    if (IPooledActor* Pooled = Cast<IPooledActor>(Actor))
        Pooled->OnReturnToPool();

    ReturnPrepared(Actor);
}

void UActorPoolManager::ReturnPrepared(AActor* Actor)
{
    if (!Actor) return;

    ApplyInert(Actor);

    // Key on the concrete class. Registration + Acquire use the same token, so this
    // lands in the matching sub-pool; FindOrAdd covers an unregistered stray.
    FActorSubPool& Pool = SubPools.FindOrAdd(Actor->GetClass());
    Pool.Inert.Add(Actor);
}

void UActorPoolManager::ApplyInert(AActor* Actor)
{
    if (!Actor) return;
    Actor->SetActorHiddenInGame(true);
    Actor->SetActorEnableCollision(false);
    Actor->SetActorTickEnabled(false);
    // Park away from any live location so a stale inert instance can't be mistaken
    // for a placed one; the parent repositions on the next acquire/finalize anyway.
    Actor->SetActorLocation(FVector::ZeroVector);

    // Belt-and-suspenders GPU quiet: the typed OnReturnToPool/ResetForPool already
    // tears down tier Niagara, but a blank prewarmed instance never ran that path.
    TInlineComponentArray<UNiagaraComponent*> NiagaraComps(Actor);
    for (UNiagaraComponent* NC : NiagaraComps)
        if (NC) NC->Deactivate();
}

int32 UActorPoolManager::NumInert(TSubclassOf<AActor> Class) const
{
    const FActorSubPool* Pool = SubPools.Find(Class);
    return Pool ? Pool->Inert.Num() : 0;
}

void UActorPoolManager::Shutdown()
{
    for (TPair<TSubclassOf<AActor>, FActorSubPool>& Entry : SubPools)
    {
        for (TObjectPtr<AActor>& Actor : Entry.Value.Inert)
            if (Actor) Actor->Destroy();
        Entry.Value.Inert.Reset();
    }
    SubPools.Reset();
    CachedWorld.Reset();
}
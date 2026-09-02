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
    FActorSubPool& Pool = SubPools.FindOrAdd(Class);
    Pool.Prewarm = FMath::Max(0, Prewarm);
}

AActor* UActorPoolManager::SpawnInert(TSubclassOf<AActor> Class)
{
    UWorld* World = CachedWorld.Get();
    if (!World || !Class) return nullptr;

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
        const int32 ToSpawn = Pool.Prewarm - Pool.Inert.Num();

        for (int32 i = 0; i < ToSpawn; ++i)
            if (AActor* Blank = SpawnInert(Entry.Key)) Pool.Inert.Add(Blank);
    }
}

AActor* UActorPoolManager::AcquireByClass(TSubclassOf<AActor> Class)
{
    if (!Class) return nullptr;

    FActorSubPool* Pool = SubPools.Find(Class);
    if (!Pool)
    {
        UE_LOG(LogTemp, Warning, TEXT("[Pool] AcquireByClass on unregistered type %s; registering with prewarm 0"), *Class->GetName());
        Pool = &SubPools.Add(Class);
    }

    AActor* Actor = nullptr;
    if (Pool->Inert.Num() > 0)
    {
        Actor = Pool->Inert.Pop(EAllowShrinking::No);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[Pool] %s exhausted; growing by 1 (raise its prewarm count)"), *Class->GetName());
        Actor = SpawnInert(Class);
    }
    if (!Actor) return nullptr;

    if (IPooledActor* Pooled = Cast<IPooledActor>(Actor)) Pooled->OnAcquired();

    return Actor;
}

void UActorPoolManager::Release(AActor* Actor)
{
    if (!Actor) return;

    if (IPooledActor* Pooled = Cast<IPooledActor>(Actor)) Pooled->OnReturnToPool();

    ReturnPrepared(Actor);
}

void UActorPoolManager::ReturnPrepared(AActor* Actor)
{
    if (!Actor) return;

    ApplyInert(Actor);

    FActorSubPool& Pool = SubPools.FindOrAdd(Actor->GetClass());

    Pool.Inert.Add(Actor);
}

void UActorPoolManager::ApplyInert(AActor* Actor)
{
    if (!Actor) return;
    Actor->SetActorHiddenInGame(true);
    Actor->SetActorEnableCollision(false);
    Actor->SetActorTickEnabled(false);
    Actor->SetActorLocation(FVector::ZeroVector);

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
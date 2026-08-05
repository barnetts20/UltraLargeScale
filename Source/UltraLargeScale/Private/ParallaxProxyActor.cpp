#include "ParallaxProxyActor.h"
#include "UltraLargeScale.h"
#include "Engine/World.h"
#include "PooledActor.h"

AParallaxProxyActor::AParallaxProxyActor()
{
    PrimaryActorTick.bCanEverTick = false;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AParallaxProxyActor::SetupWrapped(UClass* WrappedClass, double WorldRadius)
{
    if (!WrappedClass) return;
    UWorld* World = GetWorld();
    if (!World) return;

    FActorSpawnParameters SP;
    SP.Owner = this;
    SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    Wrapped = World->SpawnActor<AActor>(WrappedClass, GetActorLocation(), FRotator::ZeroRotator, SP);
    if (!Wrapped)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("AParallaxProxyActor::SetupWrapped - failed to spawn %s"), *WrappedClass->GetName());
        return;
    }

    if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnAcquired(WorldRadius);
    else Wrapped->SetActorScale3D(FVector(WorldRadius));
}

void AParallaxProxyActor::TickParallax(float DeltaTime, const FVector& PlayerPos, double InSpeedScale)
{
    SVO_GT_SCOPE("Planet::TickParallax");

    SpeedScale = InSpeedScale;

    if (!bHasLastPlayer) { LastPlayerPos = PlayerPos; bHasLastPlayer = true; }

    const FVector PlayerDelta = PlayerPos - LastPlayerPos;
    LastPlayerPos = PlayerPos;

    VirtualTraversal += PlayerDelta * InSpeedScale;

    if (Wrapped)
    {
        const FVector NewLoc = PlayerPos - VirtualTraversal;
        if (!Wrapped->GetActorLocation().Equals(NewLoc, 1.0))
            Wrapped->SetActorLocation(NewLoc);
    }
}

void AParallaxProxyActor::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Wrapped) { Wrapped->Destroy(); Wrapped = nullptr; }
    Super::EndPlay(Reason);
}

void AParallaxProxyActor::OnAcquired()
{
    bHasLastPlayer = false;
}

void AParallaxProxyActor::OnReturnToPool()
{
    if (Wrapped)
    {
        if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnReturnToPool();
        else { Wrapped->SetActorHiddenInGame(true); Wrapped->SetActorTickEnabled(false); }
    }
    VirtualTraversal = FVector::ZeroVector;
    LastPlayerPos = FVector::ZeroVector;
    bHasLastPlayer = false;
    SetActorHiddenInGame(true);
}

void AParallaxProxyActor::ReInit(UClass* BodyClass, double WorldRadius, const FVector& SpawnLoc, const FVector& InitialVT)
{
    SetActorLocation(SpawnLoc);
    VirtualTraversal = InitialVT;
    LastPlayerPos = FVector::ZeroVector;
    bHasLastPlayer = false;
    SetActorHiddenInGame(false);

    if (!Wrapped || Wrapped->GetClass() != BodyClass)
    {
        if (Wrapped) { Wrapped->Destroy(); Wrapped = nullptr; }
        SetupWrapped(BodyClass, WorldRadius);   // spawns at proxy loc + hands over radius
    }
    else
    {
        Wrapped->SetActorLocation(SpawnLoc);
        if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnAcquired(WorldRadius);
        else { Wrapped->SetActorHiddenInGame(false); Wrapped->SetActorScale3D(FVector(WorldRadius)); }
    }
}
#include "ParallaxProxyActor.h"
#include "UltraLargeScale.h"
#include "Engine/World.h"
#include "PooledActor.h"

AParallaxProxyActor::AParallaxProxyActor()
{
    // Driven by AStarSystemActor's tick loop, not UE's per-actor dispatch �
    // self-ticking would fight the layer model, same as the static-mesh body.
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

    // Proxy is spawned AT SpawnLoc, so spawn the wrapped actor there too �
    // it's already on its mark before the first TickParallax runs.
    Wrapped = World->SpawnActor<AActor>(WrappedClass, GetActorLocation(), FRotator::ZeroRotator, SP);
    if (!Wrapped)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("AParallaxProxyActor::SetupWrapped - failed to spawn %s"), *WrappedClass->GetName());
        return;
    }

    // Size via scale. For a voxel planet this IS the radius (cm) and it self-
    // initializes from scale on its first tick. Set ONCE � a planet rebuilds its
    // terrain on scale change, so never rescale per frame; we only move it.
    // Hand the body its world radius. A pooled body (IPooledActor) takes it via
    // OnAcquired(double) so it can reconfigure/rebuild; a plain body just scales.
    if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnAcquired(WorldRadius);
    else Wrapped->SetActorScale3D(FVector(WorldRadius));
}

void AParallaxProxyActor::TickParallax(float DeltaTime, const FVector& PlayerPos, double InSpeedScale)
{
    SVO_GT_SCOPE("Planet::TickParallax");

    SpeedScale = InSpeedScale;   // stored live; the meshing gate reads this

    if (!bHasLastPlayer) { LastPlayerPos = PlayerPos; bHasLastPlayer = true; }

    const FVector PlayerDelta = PlayerPos - LastPlayerPos;
    LastPlayerPos = PlayerPos;

    // Same accumulation as every layer: VT += PlayerDelta * (SpeedScale / UnitScale).
    // UnitScale is fixed at 1 here, so speed scale is the whole numerator. VT keeps
    // accumulating even at SpeedScale == 1 so the math stays continuous across speed
    // changes -- but the rendered position is then constant.
    VirtualTraversal += PlayerDelta * InSpeedScale;

    // Content renders at PlayerPos + (LocalOrigin - VT); the wrapped actor sits at the
    // layer origin, so it renders at PlayerPos - VT. Skip the push when the position
    // hasn't meaningfully changed (SpeedScale == 1 parks the actor, and a stationary
    // player never moves it) -- this avoids re-triggering the wrapped actor's
    // transform-driven LOD churn while it sits still.
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
    // Real wake (position + body reconfigure) happens in ReInit, which the star system
    // calls right after acquire. Nothing generic to do here beyond a state reset.
    bHasLastPlayer = false;
}

void AParallaxProxyActor::OnReturnToPool()
{
    // Park the wrapped body dormant (no tick/LOD/raymarch/ocean while pooled) WITHOUT
    // destroying it -- keeping it warm is the whole point. A body that doesn't implement
    // IPooledActor (the sprite placeholder) just gets hidden + tick-disabled.
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
    VirtualTraversal = InitialVT;   // positional seed so frame 0 renders on the planet's spot
    LastPlayerPos = FVector::ZeroVector;
    bHasLastPlayer = false;
    SetActorHiddenInGame(false);

    if (!Wrapped || Wrapped->GetClass() != BodyClass)
    {
        // First use, or a different body class: (re)create the wrapped body.
        if (Wrapped) { Wrapped->Destroy(); Wrapped = nullptr; }
        SetupWrapped(BodyClass, WorldRadius);   // spawns at proxy loc + hands over radius
    }
    else
    {
        // Reuse the persistent pooled body: reposition + wake/reconfigure.
        Wrapped->SetActorLocation(SpawnLoc);
        if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnAcquired(WorldRadius);
        else { Wrapped->SetActorHiddenInGame(false); Wrapped->SetActorScale3D(FVector(WorldRadius)); }
    }
}
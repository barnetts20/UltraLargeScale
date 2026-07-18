#include "ParallaxProxyActor.h"
#include "UltraLargeScale.h"
#include "Engine/World.h"

AParallaxProxyActor::AParallaxProxyActor()
{
    // Driven by AStarSystemActor's tick loop, not UE's per-actor dispatch —
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

    // Proxy is spawned AT SpawnLoc, so spawn the wrapped actor there too —
    // it's already on its mark before the first TickParallax runs.
    Wrapped = World->SpawnActor<AActor>(WrappedClass, GetActorLocation(), FRotator::ZeroRotator, SP);
    if (!Wrapped)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("AParallaxProxyActor::SetupWrapped - failed to spawn %s"), *WrappedClass->GetName());
        return;
    }

    // Size via scale. For a voxel planet this IS the radius (cm) and it self-
    // initializes from scale on its first tick. Set ONCE — a planet rebuilds its
    // terrain on scale change, so never rescale per frame; we only move it.
    Wrapped->SetActorScale3D(FVector(WorldRadius));
}

void AParallaxProxyActor::TickParallax(float DeltaTime, const FVector& PlayerPos, double InSpeedScale)
{
    SVO_GT_SCOPE("Planet::TickParallax");

    SpeedScale = InSpeedScale;   // stored live; the future meshing gate reads this
    if (!bHasLastPlayer) { LastPlayerPos = PlayerPos; bHasLastPlayer = true; }

    const FVector PlayerDelta = PlayerPos - LastPlayerPos;
    LastPlayerPos = PlayerPos;

    // Same accumulation as every layer: VT += PlayerDelta * (SpeedScale / UnitScale).
    // UnitScale is fixed at 1 here, so speed scale is the whole numerator.
    VirtualTraversal += PlayerDelta * InSpeedScale;

    // Content renders at PlayerPos + (LocalOrigin - VT); the wrapped actor sits
    // at the layer origin, so it renders at PlayerPos - VT.
    if (Wrapped)
        Wrapped->SetActorLocation(PlayerPos - VirtualTraversal);
}

void AParallaxProxyActor::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Wrapped) { Wrapped->Destroy(); Wrapped = nullptr; }
    Super::EndPlay(Reason);
}
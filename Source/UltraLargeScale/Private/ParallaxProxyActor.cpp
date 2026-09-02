#include "ParallaxProxyActor.h"
#include "UltraLargeScale.h"
#include "Engine/World.h"
#include "PooledActor.h"

AParallaxProxyActor::AParallaxProxyActor()
{
    PrimaryActorTick.bCanEverTick = false;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

// THE WRAPPED ACTOR IS OWNED BUT NOT ATTACHED, which is deliberate and worth stating
// because two things downstream depend on it. TickParallax sets the body's world location
// directly each frame rather than moving the proxy, so the body keeps full real-space
// precision independent of the proxy transform. And UActorPoolManager::ApplyInert parks
// THIS ACTOR at the origin on a pool return without moving the body -- harmless only
// because OnReturnToPool has already hidden it and ReInit repositions it before it is
// shown again.
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
        // THE BODY UNHIDES ITSELF ON THE IPooledActor PATH, and that asymmetry is the one
        // thing to check first if a reused planet goes invisible. The fallback branch below
        // unhides explicitly, and the SetupWrapped path above spawns a fresh actor, which is
        // visible by default -- so this is the only route where visibility is delegated.
        // OnAcquired(double) carries that requirement; see the note on it in PooledActor.h.
        Wrapped->SetActorLocation(SpawnLoc);
        if (IPooledActor* Body = Cast<IPooledActor>(Wrapped)) Body->OnAcquired(WorldRadius);
        else { Wrapped->SetActorHiddenInGame(false); Wrapped->SetActorScale3D(FVector(WorldRadius)); }
    }
}
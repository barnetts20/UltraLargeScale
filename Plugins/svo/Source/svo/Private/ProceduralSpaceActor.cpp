// ProceduralSpaceActor.cpp
#include "ProceduralSpaceActor.h"
#include "Misc/ScopeExit.h"
#include <Kismet/GameplayStatics.h>

AProceduralSpaceActor::AProceduralSpaceActor()
{
    PrimaryActorTick.bCanEverTick = true;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
}

bool AProceduralSpaceActor::GetPlayerLocation(const UWorld* World, FVector& OutLocation)
{
    if (!World) return false;
    if (const auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
        if (const APawn* Pawn = Controller->GetPawn())
        {
            OutLocation = Pawn->GetActorLocation();
            return true;
        }
    return false;
}

void AProceduralSpaceActor::Initialize()
{
    InitializationState = ELifecycleState::Initializing;

    // Raised HERE (before dispatch) rather than inside the task so a
    // teardown that runs between dispatch and task start can never observe
    // a false flag while the chain is pending. For pool-managed actors
    // Initialize() runs on the GT, which is what makes the fast-path check
    // in ReturnGalaxyToPool / ReturnStarSystemToPool race-free.
    bInitInProgress.store(true);

    FVector PlayerPos;
    if (GetPlayerLocation(GetWorld(), PlayerPos))
    {
        LastFrameOfReferenceLocation = PlayerPos;
        CurrentFrameOfReferenceLocation = PlayerPos;
    }

    TWeakObjectPtr<AProceduralSpaceActor> WeakThis(this);
    AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
        {
            AProceduralSpaceActor* Self = WeakThis.Get();
            if (!Self) return;

            // Cleared on EVERY exit path — Pooling aborts included. The
            // deferred pool-return worker waits on this before freeing the
            // buffers the phases below write.
            ON_SCOPE_EXIT{ Self->bInitInProgress.store(false); };

            double StartTime = FPlatformTime::Seconds();

            Self->InitializeChildPool();
            if (Self->InitializationState == ELifecycleState::Pooling) return;

            Self->InitializeData();
            if (Self->InitializationState == ELifecycleState::Pooling) return;

            Self->InitializeVolumetric();
            if (Self->InitializationState == ELifecycleState::Pooling) return;

            Self->InitializeNiagara();
            if (Self->InitializationState == ELifecycleState::Pooling) return;

            Self->InitializationState = ELifecycleState::Ready;

            double TotalDuration = FPlatformTime::Seconds() - StartTime;
            UE_LOG(LogTemp, Log, TEXT("%s::Initialize total duration: %.3f seconds"),
                *Self->GetClass()->GetName(), TotalDuration);
        });
}

void AProceduralSpaceActor::ResetForSpawn()
{
    // Do NOT re-enable tick here. Pool-managed actors (Galaxy, StarSystem)
    // are driven by their parent's TickFromParent — enabling UE tick would
    // double-tick them. Level-placed actors that need UE tick should enable
    // it explicitly after ResetForSpawn.
    InitializationState = ELifecycleState::Uninitialized;
    bPendingPlacement = false;
    PendingNodeCenter = FVector::ZeroVector;
}

void AProceduralSpaceActor::ResetForPool()
{
    SetActorTickEnabled(false);
    double StartTime = FPlatformTime::Seconds();
    InitializationState = ELifecycleState::Pooling;
    bPendingPlacement = false;
    PendingNodeCenter = FVector::ZeroVector;

    if (VolumetricComponent)
    {
        VolumetricComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
        VolumetricComponent->DestroyComponent();
        VolumetricComponent = nullptr;
    }

    // Release the transient pseudo-volume texture + its MID so GC can
    // reclaim them while pooled — the texture is large (~67MB at 256^3)
    // and otherwise stays pinned by these UPROPERTYs until the next init
    // overwrites the pointers.
    PseudoVolumeTexture = nullptr;
    VolumeMaterial = nullptr;

    double Duration = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("%s::ResetForPool took: %.3f seconds"),
        *GetClass()->GetName(), Duration);
}

void AProceduralSpaceActor::InitializeData()
{
}

void AProceduralSpaceActor::InitializeVolumetric()
{
}

void AProceduralSpaceActor::InitializeNiagara()
{
}

void AProceduralSpaceActor::InitializeChildPool()
{
}

void AProceduralSpaceActor::DrawDebugBounds()
{
    if (Octree.IsValid() && InitializationState == ELifecycleState::Ready)
    {
        if (UWorld* World = GetWorld())
        {
            FVector ActorLocation = GetActorLocation();
            double WorldExtent = GetExtent();
            FVector BoxExtent(WorldExtent, WorldExtent, WorldExtent);

            DrawDebugBox(
                World,
                ActorLocation,
                BoxExtent,
                FColor::Purple,
                false,
                -1.0f,
                0,
                WorldExtent * 0.005f
            );

            DrawDebugCoordinateSystem(
                World,
                ActorLocation,
                FRotator::ZeroRotator,
                WorldExtent * 0.1f,
                false,
                -1.0f,
                0,
                WorldExtent * 0.001f
            );
        }
    }
}
// ProceduralSpaceActor.cpp
#include "ProceduralSpaceActor.h"
#include "Misc/ScopeExit.h"
#include <Kismet/GameplayStatics.h>
#include "FTierStreamingSystem.h"

AProceduralSpaceActor::AProceduralSpaceActor()
{
    PrimaryActorTick.bCanEverTick = true;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
}

#pragma region Shared State
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

#pragma endregion

#pragma region Lifecycle
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

            // Cleared on EVERY exit path, Pooling aborts included. The
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
    // are driven by their parent's TickFromParent; enabling UE tick would
    // double-tick them. Level-placed actors that need UE tick should enable
    // it explicitly after ResetForSpawn.
    InitializationState = ELifecycleState::Uninitialized;
    bPendingPlacement = false;
    PendingNodeCenter = FVector::ZeroVector;

    // Virtual-traversal cluster. All five live on this base, so they reset here:
    // resetting them from a subclass would leave LatestVT orphaned across the
    // pool round-trip. LatestVT is the lock-guarded snapshot async push /
    // boundary-cross tasks read via GetLatestVT; if it survives reuse, a
    // re-acquired actor composites its first frames against the previous
    // occupant's traversal, offsetting the streaming window.
    VirtualTraversal = FVector::ZeroVector;
    LastPushedVirtualTraversal = FVector::ZeroVector;
    LastFrameOfReferenceLocation = FVector::ZeroVector;
    CurrentFrameOfReferenceLocation = FVector::ZeroVector;
    PublishLatestVT(FVector::ZeroVector);   // clear the async snapshot under LatestVTGuard
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
    // reclaim them while pooled; the texture is large (~67MB at 256^3)
    // and otherwise stays pinned by these UPROPERTYs until the next init
    // overwrites the pointers.
    PseudoVolumeTexture = nullptr;
    VolumeMaterial = nullptr;

    double Duration = FPlatformTime::Seconds() - StartTime;
    UE_LOG(LogTemp, Log, TEXT("%s::ResetForPool took: %.3f seconds"),
        *GetClass()->GetName(), Duration);
}

#pragma endregion

#pragma region Initialization
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

#pragma endregion

#pragma region Parallax
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

FVector AProceduralSpaceActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
    // A child actor is physically larger than the node it spawns from (this
    // actor's Extent vs the node's Extent). To subtend the same angular size
    // from the camera, the child is placed proportionally further along the
    // same view vector:
    //
    //   distance_child / size_child = distance_node / size_node
    //   distance_child = distance_node * (size_child / size_node)
    //
    // ChildUnitScale encodes the node-to-child size relationship, so
    // size_child / size_node = GetUnitScale() / ChildUnitScale.
    const double SizeRatio = GetUnitScale() / ChildUnitScale;

    // World-space position of the node's sprite.
    const FVector RenderedPos = GetActorLocation() + NodeCenter - VirtualTraversal;

    // Vector from camera to the sprite, scaled by the size ratio to give the
    // child spawn position.
    const FVector CameraToNode = RenderedPos - CurrentFrameOfReferenceLocation;
    return CurrentFrameOfReferenceLocation + CameraToNode * SizeRatio;
}

#pragma endregion

#pragma region Tier System - Grid Coord Helpers
FIntVector AProceduralSpaceActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
    return FTierStreamingSystem::PositionToGridCoord(InPos, InGridDepth, GetExtent(), GridExtentMultiplier);
}

FVector AProceduralSpaceActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
    return FTierStreamingSystem::GridCoordToCenter(InCoord, InGridDepth, GetExtent(), GridExtentMultiplier);
}

double AProceduralSpaceActor::GetGridCellExtent(int32 InGridDepth) const
{
    return FTierStreamingSystem::GetGridCellExtent(InGridDepth, GetExtent(), GridExtentMultiplier);
}

#pragma endregion
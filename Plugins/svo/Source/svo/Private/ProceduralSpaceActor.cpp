// ProceduralSpaceActor.cpp
#include "ProceduralSpaceActor.h"
#include <Kismet/GameplayStatics.h>

AProceduralSpaceActor::AProceduralSpaceActor()
{
    PrimaryActorTick.bCanEverTick = true;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
}

void AProceduralSpaceActor::Initialize()
{
    InitializationState = ELifecycleState::Initializing;

    if (const auto* World = GetWorld())
    {
        if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
        {
            if (APawn* Pawn = Controller->GetPawn())
            {
                LastFrameOfReferenceLocation = Pawn->GetActorLocation();
            }
        }
    }

    AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
        {
            double StartTime = FPlatformTime::Seconds();

            InitializeChildPool();
            if (InitializationState == ELifecycleState::Pooling) return;

            InitializeData();
            if (InitializationState == ELifecycleState::Pooling) return;

            InitializeVolumetric();
            if (InitializationState == ELifecycleState::Pooling) return;

            InitializeNiagara();
            if (InitializationState == ELifecycleState::Pooling) return;

            InitializationState = ELifecycleState::Ready;

            double TotalDuration = FPlatformTime::Seconds() - StartTime;
            UE_LOG(LogTemp, Log, TEXT("%s::Initialize total duration: %.3f seconds"),
                *GetClass()->GetName(), TotalDuration);
        });
}

void AProceduralSpaceActor::ResetForSpawn()
{
    InitializationState = ELifecycleState::Uninitialized;
}

void AProceduralSpaceActor::ResetForPool()
{
    double StartTime = FPlatformTime::Seconds();
    InitializationState = ELifecycleState::Pooling;

    if (VolumetricComponent)
    {
        VolumetricComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
        VolumetricComponent->DestroyComponent();
        VolumetricComponent = nullptr;
    }

    if (NiagaraComponent)
    {
        NiagaraComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
        NiagaraComponent->DestroyComponent();
        NiagaraComponent = nullptr;
    }

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
    // Default implementation does nothing
    // Universe and Galaxy will override this
}

FVector AProceduralSpaceActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
    const double ChildParallaxRatio = GetParentSpeedScale() / ChildUnitScale;
    const double ParentParallaxRatio = GetParentSpeedScale() / GetUnitScale();
    FVector NodeWorldPosition = NodeCenter + GetActorLocation();
    FVector PlayerToNode = CurrentFrameOfReferenceLocation - NodeWorldPosition;
    return CurrentFrameOfReferenceLocation - PlayerToNode * (ChildParallaxRatio / ParentParallaxRatio);
}

void AProceduralSpaceActor::ApplyParallaxOffset()
{
    bool bHasReference = false;
    if (const auto* World = GetWorld())
    {
        if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
        {
            if (APawn* Pawn = Controller->GetPawn())
            {
                CurrentFrameOfReferenceLocation = Pawn->GetActorLocation();
                bHasReference = true;
            }
        }
    }

    if (!bHasReference)
    {
        UE_LOG(LogTemp, Warning, TEXT("Parallax: No valid reference camera or pawn found."));
        return;
    }

    double ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
    FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
    LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
    FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
    SetActorLocation(GetActorLocation() + ParallaxOffset);
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

void AProceduralSpaceActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    ApplyParallaxOffset();
    if (IsDebug) DrawDebugBounds();
}
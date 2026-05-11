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

    TWeakObjectPtr<AProceduralSpaceActor> WeakThis(this);
    AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
        {
            AProceduralSpaceActor* Self = WeakThis.Get();
            if (!Self) return;

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
    SetActorTickEnabled(true);
    InitializationState = ELifecycleState::Uninitialized;
}

void AProceduralSpaceActor::ResetForPool()
{
    SetActorTickEnabled(false);
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
    //const double ChildParallaxRatio = GetParentSpeedScale() / ChildUnitScale;
    //const double ParentParallaxRatio = GetParentSpeedScale() / GetUnitScale();
    const double ParallaxRatio = GetUnitScale() / ChildUnitScale;
    FVector NodeWorldPosition = NodeCenter + GetActorLocation();
    FVector PlayerToNode = CurrentFrameOfReferenceLocation - NodeWorldPosition;
    return CurrentFrameOfReferenceLocation - PlayerToNode * ParallaxRatio;
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

void AProceduralSpaceActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
    // Base implementation covers StarSystemActor: apply parallax offset using
    // the already-resolved player position passed down from the parent galaxy.
    // No controller lookup needed � InPlayerPos is authoritative for this frame.
    CurrentFrameOfReferenceLocation = InPlayerPos;
    const double ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
    const FVector PlayerOffset = InPlayerPos - LastFrameOfReferenceLocation;
    LastFrameOfReferenceLocation = InPlayerPos;
    SetActorLocation(GetActorLocation() + PlayerOffset * (1.0 - ParallaxRatio));
}
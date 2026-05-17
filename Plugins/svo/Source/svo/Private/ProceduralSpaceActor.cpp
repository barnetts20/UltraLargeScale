// ProceduralSpaceActor.cpp
#include "ProceduralSpaceActor.h"
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

    FVector PlayerPos;
    if (GetPlayerLocation(GetWorld(), PlayerPos))
    {
        LastFrameOfReferenceLocation = PlayerPos;
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
    // No controller lookup needed — InPlayerPos is authoritative for this frame.
    CurrentFrameOfReferenceLocation = InPlayerPos;
    const double ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
    const FVector PlayerOffset = InPlayerPos - LastFrameOfReferenceLocation;
    LastFrameOfReferenceLocation = InPlayerPos;
    SetActorLocation(GetActorLocation() + PlayerOffset * (1.0 - ParallaxRatio));
}
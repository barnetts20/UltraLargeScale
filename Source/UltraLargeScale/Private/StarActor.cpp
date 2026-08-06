// StarActor.cpp
#include "StarActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"

// Mesh + material both live in plugin content (/UltraLargeScale/), which is not reliably
// mounted at CDO-construction time -- so NEITHER is loaded in the constructor. Both are
// LoadObject'd at runtime in BeginPlay, mirroring how AGalaxyActor loads its assets.
static const TCHAR* kDefaultStarMeshPath = TEXT("/UltraLargeScale/UnitSphere.UnitSphere");
static const TCHAR* kDefaultStarMaterialPath = TEXT("/UltraLargeScale/StarSystem/MT_Star_Inst.MT_Star_Inst");

AStarActor::AStarActor()
{
    PrimaryActorTick.bCanEverTick = true;   // min-angular-size scaling runs each frame

    StarMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StarMesh"));
    SetRootComponent(StarMeshComponent);
    StarMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    StarMeshComponent->SetMobility(EComponentMobility::Movable);
    StarMeshComponent->SetCastShadow(false);
}

void AStarActor::BeginPlay()
{
    Super::BeginPlay();

    if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, kDefaultStarMeshPath))
        StarMeshComponent->SetStaticMesh(Mesh);
    else
        UE_LOG(LogTemp, Warning, TEXT("AStarActor: star mesh '%s' failed to load"), kDefaultStarMeshPath);

    // Dynamic instance so per-star params can be set later; inherits MT_Star_Inst for now.
    if (UMaterialInterface* Parent = LoadObject<UMaterialInterface>(nullptr, kDefaultStarMaterialPath))
    {
        StarMaterialMID = UMaterialInstanceDynamic::Create(Parent, this);
        if (StarMaterialMID)
            StarMeshComponent->SetMaterial(0, StarMaterialMID);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("AStarActor: star material '%s' failed to load"), kDefaultStarMaterialPath);
    }
}

void AStarActor::OnAcquired(double WorldRadius)
{
    TrueRadius = FMath::Max(1.0, WorldRadius);
    SetActorScale3D(FVector(TrueRadius));   // unit-radius mesh -> scale == radius (cm)
    SetActorHiddenInGame(false);
    SetActorTickEnabled(true);
}

void AStarActor::OnReturnToPool()
{
    // Unused for the (unpooled) star, but required by IPooledActor.
    SetActorHiddenInGame(true);
}

void AStarActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    const UWorld* World = GetWorld();
    const APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
    if (!PC || !PC->PlayerCameraManager)
        return;

    // Same min-angular-size clamp the particle systems use: render at
    // max(trueRadius, distance * MinAngularSize). No FOV term. The floor wins when far
    // (constant apparent size), trueRadius wins once close. R / D == MinAngularSize (a
    // small constant), so the sphere never engulfs the view.
    const FVector CamLoc = PC->PlayerCameraManager->GetCameraLocation();
    const double  Dist = FVector::Distance(CamLoc, GetActorLocation());
    const double  EffRadius = FMath::Max(TrueRadius, Dist * MinAngularSize);

    SetActorScale3D(FVector(EffRadius));
}
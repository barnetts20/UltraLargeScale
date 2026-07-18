// ParallaxStaticMeshActor.cpp
#include "ParallaxStaticMeshActor.h"
#include "UObject/ConstructorHelpers.h"

AParallaxStaticMeshActor::AParallaxStaticMeshActor()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
    RootComponent = MeshComponent;
    MeshComponent->SetMobility(EComponentMobility::Movable);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);
    MeshComponent->SetRenderCustomDepth(true);
    MeshComponent->SetCustomDepthStencilValue(1);

    static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(
        TEXT("/UltraLargeScale/UnitSphere.UnitSphere"));
    if (Sphere.Succeeded())
        MeshComponent->SetStaticMesh(Sphere.Object);
}
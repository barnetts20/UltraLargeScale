// ParallaxStaticMeshActor.cpp
#include "ParallaxStaticMeshActor.h"
#include "UObject/ConstructorHelpers.h"

AParallaxStaticMeshActor::AParallaxStaticMeshActor()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
    RootComponent = MeshComponent;
    MeshComponent->SetMobility(EComponentMobility::Movable);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);// QueryAndPhysics);
    MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);
    MeshComponent->SetRenderCustomDepth(true);
    MeshComponent->SetCustomDepthStencilValue(1);

    // Virtual backdrop: hidden in the main renderer, visible only to the backdrop
    // SceneCapture (nebula meshes, black-hole billboard, etc.).
    MeshComponent->bVisibleInSceneCaptureOnly = true;

    static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/UltraLargeScale/UnitSphere.UnitSphere"));

    if (Sphere.Succeeded())
        MeshComponent->SetStaticMesh(Sphere.Object);
}
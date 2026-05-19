#include "ParallaxStaticMeshActor.h"
#include "StarSystemActor.h"

AParallaxStaticMeshActor::AParallaxStaticMeshActor()
{
	// Tick is disabled — position is driven by AStarSystemActor::TickFromParent
	// via TickFromStarSystem each frame. Self-ticking would fight the VT model.
	PrimaryActorTick.bCanEverTick = false;

	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	RootComponent = MeshComponent;

	MeshComponent->SetMobility(EComponentMobility::Movable);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);
	MeshComponent->SetRenderCustomDepth(true);
	MeshComponent->SetCustomDepthStencilValue(1);
}

void AParallaxStaticMeshActor::TickFromStarSystem(const FVector& InPlayerPos)
{
	if (!System) return;

	// The planet's rendered world position is:
	//   PlayerPos + (NodeCenter - VirtualTraversal)
	//
	// This is the same formula used by Niagara to place particles — each
	// particle renders at (PlayerPos + (LocalPos - VT)). We replicate it
	// here so the mesh sits exactly on top of the planet sprite it replaced.
	//
	// The star system actor is already pegged to InPlayerPos each tick,
	// so GetActorLocation() == InPlayerPos for the system. We compute the
	// offset directly.
	const FVector WorldPos = InPlayerPos + (NodeCenter - System->VirtualTraversal);

	SetActorLocation(WorldPos);
}
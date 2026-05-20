#include "ParallaxStaticMeshActor.h"
#include "StarSystemActor.h"

AParallaxStaticMeshActor::AParallaxStaticMeshActor()
{
	// Tick is disabled - position is driven by AStarSystemActor::TickFromParent
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

	// The mesh lives in real UE world space (UnitScale = 1). To stay visually
	// aligned with the Niagara sprite it replaced, we place it at the same
	// angular position as the sprite but scaled out by UnitScale:
	//
	//   SpritePos   = PlayerPos + (NodeCenter - VT)              [virtual space]
	//   MeshPos     = PlayerPos + (NodeCenter - VT) * UnitScale  [real space]
	//
	// This is equivalent to ComputeChildSpawnLocation(NodeCenter, 1.0).
	const FVector Offset = NodeCenter - System->VirtualTraversal;
	const FVector WorldPos = InPlayerPos + Offset * System->Params.UnitScale;

	SetActorLocation(WorldPos);
}
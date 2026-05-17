#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParallaxStaticMeshActor.generated.h"

class AStarSystemActor;

UCLASS()
class SVO_API AParallaxStaticMeshActor : public AActor
{
	GENERATED_BODY()

public:
	AParallaxStaticMeshActor();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	UStaticMeshComponent* MeshComponent;

	/** Owning star system. Set by SpawnPlanetFromPool. Used to derive
	 *  the correct SpeedScale chain (StarSystem → Galaxy → Universe). */
	UPROPERTY()
	AStarSystemActor* System = nullptr;

	/** Octree-local center of this planet's node (star-system VT space).
	 *  Stored at spawn so TickFromStarSystem can recompute world position
	 *  each frame using the current VirtualTraversal. */
	FVector NodeCenter = FVector::ZeroVector;

	/** Called every frame by AStarSystemActor::TickFromParent.
	 *  Recomputes world position from the system's current VirtualTraversal
	 *  so the planet stays locked to its parallax-correct location. */
	void TickFromStarSystem(const FVector& InPlayerPos);
};
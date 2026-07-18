// ParallaxStaticMeshActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParallaxStaticMeshActor.generated.h"

/** Default wrapped body: a unit-sphere static mesh actor. Parallax is owned by
 *  AParallaxProxyActor, which spawns, sizes, and moves this — so this class does
 *  no parallax itself. Self-loads the unit sphere so it works as a no-config
 *  default. (Name kept to avoid churn; the "Parallax" prefix is now a misnomer —
 *  rename to e.g. AUnitSphereActor when convenient; it's spawned procedurally so
 *  there are no level/asset references to redirect.) */
UCLASS()
class ULTRALARGESCALE_API AParallaxStaticMeshActor : public AActor
{
    GENERATED_BODY()
public:
    AParallaxStaticMeshActor();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    UStaticMeshComponent* MeshComponent;
};
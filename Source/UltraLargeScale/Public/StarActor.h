// StarActor.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PooledActor.h"
#include "StarActor.generated.h"

class UStaticMeshComponent;
class UMaterialInstanceDynamic;

/**
 * Minimal central-star body: a glowing sphere. Wrapped by an AParallaxProxyActor (the
 * same parallax wrapper the planets use) that the star system spawns directly and owns
 * -- NOT pooled, always present while the system is active. The proxy drives position
 * (identical parallax to planets); this actor owns its mesh, material, and a min-angular
 * -size clamp so it stays visible from any distance without a sprite.
 *
 * Mesh + material are fixed plugin assets in /UltraLargeScale/ content, which isn't
 * reliably mounted at CDO-construction time -- so both are LoadObject'd at runtime in
 * BeginPlay (mirroring AGalaxyActor), not referenced in the constructor.
 *
 * Implements IPooledActor purely so the wrapping proxy hands it its true world radius
 * via OnAcquired(double); it is never registered with the pool manager.
 */
UCLASS()
class ULTRALARGESCALE_API AStarActor : public AActor, public IPooledActor
{
    GENERATED_BODY()

public:
    AStarActor();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;

    // IPooledActor (proxy-carried): the wrapping proxy hands over the true world radius.
    using IPooledActor::OnAcquired;
    virtual void OnAcquired(double WorldRadius) override;
    virtual void OnReturnToPool() override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Star")
    TObjectPtr<UStaticMeshComponent> StarMeshComponent;

    /** Minimum angular size, matching the particle systems' clamp (they use 0.001).
     *  The star renders at max(trueRadius, distance * MinAngularSize), so it never falls
     *  below this apparent size and stays visible from any distance without a sprite. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star")
    double MinAngularSize = 0.002;

protected:
    /** True (unscaled) world radius in cm, handed over by the proxy on acquire. */
    double TrueRadius = 1.0;

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> StarMaterialMID;
};
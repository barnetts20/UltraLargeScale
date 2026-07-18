#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParallaxProxyActor.generated.h"

/** Terminal parallax layer (UnitScale = 1) that wraps a single real-space actor.
 *  The last link in the Universe -> Galaxy -> StarSystem chain: it runs the same
 *  speed-scale traversal as every layer, but at UnitScale = 1 there's no space
 *  left to compress, so the wrapped actor lives in real cm and moves with real
 *  precision. Knows nothing about what it wraps — the class is passed in — so it
 *  carries no dependency on the wrapped actor's module. The wrapped actor owns
 *  its own proceduralization; the proxy only positions it. */
UCLASS()
class ULTRALARGESCALE_API AParallaxProxyActor : public AActor
{
    GENERATED_BODY()

public:
    AParallaxProxyActor();

    /** Wrapped real-space actor (voxel planet, station, derelict, ...). */
    UPROPERTY()
    AActor* Wrapped = nullptr;

    /** Live universe speed scale — stored each frame from TickParallax. Held so a
     *  future meshing gate can check it (updates run only at SpeedScale == 1). */
    double SpeedScale = 1.0;

    /** This layer's virtual traversal, accumulated in real space. Seeded once by
     *  the star system at spawn so frame 0 renders on the planet sprite's spot. */
    FVector VirtualTraversal = FVector::ZeroVector;

    /** Spawns the wrapped actor at this proxy's location and sizes it.
     *  WorldRadius drives the wrapped actor's scale (cm). */
    void SetupWrapped(UClass* WrappedClass, double WorldRadius);

    /** Per-frame layer update, driven by AStarSystemActor's tick loop.
     *  InSpeedScale is the same value the star system resolves for itself. */
    void TickParallax(float DeltaTime, const FVector& PlayerPos, double InSpeedScale);

protected:
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
    FVector LastPlayerPos = FVector::ZeroVector;
    bool    bHasLastPlayer = false;
};
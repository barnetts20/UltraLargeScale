// ProceduralSpaceActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FOctree.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "ProceduralSpaceActor.generated.h"

/// <summary>
/// BASE GENERATION PARAMS - Shared fields across Universe/Galaxy/StarSystem
/// </summary>
USTRUCT(BlueprintType)
struct SVO_API FBaseParams {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    int Seed = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double Extent = 2147483648;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Generation")
    double UnitScale = 1.0;  // Derived - set by spawning actor

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
};

/// <summary>
/// BASE PROCEDURAL SPACE ACTOR - Abstract base for Universe/Galaxy/StarSystem
/// </summary>
UCLASS(Abstract)
class SVO_API AProceduralSpaceActor : public AActor
{
    GENERATED_BODY()

public:
    AProceduralSpaceActor();
    virtual ~AProceduralSpaceActor() = default;

#pragma region Shared State
    TSharedPtr<FOctree> Octree;
    ELifecycleState InitializationState = ELifecycleState::Uninitialized;
    bool IsDebug = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
    double SpeedScale = 1.0;  // Runtime parameter (not in Params)

    /** Utility: resolves the local player pawn's world position.
     *  Returns true on success; OutLocation is untouched on failure.
     *  Static so non-inheriting classes (e.g. AUniverseActor) can call it. */
    static bool GetPlayerLocation(const UWorld* World, FVector& OutLocation);
#pragma endregion

#pragma region Deferred Placement
    /** True while this actor has been spawned from a pool but not yet placed.
     *  The parent's tick loop checks this after InitializationState == Ready
     *  and calls the appropriate FinalizeXxxPlacement to compute the spawn
     *  position using the current frame's parallax state, toggle visibility,
     *  and begin ticking — all in the same frame, with zero parallax drift.
     *
     *  Set true in SpawnXxxFromPool; cleared by FinalizeXxxPlacement. */
    bool bPendingPlacement = false;

    /** Octree node center (local space) cached at spawn time for the parent
     *  to re-derive the correct world-space spawn position at placement time.
     *  Only meaningful while bPendingPlacement is true. */
    FVector PendingNodeCenter = FVector::ZeroVector;
#pragma endregion

#pragma region Lifecycle (overrideable)
    virtual void Initialize();      // Kicks off async init
    virtual void ResetForSpawn();   // Called before Initialize
    virtual void ResetForPool();    // Called when returning to pool
#pragma endregion

public:
#pragma region Initialization (virtual - override to implement)
    virtual void InitializeData();
    virtual void InitializeVolumetric();
    virtual void InitializeNiagara();
    virtual void InitializeChildPool();  // Optional - Universe/Galaxy override this
#pragma endregion

#pragma region Params Accessors
    virtual double GetUnitScale() const { return 1; }
    virtual double GetExtent() const { return 2147483648; }
    virtual double GetParentSpeedScale() const { return 1; }
#pragma endregion

#pragma region Shared Niagara
    UPROPERTY()
    UNiagaraComponent* NiagaraComponent;
#pragma endregion

#pragma region Shared Volumetric
    UPROPERTY()
    UTexture2D* PseudoVolumeTexture;

    UPROPERTY()
    UStaticMeshComponent* VolumetricComponent;

    UPROPERTY()
    UMaterialInstanceDynamic* VolumeMaterial;
#pragma endregion

#pragma region Parallax
    FVector LastFrameOfReferenceLocation = FVector::ZeroVector;
    FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

#pragma region Parallax Spawn Calculation
    // Computes correct spawn position for a child actor based on parallax ratios.
    // Each subclass must implement this — the formula depends on whether the actor
    // uses VirtualTraversal (Universe, Galaxy) or position-based parallax (StarSystem).
    virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const { return FVector::ZeroVector; };
#pragma endregion
    // Each subclass must implement its own parallax model. Universe and Galaxy
    // use VirtualTraversal accumulation; StarSystem uses position-based offset.
    virtual void ApplyParallaxOffset() {};
    virtual void DrawDebugBounds();
    virtual void Tick(float DeltaTime) override;

    // Called by the parent actor (Universe→Galaxy, Galaxy→StarSystem) instead of
    // UE's per-actor tick dispatch. InPlayerPos is the already-resolved player
    // world position for this frame — no child needs to query the controller.
    // Base implementation applies the standard parallax offset (covers StarSystem).
    // Galaxy overrides this to also handle VirtualTraversal, Niagara pushes,
    // tier streaming, and cascading down to its own star systems.
    virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos);
#pragma endregion

#pragma region Hierarchical Spawn Scanning
    /** Time of last scan dispatch. Used for interval throttling
     *  now that scans are driven from the tick chain instead of timers. */
    double LastScanDispatchTime = 0.0;

    /** Request a scan if enough time has elapsed and no scan is in flight.
     *  Called by the Universe's DetermineAndDispatchScan, not by a timer.
     *  Subclasses override to dispatch their async octree query. */
    virtual void RequestScan() {}

    /** Returns true if the player's VirtualTraversal is within this actor's
     *  octree bounds. Used by the Universe to determine which level to scan. */
    virtual bool IsPlayerInsideBounds() const { return false; }
#pragma endregion
};
// ProceduralSpaceActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FOctree.h"
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
    double Extent = 274877906944; // 2^38

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Generation")
    double UnitScale = 1.0;  // Derived - set by spawning actor

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
};

/// <summary>
/// BASE PROCEDURAL SPACE ACTOR - Abstract base for Universe/Galaxy/StarSystem.
///
/// Provides shared state (octree, lifecycle, parallax, virtual traversal) and
/// a default async initialization chain (InitializeChildPool → InitializeData
/// → InitializeVolumetric → InitializeNiagara). Subclasses override the
/// virtual hooks to implement level-specific generation and rendering.
///
/// Does NOT override Tick. The root actor (Universe) overrides Tick directly;
/// child actors (Galaxy, StarSystem) are driven by their parent's
/// TickFromParent cascade and should not tick independently when pool-managed.
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
    double SpeedScale = 1.0;

    /** When true the actor auto-initializes from BeginPlay. Convenient for
     *  level-placed test actors. Set to false for pool-managed actors where
     *  the parent configures params before calling Initialize(). */
    UPROPERTY(EditAnywhere, Category = "Initialization")
    bool bAutoInitializeOnBeginPlay = false;

    /** Utility: resolves the local player pawn's world position.
     *  Returns true on success; OutLocation is untouched on failure. */
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
    virtual void Initialize();      // Kicks off async init chain
    virtual void ResetForSpawn();   // Called before Initialize on pooled actors
    virtual void ResetForPool();    // Called when returning to pool
#pragma endregion

public:
#pragma region Initialization (virtual - override to implement)
    virtual void InitializeData();
    virtual void InitializeVolumetric();
    virtual void InitializeNiagara();
    virtual void InitializeChildPool();
#pragma endregion

#pragma region Params Accessors
    virtual double GetUnitScale() const { return 1; }
    virtual double GetExtent() const { return 274877906944; }
    virtual double GetParentSpeedScale() const { return 1; }
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

    /**
     * Accumulated virtual displacement of the player through this actor's
     * local space. Advances by (SpeedScale / UnitScale) * PlayerDelta each
     * tick. Used by Universe and Galaxy for camera-relative Niagara position
     * pushes; StarSystem uses it for planet placement. The actor itself is
     * pegged to the player so UE rendering stays in a clean numerical range.
     */
    FVector VirtualTraversal = FVector::ZeroVector;

    /** VirtualTraversal value at the last Niagara position push. Used to skip
     *  redundant pushes when the delta is sub-pixel. */
    FVector LastPushedVirtualTraversal = FVector::ZeroVector;

    /** Minimum VirtualTraversal delta before re-pushing camera-relative
     *  positions to Niagara. Sub-pixel changes are invisible, so skipping
     *  them avoids the full array copy+push cost per tier per tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
    double ParallaxPushThreshold = 0.5;

#pragma region Parallax Spawn Calculation
    /** Computes correct spawn position for a child actor based on parallax
     *  ratios. Each subclass must implement this — the formula depends on
     *  whether the actor uses VirtualTraversal (Universe, Galaxy) or
     *  position-based parallax (StarSystem). */
    virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const { return FVector::ZeroVector; };
#pragma endregion

    /** Per-frame parallax update. Each subclass implements its own model.
     *  Universe and Galaxy use VirtualTraversal accumulation; StarSystem
     *  uses position-based offset. Default is a no-op. */
    virtual void ApplyParallaxOffset() {};

    virtual void DrawDebugBounds();

    /** Called by the parent actor (Universe→Galaxy, Galaxy→StarSystem) instead
     *  of UE's per-actor tick dispatch. InPlayerPos is the already-resolved
     *  player world position for this frame — no child needs to query the
     *  controller. Base implementation applies the standard position-based
     *  parallax offset (covers StarSystem). Galaxy overrides to also handle
     *  VirtualTraversal, Niagara pushes, tier streaming, and cascading down
     *  to its own star systems. */
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
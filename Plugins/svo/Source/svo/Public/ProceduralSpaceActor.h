// ProceduralSpaceActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FOctree.h"
#include "Misc/ScopeLock.h"
#include <atomic>
#include "ProceduralSpaceActor.generated.h"

/// <summary>
/// BASE GENERATION PARAMS - Shared fields across Universe/Galaxy/StarSystem
/// </summary>
USTRUCT(BlueprintType)
struct SVO_API FBaseParams {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    int Seed = 0;

    //TODO: NOT SURE ABOUT ACCURACY OF BELOW COMMENT... THE EXTENT I THOUGHT WAS RELATED TO THE OCTREE BOUNDS (AND POTENTIALLY THE PARTICLE SYSTEM) NO? THATS NOT THE SAME AS THE VIRTUAL SPACE
    /** LOCAL half-extent of this actor's virtual space. 
     *  DERIVED AT SPAWN for pooled children: the spawning parent sets
     *  Extent = (parent particle's real size) / UnitScale, so real scale
     *  expresses itself as model size (and therefore content COUNT), while
     *  content sizes stay perceptually identical across instances. The
     *  template/CDO value below is used only by level-placed standalone
     *  actors and as the placeholder until first spawn. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double Extent = 2147483648.0;

    /** PER-LAYER DESIGN CONSTANT: real-world cm per one local unit of this
     *  layer. This is the single bridge between authored real-unit content
     *  sizes and local space, and it is deliberately NEVER derived per
     *  instance — a constant UnitScale is what makes star systems (and
     *  planets, and their precision budgets) identical in perceived and
     *  virtual scale across parents of every size. Set on the spawning
     *  parent's template (e.g. UniverseActor.GalaxyParams.UnitScale,
     *  FGalaxyParams.StarSystemUnitScale) and copied at spawn. Layers below
     *  the star system transition to real space (UnitScale = 1). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double UnitScale = 1.0;

    //TODO: THESE NEED A LOOK... STILL VALID? FOR ALL LAYERS?
    /** Safety clamps for the spawn-time Extent derivation, protecting
     *  against outlier parent particles or a mistuned layer UnitScale.
     *  A spawn hitting either bound logs a warning — treat that as a
     *  signal to retune the layer constant, not as normal operation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MinDerivedExtent = 1048576.0;        // 2^20

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MaxDerivedExtent = 4398046511104.0;  // 2^42

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

    //TODO: ISDEBUG FLAG AUDIT - MAKE SURE DEBUG STUFF IS GATED, COULD ALSO IMPACT OR BE INCOPRORATED INTO LOGGING
    /** Enables debug drawing and periodic verbose diagnostics on this actor.
     *  (Was a plain member — unsettable outside C++.) */
    UPROPERTY(EditAnywhere, Category = "Debug")
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

    /** True while the async Initialize() chain owns this actor's generation
     *  state (tier buffers, octree, volumetric). Set on the game thread in
     *  Initialize() BEFORE the chain dispatches; cleared by the chain task on
     *  every exit path (ON_SCOPE_EXIT). Teardown must not free tier buffers
     *  while this is set — the init-time generation ParallelFors write them
     *  and never raise bUpdateInProgress, so the transition-drain in
     *  ResetForPool does not cover them.
     *
     *  NEVER spin-wait on this from the GAME THREAD: the init chain
     *  rendezvouses with the game thread (Niagara component spawns via
     *  AsyncTask(GameThread) + Future.Wait()), so a GT wait deadlocks.
     *  Wait it out on a worker instead — see the deferred path in
     *  AUniverseActor::ReturnGalaxyToPool / AGalaxyActor::ReturnStarSystemToPool. */
    std::atomic<bool> bInitInProgress{ false };
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
     * tick. Used by Universe, Galaxy, StarSystem for camera-relative Niagara position
     * pushes; StarSystem also uses it for planet placement. The actor itself is
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

    // --- Thread-safe VirtualTraversal handoff for off-thread Niagara pushes ---
    // The game thread publishes the freshest VirtualTraversal every frame; the
    // background push tasks read it at execution time (under the tier PushCS), so a
    // push always composites against current VT -- never a value captured frames
    // earlier when the task was scheduled. This is what removes the boundary-cross
    // jitter: a full push that finishes late no longer re-seeds the GPU with a stale
    // VT. The guard is held only for the 3-double copy, never during an upload.
protected:
    void    PublishLatestVT(const FVector& InVT) { FScopeLock Lock(&LatestVTGuard); LatestVT = InVT; }
    FVector ReadLatestVT() const { FScopeLock Lock(&LatestVTGuard); return LatestVT; }

    /** Single-flight gate for the coalesced per-frame push worker (SchedulePush in
     *  each actor): bPushDirty is raised by the game thread; bPushWorkerLive keeps at
     *  most one draining worker alive. Bursts collapse to one worker that always
     *  re-reads the freshest VT. */
    std::atomic<bool> bPushDirty{ false };
    std::atomic<bool> bPushWorkerLive{ false };

private:
    mutable FCriticalSection LatestVTGuard;
    FVector LatestVT = FVector::ZeroVector;

public:
#pragma region Parallax Spawn Calculation
    /** Computes correct spawn position for a child actor based on parallax
     *  ratios. Each subclass implements this using its VirtualTraversal and
     *  unit-scale ratio; the base provides no default. */
    virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const { return FVector::ZeroVector; };
#pragma endregion

    /** Per-frame parallax update hook. Default is a no-op; overridden by layers
     *  that accumulate VirtualTraversal and push camera-relative positions. */
    virtual void ApplyParallaxOffset(const FVector& InPlayerPos) {};

    virtual void DrawDebugBounds();

    /** Called by the parent actor (Universe→Galaxy, Galaxy→StarSystem) instead
     *  of UE's per-actor tick dispatch. InPlayerPos is the already-resolved
     *  player world position for this frame — no child needs to query the
     *  controller. Each subclass accumulates VirtualTraversal, pushes camera-
     *  relative Niagara positions, runs tier streaming, and cascades to its
     *  own children. Pure virtual — no base default. */
    virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) PURE_VIRTUAL(AProceduralSpaceActor::TickFromParent, );
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
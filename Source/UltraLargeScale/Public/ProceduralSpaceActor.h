// ProceduralSpaceActor.h
#pragma once
#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "GameFramework/Actor.h"
#include "FOctree.h"
#include "Misc/ScopeLock.h"
#include <atomic>
#include "PooledActor.h"
#include "ProceduralSpaceActor.generated.h"

class UActorPoolManager;
class AUniverseActor;


/** SEED FAN-OUT.
 *
 *  A layer's Seed arrives already unique and well scattered -- FVoxelData::ComposeSeed
 *  hash-combines the parent seed with the node coordinate and the particle index. What
 *  it does NOT do is separate the several independent consumers INSIDE one actor, and
 *  FRandomStream is deterministic on its seed, so two consumers that each build
 *  FRandomStream(Seed) draw the identical sequence. An archetype roll and a rotation
 *  roll seeded that way are not merely correlated: the first draw of each is the same
 *  float, and every galaxy of one morphology comes out in one rotation band.
 *
 *  A CHANNEL NAMES A DRAW. Sharing one stream identifies a draw by its POSITION in the
 *  sequence instead, so inserting a roll shifts every draw after it and an authoring
 *  change to the arms silently reshuffles the disc, the bulge and the rotation. Seed
 *  stability across implementation changes is explicitly NOT a goal -- generated
 *  content is expected to change between passes. This narrower property is: an edit to
 *  one channel does not move another channel's output, which is what makes "what did
 *  my change actually do" answerable during authoring.
 *
 *  MECHANISM ONLY. Each layer declares its own channel constants beside the code that
 *  draws from them -- see GalaxySeed in GalaxyParams.h. Nothing layer-specific belongs
 *  in the base. */
namespace ProcSeed
{
    /** Compile-time FNV-1a over a channel name, so the identifier IS the name and
     *  nobody has to invent or collision-check magic constants. Distinctness is all
     *  that is asked of the value; MixSeed does the spreading. */
    constexpr uint32 ChannelId(const char* InName)
    {
        uint32 Hash = 2166136261u;
        while (*InName)
        {
            Hash ^= static_cast<uint32>(static_cast<unsigned char>(*InName++));
            Hash *= 16777619u;
        }
        return Hash;
    }

    /** MurmurHash3's fmix32 finalizer.
     *
     *  NOT OPTIONAL, AND HASHCOMBINE ALONE IS NOT ENOUGH. UE's HashCombine reduces to
     *  `Channel ^ (Seed + K)` once the channel-dependent shift terms are folded, so a
     *  near-sequential run of root seeds comes out near-sequential: measured over
     *  200k roots it occupied 0.047% of the int32 range, chi-squared of 1.3e7 against
     *  63 degrees of freedom. With this finalizer the same run is uniform (chi-squared
     *  48 against an expected 63).
     *
     *  FVoxelData::ComposeSeed uses bare HashCombine and is fine, because ITS inputs
     *  are already hashed coordinate values. These inputs are a raw seed and a raw
     *  channel constant, so the spreading has to happen here. */
    inline uint32 Avalanche(uint32 InValue)
    {
        InValue ^= InValue >> 16;
        InValue *= 0x85EBCA6Bu;
        InValue ^= InValue >> 13;
        InValue *= 0xC2B2AE35u;
        InValue ^= InValue >> 16;
        return InValue;
    }

    /** Root seed + channel -> an independent seed. Masked to 31 bits so it stays
     *  non-negative, matching every other seed in the system. */
    inline int32 MixSeed(int32 InRootSeed, uint32 InChannel)
    {
        const uint32 Raw = Avalanche(HashCombine(static_cast<uint32>(InRootSeed), InChannel));
        return static_cast<int32>(Raw & 0x7FFFFFFFu);
    }

    /** Indexed variant, for a channel with N instances: the three tiers, an arm index,
     *  a batch. REPLACES `Seed + Index`. Additive offsets alias -- galaxy S's index 1
     *  is galaxy S+1's index 0 -- and index 0 would otherwise hand back the unmixed
     *  root, which is the collision this whole helper exists to prevent. */
    inline int32 MixSeed(int32 InRootSeed, uint32 InChannel, int32 InIndex)
    {
        return MixSeed(InRootSeed, HashCombine(InChannel, static_cast<uint32>(InIndex)));
    }

    /** PURE, AND THAT IS THE FOOTGUN. Calling this twice with the same arguments hands
     *  back two streams that draw the identical sequence -- it reads like a getter and
     *  is not one. Take the stream ONCE at the top of the function that draws, hold it
     *  by value, and draw from the local. */
    inline FRandomStream Stream(int32 InRootSeed, uint32 InChannel)
    {
        return FRandomStream(MixSeed(InRootSeed, InChannel));
    }

    inline FRandomStream Stream(int32 InRootSeed, uint32 InChannel, int32 InIndex)
    {
        return FRandomStream(MixSeed(InRootSeed, InChannel, InIndex));
    }
}

/** Base generation params shared across Universe/Galaxy/StarSystem. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FBaseParams {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    int Seed = 0;

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
     *  instance; a constant UnitScale is what makes star systems (and
     *  planets, and their precision budgets) identical in perceived and
     *  virtual scale across parents of every size. Set on the spawning
     *  parent's template (e.g. UniverseActor.GalaxyParams.UnitScale,
     *  FGalaxyParams.StarSystemUnitScale) and copied at spawn. Layers below
     *  the star system transition to real space (UnitScale = 1). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double UnitScale = 1.0;

    /** Safety clamps for the spawn-time Extent derivation, protecting
     *  against outlier parent particles or a mistuned layer UnitScale.
     *  A spawn hitting either bound logs a warning; treat that as a
     *  signal to retune the layer constant, not as normal operation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MaxDerivedExtent = 4398046511104.0;

    /** Lower clamp for the spawn-time Extent derivation (see MaxDerivedExtent).
     *  Set to MaxDerivedExtent / INT32_MAX (~2^42 / 2^31 = 2048) so the smallest
     *  derived extent keeps its precision representable in float relative to the
     *  largest. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MinDerivedExtent = 2048;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);

    /** ProcSeed::Stream against this actor's Seed. Same purity caveat as the free
     *  function: take it once and hold it, or the second call repeats the first. */
    FRandomStream SeedStream(uint32 InChannel) const
    {
        return ProcSeed::Stream(Seed, InChannel);
    }

    FRandomStream SeedStream(uint32 InChannel, int32 InIndex) const
    {
        return ProcSeed::Stream(Seed, InChannel, InIndex);
    }
};

/** Abstract base for Universe/Galaxy/StarSystem. Provides shared state (octree,
 *  lifecycle, parallax, virtual traversal) and a default async initialization
 *  chain (InitializeChildPool -> InitializeData -> InitializeVolumetric ->
 *  InitializeNiagara); subclasses override the virtual hooks for level-specific
 *  generation and rendering.
 *
 *  Does not override Tick: the root actor (Universe) overrides Tick directly,
 *  while child actors (Galaxy, StarSystem) are driven by their parent's
 *  TickFromParent cascade and should not tick independently when pool-managed. */
UCLASS(Abstract)
class ULTRALARGESCALE_API AProceduralSpaceActor : public AActor, public IPooledActor
{
    GENERATED_BODY()

public:
    AProceduralSpaceActor();
    virtual ~AProceduralSpaceActor() = default;

#pragma region Shared State
    TSharedPtr<FOctree> Octree;
    ELifecycleState InitializationState = ELifecycleState::Uninitialized;

    //TODO: ISDEBUG FLAG AUDIT - MAKE SURE DEBUG STUFF IS GATED, COULD ALSO IMPACT OR BE INCOPRORATED INTO LOGGING
    /** Enables debug drawing and periodic verbose diagnostics on this actor;
     *  exposed as a UPROPERTY so it is settable in the editor. */
    UPROPERTY(EditAnywhere, Category = "Debug")
    bool IsDebug = false;

    /** Parallax speed multiplier for this layer. Read-only to Blueprints so all
     *  runtime changes go through SetSpeedScale (which enforces the floor and logs).
     *  Still EditAnywhere for editor-time tuning; C++ propagation writes the member
     *  directly. The authoritative copy is the Universe's; lower layers derive from
     *  it via GetEffectiveSpeedScale. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Parallax Properties")
    double SpeedScale = 1.0;

    /** Floor for SpeedScale. At 1 the layer is 1:1 with world space and the wrapped
     *  content parks + runs full-detail LOD. Values below 1 (microscopic-player
     *  territory) aren't supported yet, so SetSpeedScale clamps up to this. */
    static constexpr double MinSpeedScale = 1.0;

    /** Sets SpeedScale, clamped to >= MinSpeedScale, logging the applied value on
     *  change (and noting when a below-floor request was clamped). Route mousewheel /
     *  input here instead of writing SpeedScale directly so the floor and the log fire.
     *  Hitting exactly the floor is the trigger for parking the wrapped content. */
    UFUNCTION(BlueprintCallable, Category = "Parallax Properties")
    void SetSpeedScale(double NewSpeedScale);

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
     *  and begin ticking, all in the same frame, with zero parallax drift.
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
     *  while this is set: the init-time generation ParallelFors write them
     *  and never raise bUpdateInProgress, so the transition-drain in
     *  ResetForPool does not cover them.
     *
     *  NEVER spin-wait on this from the GAME THREAD: the init chain
     *  rendezvouses with the game thread (Niagara component spawns via
     *  AsyncTask(GameThread) + Future.Wait()), so a GT wait deadlocks.
     *  Wait it out on a worker instead; see the deferred path in
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

    /** Game-thread asset loading, invoked by Initialize() BEFORE the async chain is
     *  dispatched. LoadObject/StaticLoadObject are game-thread-only, so any asset the
     *  async InitializeXxx hooks need (e.g. Niagara systems read in BuildTierConfigs)
     *  must be loaded here, never inside those hooks. */
    virtual void LoadRuntimeAssets() {}
#pragma endregion

#pragma region Params Accessors
    virtual double GetUnitScale() const { return 1; }
    virtual double GetExtent() const { return 274877906944; }
    /** The authoritative parallax scale for this actor. Non-root layers delegate
     *  up to the Universe, which returns its owned SpeedScale. Base returns 1.0
     *  (identity) so a detached / unparented actor stays well-defined. */
    virtual double GetEffectiveSpeedScale() const { return 1.0; }

    /** Central actor pool, owned by the permanent Universe; resolved up the parent
     *  chain (mirrors GetEffectiveSpeedScale). Public so cross-object Universe->/
     *  Galaxy-> calls don't re-earn C2248. Base identity is null. */
    virtual UActorPoolManager* GetPoolManager() const { return nullptr; }

    /** Permanent Universe at the root of the parent chain (the proc-gen bounds
     *  surface). Base identity is null. */
    virtual AUniverseActor* GetUniverse() const { return nullptr; }

    /** True when this layer lives in compressed virtual space (renders to the
     *  backdrop SceneCapture) rather than real space (main pass). Derived purely
     *  from the per-layer UnitScale design constant: real space is UnitScale == 1,
     *  everything above the terminal transition is UnitScale > 1. Read it wherever a
     *  render component's bVisibleInSceneCaptureOnly is set, so the flag is a
     *  consistent runtime derivation instead of a stamped/inherited trait. */
    bool IsVirtualSpace() const { return GetUnitScale() > 1.0; }

    // IPooledActor — generic wake/teardown; see .cpp.
    using IPooledActor::OnAcquired;   // keep the OnAcquired(double) overload visible
    virtual void OnAcquired() override;
    virtual void OnReturnToPool() override;
#pragma endregion

#pragma region Tier System - Grid Coord Helpers
    /** Multiplier applied to this actor's Extent for streaming cell-size
     *  computation. Uniform across all layers.
     *  CellSize = (GetExtent() * GridExtentMultiplier) / (1 << GridDepth). */
    static constexpr double GridExtentMultiplier = 4.0;

    /** Converts a sector-local position to a grid coordinate at the given
     *  depth. Center-aligned lattice: coord (0,0,0) maps to the origin cell. */
    FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;

    /** Converts a grid coordinate back to the sector-local cell center. */
    FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;

    /** Half-extent of a grid cell at the given depth.
     *  Full cell size = 2 * GetGridCellExtent(depth). */
    double GetGridCellExtent(int32 InGridDepth) const;
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

    /** Accumulated virtual displacement of the player through this actor's local
     *  space. Advances by (SpeedScale / UnitScale) * PlayerDelta each tick. Used
     *  by Universe, Galaxy, and StarSystem for camera-relative Niagara position
     *  pushes; StarSystem also uses it for planet placement. The actor itself is
     *  pegged to the player so UE rendering stays in a clean numerical range. */
    FVector VirtualTraversal = FVector::ZeroVector;

    /** VirtualTraversal value at the last Niagara position push. Used to skip
     *  redundant pushes when the delta is sub-pixel. */
    FVector LastPushedVirtualTraversal = FVector::ZeroVector;

    /** Minimum VirtualTraversal delta before re-pushing camera-relative
     *  positions to Niagara. Sub-pixel changes are invisible, so skipping
     *  them avoids the full array copy+push cost per tier per tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
    double ParallaxPushThreshold = 0.5;

    /** Thread-safe VirtualTraversal handoff for off-thread Niagara pushes: the
     *  game thread publishes the freshest VirtualTraversal every frame; background
     *  push tasks read it at execution time (under the tier PushCS), so a push
     *  always composites against current VT, never a value captured frames earlier
     *  when the task was scheduled. This removes boundary-cross jitter: a full push
     *  that finishes late no longer re-seeds the GPU with a stale VT. The guard is
     *  held only for the 3-double copy, never during an upload. */
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
    /** Computes the world-space spawn position for a child actor at the given
     *  child UnitScale, accounting for the parallax depth ratio between this
     *  actor and the child. Uniform across all layers: reads this actor's unit
     *  scale through GetUnitScale(), so subclasses no longer override it. */
    virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;
#pragma endregion

    /** Per-frame parallax update hook. Default is a no-op; overridden by layers
     *  that accumulate VirtualTraversal and push camera-relative positions. */
    virtual void ApplyParallaxOffset(const FVector& InPlayerPos) {};

    virtual void DrawDebugBounds();

    /** Called by the parent actor (Universe -> Galaxy, Galaxy -> StarSystem) instead
     *  of UE's per-actor tick dispatch. InPlayerPos is the already-resolved
     *  player world position for this frame; no child needs to query the
     *  controller. Each subclass accumulates VirtualTraversal, pushes camera-
     *  relative Niagara positions, runs tier streaming, and cascades to its
     *  own children. Pure virtual; no base default. */
    virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) PURE_VIRTUAL(AProceduralSpaceActor::TickFromParent, );
#pragma endregion

#pragma region Hierarchical Spawn Scanning
    /** Interval in seconds between spawn-scan dispatches for this actor.
     *  Shared default across all layers; scans are still orchestrated top-down
     *  by AUniverseActor::DetermineAndDispatchScan, which throttles each layer
     *  against LastScanDispatchTime rather than a per-actor timer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    float SpawnScanInterval = 0.1f;

    /** Minimum screen-space angular size (Extent / Distance) for a node to
     *  trigger a child spawn. Squared internally before traversal.
     *  Lower values = spawn/despawn from further away. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    double SpawnScreenSpaceThreshold = 0.0018;

    /** When true, draws a debug box around each node that passes the threshold. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    bool bDebugDrawSpawnNodes = false;

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
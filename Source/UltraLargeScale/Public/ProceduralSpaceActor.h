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


/** SEED FAN-OUT: independent draw streams from one actor seed.
 *
 *  A layer's Seed arrives unique and well scattered, but does not separate the several
 *  consumers INSIDE one actor. FRandomStream is deterministic on its seed, so two consumers
 *  that each build FRandomStream(Seed) draw the IDENTICAL sequence -- an archetype roll and
 *  a rotation roll seeded that way put every galaxy of one morphology in one rotation band.
 *
 *  A CHANNEL NAMES A DRAW, rather than identifying it by position in a shared sequence. The
 *  property this buys is that an edit to one channel does not move another channel's output.
 *  Seed stability across implementation changes is explicitly NOT a goal.
 *
 *  MECHANISM ONLY: each layer declares its own channel constants beside the code that draws
 *  from them -- see GalaxySeed in GalaxyParams.h. */
namespace ProcSeed
{
    /** Compile-time FNV-1a over a channel name, so the identifier IS the name. Distinctness
     *  is all that is asked of the value; MixSeed does the spreading. */
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

    /** MurmurHash3's fmix32 finalizer. NOT OPTIONAL, AND HASHCOMBINE ALONE IS NOT ENOUGH:
     *  UE's HashCombine reduces to `Channel ^ (Seed + K)`, so a near-sequential run of root
     *  seeds comes out near-sequential, occupying a fraction of a percent of the int32 range.
     *  ComposeSeed gets away with bare HashCombine because ITS inputs are already hashed
     *  coordinates; these are a raw seed and a raw channel constant. */
    inline uint32 Avalanche(uint32 InValue)
    {
        InValue ^= InValue >> 16;
        InValue *= 0x85EBCA6Bu;
        InValue ^= InValue >> 13;
        InValue *= 0xC2B2AE35u;
        InValue ^= InValue >> 16;
        return InValue;
    }

    /** Runtime channel from an FName, for channels that are DATA rather than code -- a rolled
     *  parameter identified by its property name, for instance.
     *
     *  NOT GetTypeHash(FName), which derives from the name table's registration order and so
     *  differs between runs. UPPERCASED FIRST because FName comparison is case-insensitive:
     *  "ArmRadius" and "armradius" name one property and must name one stream. */
    inline uint32 ChannelId(FName InName)
    {
        const FString Text = InName.ToString().ToUpper();
        uint32 Hash = 2166136261u;
        for (const TCHAR Char : Text)
        {
            Hash ^= static_cast<uint32>(Char);
            Hash *= 16777619u;
        }
        return Hash;
    }

    /** Root seed + channel -> an independent seed. Masked to 31 bits so it stays
     *  non-negative, matching every other seed in the system. */
    inline int32 MixSeed(int32 InRootSeed, uint32 InChannel)
    {
        const uint32 Raw = Avalanche(HashCombine(static_cast<uint32>(InRootSeed), InChannel));
        return static_cast<int32>(Raw & 0x7FFFFFFFu);
    }

    /** Indexed variant, for a channel with N instances: the three tiers, an arm index, a
     *  batch. USE THIS RATHER THAN `Seed + Index` -- additive offsets alias, since galaxy S's
     *  index 1 is galaxy S+1's index 0, and index 0 hands back the unmixed root. */
    inline int32 MixSeed(int32 InRootSeed, uint32 InChannel, int32 InIndex)
    {
        return MixSeed(InRootSeed, HashCombine(InChannel, static_cast<uint32>(InIndex)));
    }

    /** PURE, AND THAT IS THE FOOTGUN: calling this twice with the same arguments hands back
     *  two streams that draw the identical sequence. Take the stream ONCE at the top of the
     *  function that draws, hold it by value, and draw from the local. */
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

    /** LOCAL half-extent of this actor's virtual space. DERIVED AT SPAWN for pooled children
     *  as (parent particle's real size) / UnitScale, so real scale expresses itself as content
     *  COUNT while content sizes stay perceptually identical. The value below is used only by
     *  level-placed standalone actors, and as the placeholder until first spawn. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double Extent = 2147483648.0;

    /** PER-LAYER DESIGN CONSTANT: real-world cm per one local unit of this layer, and the
     *  single bridge between authored real-unit content sizes and local space. NEVER derived
     *  per instance -- a constant UnitScale is what makes star systems identical in perceived
     *  and virtual scale across parents of every size. Set on the spawning parent's template
     *  and copied at spawn. Layers below the star system are real space, UnitScale = 1. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double UnitScale = 1.0;

    /** Safety clamps for the spawn-time Extent derivation, against outlier parent particles
     *  or a mistuned layer UnitScale. A spawn hitting either bound logs a warning; treat that
     *  as a signal to retune the layer constant, not as normal operation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MaxDerivedExtent = 4398046511104.0;

    /** Lower clamp for the spawn-time Extent derivation. Set to MaxDerivedExtent / INT32_MAX
     *  so the smallest derived extent stays representable in float against the largest. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double MinDerivedExtent = 2048;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);

    /** ProcSeed::Stream against this actor's Seed. Same purity caveat as the free function:
     *  take it once and hold it, or the second call repeats the first. */
    FRandomStream SeedStream(uint32 InChannel) const
    {
        return ProcSeed::Stream(Seed, InChannel);
    }

    FRandomStream SeedStream(uint32 InChannel, int32 InIndex) const
    {
        return ProcSeed::Stream(Seed, InChannel, InIndex);
    }
};

/** Abstract base for Universe, Galaxy and StarSystem. Provides shared state -- octree,
 *  lifecycle, parallax, virtual traversal -- and a default async initialization chain
 *  (InitializeChildPool -> InitializeData -> InitializeVolumetric -> InitializeNiagara),
 *  whose hooks subclasses override for level-specific generation and rendering.
 *
 *  Does not override Tick: the Universe overrides Tick directly, while child actors are
 *  driven by their parent's TickFromParent cascade and must not tick independently. */
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

    /** Enables debug drawing and periodic verbose diagnostics on this actor.
     *
     *  IT DOES NOT COVER THE WHOLE HIERARCHY: only AGalaxyActor and AStarSystemActor read it,
     *  AUniverseActor never does, and the spawn-node debug draw all three share is gated by
     *  bDebugDrawSpawnNodes instead. Setting this on a universe actor turns nothing on. */
    UPROPERTY(EditAnywhere, Category = "Debug")
    bool IsDebug = false;

    /** Parallax speed multiplier for this layer. Read-only to Blueprints so runtime changes
     *  go through SetSpeedScale, which enforces the floor and logs; still EditAnywhere for
     *  editor-time tuning. The authoritative copy is the Universe's, and lower layers derive
     *  from it via GetEffectiveSpeedScale. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Parallax Properties")
    double SpeedScale = 1.0;

    /** Floor for SpeedScale. At 1 the layer is 1:1 with world space and the wrapped content
     *  parks and runs full-detail LOD. Values below 1 are not supported yet. */
    static constexpr double MinSpeedScale = 1.0;

    /** Sets SpeedScale, clamped to >= MinSpeedScale, logging the applied value on change and
     *  noting a clamped request. Route input here instead of writing SpeedScale directly, so
     *  the floor and the log fire. Hitting the floor exactly parks the wrapped content. */
    UFUNCTION(BlueprintCallable, Category = "Parallax Properties")
    void SetSpeedScale(double NewSpeedScale);

    /** When true the actor auto-initializes from BeginPlay, which suits level-placed test
     *  actors. Set false for pool-managed actors, whose parent configures params first. */
    UPROPERTY(EditAnywhere, Category = "Initialization")
    bool bAutoInitializeOnBeginPlay = false;

    /** Resolves the local player pawn's world position. Returns true on success; OutLocation
     *  is untouched on failure. */
    static bool GetPlayerLocation(const UWorld* World, FVector& OutLocation);
#pragma endregion

#pragma region Deferred Placement
    /** True while this actor has been spawned from a pool but not yet placed. The parent's
     *  tick loop checks this once InitializationState is Ready and calls FinalizeXxxPlacement,
     *  which computes the spawn position from the current frame's parallax state, toggles
     *  visibility and begins ticking in one frame. Set in SpawnXxxFromPool. */
    bool bPendingPlacement = false;

    /** Octree node centre in local space, cached at spawn time so the parent can re-derive
     *  the world-space position at placement. Only meaningful while bPendingPlacement. */
    FVector PendingNodeCenter = FVector::ZeroVector;

    /** True while the async Initialize() chain owns this actor's generation state -- tier
     *  buffers, octree, volumetric. Set on the game thread before the chain dispatches,
     *  cleared by the chain on every exit path. TEARDOWN MUST NOT FREE TIER BUFFERS WHILE THIS
     *  IS SET: the init-time generation ParallelFors write them without raising
     *  bUpdateInProgress, so ResetForPool's transition-drain does not cover them.
     *
     *  NEVER spin-wait on this from the GAME THREAD -- the chain rendezvouses with the game
     *  thread for Niagara component spawns, so a GT wait deadlocks. Wait it out on a worker;
     *  see the deferred path in AUniverseActor::ReturnGalaxyToPool. */
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

    /** Game-thread asset loading, invoked by Initialize() BEFORE the async chain dispatches.
     *  LoadObject is game-thread-only, so any asset the async InitializeXxx hooks need must be
     *  loaded here, never inside those hooks. */
    virtual void LoadRuntimeAssets() {}
#pragma endregion

#pragma region Params Accessors
    virtual double GetUnitScale() const { return 1; }
    virtual double GetExtent() const { return 274877906944; }
    /** The authoritative parallax scale for this actor. Non-root layers delegate up to the
     *  Universe, which returns its owned SpeedScale; base returns identity. */
    virtual double GetEffectiveSpeedScale() const { return 1.0; }

    /** Central actor pool, owned by the permanent Universe and resolved up the parent chain,
     *  mirroring GetEffectiveSpeedScale. Base identity is null. */
    virtual UActorPoolManager* GetPoolManager() const { return nullptr; }

    /** Permanent Universe at the root of the parent chain. Base identity is null. */
    virtual AUniverseActor* GetUniverse() const { return nullptr; }

    /** True when this layer lives in compressed virtual space, rendering to the backdrop
     *  SceneCapture rather than the main pass. Derived purely from UnitScale: real space is 1,
     *  everything above the terminal transition is greater. Read it wherever a component's
     *  bVisibleInSceneCaptureOnly is set, so the flag is derived rather than stamped. */
    bool IsVirtualSpace() const { return GetUnitScale() > 1.0; }

    // IPooledActor — generic wake/teardown; see .cpp.
    using IPooledActor::OnAcquired;   // keep the OnAcquired(double) overload visible
    virtual void OnAcquired() override;
    virtual void OnReturnToPool() override;
#pragma endregion

#pragma region Tier System - Grid Coord Helpers
    /** Multiplier applied to this actor's Extent for streaming cell-size computation, uniform
     *  across layers: CellSize = (GetExtent() * GridExtentMultiplier) / (1 << GridDepth). */
    static constexpr double GridExtentMultiplier = 4.0;

    /** Sector-local position to grid coordinate at the given depth. Centre-aligned lattice:
     *  coord (0,0,0) maps to the origin cell. */
    FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;

    /** Converts a grid coordinate back to the sector-local cell center. */
    FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;

    /** Half-extent of a grid cell at the given depth; full size is twice this. */
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

    /** Accumulated virtual displacement of the player through this actor's local space,
     *  advancing by (SpeedScale / UnitScale) * PlayerDelta each tick. Every layer uses it for
     *  camera-relative Niagara pushes, and StarSystem for planet placement. The actor itself
     *  is pegged to the player, so UE rendering stays in a clean numerical range. */
    FVector VirtualTraversal = FVector::ZeroVector;

    /** VirtualTraversal at the last Niagara position push, against ParallaxPushThreshold. */
    FVector LastPushedVirtualTraversal = FVector::ZeroVector;

    /** Minimum VirtualTraversal delta before re-pushing camera-relative positions to Niagara.
     *  Sub-pixel changes are invisible, so skipping them avoids a copy and push per tier. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
    double ParallaxPushThreshold = 0.5;

    /** Thread-safe VirtualTraversal handoff for off-thread Niagara pushes. The game thread
     *  publishes the freshest VT every frame and background push tasks read it at EXECUTION
     *  time, so a push that finishes late composites against current VT rather than a value
     *  captured when it was scheduled. The guard is held only for the copy, never an upload. */
protected:
    void    PublishLatestVT(const FVector& InVT) { FScopeLock Lock(&LatestVTGuard); LatestVT = InVT; }
    FVector ReadLatestVT() const { FScopeLock Lock(&LatestVTGuard); return LatestVT; }

    /** Single-flight gate for the coalesced per-frame push worker. bPushDirty is raised by
     *  the game thread; bPushWorkerLive keeps at most one draining worker alive, so bursts
     *  collapse to one worker that always re-reads the freshest VT. */
    std::atomic<bool> bPushDirty{ false };
    std::atomic<bool> bPushWorkerLive{ false };

private:
    mutable FCriticalSection LatestVTGuard;
    FVector LatestVT = FVector::ZeroVector;

public:
#pragma region Parallax Spawn Calculation
    /** World-space spawn position for a child actor at the given child UnitScale, accounting
     *  for the parallax depth ratio between this actor and the child. Uniform across layers,
     *  reading this actor's scale through GetUnitScale(). */
    virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;
#pragma endregion

    /** Per-frame parallax update hook. No-op by default; overridden by layers that accumulate
     *  VirtualTraversal and push camera-relative positions. */
    virtual void ApplyParallaxOffset(const FVector& InPlayerPos) {};

    virtual void DrawDebugBounds();

    /** Called by the parent actor instead of UE's per-actor tick dispatch. InPlayerPos is the
     *  already-resolved player world position for this frame, so no child queries the
     *  controller. Each subclass accumulates VirtualTraversal, pushes camera-relative Niagara
     *  positions, runs tier streaming, and cascades to its own children. */
    virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) PURE_VIRTUAL(AProceduralSpaceActor::TickFromParent, );
#pragma endregion

#pragma region Hierarchical Spawn Scanning
    /** Interval in seconds between spawn-scan dispatches for this actor. Scans are
     *  orchestrated top-down by AUniverseActor::DetermineAndDispatchScan, which throttles each
     *  layer against LastScanDispatchTime rather than a per-actor timer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    float SpawnScanInterval = 0.1f;

    /** Minimum screen-space angular size (Extent / Distance) for a node to trigger a child
     *  spawn; squared internally before traversal. Lower values spawn from further away. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    double SpawnScreenSpaceThreshold = 0.0018;

    /** When true, draws a debug box around each node that passes the threshold. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
    bool bDebugDrawSpawnNodes = false;

    /** Time of the last scan dispatch, for interval throttling. */
    double LastScanDispatchTime = 0.0;

    /** Request a scan if enough time has elapsed and none is in flight. Called by the
     *  Universe's DetermineAndDispatchScan; subclasses dispatch their async octree query. */
    virtual void RequestScan() {}

    /** True if the player's VirtualTraversal is within this actor's octree bounds. Used by
     *  the Universe to decide which level to scan. */
    virtual bool IsPlayerInsideBounds() const { return false; }
#pragma endregion
};
#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UniverseDataGenerator.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "FOctree.h"
#include "DataTypes.h"
#include "FNiagaraParticleBuffer.h"
#include "FTierStreamingContext.h"
#include "FTierStreamingSystem.h"
#include "UniverseActor.generated.h"
class AGalaxyActor;
#pragma endregion



#pragma region AUniverseActor
/**
 * Sector-scale universe actor. Owns the three-tier particle streaming system
 * (Large / Mid / Small), the persistent spatial octree, the parallax
 * traversal model, and the galaxy spawn-scan pipeline.
 *
 * Initialization is fully asynchronous: BeginPlay kicks off a background
 * chain (InitializeData → InitializeNiagara) with game-thread rendezvous
 * only where Niagara component creation requires it. After initialization the
 * actor runs entirely from Tick with no blocking calls.
 *
 * Scale model: the actor is pegged to the player every tick so UE's rendering
 * stays in a clean numerical range. All virtual movement is accumulated in
 * VirtualTraversal and applied to particle positions as camera-relative
 * offsets before each Niagara push.
 */
UCLASS()
class SVO_API AUniverseActor : public AActor
{
	GENERATED_BODY()

public:
	AUniverseActor();

#pragma region Editor Parameters

	/** Full universe generation and tier streaming parameters. Editable in the
	 *  Details panel; changes take effect on the next Initialize() call. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;

	/** Parallax speed multiplier. Per-tier virtual traversal ratio =
	 *  SpeedScale / Params.UnitScale. Higher values make the sector appear
	 *  closer (particles move faster relative to the player). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
	double SpeedScale = 1.0;

#pragma endregion

#pragma region Public Octree Queries
	/**
	 * Returns all octree nodes whose screen-space angular size
	 * (Extent * (1 + ScaleFactor))^2 / DistSq exceeds InScreenSpaceThreshold^2.
	 * Traversal prunes entire subtrees whose maximum possible screen size
	 * cannot pass the threshold, making it significantly faster than a full
	 * range query for sparse large-scale distributions.
	 *
	 * @param InCenter                Query origin in sector-local space (typically VirtualTraversal).
	 * @param InScreenSpaceThreshold  Minimum angular size ratio to pass. Squared internally.
	 * @param InTypeId                TypeId filter. Pass -1 to return all types.
	 * @return                        Nodes passing the screen-space threshold.
	 */
	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(
		const FVector& InCenter, double InScreenSpaceThreshold, int32 InTypeId = -1) const;

#pragma endregion

#pragma region Spatial Index

	/**
	 * Persistent sector-scope spatial index. Survives boundary crosses --
	 * nodes are inserted on cell entry and left in place on exit, forming a
	 * spatial registry of all visited particles. Rebased only when
	 * VirtualTraversal approaches the root bounds (extremely rare).
	 *
	 * Data.ObjectId on each node carries the flat slot index into the tier's
	 * particle buffers. Queries dereference back through that index to read
	 * pre-computed positions and extents.
	 *
	 * Sized to PersistentTreeMultiplier * Params.Extent to give many cell-widths
	 * of travel before rebasing is needed.
	 */
	TSharedPtr<FOctree> Octree;

	/** Multiplier applied to Params.Extent for streaming cell size computation.
	 *  CellSize = (Params.Extent * GridExtentMultiplier) / (1 << GridDepth). */
	static constexpr double GridExtentMultiplier = 4.0;

	/** Multiplier applied to Params.Extent for octree root size.
	 *  Must be a power of 2. 64 = 2^6, tree covers +/-32x the sector extent
	 *  per axis, giving ~2^37 units of traversal range before a rebase. */
	static constexpr double PersistentTreeMultiplier = 64.0;

	/** TypeId tag written into every octree node inserted by the tier system.
	 *  Allows spatial queries to filter for galaxy/sector content vs. other
	 *  future node types. */
	static constexpr int32 GalaxyTypeId = 0;

#pragma endregion

#pragma region Galaxy Spawn Hooks

	/** Maps each live octree node to its pooled galaxy actor instance. */
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;

	/**
	 * Pops a galaxy from the pool, configures it from InNode's data
	 * (UnitScale, Seed, ParentColor, Rotation), positions it via
	 * ComputeChildSpawnLocation, and calls Initialize(). No-ops if the pool
	 * is empty or InNode is already spawned.
	 *
	 * @param InNode  Octree node representing the galaxy to spawn.
	 */
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);

	/**
	 * Returns the galaxy associated with InNode to the pool. Calls
	 * ResetForPool() on the game thread (component teardown), then flushes
	 * the galaxy's octree on a background thread before re-inserting it into
	 * the pool on the game thread.
	 *
	 * @param InNode  Octree node whose associated galaxy should be returned.
	 */
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);

#pragma endregion

#pragma region Sector Grid Identity

	/** Grid coordinate of this sector within the universe cell lattice.
	 *  Set once by ConfigureCell before Initialize(). */
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FIntVector CellCoord = FIntVector::ZeroValue;

	/** World-space center of this sector's cell. Derived from CellCoord:
	 *  CellOrigin = CellCoord * (2 * Params.Extent). Used as the actor's
	 *  initial placement and as the cross-sector child-spawn origin. */
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FVector CellOrigin = FVector::ZeroVector;

	/** When true the sector auto-initializes from BeginPlay. Convenient for
	 *  level-placed test actors. Set to false for pool-managed sectors where
	 *  ConfigureCell must run before Initialize(). */
	UPROPERTY()
	bool bAutoInitializeOnBeginPlay = true;

	/**
	 * Sets CellCoord and derives CellOrigin, then repositions the actor.
	 * Also rebuilds the octree against the actual Params.Extent in case Params
	 * were overridden after construction. Must be called before Initialize().
	 *
	 * @param InCellCoord  Universe grid coordinate for this sector.
	 */
	void ConfigureCell(FIntVector InCellCoord);

#pragma endregion

#pragma region Spawn Range Scanning

	/** Interval in seconds between spawn-scan background queries. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

	/** Minimum screen-space angular size (Extent / Distance) for a node to
	 *  trigger a spawn event. Lower values allow smaller/more-distant objects
	 *  to pass. Squared internally before traversal for performance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScreenSpaceThreshold = 0.033;

	/** When true, draws a debug box around each node that passes the
	 *  spawn-scan threshold each interval. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bDebugDrawSpawnNodes = false;

	/** When true, logs the live particle count for each entering node's
	 *  buffer slot. Useful for generation tuning; noisy in normal play. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bLogSpawnEnterExitBuffers = false;

#pragma endregion

#pragma region Lifecycle

	/** Current initialization state. Drives async task guard checks and
	 *  UpdateTier / ApplyParallaxOffset early-outs. */
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	/**
	 * Kicks off the async initialization chain:
	 * InitializeChildPool → InitializeData → InitializeNiagara.
	 * Each step checks InitializationState and bails if Pooling or Destroying
	 * is set. Safe to call from BeginPlay or externally before FinishSpawning.
	 */
	void Initialize();

#pragma endregion

protected:
#pragma region Initialization

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;

	/** Placeholder for galaxy actor pool pre-warming. Currently a no-op pending
	 *  galaxy refactor. */
	void InitializeChildPool();

	/** Copies Params into UniverseGenerator and builds the noise graph.
	 *  Called on a background thread from Initialize(). */
	void InitializeData();

	/** Calls BuildTierConfigs then InitializeTier for all three tiers.
	 *  Called on a background thread; each InitializeTier rendezvouses with
	 *  the game thread via TPromise/TFuture for Niagara component creation. */
	void InitializeNiagara();

#pragma endregion

#pragma region Data Generation

	/** Owns all noise composition and per-tier particle generation logic.
	 *  Tier callbacks in BuildTierConfigs delegate here, keeping the actor
	 *  free of noise and procgen implementation details. */
	UniverseDataGenerator UniverseGenerator;

#pragma endregion

#pragma region Niagara Assets

	/** Large-tier galaxy sprite system. Renders galaxy cluster positions with
	 *  face-normal rotation data for non-billboard shading. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorLargeCloud;

	/** Mid-tier galaxy sprite system. Intermediate scale band between large
	 *  clusters and small individual galaxies. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorMidCloud;

	/** Small-tier galaxy sprite system. Highest-resolution scale band,
	 *  closest to the player's virtual position. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorSmallCloud;

	/** Gas cloud sprite system. Paired with the Large tier; shares positions
	 *  but uses a separate material and extent range for nebula rendering. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGasCloud;

#pragma endregion

#pragma region Tier System - Config / State

	/** Large-tier config (cluster + gas, GridDepth = 1, NeighborhoodRadius = 1). */
	FParticleTierConfig CoarseTierConfig;
	FParticleTierState  CoarseTierState;

	/** Mid-tier config (GridDepth = 4, NeighborhoodRadius = 1). */
	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;

	/** Small-tier config (GridDepth = 7, NeighborhoodRadius = 1). */
	FParticleTierConfig SmallTierConfig;
	FParticleTierState  SmallTierState;

	/** GC-safe owner for all Niagara components created by InitializeTier.
	 *  FParticleTierState::NiagaraComponents holds raw aliases into this array.
	 *  Do not reorder or remove entries at runtime. */
	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;

#pragma endregion

#pragma region Tier System - Pipeline

	/**
	 * Populates all three tier configs from Params and the assigned Niagara
	 * assets. Derives scale ranges, wires generation callbacks, and builds the
	 * shared ComputeBounds lambda. Called once at the start of InitializeNiagara.
	 */
	void BuildTierConfigs();

	/**
	 * Builds a FTierStreamingContext snapshot for the current frame.
	 * Called by Tick before tier updates and by InitializeNiagara.
	 */
	FTierStreamingContext BuildStreamingContext() const;

#pragma endregion

#pragma region Tier System - Octree Integration
	/** True while an async rebase task owns the octree. UpdateTier and
	 *  CheckOctreeBounds must not proceed while this is set. */
	std::atomic<bool> bRebaseInProgress{ false };

	/**
	 * Checks whether VirtualTraversal is within 25% of the octree boundary
	 * on any axis. If so, and no tier update is in progress, triggers RebaseOctree.
	 * Called from Tick after all tier updates.
	 */
	void CheckOctreeBounds();

#pragma endregion

#pragma region Tier System - Grid Coord Helpers

	/**
	 * Converts a sector-local position to a grid coordinate at the given depth.
	 * Uses a center-aligned lattice: coord (0,0,0) maps to the origin cell.
	 *
	 * @param InPos       Position in sector-local space.
	 * @param InGridDepth Tier grid depth (determines cell size).
	 * @return            Integer grid coordinate.
	 */
	FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;

	/**
	 * Converts a grid coordinate back to the world-local cell center position.
	 *
	 * @param InCoord     Integer grid coordinate.
	 * @param InGridDepth Tier grid depth.
	 * @return            Cell center in sector-local space.
	 */
	FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;

	/**
	 * Returns the half-extent of a grid cell at the given depth.
	 * Full cell size = 2 * GetGridCellExtent(depth).
	 *
	 * @param InGridDepth Tier grid depth.
	 * @return            Cell half-extent in sector-local units.
	 */
	double GetGridCellExtent(int32 InGridDepth) const;

#pragma endregion

#pragma region Parallax

	/** Previous frame's player world position. Used to compute PlayerDelta
	 *  each tick for VirtualTraversal accumulation. */
	FVector LastFrameOfReferenceLocation = FVector::ZeroVector;

	/** Current frame's player world position. Cached for ComputeChildSpawnLocation. */
	FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

	/**
	 * Accumulated virtual displacement of the player through sector space.
	 * Advances by (SpeedScale / Params.UnitScale) * PlayerDelta each tick.
	 * The actor itself is pegged to the player so UE's rendering stays in a
	 * clean numerical range; VirtualTraversal encodes how far the player has
	 * "really" moved at sector scale.
	 *
	 * Used by:
	 *   - PushTierToNiagara: RelativePos = LocalPos - VirtualTraversal.
	 *   - ApplyParallaxOffset: per-frame position re-push.
	 *   - UpdateTier: grid coord derivation for boundary-cross detection.
	 *   - CheckOctreeBounds: rebase trigger.
	 *   - DebugDrawSpawnNode: world-space node visualization.
	 */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** VirtualTraversal value at the last Niagara position push. Used to skip
	 *  redundant pushes when the delta is sub-pixel. */
	FVector LastPushedVirtualTraversal = FVector::ZeroVector;

	/** Minimum VirtualTraversal delta (in octree units) before re-pushing
	 *  camera-relative positions to Niagara. Sub-pixel changes are invisible,
	 *  so skipping them avoids the full array copy+push cost per tier per tick.
	 *  Tune via console: r.ParallaxPushThreshold */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
	double ParallaxPushThreshold = 0.5;

	/**
	 * Per-frame parallax update. Pegs the actor to the current player position,
	 * advances VirtualTraversal, and re-pushes camera-relative positions to all
	 * active Niagara components. Must be called before UpdateTier each tick so
	 * streaming coord checks use the latest VirtualTraversal.
	 */
	void ApplyParallaxOffset();

#pragma endregion

#pragma region Galaxy Pool

	/** Class used when pre-warming the galaxy actor pool. */
	TSubclassOf<AGalaxyActor> GalaxyActorClass;

	/** Target pool size. Pool is pre-warmed during InitializeChildPool. */
	int32 GalaxyPoolSize = 5;

	/** Available galaxy actors ready for spawn. Managed as a stack (Pop/Insert). */
	TArray<AGalaxyActor*> GalaxyPool;

	/**
	 * Computes the world-space spawn position for a child actor at a given
	 * unit scale, accounting for the parallax depth ratio between this sector
	 * and the child.
	 *
	 * SpawnPos = CurrentPlayerPos + (NodeCenter - CellOrigin) * (ThisUnitScale / ChildUnitScale)
	 *
	 * @param NodeCenter     Octree node center in sector-local space.
	 * @param ChildUnitScale The child actor's UnitScale (determines parallax depth).
	 * @return               World-space spawn position.
	 */
	FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;

#pragma endregion

private:
#pragma region Spawn Scan - Internal

	/** Guards against overlapping spawn-scan background tasks. Set true when a
	 *  scan is dispatched; cleared on the game thread after TrackedSpawnNodes
	 *  is updated. */
	std::atomic<bool> bSpawnScanInProgress{ false };

	/** Timer handle for the recurring spawn-scan interval. */
	FTimerHandle SpawnScanTimerHandle;

	/** Set of nodes currently inside the spawn threshold. Diffed each scan
	 *  interval to produce enter/exit events. Game-thread only. */
	TSet<TSharedPtr<FOctreeNode>> TrackedSpawnNodes;

	/** Pending scan results written by the async callback, consumed by Tick.
	 *  Processing is deferred to Tick so that SpawnGalaxyFromPool always sees
	 *  the current frame's VirtualTraversal and player position (set by
	 *  ApplyParallaxOffset), eliminating the 1-frame parallax offset that
	 *  occurs when the timer callback lands before or after the parallax update. */
	bool bHasPendingScanResults = false;
	TArray<TSharedPtr<FOctreeNode>> PendingScanResults;

	/** Starts the recurring spawn-scan timer. Called on the game thread after
	 *  initialization completes. */
	void StartSpawnScanTimer();

	/** Clears the timer and empties TrackedSpawnNodes. Called from EndPlay. */
	void StopSpawnScanTimer();

	/**
	 * Timer callback. Dispatches a background octree query, then stores
	 * results into PendingScanResults for deferred processing in Tick.
	 * Skips if a scan is already in progress (bSpawnScanInProgress).
	 */
	void UpdateSpawnRangeNodes();

	/**
	 * Processes pending scan results after ApplyParallaxOffset has resolved
	 * the current frame's player position and VirtualTraversal. Called from Tick.
	 */
	void ProcessPendingScanResults();

	/** Logs an ENTER event for a node that crossed into the spawn threshold. */
	void LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const;

	/** Logs an EXIT event for a node that left the spawn threshold. */
	void LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const;

	/** Draws a debug box in world space around InNode for one scan interval. */
	void DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const;

#pragma endregion
};
#pragma endregion
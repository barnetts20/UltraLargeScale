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
#include "UniverseActor.generated.h"
class AGalaxyActor;
#pragma endregion

#pragma region Internal Data Structures
// ============================================================================
//  Tier System — Supporting Structs
//  Defined outside the class so they can be forward-declared by subsystems
//  that only need to read tier data (e.g. volumetric streaming callbacks).
// ============================================================================
/**
 * Maps one active grid cell to its flat-buffer slot and the octree nodes
 * inserted from that slot. Needed for O(1) slot recycling on cell exit and
 * for targeted octree node removal without a full tree scan.
 *
 * Replaces the old FCoarseSlotEntry / FProximitySlotEntry pair, which were
 * structurally identical.
 */
struct FSlotEntry
{
	/** Index into the tier's flat particle buffer. -1 = unassigned. */
	int32 SlotIndex = -1;

	/** Octree nodes inserted from this slot's live particles. Cleared and
	 *  repopulated on each boundary cross; left persistent on cell exit so
	 *  the spatial index survives beyond the streaming window. */
	TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
};

/**
 * Immutable descriptor for one particle streaming tier. Populated once by
 * BuildTierConfigs() and never mutated at runtime. All behavioural differences
 * between Large / Mid / Small tiers are encoded here; the generic pipeline
 * (InitializeTier, UpdateTier) reads these fields and delegates generation
 * via the callbacks.
 */
struct FParticleTierConfig
{
	/** Human-readable name used in log output (e.g. "Large", "Mid", "Small"). */
	FString TierName;

	/**
	 * Octree depth that defines this tier's streaming cell size.
	 * Cell half-extent = (Params.Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)).
	 *   Large = 1  →  cell half-extent = 2 * Extent
	 *   Mid   = 4  →  cell half-extent = Extent / 8
	 *   Small = 7  →  cell half-extent = Extent / 64
	 */
	int32 GridDepth = 1;

	/**
	 * Half-width of the 3D cell neighborhood streamed around the player.
	 * Total active slots = (2 * NeighborhoodRadius + 1)^3.
	 * e.g. radius 1 → 3×3×3 = 27 slots.
	 */
	int32 NeighborhoodRadius = 1;

	/** Maximum particles written per slot (candidate count before rejection). */
	int32 SlotCapacity = 0;

	/** Tier index written into octree node TypeId on insert.
	 *  Used by spawn hooks to identify which tier a node came from
	 *  without searching scale ranges. Large=0, Mid=1, Small=2. */
	int32 TierIndex = 0;

	/**
	 * One Niagara system template per logical buffer in this tier.
	 * Large tier has two (cluster + gas); Mid and Small have one each.
	 * InitializeTier spawns one UNiagaraComponent per entry.
	 */
	TArray<UNiagaraSystem*> NiagaraAssets;

	/**
	 * Parallel to NiagaraAssets. True if the corresponding buffer should
	 * allocate the Rotations array (face normals for non-billboard rendering).
	 * Large cluster = true; gas, Mid, Small = false.
	 */
	TArray<bool> bWantRotations;

	/**
	 * Index into NiagaraAssets / Buffers that is walked during octree insertion.
	 * Set to -1 to skip octree insertion entirely for this tier.
	 * Large = 0 (cluster buffer); Mid = 0; Small = 0.
	 */
	int32 OctreeInsertBufferIndex = 0;

	/**
	 * Particle generation callback. Invoked once per entering cell during
	 * parallel generation (both InitializeTier and UpdateTier). Writes
	 * directly into the slot region of each buffer pointer provided.
	 *
	 * @param Coord      Grid coordinate of the cell being generated.
	 * @param SlotIndex  Flat slot index within the tier's particle buffers.
	 * @param Buffers    One raw pointer per NiagaraAsset, each pointing to
	 *                   the back buffer for this tier.
	 */
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	/**
	 * Returns the axis-aligned bounding box used as the Niagara fixed bounds
	 * for all components in this tier. Recomputed on each boundary cross push
	 * so Lumen/radiance-cache always sees a tight box around actual particles.
	 */
	TFunction<FBox()> ComputeBounds;

	/**
	 * Optional hook fired inside UpdateTier's async task after exiting slots
	 * are freed but before particle generation begins. Used by the streaming
	 * volumetric to update density sub-regions in lockstep with Large-tier
	 * boundary crosses. Not set for Mid or Small tiers.
	 *
	 * @param Entering   Grid coords of cells entering the streaming window.
	 * @param Exiting    Grid coords of cells leaving the streaming window.
	 * @param NewCenter  The tier's new center coord after the boundary cross.
	 */
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;
};

/**
 * Mutable runtime state for one particle streaming tier. Fully owned by the
 * tier pipeline; generation callbacks write into Buffers directly but do not
 * touch any other fields.
 *
 * @note This is a plain struct, not a USTRUCT. UNiagaraComponent* pointers
 *       stored here alias entries in AUniverseActor::TierNiagaraComponents
 *       (a UPROPERTY TArray) for GC safety. Do not store them elsewhere.
 *
 * Threading contract:
 *   - CenterCoord, ActiveSlots, FreeSlots, SlotCounts, CellCache, and the
 *     back-buffer are written exclusively by the async task spawned from
 *     UpdateTier. The game thread must not read them while bUpdateInProgress
 *     is true.
 *   - FrontIdx, bUpdateInProgress, and bNeedsPush are std::atomic and safe
 *     for lock-free cross-thread reads.
 *   - The front-buffer (Buffers[b][FrontIdx]) is read-only on the game thread
 *     (ApplyParallaxOffset, PushTierToNiagara). Async tasks must not write it.
 *   - NiagaraComponents are only touched on the game thread.
 */

struct FParticleTierState
{
	/**
	 * Double-buffered particle data, one pair per Niagara asset.
	 * Outer index = asset/buffer index. Inner index is always 2:
	 * [FrontIdx] = live data read by game thread.
	 * [1-FrontIdx] = back buffer written by async generation.
	 */
	TArray<TArray<FNiagaraParticleBuffer>> Buffers;

	/** Raw component pointers aliasing TierNiagaraComponents. Parallel to
	 *  FParticleTierConfig::NiagaraAssets. Game-thread only. */
	TArray<UNiagaraComponent*> NiagaraComponents;

	/** Index of the currently live buffer (0 or 1). Swapped atomically by
	 *  the async task after generation completes. */
	std::atomic<int32> FrontIdx{ 0 };

	/** True while an async boundary-cross task owns the back-buffer and state.
	 *  Game thread must not begin a new update while this is set. */
	std::atomic<bool> bUpdateInProgress{ false };

	/** Set by the async task when a new back-buffer is ready to push.
	 *  Cleared by the game thread after PushTierToNiagara completes. */
	std::atomic<bool> bNeedsPush{ false };

	/** Grid coordinate of the cell currently at the center of the streaming
	 *  neighborhood. Written on the game thread before the async task starts,
	 *  then treated as read-only by both sides until the task clears
	 *  bUpdateInProgress. Initialized to INT32_MIN to force a full generate
	 *  on the first UpdateTier call. */
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	/** Maps grid coord → slot entry (index + inserted octree nodes).
	 *  Written only by the async task while bUpdateInProgress is true. */
	TMap<FIntVector, FSlotEntry> ActiveSlots;

	/** Stack of available slot indices. Popped on cell enter, pushed on exit.
	 *  Written only by the async task while bUpdateInProgress is true. */
	TArray<int32> FreeSlots;

	/** Per-slot accepted particle count, written by GenerateCallback.
	 *  Used by CacheCellFromBuffers and InsertSlotIntoOctree to skip dead
	 *  padding without iterating the full SlotCapacity. */
	TArray<int32> SlotCounts;

	/**
	 * Persistent procgen cache keyed by grid coord. On first visit a cell is
	 * generated and its output stored here (cache-miss). On re-entry the
	 * stored data is blitted directly into the back-buffer, skipping all noise
	 * and rejection sampling (cache-hit). The slot is recycled on exit but the
	 * cache entry survives. Entries beyond NeighborhoodRadius + 4 cells
	 * (Chebyshev) are evicted by CullTierCache after each boundary cross.
	 */
	TMap<FIntVector, FCachedCellData> CellCache;
};
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
	 * Full tier initialization: allocates double-buffered particle data,
	 * builds the initial neighborhood, runs parallel generation, inserts into
	 * the octree, mirrors front→back, then rendezvouses with the game thread
	 * to spawn Niagara components and activate them exactly once.
	 *
	 * Particle IDs are stable from this point forward — Activate is never
	 * called again for the lifetime of the tier.
	 *
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state to initialize.
	 */
	void InitializeTier(FParticleTierConfig& Config, FParticleTierState& State);

	/**
	 * Per-tick streaming update. On game thread: checks bNeedsPush and pushes
	 * if ready, then checks for a neighborhood center change. If the player has
	 * crossed a cell boundary, kicks an async task that: copies front→back,
	 * diffs old/new neighborhoods, frees exiting slots, generates entering slots
	 * (cache-hit or procgen), inserts into the octree, culls the cell cache,
	 * then swaps FrontIdx and sets bNeedsPush.
	 *
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state.
	 */
	void UpdateTier(FParticleTierConfig& Config, FParticleTierState& State);

	/**
	 * Pushes the front buffer of each buffer pair to its Niagara component.
	 * Recomputes and sets the fixed bounds box each call so Lumen always
	 * sees a tight volume around actual particle positions.
	 *
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state.
	 */
	void PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State);

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

	/**
	 * Inserts all active slots of one tier's buffer into the octree.
	 * Used by InitializeTier (full initial insert) and RebaseOctree (full
	 * rebuild). For incremental per-cell insert during streaming use
	 * InsertSlotIntoOctree instead.
	 *
	 * @param Config     Tier descriptor (provides OctreeInsertBufferIndex).
	 * @param State      Tier state (provides ActiveSlots and Buffers).
	 * @param BufferIdx  Which of the two buffers to read (front or back).
	 */
	void InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx);

	/**
	 * Inserts a single slot's live particles into the octree. Used by
	 * UpdateTier for incremental insert of entering cells without a full
	 * tree rebuild. Iterates only SlotCounts[SlotIndex] particles.
	 *
	 * @param Config     Tier descriptor.
	 * @param State      Tier state.
	 * @param SlotIndex  The flat slot index to insert.
	 * @param BufferIdx  Which of the two buffers to read.
	 */
	void InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State,
		int32 SlotIndex, int32 BufferIdx);

	/**
	 * Shared inner loop for InsertTierIntoOctree and InsertSlotIntoOctree.
	 * Computes insert depth from Extent via MakePointDataFromWorldScale,
	 * calls Octree->InsertPosition, and appends the result to Entry.InsertedNodes.
	 *
	 * @param Entry       Slot entry to append the inserted node to.
	 * @param Position    Particle position in sector-local space.
	 * @param Extent      Particle extent in sector-local units (UnitScale = 1.0 assumed).
	 * @param SlotIndex   Written into ObjectId for buffer dereference on query.
	 * @param TreeExtent  Current octree root extent, used to derive insert depth.
	 * @param TierIndex   Written into TypeId so spawn hooks can identify the source tier.
	 */
	void InsertParticleIntoOctree(FSlotEntry& Entry, const FVector& Position,
		float Extent, int32 SlotIndex, double TreeExtent, int32 TierIndex = 0);

#pragma endregion

#pragma region Tier System - Cell Cache

	/**
	 * Snapshots the live particles from a buffer slot into State.CellCache[Coord].
	 * Called after generation (cache-write on cache-miss). On re-entry the
	 * stored arrays are blitted directly into the back-buffer, skipping procgen.
	 *
	 * @param Config     Tier descriptor (provides buffer count).
	 * @param State      Tier state (provides Buffers and SlotCounts).
	 * @param Coord      Grid coordinate key for the cache entry.
	 * @param SlotIndex  Source slot to snapshot.
	 * @param BufferIdx  Which of the two buffers to read from.
	 */
	void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx);

	/**
	 * Evicts CellCache entries further than (NeighborhoodRadius + 4) cells
	 * from NewCenter in Chebyshev distance. Called at the end of each
	 * UpdateTier async task, before bUpdateInProgress is cleared, ensuring
	 * exclusive write access to CellCache.
	 *
	 * The +4 buffer keeps two rings beyond the active neighborhood warm for
	 * immediate cache-hits on the next boundary cross in any direction.
	 *
	 * @param Config     Tier descriptor (provides NeighborhoodRadius).
	 * @param State      Tier state (provides CellCache).
	 * @param NewCenter  The tier's new center coord after the boundary cross.
	 */
	void CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& NewCenter);

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

	/** Starts the recurring spawn-scan timer. Called on the game thread after
	 *  initialization completes. */
	void StartSpawnScanTimer();

	/** Clears the timer and empties TrackedSpawnNodes. Called from EndPlay. */
	void StopSpawnScanTimer();

	/**
	 * Timer callback. Dispatches a background octree query, then marshals
	 * enter/exit events back to the game thread. Skips if a scan is already
	 * in progress (bSpawnScanInProgress).
	 */
	void UpdateSpawnRangeNodes();

	/** Logs an ENTER event for a node that crossed into the spawn threshold. */
	void LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const;

	/** Logs an EXIT event for a node that left the spawn threshold. */
	void LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const;

	/** Draws a debug box in world space around InNode for one scan interval. */
	void DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const;

#pragma endregion
};
#pragma endregion
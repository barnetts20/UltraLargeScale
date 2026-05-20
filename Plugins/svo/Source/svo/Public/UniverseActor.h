#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "GalaxyDataGenerator.h"    // FGalaxyParams — exposed at universe level for editor tuning
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
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
class SVO_API AUniverseActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AUniverseActor();

#pragma region Editor Parameters

	/** Full universe generation and tier streaming parameters. Editable in the
	 *  Details panel; changes take effect on the next Initialize() call. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;

	/** Galaxy generation parameters. Owned at the universe level so the
	 *  spiral density field, tier configs, and volume material settings can
	 *  be tuned in-editor without rebuilding. Passed to each galaxy on spawn.
	 *  Will eventually become a proceduralization bounds struct randomized
	 *  per galaxy instance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Properties")
	FGalaxyParams GalaxyParams;

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
	 * Pops a galaxy from the pool, configures params (UnitScale, Seed,
	 * ParentColor, Rotation), marks it hidden with bPendingPlacement = true,
	 * and calls Initialize(). Does NOT position the galaxy or make it visible;
	 * that is deferred to FinalizeGalaxyPlacement once async init completes.
	 *
	 * @param InNode  Octree node representing the galaxy to spawn.
	 */
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);

	/**
	 * Called from Tick on the first frame a galaxy reaches ELifecycleState::Ready
	 * while bPendingPlacement is still true. Computes the spawn position using
	 * the current frame's resolved VirtualTraversal and player position,
	 * initializes the galaxy's VirtualTraversal, makes it visible, and clears
	 * the pending flag. The galaxy's first TickFromParent runs immediately after
	 * in the same frame — zero frames of parallax drift.
	 *
	 * @param Galaxy  The galaxy actor to finalize.
	 */
	void FinalizeGalaxyPlacement(AGalaxyActor* Galaxy);

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

	/** Interval in seconds between spawn-scan dispatches. */
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

	/**
	 * Kicks off the async initialization chain:
	 * InitializeChildPool → InitializeData → InitializeNiagara.
	 * Each step checks InitializationState and bails if Pooling or Destroying
	 * is set. Safe to call from BeginPlay or externally before FinishSpawning.
	 */
	virtual void Initialize() override;

#pragma endregion

protected:
#pragma region Initialization

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;

	virtual void InitializeChildPool() override;
	virtual void InitializeData() override;
	virtual void InitializeNiagara() override;

#pragma endregion

#pragma region Params Accessors
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
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

	/**
	 * Per-frame parallax update. Pegs the actor to the current player position,
	 * advances VirtualTraversal, and re-pushes camera-relative positions to all
	 * active Niagara components. Must be called before UpdateTier each tick so
	 * streaming coord checks use the latest VirtualTraversal.
	 */
	virtual void ApplyParallaxOffset() override;

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
	virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const override;

#pragma endregion

private:
#pragma region Spawn Scan - Internal

	/** Guards against overlapping spawn-scan background tasks. Set true when a
	 *  scan is dispatched; cleared on the game thread after TrackedSpawnNodes
	 *  is updated. */
	std::atomic<bool> bSpawnScanInProgress{ false };

	/** Time of last scan dispatch. Used for interval throttling. */
	double LastScanDispatchTime = 0.0;

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

	/** Dispatches an async octree scan if enough time has elapsed
	 *  since the last dispatch and no scan is already in flight.
	 *  Called by DetermineAndDispatchScan — not by a timer. */
	void RequestScan();

	/**
	 * Walks the active hierarchy deepest-first (star systems → galaxies →
	 * universe) and dispatches exactly one scan per tick to the deepest
	 * level the player is currently inside. Replaces all per-level timers.
	 */
	void DetermineAndDispatchScan();

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
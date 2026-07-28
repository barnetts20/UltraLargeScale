#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "GalaxyDataGenerator.h"
#include "StarSystemDataGenerator.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "DataTypes.h"
#include "FNiagaraParticleBuffer.h"
#include "FTierStreamingSystem.h"
#include "UniverseParams.h"
#include "GalaxyParams.h"
#include "StarSystemParams.h"
#include "UniverseActor.generated.h"
class AGalaxyActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class APostProcessVolume;
class UMaterialInterface;
class UMaterialInstanceDynamic;


#pragma region AUniverseActor
/**
 * Sector-scale universe actor. Owns the three-tier particle streaming system
 * (Large / Mid / Small), the persistent spatial octree, the parallax
 * traversal model, and the galaxy spawn-scan pipeline.
 *
 * Initialization is fully asynchronous: BeginPlay kicks off a background
 * chain (InitializeData -> InitializeNiagara) with game-thread rendezvous
 * only where Niagara component creation requires it. After initialization the
 * actor runs entirely from Tick with no blocking calls.
 *
 * Scale model: the actor is pegged to the player every tick so UE's rendering
 * stays in a clean numerical range. All virtual movement is accumulated in
 * VirtualTraversal and applied to particle positions as camera-relative
 * offsets before each Niagara push.
 */
UCLASS()
class ULTRALARGESCALE_API AUniverseActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AUniverseActor();

#pragma region Backdrop Capture
	/** Master switch for the virtual-backdrop scene capture. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Capture")
	bool bEnableBackdropCapture = true;

	/** Render-target resolution as a fraction of the viewport (1.0 = full res).
	 *  The backdrop is low-frequency (sprites + smooth marches), so it tolerates a
	 *  reduced-res capture well; this is the first knob to drop if the extra pass
	 *  costs too much. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Capture", meta = (ClampMin = "0.25", ClampMax = "1.0"))
	float BackdropResolutionScale = 1.0f;

	/** The HDR render target the virtual stack is captured into. The composite
	 *  post-process material samples this as the backdrop. Null until BeginPlay. */
	UTextureRenderTarget2D* GetBackdropRenderTarget() const { return BackdropRT; }

	/** Debug: hand-assign an RT asset to view the capture in the RT asset editor.
	 *  When set, EnsureBackdropRenderTarget uses it verbatim and skips sizing/alloc. */
	UPROPERTY(EditAnywhere, Category = "Backdrop Capture")
	UTextureRenderTarget2D* DebugRTOverride = nullptr;

	/** SceneDepth (world cm) at/above which a pixel is treated as sky and filled with
	 *  the backdrop. Must sit ABOVE your farthest real geometry (the near planet at max
	 *  view distance) and BELOW the far-plane depth. Too low: backdrop bleeds over a
	 *  distant planet. Too high: a thin sky halo at the planet limb. Tune this first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Composite")
	float BackdropDepthThreshold = 1.0e9f;

	/** Material parameter names on MT_BackdropPostProcess. Must match the asset. */
	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropRTParamName = TEXT("BackdropRT");

	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropDepthThresholdParamName = TEXT("FarThreshold");

	/** Global brightness multiplier for the backdrop, applied in the composite before it
	 *  feeds the main scene's bloom/tonemap. Galaxy sprites were authored hot for the old
	 *  tonemapped path; drop this (start ~0.15-0.3) so they don't over-bloom or trigger
	 *  lens-flare artifacts once composited. Per-emitter emissive is the granular lever. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Composite", meta = (ClampMin = "0.0"))
	float BackdropIntensity = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropIntensityParamName = TEXT("BackdropIntensity");
#pragma endregion

#pragma region Editor Parameters

	FUniverseParams UniverseParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParamBounds UniverseParamBounds;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Properties")
	FGalaxyParamBounds GalaxyParamBounds;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star System Properties")
	FStarSystemParamBounds StarSystemParamBounds;

#pragma endregion

#pragma region Public Octree Queries
	/**
	 * Returns all octree nodes whose screen-space angular size
	 * (Extent * (1 + ScaleFactor))^2 / DistSq exceeds InScreenSpaceThreshold^2.
	 * Traversal prunes entire subtrees whose maximum possible screen size
	 * cannot pass the threshold, making it significantly faster than a full
	 * range query for sparse large-scale distributions. InCenter is the query
	 * origin in sector-local space (typically VirtualTraversal);
	 * InScreenSpaceThreshold is the minimum angular size ratio to pass (squared
	 * internally); InTypeId filters by TypeId (-1 returns all types). Returns the
	 * nodes that pass.
	 */
	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(
		const FVector& InCenter, double InScreenSpaceThreshold, int32 InTypeId = -1) const;

#pragma endregion

#pragma region Spatial Index

	/** Multiplier applied to Params.Extent for octree root size.
	 *  Must be a power of 2. 128 = 2^7: the tree covers +/-128x the sector
	 *  extent per axis (~2^38 local units with the default 2^31 extent);
	 *  CheckOctreeBounds triggers a rebase at 75% of that. */
	static constexpr double PersistentTreeMultiplier = 128.0;

#pragma endregion

#pragma region Galaxy Spawn Hooks

	/** Maps each live octree node to its pooled galaxy actor instance. */
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;

	/**
	 * Pops a galaxy from the pool for InNode (the octree node representing the
	 * galaxy to spawn), configures params (UnitScale, Seed, ParentColor,
	 * Rotation), marks it hidden with bPendingPlacement = true, and calls
	 * Initialize(). Does NOT position the galaxy or make it visible; that is
	 * deferred to FinalizeGalaxyPlacement once async init completes.
	 */
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);

	/**
	 * Called from Tick on the first frame a galaxy reaches ELifecycleState::Ready
	 * while bPendingPlacement is still true. Computes the spawn position using
	 * the current frame's resolved VirtualTraversal and player position,
	 * initializes the galaxy's VirtualTraversal, makes it visible, and clears
	 * the pending flag. The galaxy's first TickFromParent runs immediately after
	 * in the same frame, zero frames of parallax drift. Galaxy is the galaxy
	 * actor to finalize.
	 */
	void FinalizeGalaxyPlacement(AGalaxyActor* Galaxy);

	/**
	 * Returns the galaxy associated with InNode to the pool. Calls
	 * ResetForPool() on the game thread (component teardown), then flushes
	 * the galaxy's octree on a background thread before re-inserting it into
	 * the pool on the game thread. InNode is the octree node whose associated
	 * galaxy should be returned.
	 */
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);

	/**
	 * Tail of ReturnGalaxyToPool: background octree flush (fresh tree built
	 * in a local), then a game-thread hop that swaps the galaxy's Octree
	 * member and re-inserts the galaxy into the pool. Split out so the
	 * deferred (init-draining) return path can share it. The Octree member
	 * swap MUST stay on the game thread; a background assign races GT
	 * readers of the TSharedPtr (BuildStreamingContext, IsPlayerInsideBounds).
	 */
	void FinishGalaxyPoolReturn(TWeakObjectPtr<AGalaxyActor> WeakGalaxy);

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
	 * Sets CellCoord from InCellCoord (the universe grid coordinate for this
	 * sector) and derives CellOrigin, then repositions the actor. Also rebuilds
	 * the octree against the actual Params.Extent in case Params were overridden
	 * after construction. Must be called before Initialize().
	 */
	void ConfigureCell(FIntVector InCellCoord);

#pragma endregion

#pragma region Spawn Range Scanning

	/** When true, logs the live particle count for each entering node's
	 *  buffer slot. Useful for generation tuning; noisy in normal play. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bLogSpawnEnterExitBuffers = false;

#pragma endregion

#pragma region Lifecycle

	/**
	 * Kicks off the async initialization chain:
	 * InitializeChildPool -> InitializeData -> InitializeNiagara.
	 * Each step checks InitializationState and bails if Pooling or Destroying
	 * is set. Safe to call from BeginPlay or externally before FinishSpawning.
	 */
	virtual void Initialize() override;

#pragma endregion

protected:
#pragma region Initialization

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Universe is the root of the tick cascade - it drives itself from Tick and
	*  is never ticked by a parent. Implemented as a no-op purely to satisfy the
	*  abstract base contract. */
	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override {}
	virtual void Tick(float DeltaTime) override;

	virtual void InitializeChildPool() override;
	virtual void InitializeData() override;
	virtual void InitializeNiagara() override;

#pragma endregion

#pragma region Backdrop Capture (internal)
	/** Scene capture that renders ONLY the virtual stack into BackdropRT from the
	 *  main camera's POV each frame. Real geometry (terrain/ocean) is excluded via
	 *  bHiddenInSceneCapture; the virtual stack is bVisibleInSceneCaptureOnly, so no
	 *  ShowOnly list is needed. Composited behind the main scene at Scene Color After
	 *  DOF, so DOF/translucency/atmosphere all draw over it. Owned here because the
	 *  backdrop is the visual form of the universe hierarchy this actor governs. */
	UPROPERTY()
	USceneCaptureComponent2D* BackdropCapture = nullptr;

	/** HDR (RGBA16f) target for BackdropCapture. Transient; recreated on resize. */
	UPROPERTY(Transient)
	UTextureRenderTarget2D* BackdropRT = nullptr;

	/** Last viewport size the RT was sized to, so resolution changes trigger a recreate. */
	FIntPoint BackdropRTSize = FIntPoint::ZeroValue;

	/** Composite base material (MT_BackdropPostProcess), constructor-loaded by path. */
	UPROPERTY()
	UMaterialInterface* CompositeMaterial = nullptr;

	/** Dynamic instance of CompositeMaterial: holds the live RT + threshold params. */
	UPROPERTY()
	UMaterialInstanceDynamic* CompositeMID = nullptr;

	/** Unbound post-process volume this actor spawns and owns; carries CompositeMID
	 *  as a blendable. Always-on, so the backdrop composites in deep space too. */
	UPROPERTY()
	APostProcessVolume* BackdropPPVolume = nullptr;

	/** One-time capture config: capture source, manual-drive, and the show flags
	 *  that keep the real planet's atmosphere/fog out of the backdrop. */
	void InitializeBackdropCapture();

	/** One-time composite setup: MID from CompositeMaterial + owned unbound PP volume. */
	void InitializeBackdropComposite();

	/** Per-frame: sync the capture to the camera POV, then trigger CaptureScene. */
	void UpdateBackdropCapture();

	/** Creates or resizes BackdropRT to the current viewport * BackdropResolutionScale,
	 *  and (re)binds it onto CompositeMID. */
	void EnsureBackdropRenderTarget();
#pragma endregion

#pragma region Params Accessors
	virtual double GetUnitScale() const override { return UniverseParams.UnitScale; }
	virtual double GetExtent() const override { return UniverseParams.Extent; }
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

	/** Large-tier config (cluster + gas). Depth/radius/capacity come from
	 *  UniverseParams.LargeTier (defaults: GridDepth 1, NeighborhoodRadius 1). */
	FParticleTierConfig LargeTierConfig;
	FParticleTierState  LargeTierState;

	/** Mid-tier config. From UniverseParams.MidTier (defaults: GridDepth 3,
	 *  NeighborhoodRadius 1). */
	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;

	/** Small-tier config. From UniverseParams.SmallTier (defaults: GridDepth 5,
	 *  NeighborhoodRadius 1). */
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

#pragma region Parallax

	/**
	 * Per-frame parallax update. Pegs the actor to the current player position,
	 * advances VirtualTraversal, and re-pushes camera-relative positions to all
	 * active Niagara components. Must be called before UpdateTier each tick so
	 * streaming coord checks use the latest VirtualTraversal.
	 */
	virtual void ApplyParallaxOffset(const FVector& InPlayerPos) override;

	/** Schedules a coalesced background per-frame VT push for this actor's tiers.
	 *  Single-flight: bursts collapse to one worker that re-reads the freshest VT. */
	void SchedulePush();

#pragma endregion

#pragma region Galaxy Pool

	/** Class used when pre-warming the galaxy actor pool. */
	TSubclassOf<AGalaxyActor> GalaxyActorClass;

	/** Target pool size. Pool is pre-warmed during InitializeChildPool. */
	int32 GalaxyPoolSize = 5;

	/** Available galaxy actors ready for spawn. Managed as a stack (Pop/Insert). */
	TArray<AGalaxyActor*> GalaxyPool;

#pragma endregion

private:
#pragma region Spawn Scan - Internal

	/** Guards against overlapping spawn-scan background tasks. Set true when a
	 *  scan is dispatched; cleared on the game thread after TrackedSpawnNodes
	 *  is updated. */
	std::atomic<bool> bSpawnScanInProgress{ false };

	// LastScanDispatchTime lives on AProceduralSpaceActor (shared by all
	// scan-capable layers); not redeclared here.

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
	 *  Called by DetermineAndDispatchScan, not by a timer. */
	virtual void RequestScan() override;

	/**
	 * Walks the active hierarchy deepest-first (star systems -> galaxies ->
	 * universe) and dispatches exactly one scan per tick to the deepest
	 * level the player is currently inside. Tick-driven, with no per-level timers.
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
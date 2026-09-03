#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "GalaxyDataGenerator.h"
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
class UActorPoolManager;
class AStarSystemActor;
class AParallaxProxyActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class APostProcessVolume;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UVolumeTexture;


#pragma region AUniverseActor
/**
 * Sector-scale universe actor. Owns the three-tier particle streaming system, the persistent
 * spatial octree, the parallax traversal model, and the galaxy spawn-scan pipeline.
 *
 * Initialization is fully asynchronous, with game-thread rendezvous only where Niagara
 * component creation requires it; afterwards the actor runs entirely from Tick with no
 * blocking calls. The actor is pegged to the player every tick so UE's rendering stays in a
 * clean numerical range, and all virtual movement accumulates in VirtualTraversal.
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

	/** Render-target resolution as a fraction of the viewport. The backdrop is low-frequency,
	 *  so it tolerates a reduced-res capture well; the first knob to drop if the pass costs
	 *  too much. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Capture", meta = (ClampMin = "0.25", ClampMax = "1.0"))
	float BackdropResolutionScale = 1.0f;

	/** Debug: hand-assign an RT asset to view the capture live in its asset editor. When set,
	 *  EnsureBackdropRenderTarget uses it verbatim and skips sizing. */
	UPROPERTY(EditAnywhere, Category = "Backdrop Capture")
	UTextureRenderTarget2D* DebugRTOverride = nullptr;

	/** SceneDepth in world cm at or above which a pixel is treated as sky and filled with the
	 *  backdrop. Must sit ABOVE the farthest real geometry and BELOW the far plane. Too low
	 *  bleeds the backdrop over a distant planet; too high leaves a sky halo at its limb. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Composite")
	float BackdropDepthThreshold = 1.0e9f;

	/** Material parameter names on MT_BackdropPostProcess. Must match the asset. */
	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropRTParamName = TEXT("BackdropRT");

	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropDepthThresholdParamName = TEXT("FarThreshold");

	/** Global brightness multiplier for the backdrop, applied in the composite before it feeds
	 *  the main scene's bloom and tonemap. Galaxy sprites are authored hot, so values around
	 *  0.15-0.3 avoid over-bloom; per-emitter emissive is the granular lever. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Backdrop Composite", meta = (ClampMin = "0.0"))
	float BackdropIntensity = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Backdrop Composite")
	FName BackdropIntensityParamName = TEXT("BackdropIntensity");
#pragma endregion

#pragma region Editor Parameters

	/** THE authored configuration for this sector: the density field, the march, the tier
	 *  streaming setup and the legacy noise graph. The only authoring surface for it --
	 *  nothing copies into it at spawn, so an edit on a placed actor takes effect directly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams UniverseParams;

	/** THE authored configuration for every galaxy this universe spawns: the archetype array,
	 *  the default-mode toggle, and the shared config block. Nothing else authors galaxy
	 *  params -- AGalaxyActor::Params is resolved OUTPUT, overwritten at spawn. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Properties")
	FGalaxySpawnConfig GalaxySpawnConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star System Properties")
	FStarSystemParamBounds StarSystemParamBounds;

#pragma endregion

#pragma region Public Octree Queries
	/** All octree nodes whose screen-space angular size (Extent * (1 + ScaleFactor))^2 / DistSq
	 *  exceeds InScreenSpaceThreshold^2. Traversal prunes subtrees whose maximum possible
	 *  screen size cannot pass, so it beats a full range query on sparse distributions.
	 *
	 *  InCenter is the query origin in sector-local space, typically VirtualTraversal;
	 *  InTypeId of -1 returns all types. */
	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(
		const FVector& InCenter, double InScreenSpaceThreshold, int32 InTypeId = -1) const;

#pragma endregion

#pragma region Spatial Index

	/** Multiplier applied to Params.Extent for octree root size. MUST BE A POWER OF 2. At 128
	 *  the tree covers +/-128x the sector extent per axis, and CheckOctreeBounds triggers a
	 *  rebase at 75% of that. */
	static constexpr double PersistentTreeMultiplier = 128.0;

	/** THE OCTREE ROOT'S HALF-EXTENT, and the one place the product is taken. The
	 *  constructor's tree and the rebase's replacement must be the SAME SIZE: the bounds test
	 *  measures the player against a fraction of the live tree's extent, so a smaller
	 *  replacement brings each rebase closer than the last until they run every tick. */
	double GetPersistentTreeExtent() const
	{
		return UniverseParams.Extent * PersistentTreeMultiplier;
	}

#pragma endregion

#pragma region Galaxy Spawn Hooks

	/** Maps each live octree node to its pooled galaxy actor instance. */
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;

	/** Pops a galaxy from the pool for InNode, configures its params, marks it hidden with
	 *  bPendingPlacement, and calls Initialize(). Does NOT position the galaxy or make it
	 *  visible -- that is deferred to FinalizeGalaxyPlacement once async init completes. */
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);

	/** Called from Tick on the first frame a galaxy reaches Ready while bPendingPlacement is
	 *  still set. Computes the spawn position from the current frame's resolved
	 *  VirtualTraversal and player position, seeds the galaxy's own VirtualTraversal, makes it
	 *  visible, and clears the flag -- so its first TickFromParent runs in the same frame. */
	void FinalizeGalaxyPlacement(AGalaxyActor* Galaxy);

	/** Returns the galaxy associated with InNode to the pool: ResetForPool on the game thread
	 *  for component teardown, then a background octree flush before re-insertion. */
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);

	/** Tail of ReturnGalaxyToPool: background octree flush into a local, then a game-thread hop
	 *  that swaps the galaxy's Octree member and re-pools it. Split out so the deferred
	 *  init-draining return path can share it. THE OCTREE SWAP MUST STAY ON THE GAME THREAD --
	 *  a background assign races GT readers of the TSharedPtr. */
	void FinishGalaxyPoolReturn(TWeakObjectPtr<AGalaxyActor> WeakGalaxy);

#pragma endregion

#pragma region Spawn Range Scanning

	/** Logs the live particle count for each entering node's buffer slot. Useful for
	 *  generation tuning, noisy in normal play. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bLogSpawnEnterExitBuffers = false;

#pragma endregion

#pragma region Lifecycle

	/** Kicks off the async initialization chain: InitializeChildPool -> InitializeData ->
	 *  InitializeNiagara. Each step bails if InitializationState is Pooling or Destroying.
	 *  Safe to call from BeginPlay or externally before FinishSpawning. */
	virtual void Initialize() override;

#pragma endregion

protected:
#pragma region Initialization

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Universe is the root of the tick cascade: it drives itself from Tick and is never ticked
	*  by a parent. A no-op purely to satisfy the abstract base contract. */
	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override {}
	virtual void Tick(float DeltaTime) override;

	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void LoadRuntimeAssets() override;

#pragma endregion

#pragma region Volumetric
	/** The cosmic-web raymarcher. Evaluates UniverseDensityCore.ush directly rather than
	 *  sampling a baked pseudovolume, so the structure it draws is the same function entity
	 *  generation places against. */
	FString VolumetricMaterialPath = FString("/UltraLargeScale/Sector/MT_UniverseRaymarchAnalytic_Inst.MT_UniverseRaymarchAnalytic_Inst");

	/** Pushes the authored field onto a material instance. Draw-constant, so it runs once at
	 *  material creation; the offset has its own per-frame path.
	 *
	 *  THE GRAPH IS A PASS-THROUGH: these are the identical raw values Pack hands
	 *  MakeUniverseDensityParams, and every correlation between them is resolved inside that
	 *  shared derivation. Scaling or combining anything here desyncs render from placement. */
	void PushDensityParams(UMaterialInstanceDynamic* InMID) const;

	/** THE MARCH CONTROLS, pushed separately from the field because none of them reaches
	 *  MakeUniverseDensityParams: they are performance controls, and the entity path samples
	 *  the same field with none of them. */
	void PushMarchParams(UMaterialInstanceDynamic* InMID) const;

	/** THE FIELD OFFSET IS THE ONLY PER-FRAME PARAMETER. This proxy is pegged to the CAMERA
	 *  rather than sitting at a fixed origin as the galaxy layer's does, so the field scrolls
	 *  under it and the offset is how far it has scrolled.
	 *
	 *  Two vector pins, split so the integer part carries magnitude and the fraction carries
	 *  precision. Cheap enough to run unconditionally. */
	void PushFieldOffset(UMaterialInstanceDynamic* InMID) const;

	/** THE FIELD'S REPEAT PERIOD IN SMALL CELLS, derived exactly as MakeUniverseDensityParams
	 *  derives it. Delegates to UniverseCellWrap::FieldCellPeriod so this and
	 *  UniverseDataGenerator cannot arrive at different periods.
	 *
	 *  PUBLIC because a reader needs it to INTERPRET ComputeFieldOffset: the cell index comes
	 *  back in [0, period), so one cell left of the origin reports period-1 rather than -1.
	 *  Correct, but unfoldable back to signed without the period. See UniverseCellWrap. */
public:
	int32 GetFieldCellPeriod() const;
protected:

	/** HALF-EXTENT OF THE PROXY, and the single number both the box scale and the offset
	 *  normalization derive from -- they are the same frame, and two expressions for it drift
	 *  apart without anything looking obviously wrong.
	 *
	 *  IT IS THE LARGE TIER'S NEIGHBOURHOOD, not the sector extent: the marched volume has to
	 *  cover the span the streaming grid has resident, which is (2 * NeighborhoodRadius + 1)
	 *  cells at that tier's depth. At the defaults a proxy sized at Extent would cover the
	 *  centre cell alone, a twenty-seventh of the volume it should.
	 *
	 *  Reads UniverseParams.LargeTier rather than LargeTierConfig, because InitializeVolumetric
	 *  runs before BuildTierConfigs has populated the latter. */
	double GetVolumetricProxyExtent() const;

	/** ONE FIELD CELL IN CALLER UNITS: the proxy half-extent times the authored small cell
	 *  size, and the unit ComputeFieldOffset counts in.
	 *
	 *  UniverseDataGenerator::FieldCellSize IS THE SAME QUANTITY on the other side of the
	 *  handoff, agreeing by construction because the actor sets the generator's FieldExtent
	 *  from GetVolumetricProxyExtent. If that handoff stops, entities are placed against a
	 *  field a fraction of a cell from the one being drawn. */
	double GetFieldCellSize() const;

	/** VirtualTraversal expressed as an exact small-cell count plus a fraction. Computed in
	 *  double and split by floor, because VirtualTraversal reaches the magnitudes this
	 *  coordinate design exists for; what it is handed to is narrower. See the precision note
	 *  on FUniverseFieldOffset.
	 *
	 *  PUBLIC because it is the exact value PushFieldOffset marshals to the material, so a
	 *  reader wanting to DISPLAY a field-space position calls this rather than repeating the
	 *  two divisions against its own copy of the proxy extent and cell size. */
public:
	FUniverseFieldOffset ComputeFieldOffset() const;
protected:

#pragma endregion

#pragma region Backdrop Capture (internal)
	/** Scene capture that renders ONLY the virtual stack into BackdropRT from the main camera's
	 *  POV each frame. Real geometry is excluded via bHiddenInSceneCapture and the virtual
	 *  stack is bVisibleInSceneCaptureOnly, so no ShowOnly list is needed. Composited behind
	 *  the main scene at Scene Color After DOF. */
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

	/** Unbound post-process volume this actor spawns and owns, carrying CompositeMID as a
	 *  blendable. Always on, so the backdrop composites in deep space too. */
	UPROPERTY()
	APostProcessVolume* BackdropPPVolume = nullptr;

	/** One-time capture config: capture source, manual drive, and the show flags that keep the
	 *  real planet's atmosphere and fog out of the backdrop. */
	void InitializeBackdropCapture();

	/** One-time composite setup: MID from CompositeMaterial + owned unbound PP volume. */
	void InitializeBackdropComposite();

	/** Per-frame: sync the capture to the camera POV, then trigger CaptureScene. */
	void UpdateBackdropCapture();

	/** Creates or resizes BackdropRT to the viewport times BackdropResolutionScale, and
	 *  re-binds it onto CompositeMID -- the RT is a new pointer after a resize. */
	void EnsureBackdropRenderTarget();
#pragma endregion

#pragma region Params Accessors
	virtual double GetUnitScale() const override { return UniverseParams.UnitScale; }
	virtual double GetExtent() const override { return UniverseParams.Extent; }
public:
	/** Universe owns the authoritative parallax scale; all lower layers resolve to this. */
	virtual double GetEffectiveSpeedScale() const override { return SpeedScale; }
	virtual UActorPoolManager* GetPoolManager() const override { return PoolManager; }
	virtual AUniverseActor* GetUniverse() const override { return const_cast<AUniverseActor*>(this); }
protected:
#pragma endregion

#pragma region Data Generation

	/** Owns per-tier particle generation. Tier callbacks in BuildTierConfigs delegate here,
	 *  keeping the actor free of procgen implementation details. */
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

	/** The four volume textures the density field fetches, resolved from the authored paths in
	 *  MaterialParams.
	 *
	 *  FOUR ASSETS, TWO CONSUMERS, RESOLVED ONCE: the material instance and the entity-gen
	 *  dispatch are handed the same pointers, so they cannot sample different assets.
	 *
	 *  SEPARATE UPROPERTY MEMBERS rather than an FUniverseFieldTextures member, because this is
	 *  where the REFERENCES live and UPROPERTY is what keeps them alive across a GC.
	 *  FUniverseFieldTextures is a transport struct of raw pointers with no ownership.
	 *
	 *  Loaded in LoadRuntimeAssets, on the game thread, because LoadObject is not thread safe
	 *  and the generator's dispatch path may run on a worker. */
	UPROPERTY()
	UVolumeTexture* FieldVarianceTexA = nullptr;

	UPROPERTY()
	UVolumeTexture* FieldVarianceTexB = nullptr;

	UPROPERTY()
	UVolumeTexture* FieldWarpTexLarge = nullptr;

	UPROPERTY()
	UVolumeTexture* FieldWarpTexSmall = nullptr;

	/** The four as a transport bundle, for the generator and the dispatch. BUILT ON DEMAND
	 *  rather than cached, so it cannot go stale after a pooled reuse reloads them. */
	FUniverseFieldTextures GetFieldTextures() const
	{
		FUniverseFieldTextures Out;
		Out.VarianceA = FieldVarianceTexA;
		Out.VarianceB = FieldVarianceTexB;
		Out.WarpLarge = FieldWarpTexLarge;
		Out.WarpSmall = FieldWarpTexSmall;
		return Out;
	}

#pragma endregion

#pragma region Tier System - Config / State

	/** Large-tier config. Depth, radius and capacity come from UniverseParams.LargeTier. */
	FParticleTierConfig LargeTierConfig;
	FParticleTierState  LargeTierState;

	/** Mid-tier config, from UniverseParams.MidTier. */
	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;

	/** Small-tier config, from UniverseParams.SmallTier. */
	FParticleTierConfig SmallTierConfig;
	FParticleTierState  SmallTierState;

public:
	/** THE FINEST TIER'S CURRENT GRID COORD, and the one worth watching. All three tiers key
	 *  cells on an absolute grid coord that ComposeSeed turns into every entity's seed, and
	 *  unlike the field cell index it does NOT wrap. The finest grid has the smallest cells,
	 *  so its coord climbs fastest and reaches int32 first -- the other two are strictly
	 *  further from their limit.
	 *
	 *  INT32_MIN on every axis is the sentinel for a tier that has never streamed, not a
	 *  coordinate. See FParticleTierState::CenterCoord. */
	FIntVector GetFinestTierCoord() const { return SmallTierState.CenterCoord; }
protected:

	/** GC-safe owner for all Niagara components created by InitializeTier.
	 *  FParticleTierState::NiagaraComponents holds raw aliases into this array, so DO NOT
	 *  reorder or remove entries at runtime. */
	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;

#pragma endregion

#pragma region Tier System - Pipeline

	/** Populates all three tier configs from Params and the assigned Niagara assets: derives
	 *  scale ranges, wires generation callbacks, and builds the shared ComputeBounds lambda.
	 *  Called once at the start of InitializeNiagara. */
	void BuildTierConfigs();

	/** Builds the GPU batch callback for one tier. See the definition for why this is a
	 *  factory rather than three written-out lambdas, and why InSeedOffset must never be
	 *  renumbered. */
	TFunction<bool(const TArray<TPair<FIntVector, int32>>&, TArray<int32>&)>
		MakeTierBatchCallback(
			FParticleTierConfig& InConfig,
			FParticleTierState& InState,
			const FTierParams& InTierParams,
			int32 InSeedOffset);

	/** A FTierStreamingContext snapshot for the current frame. Called by Tick before tier
	 *  updates, and by InitializeNiagara. */
	FTierStreamingContext BuildStreamingContext() const;

#pragma endregion

#pragma region Tier System - Octree Integration
	/** True while an async rebase task owns the octree. UpdateTier and
	 *  CheckOctreeBounds must not proceed while this is set. */
	std::atomic<bool> bRebaseInProgress{ false };

	/** Triggers RebaseOctree when VirtualTraversal comes within 25% of the octree boundary on
	 *  any axis and no tier update is in progress. Called from Tick after all tier updates. */
	void CheckOctreeBounds();

#pragma endregion

#pragma region Parallax

	/** Per-frame parallax update: pegs the actor to the current player position, advances
	 *  VirtualTraversal, and re-pushes camera-relative positions to all active Niagara
	 *  components. MUST run before UpdateTier each tick, so streaming coord checks see the
	 *  latest VirtualTraversal. */
	virtual void ApplyParallaxOffset(const FVector& InPlayerPos) override;

	/** Schedules a coalesced background per-frame VT push for this actor's tiers.
	 *  Single-flight: bursts collapse to one worker that re-reads the freshest VT. */
	void SchedulePush();

#pragma endregion

#pragma region Pool Registration

	/** Galaxy spawn class and prewarm count, fed to the central UActorPoolManager in BeginPlay.
	 *  Acquire and release go through the manager. */
	TSubclassOf<AGalaxyActor> GalaxyActorClass;

	int32 GalaxyPoolSize = 5;

	/** Star-system spawn class and GLOBAL prewarm count -- global concurrent-visible, NOT
	 *  galaxies times per-galaxy, since the player is inside one galaxy at a time. The manager
	 *  grows and warns on exhaustion, so brief transition overflow self-heals. */
	TSubclassOf<AStarSystemActor> StarSystemActorClass;

	int32 StarSystemPoolSize = 5;

	/** Planet proxy spawn class + prewarm count. Proxies wrap the real voxel/mesh body;
	 *  the body persists across the pool round-trip (parked dormant) so it stays warm. */
	TSubclassOf<AParallaxProxyActor> ProxyActorClass;

	int32 PlanetPoolSize = 5;

	/** The single central actor pool. Created and prewarmed in BeginPlay before any child
	 *  activates, torn down in EndPlay, and resolved by every layer via GetPoolManager(). */
	UPROPERTY()
	UActorPoolManager* PoolManager = nullptr;

#pragma endregion

private:
#pragma region Spawn Scan - Internal

	/** Guards against overlapping spawn-scan background tasks. Set when a scan is dispatched,
	 *  cleared on the game thread once TrackedSpawnNodes is updated. */
	std::atomic<bool> bSpawnScanInProgress{ false };

	// LastScanDispatchTime lives on AProceduralSpaceActor, shared by all scan-capable layers.

	/** Nodes currently inside the spawn threshold, diffed each scan interval to produce
	 *  enter/exit events. GAME THREAD ONLY. */
	TSet<TSharedPtr<FOctreeNode>> TrackedSpawnNodes;

	/** Pending scan results written by the async callback, consumed by Tick. Deferred so
	 *  SpawnGalaxyFromPool always sees the current frame's VirtualTraversal and player
	 *  position, rather than landing either side of the parallax update. */
	bool bHasPendingScanResults = false;
	TArray<TSharedPtr<FOctreeNode>> PendingScanResults;

	/** Dispatches an async octree scan if enough time has elapsed and none is in flight.
	 *  Called by DetermineAndDispatchScan, not by a timer. */
	virtual void RequestScan() override;

	/** Walks the active hierarchy deepest-first and dispatches exactly one scan per tick, to
	 *  the deepest level the player is currently inside. */
	void DetermineAndDispatchScan();

	/** Processes pending scan results, after ApplyParallaxOffset has resolved the current
	 *  frame's player position and VirtualTraversal. Called from Tick. */
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
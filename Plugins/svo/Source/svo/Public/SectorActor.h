#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UniverseDataGenerator.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "FOctree.h"
#include "DataTypes.h"
#include "FNiagaraParticleBuffer.h"
#include "SectorActor.generated.h"

class AGalaxyActor;

// ---------------------------------------------------------------------------
// Unified Particle Tier System — structs
// ---------------------------------------------------------------------------

// Replaces FCoarseSlotEntry and FProximitySlotEntry (they were identical).
// Maps a grid coord to its slot index in the flat particle buffers, plus the
// octree nodes inserted from that slot (needed for cleanup on exit).
struct FSlotEntry
{
	int32 SlotIndex = -1;
	TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
};

// Static descriptor for one particle streaming tier. Filled once before
// InitializeTier() and not mutated at runtime. All behavioural differences
// between tiers live here; the generic pipeline reads these fields.
struct FParticleTierConfig
{
	// For logging only (e.g. "Coarse", "Proximity").
	FString TierName;

	// Depth in the octree that defines this tier's grid cell size.
	// Cell extent = GridExtent / (1 << GridDepth), where
	// GridExtent = Params.Extent * GridExtentMultiplier.
	//   Large    = 1  → cell extent = 2*Extent.
	//   Small    = ScanDepth + 1 → cell extent = Extent / 2^ScanDepth.
	int32 GridDepth = 1;

	// Half-width of the 3D neighborhood around the player's current cell.
	// Coarse reads from Params.CoarseNeighborhoodRadius; proximity uses 1.
	// Total slots = (2*NeighborhoodRadius + 1)^3.
	int32 NeighborhoodRadius = 1;

	// Max particles per slot. Coarse = Params.MaxClusterPerCoarseNode;
	// proximity = Params.MaxParticlesPerNode.
	int32 SlotCapacity = 0;

	// One Niagara system template per buffer set. Coarse has two (cluster + gas);
	// proximity has one. InitializeTier creates one UNiagaraComponent per entry.
	TArray<UNiagaraSystem*> NiagaraAssets;

	// Parallel to NiagaraAssets — whether each buffer should allocate the
	// Rotations array. Coarse = {true, false}; proximity = {false}.
	TArray<bool> bWantRotations;

	// Called during parallel generation. Receives the grid coord, slot index,
	// and raw buffer pointers (one per Niagara asset) sized to match
	// NiagaraAssets. The callback writes directly into the slot.
	// Delegates to UniverseDataGenerator generation methods.
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	// Returns the fixed bounds box for all Niagara components in this tier.
	TFunction<FBox()> ComputeBounds;

	// Which buffer index (into the NiagaraAssets array) to walk for octree
	// insertion. Set to -1 to skip octree insert entirely.
	// Coarse = 0 (cluster buffer); proximity = 0 (only buffer).
	int32 OctreeInsertBufferIndex = 0;

	// Optional callback fired inside UpdateTier's async task after the
	// entering/exiting coord sets are computed and exiting slots are freed,
	// but before particle generation begins. Used by the streaming volumetric
	// to update sub-regions of the density buffer in lockstep with the large
	// tier's boundary crosses. Not set for mid/small tiers.
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;
};

// Runtime state for one particle streaming tier. Fully owned by the tier
// pipeline — generation callbacks don't touch these fields directly.
// NOTE: This is a plain struct, not a USTRUCT. UNiagaraComponent* pointers
// stored here are aliased from a UPROPERTY TArray on the actor for GC safety.
//
// Threading contract:
//   - CenterCoord, ActiveSlots, FreeSlots, SlotCounts, and the back-buffer
//     are written exclusively by the async task spawned from UpdateTier.
//     The game thread must not read them while bUpdateInProgress is true.
//   - FrontIdx, bUpdateInProgress, and bNeedsPush are std::atomic and safe
//     for cross-thread reads.
//   - The front-buffer (Buffers[b][FrontIdx]) is read-only to the game
//     thread (PushTierToNiagara) and must not be written by async tasks.
//   - NiagaraComponents are only touched on the game thread.
struct FParticleTierState
{
	// One double-buffered pair per Niagara asset. Outer index = asset index,
	// inner is always size 2 indexed by FrontIdx / (1 - FrontIdx).
	TArray<TArray<FNiagaraParticleBuffer>> Buffers;

	// Raw pointers into the actor's UPROPERTY component array. Parallel to
	// config NiagaraAssets.
	TArray<UNiagaraComponent*> NiagaraComponents;

	// Which of the two buffers is live. Shared across all buffer pairs —
	// they swap in lockstep.
	std::atomic<int32> FrontIdx{ 0 };
	std::atomic<bool> bUpdateInProgress{ false };
	std::atomic<bool> bNeedsPush{ false };

	// Current neighborhood center in this tier's grid coords.
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	// Maps grid coord → slot index + inserted octree nodes.
	TMap<FIntVector, FSlotEntry> ActiveSlots;

	// Available slot indices (used as a stack).
	TArray<int32> FreeSlots;

	// Per-slot live particle count, written by the generate callback.
	TArray<int32> SlotCounts;

	// Persistent cache of generated particle data, keyed by grid coord.
	// Populated on first generation (cache-miss); read back on re-entry
	// (cache-hit) to skip procgen. Data survives cell exit — only the
	// particle buffer slot is recycled, not the cached arrays.
	TMap<FIntVector, FCachedCellData> CellCache;
};

// ---------------------------------------------------------------------------

UCLASS()
class SVO_API ASectorActor : public AActor
{
	GENERATED_BODY()

public:
	ASectorActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;

	// Parallax strength knob — per-tier parallax ratios are derived as
	// SpeedScale / TierUnitScale, so this is the numerator shared across tiers.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
	double SpeedScale = 1.0;
#pragma endregion

#pragma region Lifecycle State
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	// Sector-scope spatial index. Persistent across boundary crosses —
	// nodes survive cell exit and are reused on cache-hit re-entry. The
	// tree is sized to PersistentTreeMultiplier * Params.Extent, large
	// enough that normal traversal stays within bounds. Rebasing only
	// occurs in the exceptional case that VirtualTraversal exits the
	// root bounds.
	//
	// Particles (cluster sprites + galaxy points) are inserted individually
	// at depths derived from each particle's extent via
	// FPointData::MakePointDataFromWorldScale. The tree is a registry, not
	// authoritative storage — Data.ObjectId carries the slot index into the
	// flat particle buffers, and queries dereference back through that
	// index to read pre-quantized positions/extents.
	TSharedPtr<FOctree> Octree;

	// Grid cell sizing multiplier. Defines the streaming cell sizes via
	// CellSize = (Params.Extent * GridExtentMultiplier) / (1 << GridDepth).
	// Kept at 4.0 to preserve existing cell sizes and generation behavior.
	static constexpr double GridExtentMultiplier = 4.0;

	// Octree spatial extent multiplier. The octree is sized to
	// PersistentTreeMultiplier * Params.Extent, giving the player many
	// cell-widths of travel before hitting the bounds. Must be a power
	// of 2 for clean octree subdivision. 128 = 2^7, so tree extent =
	// 2^(31+7) = 2^38, covering ±64× the sector extent per axis.
	static constexpr double PersistentTreeMultiplier = 128.0;

	// Both tiers' particles tag their octree nodes with this TypeId so
	// proximity queries can filter for galaxy content vs. anything else
	// that may live in the tree later.
	static constexpr int32 GalaxyTypeId = 0;

	// Kicks off async init — InitializeChildPool → InitializeData →
	// InitializeVolumetric → InitializeNiagara, with Pooling-state guards
	// between each step.
	void Initialize();
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	// TODO: Wire these into the spawn scanning system once the clean-up pass is complete.
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

#pragma region Sector Grid Identity
	// Which cell in the universe grid this sector represents. Set by the
	// universe before init via ConfigureCell(). All sector position/noise math
	// is offset by CellOrigin so adjacent sectors form a continuous field.
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FIntVector CellCoord = FIntVector::ZeroValue;

	// World-space center of this sector's cell. Derived: CellCoord * (2 * Params.Extent).
	// Set once in ConfigureCell and used as the initial actor placement
	// location, plus as the authoritative sector-grid origin for cross-
	// sector child-spawn math.
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FVector CellOrigin = FVector::ZeroVector;

	// When true (default), the sector calls Initialize() automatically from
	// BeginPlay — convenient for sectors dragged into the level for testing.
	// When spawned by AUniverseActor, ConfigureCell() runs before
	// FinishSpawning so Params are already set when BeginPlay fires.
	UPROPERTY()
	bool bAutoInitializeOnBeginPlay = true;

	// Configure cell identity before Initialize(). Sets CellCoord, derives
	// CellOrigin, places the actor at CellOrigin in world space. Must be
	// called before Initialize().
	void ConfigureCell(FIntVector InCellCoord);
#pragma endregion

protected:
#pragma region Initialization
	void InitializeData();
	void InitializeVolumetric();
	void InitializeNiagara();
	void InitializeChildPool();
#pragma endregion

#pragma region Data Generation
	// Owns all noise composition and particle generation logic.
	// The tier callbacks in BuildTierConfigs() delegate to this generator,
	// keeping the actor free of noise/generation implementation details.
	UniverseDataGenerator UniverseGenerator;
#pragma endregion

#pragma region Niagara Assets
	// Galaxy sprite systems — one per scale band, each using a material instance
	// of the shared MT_GalaxySprite base with its own fade-out range.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorLargeCloud;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorMidCloud;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorSmallCloud;

	// Gas sprite layer — separate material, paired with the large tier.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGasCloud;
#pragma endregion

#pragma region Unified Particle Tier System
	// --- Tier Config/State Pairs ---
	FParticleTierConfig CoarseTierConfig;
	FParticleTierState  CoarseTierState;
	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;
	FParticleTierConfig ProximityTierConfig;
	FParticleTierState  ProximityTierState;

	// GC-safe storage for all Niagara components created by the tier system.
	// FParticleTierState::NiagaraComponents holds raw pointers that alias
	// entries in this array. Do not reorder or remove entries at runtime.
	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;

	// --- Generic Tier Pipeline ---
	// Populates CoarseTierConfig, MidTierConfig, and ProximityTierConfig
	// from Params and the Niagara asset pointers. Generation callbacks
	// delegate to UniverseGenerator methods. Called once at the start of
	// InitializeNiagara.
	void BuildTierConfigs();

	// Allocate buffers → build neighborhood → serial slot alloc → parallel
	// generate (via Config.GenerateCallback) → serial octree insert (if
	// OctreeInsertBufferIndex >= 0) → mirror front→back → game-thread
	// Niagara spawn + bounds + push.
	void InitializeTier(FParticleTierConfig& Config, FParticleTierState& State);

	// Per-tick streaming update. Checks bNeedsPush → push + reinit → checks
	// bUpdateInProgress → coord-diff old vs new neighborhood → free exiting
	// → alloc entering → parallel generate into back → octree insert → swap
	// FrontIdx → set bNeedsPush.
	void UpdateTier(FParticleTierConfig& Config, FParticleTierState& State);

	// For each buffer pair, push Buffers[i][FrontIdx] to NiagaraComponents[i].
	// Also updates the Niagara fixed bounds relative to VirtualTraversal so
	// Lumen/rendering sees a tight bounding box around the actual particles.
	void PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State);

	// Exceptional rebase: destroy the octree, re-create it centered on
	// VirtualTraversal, and re-insert all active slots from all tiers.
	// Only called when VirtualTraversal exits the root bounds — should
	// be extremely rare in normal play.
	void RebaseOctree();

	// Check whether VirtualTraversal is approaching the octree bounds.
	// If so, trigger RebaseOctree. Called from Tick.
	void CheckOctreeBounds();

	// Insert one tier's active front-buffer particles into the octree.
	// Helper for RebaseOctree and InitializeTier.
	void InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx);

	// Insert a single slot's particles into the octree. Used by UpdateTier
	// for incremental insert of entering cells (both cache-hit and
	// cache-miss) without a full tree rebuild.
	void InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State,
		int32 SlotIndex, int32 BufferIdx);

	// Extract the live particles from a buffer slot into a FCachedCellData
	// entry in State.CellCache[Coord]. Called after generation for both
	// InitializeTier and UpdateTier (cache-write on generation).
	void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx);

	// --- Generic Grid Coord Helpers ---
	// All parameterized by GridDepth so both tiers share one implementation.

	// World-local position → grid coord at a given depth.
	// Uses center-aligned lattice: coord (0,0,0) is centered at the origin.
	FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;

	// Grid coord → cell center position.
	FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;

	// Cell half-extent at a given depth.
	double GetGridCellExtent(int32 InGridDepth) const;
#pragma endregion

#pragma region Galaxy Pool
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;

	// In the player-pegged frame, a child actor placed to match parallax depth sits at:
	//   CurrentPlayerPos + (NodeCenter - CellOrigin) * (ThisUnitScale / ChildUnitScale)
	FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;
#pragma endregion

#pragma region AActor Overrides
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;
#pragma endregion

#pragma region Player-Centered Parallax
	// Reference positions tracked frame-to-frame for parallax math.
	// LastFrameOfReferenceLocation is used to compute PlayerDelta;
	// CurrentFrameOfReferenceLocation is the most recent sampled player pos.
	FVector LastFrameOfReferenceLocation = FVector::ZeroVector;
	FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

	// Accumulated virtual-space traversal. Advances by Ratio * PlayerDelta
	// each tick inside ApplyParallaxOffset. The actor itself stays pegged to
	// the player so UE's rendering systems remain in a clean numerical range;
	// VirtualTraversal is the player's position in the sector's virtual frame.
	//
	// Used by:
	//   - PushTierToNiagara to compute camera-relative particle positions
	//     (Relative = LocalPos - VirtualTraversal).
	//   - Streaming coord tracking (PositionToGridCoord consumes
	//     VirtualTraversal directly via UpdateTier).
	//   - DebugDrawSpawnNode and UpdateSpawnRangeNodes.
	//   - Scratch pad broadcast: User.ParallaxOffset = -Ratio * PlayerDelta
	//     per tick, drifts stored particle positions to stay aligned with
	//     VirtualTraversal between pushes.
	FVector VirtualTraversal = FVector::ZeroVector;

	// Per-frame update: peg actor to player, advance VirtualTraversal, and
	// broadcast User.ParallaxOffset to all Niagara scratch pads.
	void ApplyParallaxOffset();
#pragma endregion

#pragma region Public Octree Queries
public:
	// Spatial range query — returns every node within InRadius of InCenter
	// (sector-actor-local space) matching the given TypeId. Pass -1 for no
	// type filter. Both tiers' particles are interleaved in the result.
	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(const FVector& InCenter, double InRadius, int32 InTypeId = -1) const;

	// Screen-space-culled variant. Traverses the tree top-down and prunes nodes
	// whose (Extent * (1 + ScaleFactor)) / Distance falls below InScreenSpaceThreshold.
	// Pass -1 for InTypeId for no type filter.
	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(const FVector& InCenter, double InExtent, double InScreenSpaceThreshold, int32 InTypeId = -1) const;
#pragma endregion

#pragma region Spawn Range Scanning
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

	// Extent of the spatial query box around the player (sector-actor-local space).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScanExtent = 200000.0;

	// Screen-space size threshold, angular-size proxy (Extent / Distance).
	// Lower => smaller things pass => more results.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScreenSpaceThreshold = 0.0001;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bDebugDrawSpawnNodes = true;

	// When true, on enter/exit we log the full cell buffer slice.
	// Useful while tuning but noisy — off by default.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bLogSpawnEnterExitBuffers = false;

private:
	FTimerHandle SpawnScanTimerHandle;
	TSet<TSharedPtr<FOctreeNode>> TrackedSpawnNodes;

	void StartSpawnScanTimer();
	void StopSpawnScanTimer();
	void UpdateSpawnRangeNodes();

	void LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const;
	void LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const;
	void DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const;
#pragma endregion
};
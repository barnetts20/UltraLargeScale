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
	// For logging only (e.g. "Large", "Mid", "Small").
	FString TierName;

	// Depth in the octree that defines this tier's grid cell size.
	// Cell extent = TreeExtent / (1 << GridDepth), where
	// TreeExtent = Params.Extent * TreeExtentMultiplier.
	int32 GridDepth = 1;

	// Half-width of the 3D neighborhood around the player's current cell.
	// Total slots = (2*NeighborhoodRadius + 1)^3.
	int32 NeighborhoodRadius = 1;

	// Max particles per slot.
	int32 SlotCapacity = 0;

	// One Niagara system template per buffer set. Large has two (cluster + gas);
	// mid and small have one each. InitializeTier creates one UNiagaraComponent
	// per entry.
	TArray<UNiagaraSystem*> NiagaraAssets;

	// Parallel to NiagaraAssets — whether each buffer should allocate the
	// Rotations array. Large = {true, false}; mid = {true}; small = {false}.
	TArray<bool> bWantRotations;

	// Called during parallel generation. Receives the grid coord, slot index,
	// and raw buffer pointers (one per Niagara asset) sized to match
	// NiagaraAssets. The callback writes directly into the slot and returns
	// the number of accepted particles.
	TFunction<int32(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	// Returns the fixed bounds box for all Niagara components in this tier.
	TFunction<FBox()> ComputeBounds;

	// Which buffer index (into the NiagaraAssets array) to walk for octree
	// insertion. Set to -1 to skip octree insert entirely.
	// Large = 0 (cluster buffer); mid = 0; small = 0.
	int32 OctreeInsertBufferIndex = 0;

	// Optional callback fired inside UpdateTier's async task after the
	// entering/exiting coord sets are computed and exiting slots are freed,
	// but before particle generation begins. Not currently set for any tier.
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;
};

// Runtime state for one particle streaming tier. Fully owned by the tier
// pipeline — generation callbacks don't touch these fields directly.
// NOTE: This is a plain struct, not a USTRUCT. UNiagaraComponent* pointers
// stored here are aliased from a UPROPERTY TArray on the actor for GC safety.
struct FParticleTierState
{
	// One pair per Niagara asset. Outer index = asset index,
	// inner is always size 2 indexed by ActiveIdx / (1 - ActiveIdx).
	// On boundary cross, continuing slots are copied from active → inactive,
	// entering slots are generated fresh, and exiting slots are dead-stubbed,
	// then ActiveIdx is swapped. This avoids copying the full buffer while
	// keeping both sides consistent.
	TArray<TArray<FNiagaraParticleBuffer>> Buffers;

	// Raw pointers into the actor's UPROPERTY component array. Parallel to
	// config NiagaraAssets.
	TArray<UNiagaraComponent*> NiagaraComponents;

	// Which of the two buffers is currently being read by Niagara.
	// Toggled atomically after the inactive buffer has been updated.
	std::atomic<int32> ActiveIdx{ 0 };
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

	// Sector-scope spatial index. Sized to 4 * Params.Extent and corner-
	// aligned with center = (Params.Extent, Params.Extent, Params.Extent)
	// so that the tree's depth-2 grid lines up with the sector's coarse
	// cell grid.
	TSharedPtr<FOctree> Octree;

	// Tree sizing convention. Multiplier is fixed by the 3x3 coarse
	// neighborhood requirement; if CoarseNeighborhoodRadius ever exceeds 1
	// this needs to scale.
	static constexpr double TreeExtentMultiplier = 4.0;

	// Both tiers' particles tag their octree nodes with this TypeId so
	// proximity queries can filter for galaxy content vs. anything else
	// that may live in the tree later.
	static constexpr int32 GalaxyTypeId = 0;

	// Kicks off async init — InitializeChildPool → InitializeNiagara,
	// with Pooling-state guards between each step.
	void Initialize();
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
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
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FVector CellOrigin = FVector::ZeroVector;

	// Read-only debug display — current player grid coord for each tier.
	// Updated every tick from each tier's CenterCoord.
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid|Debug")
	FIntVector LargeTierPlayerCoord = FIntVector(INT32_MIN);

	UPROPERTY(VisibleAnywhere, Category = "Sector Grid|Debug")
	FIntVector MidTierPlayerCoord = FIntVector(INT32_MIN);

	UPROPERTY(VisibleAnywhere, Category = "Sector Grid|Debug")
	FIntVector SmallTierPlayerCoord = FIntVector(INT32_MIN);

	// When true (default), the sector calls Initialize() automatically from
	// BeginPlay — convenient for sectors dragged into the level for testing.
	// AUniverseActor sets this to false on the sectors it spawns so it can
	// ConfigureCell() first, then drive Initialize() itself.
	UPROPERTY()
	bool bAutoInitializeOnBeginPlay = true;

	// Configure cell identity before Initialize(). Sets CellCoord, derives
	// CellOrigin, places the actor at CellOrigin in world space. Must be
	// called before Initialize().
	void ConfigureCell(FIntVector InCellCoord);
#pragma endregion

protected:
#pragma region Initialization
	void InitializeNiagara();
	void InitializeChildPool();
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
	FParticleTierConfig LargeTierConfig;
	FParticleTierState  LargeTierState;
	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;
	FParticleTierConfig SmallTierConfig;
	FParticleTierState  SmallTierState;

	// Density noise graph — built once during initialization and shared
	// (read-only) across all tiers and threads for generation.
	FastNoise::SmartNode<> DensityNoise;

	// Generator instance — owns generation methods and noise construction.
	// Params are synced from the actor's Params before use.
	UniverseDataGenerator Generator;

	// GC-safe storage for all Niagara components created by the tier system.
	// FParticleTierState::NiagaraComponents holds raw pointers that alias
	// entries in this array. Do not reorder or remove entries at runtime.
	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;

	// --- Generic Tier Pipeline ---
	// Populates tier configs from Params and the Niagara asset pointers.
	// Called once at the start of InitializeNiagara.
	void BuildTierConfigs();

	// Allocate buffers → build neighborhood → serial slot alloc → parallel
	// generate (via Config.GenerateCallback) → serial octree insert (if
	// OctreeInsertBufferIndex >= 0) → game-thread Niagara spawn + bounds + push.
	void InitializeTier(FParticleTierConfig& Config, FParticleTierState& State);

	// Per-tick streaming update. Checks bNeedsPush → push + reinit → checks
	// bUpdateInProgress → coord-diff old vs new neighborhood → clear exiting
	// in inactive buffer → generate entering in inactive buffer → octree
	// insert → swap ActiveIdx → set bNeedsPush.
	void UpdateTier(FParticleTierConfig& Config, FParticleTierState& State);

	// For each buffer pair, push Buffers[i][ActiveIdx] to NiagaraComponents[i].
	void PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State);

	// --- Generic Grid Coord Helpers ---
	// All parameterized by GridDepth so all tiers share one implementation.

	// World-local position → grid coord at a given depth.
	// Uses center-aligned lattice: coord (0,0,0) is centered at the origin.
	FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;
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
	// Player world position from previous frame — used to compute per-frame delta.
	FVector LastFrameOfReferenceLocation = FVector::ZeroVector;

	// Current player world position, sampled once per frame.
	FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

	// --- Parallax Drift with Threshold Rebase ---
	//
	// Rendered position for each particle:
	//   P = Array[i] + RebaseAccum + Drift
	//
	// Array[i]     = raw virtual-space position, only written on cell
	//                enter/exit pushes. CPU buffers are the source of truth.
	// RebaseAccum  = accumulated sum of all previous rebases. Grows in
	//                steps each time Drift exceeds the threshold. Sent to
	//                Niagara as User.ParallaxRebase every frame.
	// Drift        = accumulated Ratio * PlayerDelta since last rebase.
	//                Always small (bounded by RebaseThreshold). Sent to
	//                Niagara as User.ParallaxOffset every frame.
	//
	// On rebase: RebaseAccum += Drift, Drift = 0. No buffer writes,
	// no scratch pad writes. The equation stays balanced because what
	// left Drift got added to RebaseAccum.
	//
	// GridVirtualOffset: separate accumulator for grid coord lookups.
	// Runs forever, never resets. Precision doesn't matter.
	//
	// Niagara Particle Update scratch pad:
	//   Position = PositionArray[UniqueID] + ParallaxRebase + ParallaxOffset
	FVector GridVirtualOffset = FVector::ZeroVector;
	FVector Drift = FVector::ZeroVector;
	FVector RebaseAccum = FVector::ZeroVector;

	// Rebase when any component of Drift exceeds this value.
	// Float precision at 1e6 is ~0.06 units — still sub-unit accuracy.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
	double RebaseThreshold = 2000000000.0;

	// Per-frame update: peg actor to player, advance Drift, check
	// threshold, broadcast User.ParallaxOffset and User.ParallaxRebase
	// to all Niagara components.
	void ApplyParallaxOffset();
#pragma endregion

#pragma region Public Octree Queries
public:
	// Spatial range query — returns every node within InRadius of InCenter
	// (sector-actor-local space) matching the given TypeId. Pass -1 for no
	// type filter. All tiers' particles are interleaved in the result.
	TArray<TSharedPtr<FOctreeNode>> GetNodesInRange(const FVector& InCenter, double InRadius, int32 InTypeId = -1) const;

	// Screen-space-culled variant. Traverses the tree top-down and prunes nodes
	// whose (Extent * (1 + ScaleFactor)) / Distance falls below InScreenSpaceThreshold.
	TArray<TSharedPtr<FOctreeNode>> GetNodesByScreenSpace(const FVector& InCenter, double InExtent, double InScreenSpaceThreshold, int32 InTypeId = -1) const;
#pragma endregion

#pragma region Spawn Range Scanning
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

	// Extent of the spatial query box around the player (sector-actor-local space).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScanExtent = 20000.0;

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

	// Identifies which tier a slot index belongs to. Returns the tier config
	// and state, or nullptr pair if not found.
	TPair<const FParticleTierConfig*, const FParticleTierState*> FindTierForSlot(int32 InSlotIndex) const;
#pragma endregion
};
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
	// Cell extent = TreeExtent / (1 << GridDepth), where
	// TreeExtent = Params.Extent * TreeExtentMultiplier.
	//   Coarse   = 1  → cell extent = 2*Extent (matches current coarse cell size).
	//   Proximity = ScanDepth + 1 → cell extent = Extent / 2^ScanDepth.
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
	// Captures `this` — needs Params, BuildNoise, Octree, etc.
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	// Returns the fixed bounds box for all Niagara components in this tier.
	TFunction<FBox()> ComputeBounds;

	// Which buffer index (into the NiagaraAssets array) to walk for octree
	// insertion. Set to -1 to skip octree insert entirely.
	// Coarse = 0 (cluster buffer); proximity = 0 (only buffer).
	int32 OctreeInsertBufferIndex = 0;
};

// Runtime state for one particle streaming tier. Fully owned by the tier
// pipeline — generation callbacks don't touch these fields directly.
// NOTE: This is a plain struct, not a USTRUCT. UNiagaraComponent* pointers
// stored here are aliased from a UPROPERTY TArray on the actor for GC safety.
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
	// cell grid: 3 of the 4 depth-2 cells along each axis correspond to
	// real sector cells (-1, 0, +1 coords); the 4th is unused buffer.
	//
	// Particles (cluster sprites + galaxy points) are inserted individually
	// at depths derived from each particle's extent via
	// FPointData::MakePointDataFromWorldScale. The tree is a registry, not
	// authoritative storage — Data.ObjectId carries the slot index into the
	// flat particle buffers, and queries dereference back through that
	// index to read pre-quantized positions/extents.
	TSharedPtr<FOctree> Octree;

	// Tree sizing convention. Multiplier is fixed by the 3x3 coarse
	// neighborhood requirement; if CoarseNeighborhoodRadius ever exceeds 1
	// this needs to scale.
	static constexpr double TreeExtentMultiplier = 4.0;

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
	void InitializeData();
	void InitializeVolumetric();
	void InitializeNiagara();
	void InitializeChildPool();
	FastNoise::SmartNode<> BuildNoise(int InSeed);
#pragma endregion

#pragma region Data Generation
	UniverseDataGenerator UniverseGenerator;
#pragma endregion

#pragma region Niagara Assets
	// Niagara system asset for the cluster visualization layer.
	// Assign in the sector actor's Blueprint defaults (e.g. NG_SectorClusterCloud).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorClusterCloud;

	// Niagara system asset for the gas sprite layer.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGasCloud;

	// Niagara system asset for the proximity/galaxy-scale sprite layer.
	// Distinct from the cluster layer so it can carry its own material and fade range.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGalaxyCloud;
#pragma endregion

#pragma region Unified Particle Tier System
	// --- Tier Config/State Pairs ---
	FParticleTierConfig CoarseTierConfig;
	FParticleTierState  CoarseTierState;
	FParticleTierConfig ProximityTierConfig;
	FParticleTierState  ProximityTierState;

	// GC-safe storage for all Niagara components created by the tier system.
	// FParticleTierState::NiagaraComponents holds raw pointers that alias
	// entries in this array. Do not reorder or remove entries at runtime.
	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;

	// --- Generic Tier Pipeline ---
	// Populates CoarseTierConfig and ProximityTierConfig from Params and
	// the Niagara asset pointers. Called once at the start of InitializeNiagara.
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
	void PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State);

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

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");

	// Per-sector raymarched volumetric. Default OFF — 27 simultaneous raymarch
	// volumes in a full grid are unaffordable. Enable on a single directly-placed
	// sector for volumetric debugging or single-sector dev work.
	// Retained as a debug/reference path; the grid-aligned octree may eventually
	// allow a lower-resolution per-cell volumetric to be revisited.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volumetric")
	bool bEnableVolumetric = false;

	// Only populated when bEnableVolumetric is true.
	UPROPERTY()
	UTexture2D* PseudoVolumeTexture;

	UPROPERTY()
	UStaticMeshComponent* VolumetricComponent;

	UPROPERTY()
	UMaterialInstanceDynamic* VolumeMaterial;
#pragma endregion

#pragma region Density Field (CPU-side authoritative copy)
	// Persistent uint8 BGRA8 buffer from SampleNoiseToVolume.
	// Only populated when bEnableVolumetric is true.
	TArray<uint8> DensityBuffer;

	// Non-owning view over DensityBuffer with source-space metadata.
	// Rebuilt whenever DensityBuffer is (re)generated.
	FDensityVolume DensityVolume;
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

#pragma region Tier-Specific Generation Callbacks
	// These remain as named methods wired into FParticleTierConfig.GenerateCallback.
	// Each receives raw buffer pointers and writes directly into the slot.

	// Coarse tier: generates cluster + gas particles using batched noise.
	// Candidates scatter within ±Extent of the cell center; noise offset is
	// shared across all candidates in a cell (coord-derived).
	void GenerateCoarseNode(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InClusterBuffer, FNiagaraParticleBuffer& InGasBuffer);

	// Proximity tier: generates galaxy-scale particles using batched noise.
	// Candidates scatter within ±CellExtent of the cell center; noise offset
	// is computed per-candidate (may straddle coarse cell boundaries).
	void GenerateNodeGalaxies(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer);
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
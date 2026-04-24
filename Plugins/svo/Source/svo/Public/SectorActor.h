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

	// Number of candidate positions to test for gas sprite placement.
	// Final particle count is the subset that pass the density rejection gate.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	int32 GasParticleCount = 15000;

	// Per-particle extent at density=0 (lower bound of the density-driven extent lerp).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	float GasMinExtent = 1e15f;

	// Per-particle extent at density=1 (upper bound of the density-driven extent lerp).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	float GasMaxExtent = 5e16f;
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
	//   - Push* to compute camera-relative particle positions
	//     (Relative = LocalPos - VirtualTraversal).
	//   - Streaming coord tracking (PositionToCoarseCoord / PositionToScanCoord
	//     consume VirtualTraversal directly).
	//   - DebugDrawSpawnNode and UpdateSpawnRangeNodes.
	//   - Scratch pad broadcast: User.ParallaxOffset = -Ratio * PlayerDelta
	//     per tick, drifts stored particle positions to stay aligned with
	//     VirtualTraversal between pushes.
	FVector VirtualTraversal = FVector::ZeroVector;

	// Per-frame update: peg actor to player, advance VirtualTraversal, and
	// broadcast User.ParallaxOffset to all Niagara scratch pads.
	void ApplyParallaxOffset();
#pragma endregion

#pragma region Coarse Cluster Streaming
	// Streams a player-centered 3x3x3 neighborhood of coarse-scale nodes and
	// feeds generated cluster + gas sprite data into two dedicated Niagara
	// components. Cluster and gas share slot indexing (1:1 per coarse node).

	// --- Configuration ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coarse")
	int32 CoarseNeighborhoodRadius = 1;

	// Slot capacity per coarse node. Should be >= Params.Count so every node's
	// full candidate set fits; unused entries are written as dead-particle stubs.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coarse")
	int32 MaxClusterPerCoarseNode = 500;

	// --- Slot State (only touched by async task, guarded by bCoarseUpdateInProgress) ---
	struct FCoarseSlotEntry
	{
		int32 SlotIndex = -1;
		TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
	};

	FIntVector CoarseCenterCell = FIntVector(INT32_MIN);
	TMap<FIntVector, FCoarseSlotEntry> ActiveCoarseNodes;
	TArray<int32> CoarseFreeSlots;
	TArray<int32> CoarseSlotCounts;

	// --- Double-Buffered Particle Data ---
	// Cluster and gas share slot indices but are separate buffers so each can
	// be pushed to its own Niagara component independently.
	FNiagaraParticleBuffer CoarseClusterBuffers[2];
	FNiagaraParticleBuffer CoarseGasBuffers[2];
	std::atomic<int32> CoarseFrontIdx{ 0 };
	std::atomic<bool> bCoarseUpdateInProgress{ false };
	std::atomic<bool> bCoarseNeedsPush{ false };

	// --- Niagara Components ---
	UPROPERTY()
	UNiagaraComponent* CoarseClusterNiagara;

	UPROPERTY()
	UNiagaraComponent* CoarseGasNiagara;

	// --- Methods ---
	void InitializeCoarseSystem();
	void UpdateCoarseNodes();
	// Generation produces particle data using the cell's logical center
	// (CoarseCoordToCenter). Octree insert is a separate post-generation pass.
	void GenerateCoarseNode(const FIntVector& InCoarseCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InClusterBuffer, FNiagaraParticleBuffer& InGasBuffer);
	// Walks the cluster buffer slot and inserts each live particle into the
	// octree. Gas shares 1:1 positions with cluster so only cluster is inserted.
	void InsertCoarseCellIntoOctree(const FIntVector& InCoarseCoord, int32 InSlotIndex, const FNiagaraParticleBuffer& InClusterBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const;
	void PushCoarseToNiagara();

	FIntVector PositionToCoarseCoord(const FVector& InWorldPos) const;
	FVector CoarseCoordToCenter(const FIntVector& InCoord) const;
#pragma endregion

#pragma region Proximity Galaxy Streaming
	// --- Configuration ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 ScanDepth = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 MaxParticlesPerNode = 2000;

	// --- Slot State (only touched by async task, guarded by bProximityUpdateInProgress) ---
	struct FProximitySlotEntry
	{
		int32 SlotIndex = -1;
		TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
	};

	FIntVector CurrentScanCoord = FIntVector(INT32_MIN);
	TMap<FIntVector, FProximitySlotEntry> ActiveNodeSlots;
	TArray<int32> FreeSlots;
	TArray<int32> SlotParticleCounts;

	// --- Double-Buffered Particle Data ---
	FNiagaraParticleBuffer ProximityBuffers[2];
	std::atomic<int32> FrontBufferIndex{ 0 };
	std::atomic<bool> bProximityUpdateInProgress{ false };
	std::atomic<bool> bProximityNeedsPush{ false };

	// --- Proximity Niagara ---
	UPROPERTY()
	UNiagaraComponent* ProximityNiagaraComponent;

	// --- Methods ---
	void InitializeProximitySystem();
	void UpdateProximityNodes();
	// Generation produces particles using the cell's logical center (ScanCoordToCenter).
	// Octree insert is a separate post-generation pass.
	void GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InBuffer);
	void InsertProximityCellIntoOctree(const FIntVector& InNodeCoord, int32 InSlotIndex, const FNiagaraParticleBuffer& InBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const;
	void PushProximityToNiagara();

	FIntVector PositionToScanCoord(const FVector& InLocalPos) const;
	FVector ScanCoordToCenter(const FIntVector& InCoord) const;
	double GetScanNodeExtent() const;
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
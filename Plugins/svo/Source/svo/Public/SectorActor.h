#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UniverseDataGenerator.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "FOctree.h"
#include "DataTypes.h"
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

	// Parallax strength knob — analogous to the base-class SpeedScale that
	// lived on AProceduralSpaceActor. Per-tier parallax ratios are derived as
	// SpeedScale / TierUnitScale, so this is the numerator shared across tiers.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
	double SpeedScale = 1.0;
#pragma endregion

#pragma region Lifecycle State
	// Copied down from the former AProceduralSpaceActor base so external
	// callers (UniverseActor pooling, etc.) and internal async init guards
	// keep working without a base class.
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	// Octree still lives on the sector — retained for the Task 2 proximity
	// data structure work. Not driven by any data pipeline currently; the
	// coarse/fine streaming both sample noise directly.
	TSharedPtr<FOctree> Octree;

	// Kicks off async init — InitializeChildPool → InitializeData →
	// InitializeVolumetric → InitializeNiagara, with Pooling-state guards
	// between each step. Copied from the former base.
	void Initialize();
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

#pragma region Sector Grid Identity
	// Which cell in the universe grid this sector represents. Set by the
	// universe before init via Configure(). All sector position/noise math
	// is offset by CellOrigin so adjacent sectors form a continuous field.
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FIntVector CellCoord = FIntVector::ZeroValue;

	// World-space center of this sector's cell. Derived: CellCoord * (2 * Params.Extent).
	// Set once in ConfigureCell and used as the initial actor placement
	// location, plus as the authoritative sector-grid origin for cross-
	// sector child-spawn math in ComputeChildSpawnLocation. Note the
	// actor itself drifts away from CellOrigin during play due to the
	// parallax offset applied in ApplyParallaxOffset.
	UPROPERTY(VisibleAnywhere, Category = "Sector Grid")
	FVector CellOrigin = FVector::ZeroVector;

	// When true (default), the sector calls Initialize() automatically from
	// BeginPlay — convenient for sectors dragged into the level for testing.
	// AUniverseActor sets this to false on the sectors it spawns so it can
	// ConfigureCell() first, then drive Initialize() itself.
	UPROPERTY()
	bool bAutoInitializeOnBeginPlay = true;

	// Configure cell identity before Initialize(). Sets CellCoord, derives
	// CellOrigin, places the actor at CellOrigin in world space (initial
	// position — ApplyParallaxOffset will peg the actor to the player once
	// init completes). Must be called before Initialize().
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
	// Niagara system asset for the always-loaded cluster visualization layer.
	// Assign in the sector actor's Blueprint defaults (e.g. NG_SectorClusterCloud).
	// Material fade range is configured directly on the material instance.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorClusterCloud;

	// Niagara system asset for the gas sprite layer (rejection-sampled
	// against the sector density volume). Default loaded in the constructor;
	// override in BP defaults if needed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGasCloud;

	// Niagara system asset for the nearby/galaxy-scale sprite layer —
	// populated at runtime by the proximity streaming system (see
	// InitializeProximitySystem / UpdateProximityNodes). Distinct from the
	// cluster layer so it can carry its own material and fade range.
	// Assign in the sector actor's Blueprint defaults (e.g. NG_SectorGalaxyCloud).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	UNiagaraSystem* SectorGalaxyCloud;

	// Number of candidate positions to test for gas sprite placement.
	// Final particle count is the subset that pass the density rejection
	// gate, so denser sectors yield more particles than empty ones.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	int32 GasParticleCount = 15000;

	// Per-particle extent at density=0 (lower bound of the density-driven
	// extent lerp). Tune in the editor to taste.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	float GasMinExtent = 1e15f;

	// Per-particle extent at density=1 (upper bound of the density-driven
	// extent lerp). Tune in the editor to taste.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Niagara")
	float GasMaxExtent = 5e16f;
#pragma endregion

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");

	// Per-sector raymarched volumetric. Default OFF — when running in a
	// multi-sector grid (3x3x3 = 27 sectors), 27 raymarch volumes are
	// unaffordable. Enable on a single directly-placed sector for
	// volumetric debugging or single-sector dev work.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volumetric")
	bool bEnableVolumetric = false;

	// Volumetric component resources. Previously inherited from the base;
	// now local since the sector is a bare AActor. Only populated when
	// bEnableVolumetric is true. VolumeMaterial is kept as a member for
	// symmetry with the former shared layout; it is unused today but keeps
	// the cookbook intact for future material updates.
	UPROPERTY()
	UTexture2D* PseudoVolumeTexture;

	UPROPERTY()
	UStaticMeshComponent* VolumetricComponent;

	UPROPERTY()
	UMaterialInstanceDynamic* VolumeMaterial;
#pragma endregion

#pragma region Density Field (CPU-side authoritative copy)
	// Persistent uint8 BGRA8 buffer from SampleNoiseToVolume. Kept alive for the
	// lifetime of the sector so CPU systems (rejection sampling, etc.) can query
	// density directly instead of going through the octree.
	TArray<uint8> DensityBuffer;

	// Non-owning view over DensityBuffer with source-space metadata. Rebuilt
	// whenever DensityBuffer is (re)generated. Sample via SampleDensityAtLocalPos.
	FDensityVolume DensityVolume;
#pragma endregion

#pragma region Galaxy Pool
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;

	// Reimplemented locally now that there's no AProceduralSpaceActor to
	// inherit from. In the new player-pegged frame, a child actor placed to
	// "look right" given the parallax depth story sits at:
	//   CurrentPlayerPos + (NodeCenter - CellOrigin) * (ThisUnitScale / ChildUnitScale)
	// i.e. scale the cell-local offset by the unit-scale ratio so nearer-tier
	// spawns appear closer to the player. Preserved for the SpawnGalaxyFromPool
	// surface; not wired into any streaming path yet (see Task 3).
	FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;
#pragma endregion

#pragma region AActor Overrides
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;
#pragma endregion

#pragma region Player-Centered Parallax
	// Reference positions tracked frame-to-frame for parallax math. Copied
	// down from the former base class; LastFrameOfReferenceLocation is used
	// to compute PlayerDelta, CurrentFrameOfReferenceLocation is the most
	// recent sampled player pos (useful for child spawn calls).
	FVector LastFrameOfReferenceLocation = FVector::ZeroVector;
	FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

	// Per-frame update: compute player delta, drift actor by
	// -PlayerDelta*Ratio, broadcast ParallaxOffset to each Niagara
	// component (they also SetWorldLocation(PlayerPos)). Single-ratio
	// model — Ratio = SpeedScale / Params.UnitScale, same as the
	// pre-refactor AProceduralSpaceActor::ApplyParallaxOffset.
	void ApplyParallaxOffset();
#pragma endregion

#pragma region Coarse Cluster Streaming
	// Streams a player-centered 3x3x3 neighborhood of coarse-scale nodes
	// (each node == one sector's worth of extent today) and feeds their
	// generated cluster + gas sprite data into two dedicated Niagara
	// components. Parallel to the Proximity Galaxy Streaming region below —
	// same pattern (slot pool, double-buffered arrays, OldSet/NewSet diff on
	// boundary cross, async generation), one tier up.
	//
	// Cluster and gas share slot indexing: each coarse node owns one slot
	// that holds both its cluster sprites and its 1:1 matched gas sprites.

	// --- Configuration ---
	// Half-width of the active coarse grid (1 = 3x3x3 = 27 nodes). Keep at 1
	// for now; wider radii multiply streaming work and memory proportionally.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coarse")
	int32 CoarseNeighborhoodRadius = 1;

	// Slot capacity per coarse node. Each slot reserves this many contiguous
	// particle entries in the flat packed buffer; unused entries get written
	// as dead-particle stubs (extent=0, off-screen position). Should be >=
	// Params.Count so every node's full candidate set fits.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coarse")
	int32 MaxClusterPerCoarseNode = 500;

	// --- Slot State (only touched by async task, guarded by bCoarseUpdateInProgress) ---
	FIntVector CoarseCenterCell = FIntVector(INT32_MIN);
	TMap<FIntVector, int32> ActiveCoarseNodes;
	TArray<int32> CoarseFreeSlots;
	TArray<int32> CoarseSlotCounts;

	// --- Double-Buffered Particle Data ---
	// Cluster and gas share slot indexing (1:1 per coarse node), so both
	// live in the same buffer struct and get swapped together.
	struct FCoarseBuffer
	{
		// Cluster arrays (one entry per cluster sprite)
		TArray<FVector>      ClusterPositions;
		TArray<FVector>      ClusterRotations;
		TArray<float>        ClusterExtents;
		TArray<FLinearColor> ClusterColors;

		// Gas arrays (one entry per gas sprite, 1:1 with cluster)
		TArray<FVector>      GasPositions;
		TArray<float>        GasExtents;
		TArray<FLinearColor> GasColors;

		void Allocate(int32 TotalParticles)
		{
			ClusterPositions.SetNumZeroed(TotalParticles);
			ClusterRotations.SetNumZeroed(TotalParticles);
			ClusterExtents.SetNumZeroed(TotalParticles);
			ClusterColors.SetNumZeroed(TotalParticles);
			GasPositions.SetNumZeroed(TotalParticles);
			GasExtents.SetNumZeroed(TotalParticles);
			GasColors.SetNumZeroed(TotalParticles);
		}
	};

	FCoarseBuffer CoarseBuffers[2];
	std::atomic<int32> CoarseFrontIdx{ 0 };
	std::atomic<bool> bCoarseUpdateInProgress{ false };
	std::atomic<bool> bCoarseNeedsPush{ false };

	// --- Niagara Components ---
	// Two sector-level components, one per visual layer. Streamed buffers,
	// double-buffered, slot-packed — so they have their own lifecycle rather
	// than fitting into any one-shot init pattern.
	UPROPERTY()
	UNiagaraComponent* CoarseClusterNiagara;

	UPROPERTY()
	UNiagaraComponent* CoarseGasNiagara;

	// --- Methods ---
	void InitializeCoarseSystem();
	void UpdateCoarseNodes();
	void GenerateCoarseNode(const FIntVector& InCoarseCoord, int32 InSlotIndex, FCoarseBuffer& InBuffer);
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
	FIntVector CurrentScanCoord = FIntVector(INT32_MIN);
	TMap<FIntVector, int32> ActiveNodeSlots;
	TArray<int32> FreeSlots;
	TArray<int32> SlotParticleCounts;

	// --- Double-Buffered Particle Data ---
	struct FProximityBuffer
	{
		TArray<FVector> Positions;
		TArray<float> Extents;
		TArray<FLinearColor> Colors;

		void Allocate(int32 TotalParticles)
		{
			Positions.SetNumZeroed(TotalParticles);
			Extents.SetNumZeroed(TotalParticles);
			Colors.SetNumZeroed(TotalParticles);
		}
	};

	FProximityBuffer ProximityBuffers[2];
	std::atomic<int32> FrontBufferIndex{ 0 };
	std::atomic<bool> bProximityUpdateInProgress{ false };
	std::atomic<bool> bProximityNeedsPush{ false };

	// --- Proximity Niagara ---
	UPROPERTY()
	UNiagaraComponent* ProximityNiagaraComponent;

	// --- Methods ---
	void InitializeProximitySystem();
	void UpdateProximityNodes();
	void GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FProximityBuffer& InBuffer);
	void PushProximityToNiagara();

	FIntVector PositionToScanCoord(const FVector& InLocalPos) const;
	FVector ScanCoordToCenter(const FIntVector& InCoord) const;
	double GetScanNodeExtent() const;
#pragma endregion
};
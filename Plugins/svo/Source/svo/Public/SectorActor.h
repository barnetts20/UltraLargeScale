#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "FOctree.h"
#include "SectorActor.generated.h"

class AGalaxyActor;

/// <summary>
/// Per-layer data bundle for one Niagara visualization system on a sector.
/// Holds the unified set of particle arrays (Positions/Rotations/Extents/Colors)
/// and a pointer to the Niagara system asset the sector will spawn for this
/// layer. Behavioural differences between layers (cluster vs. gas vs. sector-
/// wide galaxies) are expressed entirely in the Niagara asset + material �
/// this struct is just data.
///
/// Population helpers like PopulateFromPointNodes fill the arrays from a
/// specific data source. Add more helpers as new layer sources appear (e.g.
/// PopulateFromDensityVolume for the gas system, PopulateFromLargeObjects for
/// sector-wide galaxies).
///
/// Sector owns a TArray<FSectorNiagaraLayerData> built during InitializeData;
/// InitializeNiagara then walks it, spawns a UNiagaraComponent per layer, and
/// pushes the User.* arrays in one unified block.
/// </summary>
USTRUCT()
struct SVO_API FSectorNiagaraLayerData
{
	GENERATED_BODY()

	// Niagara system asset to spawn for this layer. Hard ref (UPROPERTY) so
	// the cooker tracks it and GC keeps it pinned.
	UPROPERTY()
	TObjectPtr<UNiagaraSystem> SystemAsset = nullptr;

	// Layer name for debug logging. Not shown to users.
	FName LayerName;

	// Unified particle data. All layers get the same array set; any layer
	// whose Niagara asset doesn't read a given binding simply ignores it.
	TArray<FVector>       Positions;
	TArray<FVector>       Rotations;
	TArray<float>         Extents;
	TArray<FLinearColor>  Colors;

	FSectorNiagaraLayerData() = default;

	/// <summary>
	/// Fill the arrays from a list of point nodes inserted into the octree
	/// by UniverseDataGenerator. Builds Position/Rotation/Extent/Color per
	/// node — rotation is derived from a stable per-ObjectId random stream.
	/// InWorldOffset is added to every output position; pass the sector's
	/// CellOrigin to render at the correct world location in a multi-sector
	/// grid. Safe to call on any thread (ParallelFor inside).
	/// </summary>
	void PopulateFromPointNodes(
		const TArray<TSharedPtr<FOctreeNode>>& InPointNodes,
		UNiagaraSystem* InSystemAsset,
		FName InLayerName,
		FVector InWorldOffset = FVector::ZeroVector);

	/// <summary>
	/// Fill the arrays by uniform-random rejection sampling against a
	/// density volume — InSampleCount candidate positions are tested and the
	/// accepted subset is written. Output array size equals the accepted
	/// count, not InSampleCount.
	///
	/// Per accepted particle:
	///  - Position: candidate sector-local position + InWorldOffset
	///  - Extent:   lerp(InMinExtent, InMaxExtent, density)
	///  - Color:    FLinearColor(1, 1, 1, density) — RGB free for material
	///              tinting, alpha carries the per-particle density value
	///  - Rotation: zero (gas sprites are billboards / radially symmetric)
	///
	/// InWorldOffset shifts every output position; density is sampled
	/// pre-offset (in sector-local space). Threadsafe; uses ParallelFor with
	/// an atomic write index.
	/// </summary>
	void PopulateFromDensityVolume(
		const FDensityVolume& InDensityVolume,
		double InSectorExtent,
		int32 InSampleCount,
		float InMinExtent,
		float InMaxExtent,
		int32 InSeed,
		UNiagaraSystem* InSystemAsset,
		FName InLayerName,
		FVector InWorldOffset = FVector::ZeroVector);
};

UCLASS()
class SVO_API ASectorActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	ASectorActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;
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
	// called before Initialize() — does not trigger generation itself.
	void ConfigureCell(FIntVector InCellCoord);
#pragma endregion

protected:
#pragma region Params Accessors
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override { return SpeedScale; }
#pragma endregion

#pragma region Initialization
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void InitializeChildPool() override;
	FastNoise::SmartNode<> BuildNoise(int InSeed);
#pragma endregion

#pragma region Data Generation
	UniverseDataGenerator UniverseGenerator;
#pragma endregion

#pragma region Niagara Layers
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

	// CPU-side data for each layer. Populated during InitializeData,
	// consumed during InitializeNiagara. Index-parallel with LayerComponents.
	TArray<FSectorNiagaraLayerData> LayerData;

	// Live UNiagaraComponents spawned from LayerData entries during
	// InitializeNiagara. Index-parallel with LayerData. UPROPERTY keeps them
	// GC-rooted; EndPlay explicitly destroys them before the engine's PIE-end
	// stale-reference scan runs.
	UPROPERTY()
	TArray<TObjectPtr<UNiagaraComponent>> LayerComponents;
#pragma endregion

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");

	// Per-sector raymarched volumetric. Default OFF — when running in a
	// multi-sector grid (3x3x3 = 27 sectors), 27 raymarch volumes are
	// unaffordable. Enable on a single directly-placed sector for
	// volumetric debugging or single-sector dev work.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volumetric")
	bool bEnableVolumetric = false;
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
#pragma endregion

#pragma region Overrides
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void ApplyParallaxOffset() override;
	virtual void Tick(float DeltaTime) override;
#pragma endregion

#pragma region Player-Centered Parallax
	double ParallaxRatio = 0.0;
#pragma endregion

#pragma region Proximity Galaxy Streaming
	// --- Configuration ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 ScanDepth = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 MaxParticlesPerNode = 2000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 RejectionOversampleFactor = 4;

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
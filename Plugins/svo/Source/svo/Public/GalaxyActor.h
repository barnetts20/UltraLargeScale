// GalaxyActor.h
// Full tier streaming system mirroring UniverseActor.
// Large tier: exhaustive single-pass (NeighborhoodRadius=0), always loaded.
// Mid/Small tiers: neighborhood streaming with cell cache.
// Spawn scan: timer-based octree query (VirtualTraversal space) drives
//             SpawnStarSystemFromPool / ReturnStarSystemToPool, mirroring
//             the universe-level galaxy spawn pipeline exactly.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyDataGenerator.h"
#include "FTierStreamingSystem.h"
#include "UniverseActor.h"
#include "GalaxyActor.generated.h"

class AStarSystemActor;

UCLASS()
class SVO_API AGalaxyActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AGalaxyActor();
	~AGalaxyActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Galaxy Properties")
	FGalaxyParams Params;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Galaxy Parent Actor")
	AUniverseActor* Universe;

	UPROPERTY(EditAnywhere, Category = "Galaxy Properties")
	bool bAutoInitializeOnBeginPlay = false;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Virtual traversal of the player through galaxy-local space.
	FVector VirtualTraversal = FVector::ZeroVector;

	/** VirtualTraversal value at the last Niagara position push. */
	FVector LastPushedVirtualTraversal = FVector::ZeroVector;

	/** Minimum VT delta before re-pushing positions to Niagara. */
	double ParallaxPushThreshold = 0.5;
#pragma endregion

#pragma region Spawn Range Scanning (public - tunable in editor)
	/** Interval in seconds between spawn-scan background queries. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

	/** Minimum screen-space angular size (Extent / Distance) for a node to
	 *  trigger a star system spawn. Squared internally before traversal.
	 *  Lower values = spawn/despawn from further away.
	 *  UniverseActor default is 0.033. For star systems to load earlier
	 *  (before you're right on top of the sprite), try 0.005–0.015. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScreenSpaceThreshold = 0.01;

	/** When true, draws a debug box around each node that passes the threshold. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bDebugDrawSpawnNodes = false;
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AStarSystemActor>> SpawnedStarSystems;
	void SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

#pragma region Initialization
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void InitializeChildPool() override;
	virtual void ResetForPool() override;
	virtual void ResetForSpawn() override;
#pragma endregion

	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override;

protected:
#pragma region Params Accessors
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override {
		return Universe ? Universe->SpeedScale : SpeedScale;
	}
#pragma endregion

#pragma region Data Generation
	GalaxyDataGenerator GalaxyGenerator;
#pragma endregion

#pragma region Niagara Assets
	UPROPERTY()
	UNiagaraSystem* GalaxyLargeCloud;

	UPROPERTY()
	UNiagaraSystem* GalaxyMidCloud;

	UPROPERTY()
	UNiagaraSystem* GalaxySmallCloud;
#pragma endregion

#pragma region Tier System - Config / State
	FParticleTierConfig LargeTierConfig;
	FParticleTierState  LargeTierState;

	FParticleTierConfig MidTierConfig;
	FParticleTierState  MidTierState;

	FParticleTierConfig SmallTierConfig;
	FParticleTierState  SmallTierState;

	UPROPERTY()
	TArray<UNiagaraComponent*> TierNiagaraComponents;
#pragma endregion

#pragma region Tier System - Pipeline
	void BuildTierConfigs();
	FTierStreamingContext BuildStreamingContext() const;

	/// Returns true if the given grid coord's cell overlaps the galaxy volume.
	bool CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const;
#pragma endregion

#pragma region Tier System - Grid Coord Helpers
	FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;
	FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;
	double GetGridCellExtent(int32 InGridDepth) const;
	static constexpr double GridExtentMultiplier = 4.0;
#pragma endregion

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_GalaxyRaymarchPseudoVolume_Inst.MT_GalaxyRaymarchPseudoVolume_Inst");
#pragma endregion

#pragma region Star System Pool
	TSubclassOf<AStarSystemActor> StarSystemActorClass;
	int32 StarSystemPoolSize = 5;
	TArray<AStarSystemActor*> StarSystemPool;

	/// Mirrors AUniverseActor::ComputeChildSpawnLocation, accounts for VT.
	virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const override;

	/// Galaxy parallax is handled inline in TickFromParent (VirtualTraversal model).
	virtual void ApplyParallaxOffset() override;

	/** Deferred placement: finalizes world position and VirtualTraversal for a
	 *  star system on the first tick after async init completes, mirroring
	 *  AUniverseActor::FinalizeGalaxyPlacement. */
	void FinalizeStarSystemPlacement(AStarSystemActor* System);
#pragma endregion

#pragma region Tick
	virtual void Tick(float DeltaTime) override;
#pragma endregion

#pragma region Diagnostics
	int32 DiagTickCount = 0;
#pragma endregion

private:
#pragma region Spawn Scan - Internal
	/** Guards against overlapping spawn-scan background tasks. */
	std::atomic<bool> bSpawnScanInProgress{ false };

	/** Timer handle for the recurring spawn-scan interval. */
	FTimerHandle SpawnScanTimerHandle;

	/** Nodes currently inside the spawn threshold. Diffed each interval. */
	TSet<TSharedPtr<FOctreeNode>> TrackedSpawnNodes;

	/** Pending results from the async octree query, consumed in TickFromParent
	 *  after VirtualTraversal is resolved for the current frame. */
	bool bHasPendingScanResults = false;
	TArray<TSharedPtr<FOctreeNode>> PendingScanResults;

	void StartSpawnScanTimer();
	void StopSpawnScanTimer();
	void UpdateSpawnRangeNodes();
	void ProcessPendingScanResults();

	void LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const;
	void LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const;
	void DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const;
#pragma endregion
};
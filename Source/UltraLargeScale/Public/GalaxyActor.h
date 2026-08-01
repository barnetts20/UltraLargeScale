// GalaxyActor.h
// Full tier streaming system mirroring UniverseActor.
// Large tier: exhaustive single-pass (NeighborhoodRadius=0), always loaded.
// Mid/Small tiers: neighborhood streaming with cell cache.
// Spawn scan: driven by Universe::DetermineAndDispatchScan via RequestScan(),
//             dispatches async octree query (VirtualTraversal space) to drive
//             SpawnStarSystemFromPool / ReturnStarSystemToPool.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyDataGenerator.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "UniverseActor.h"
#include "GalaxyActor.generated.h"

class AStarSystemActor;

UCLASS()
class ULTRALARGESCALE_API AGalaxyActor : public AProceduralSpaceActor
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

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AStarSystemActor>> SpawnedStarSystems;
	void SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode);

	/**
	 * Tail of ReturnStarSystemToPool: background octree flush (fresh tree
	 * built in a local), then a game-thread hop that swaps the system's
	 * Octree member and re-inserts it into the pool. Split out so the
	 * deferred (init-draining) return path can share it. The Octree member
	 * swap must stay on the game thread - a background assign races GT
	 * readers of the TSharedPtr.
	 */
	void FinishStarSystemPoolReturn(TWeakObjectPtr<AStarSystemActor> WeakSystem);
#pragma endregion

#pragma region Initialization
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void LoadRuntimeAssets() override;
	virtual void ResetForPool() override;
	virtual void ResetForSpawn() override;

	/** Typed re-init for a pooled galaxy: sets Params, arms deferred placement from
	 *  InXform's location (final placement stays with FinalizeGalaxyPlacement), and
	 *  runs the existing async init chain. Called by the parent after Acquire<>() +
	 *  association. Consumed in Phase B. */
	void ReInit(const FGalaxyParams& InParams, const FTransform& InXform);
#pragma endregion

	virtual void ApplyParallaxOffset(const FVector& InPlayerPos) override;

	/** Schedules a coalesced background per-frame VT push for this actor's tiers.
	 *  Single-flight: bursts collapse to one worker that re-reads the freshest VT. */
	void SchedulePush();
	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override;

protected:
#pragma region Params Accessors
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
public:
	virtual double GetEffectiveSpeedScale() const override { return Universe ? Universe->GetEffectiveSpeedScale() : 1.0; }
	virtual UActorPoolManager* GetPoolManager() const override { return Universe ? Universe->GetPoolManager() : nullptr; }
	virtual AUniverseActor* GetUniverse() const override { return Universe; }
protected:
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

	/** Returns true if the given grid coord's cell overlaps the galaxy volume. */
	bool CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const;
#pragma endregion

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/UltraLargeScale/Galaxy/MT_GalaxyRaymarchPseudoVolume_Inst.MT_GalaxyRaymarchPseudoVolume_Inst");
#pragma endregion

#pragma region Star System Placement
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

	/** Nodes currently inside the spawn threshold. Diffed each interval. */
	TSet<TSharedPtr<FOctreeNode>> TrackedSpawnNodes;

	/** Pending results from the async octree query, consumed in TickFromParent
	 *  after VirtualTraversal is resolved for the current frame. */
	bool bHasPendingScanResults = false;
	TArray<TSharedPtr<FOctreeNode>> PendingScanResults;

	void ProcessPendingScanResults();

	void LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const;
	void LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const;
	void DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const;
#pragma endregion

public:
#pragma region Hierarchical Scan (called by Universe)
	/** Dispatches an async scan if enough time has elapsed. Called by
	 *  Universe::DetermineAndDispatchScan, not by a timer. */
	virtual void RequestScan() override;

	/** Returns true if VirtualTraversal is within this galaxy's octree bounds. */
	virtual bool IsPlayerInsideBounds() const override;
#pragma endregion
};
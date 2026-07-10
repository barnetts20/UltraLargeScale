// StarSystemActor.h
// Tier streaming system mirroring GalaxyActor.
// Large tier: planets along a line at orbit distance, always loaded, neighbor-scanned.
// Mid/Small tiers: present but zero-particle placeholders for now.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "StarSystemDataGenerator.h"
#include "FTierStreamingSystem.h"
#include "StarSystemParams.h"
#include "StarSystemActor.generated.h"

class AGalaxyActor;
class AParallaxStaticMeshActor;

// ---------------------------------------------------------------------------
// AStarSystemActor
// ---------------------------------------------------------------------------
UCLASS()
class SVO_API AStarSystemActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AStarSystemActor();
	virtual ~AStarSystemActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "StarSystem Properties")
	FStarSystemParams Params;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "StarSystem Parent Actor")
	AGalaxyActor* Galaxy;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#pragma endregion

#pragma region Spawn Range Scanning (public - tunable in editor)
	//TODO: SCAN INTERVAL SHOULD PROBABLY BE MANAGED CENTRALLY FOR ALL LAYERS... NEED TO CHECK WHATS GOING ON WITH THIS AND POTENTIALLY HOIST. SIMILAR TO TICK, THIS SHOULD PROBABLY BE ORCHESTRATED TOP DOWN (MAYBE IT ALREADY IS)
	/** Interval in seconds between planet spawn-scan dispatches. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

	//TODO: HOIST?
	/** Screen-space threshold for planet spawn/despawn.
	 *  Smaller = spawn from further away. Try 0.01-0.05. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	double SpawnScreenSpaceThreshold = 0.02;

	/** Draw debug boxes around nodes that pass the spawn threshold. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	bool bDebugDrawSpawnNodes = false;
#pragma endregion

#pragma region Pool Lifecycle
	virtual void ResetForPool() override;
	virtual void ResetForSpawn() override;
#pragma endregion

#pragma region Tick
	virtual void Tick(float DeltaTime) override;
	virtual void ApplyParallaxOffset(const FVector& InPlayerPos) override;

	/** Schedules a coalesced background per-frame VT push for this actor's tiers.
	 *  Single-flight: bursts collapse to one worker that re-reads the freshest VT. */
	void SchedulePush();
	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override;
#pragma endregion

#pragma region Planet Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AActor>> SpawnedPlanets;

	void SpawnPlanetFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnPlanetToPool(TSharedPtr<FOctreeNode> InNode);
	void FinalizePlanetPlacement(AActor* Planet, TSharedPtr<FOctreeNode> InNode);
#pragma endregion

protected:
#pragma region Params Accessors (pure virtual implementations)
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override;
#pragma endregion

#pragma region Initialization
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	// TOOD: COMMENT BELOW IS ONLY ACCURATE CURRENTLY... THIS WILL EVENTUALLY BE MERGED WITH OUR PLANET PROJECT AND THEN IT WOULD LIKELY BE USING A HYRBID POOL APPROACH
	// COMPLEX ACTORS SUCH AS PLANETS WOULD BE POOLED, SIMPLE ACTORS (STATIC MESHES ETC) MAY JUST SPAWN WITH NO POOL
	// No InitializeChildPool — star systems manage individual actor spawns, not a pool.

#pragma endregion

#pragma region Data Generation
	StarSystemDataGenerator SystemGenerator;

	TArray<FVector>       PlanetPositions;
	TArray<float>         PlanetExtents;
	TArray<FLinearColor>  PlanetColors;
#pragma endregion

#pragma region Niagara Assets
	UPROPERTY()
	UNiagaraSystem* StarSystemLargeCloud;

	UPROPERTY()
	UNiagaraSystem* StarSystemMidCloud;

	UPROPERTY()
	UNiagaraSystem* StarSystemSmallCloud;
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
#pragma endregion

#pragma region Tier System - Grid Coord Helpers
	FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const;
	FVector    GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const;
	double     GetGridCellExtent(int32 InGridDepth) const;

	/** True if the cell's AABB overlaps the system's orbital volume
	 *  (±Extent * OuterOrbitFraction cube — the same region the Large tier's
	 *  bounds cover). Serves as ShouldSkipCell for the streaming tiers: both
	 *  the per-cell cull and the streaming gate. Mirrors
	 *  AGalaxyActor::CellOverlapsVolume. */
	bool CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const;

	//TODO: IS THIS ALWAYS THE SAME FOR ALL LAYERS? IF SO WE COULD HOIST
	/** GridExtentMultiplier mirrors GalaxyActor. The star system's spatial grid
	 *  is sized relative to Params.Extent * this multiplier so the large tier
	 *  single cell comfortably covers all planetary orbits. */
	static constexpr double GridExtentMultiplier = 4.0;
#pragma endregion

#pragma region Diagnostics
	int32 DiagTickCount = 0;

	void DrawPlanetDebugPositions() const;
#pragma endregion

private:
#pragma region Spawn Scan - Internal
	std::atomic<bool> bSpawnScanInProgress{ false };
	TSet<TSharedPtr<FOctreeNode>> TrackedPlanetNodes;
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

	/** Returns true if VirtualTraversal is within this system's octree bounds. */
	virtual bool IsPlayerInsideBounds() const override;
#pragma endregion
};
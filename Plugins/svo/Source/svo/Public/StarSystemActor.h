// StarSystemActor.h
// Tier streaming system mirroring GalaxyActor.
// Large tier: planets along a line at orbit distance, always loaded, neighbor-scanned.
// Mid/Small tiers: present but zero-particle placeholders for now.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "StarSystemDataGenerator.h"
#include "FTierStreamingSystem.h"
#include "StarSystemActor.generated.h"

class AGalaxyActor;
class AParallaxStaticMeshActor;

// ---------------------------------------------------------------------------
// FStarSystemTierParams  (mirrors FGalaxyTierParams for the tier pipeline)
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct SVO_API FStarSystemTierParams
{
	GENERATED_BODY()

	/** Grid depth for this tier.
	 *  Large = 1 (whole system in one cell), Mid/Small unused but present. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier")
	int32 GridDepth = 1;

	/** Neighborhood radius. Large=0 (single exhaustive cell). Mid/Small=1. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier")
	int32 NeighborhoodRadius = 0;

	/** Particle slot capacity per cell. Mid/Small = 0 for now. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier")
	int32 MaxParticlesPerSlot = 64;
};

// ---------------------------------------------------------------------------
// FStarSystemParams
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct SVO_API FStarSystemParams : public FBaseParams
{
	GENERATED_BODY()

	/** Color of this system's star (inherited from parent node Composition). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star")
	FLinearColor StarColor = FLinearColor(1, 1, 1, 1);

	/** How much larger the star system bounds are relative to the star sprite.
	 *  The star's world radius = ParticleExtent * Galaxy.UnitScale.
	 *  The star system's virtual space covers BoundsScaleMultiplier times that,
	 *  so planets can orbit well outside the star sprite's footprint.
	 *  Tune this until planets are visibly spread out when approaching the star.
	 *  1000 is a good starting point for a solar-system-scale first pass. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star")
	double BoundsScaleMultiplier = 30.0;

	/** Maximum number of planet sprites to generate (line layout). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	int32 MaxPlanets = 8;

	/** Absolute world-cm scale ranges for planet generation.
	 *  Inner rocky planets are drawn from TerrestrialMinScale..TerrestrialMaxScale.
	 *  Outer gas giants are drawn from GasGiantMinScale..GasGiantMaxScale.
	 *  MakePointDataFromWorldScale converts these to octree-local extents
	 *  using UnitScale, so planets have consistent physical sizes regardless
	 *  of which galaxy or star spawned them.
	 *
	 *  Real references (diameters in cm):
	 *    Mercury ≈ 4.9e8, Mars ≈ 6.8e8, Earth ≈ 1.27e9, Venus ≈ 1.2e9
	 *    Neptune ≈ 4.95e9, Saturn ≈ 1.16e10, Jupiter ≈ 1.4e10 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Terrestrial")
	double TerrestrialMinScale = 1e8;   // Small rocky (Ceres-to-Mercury class)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Terrestrial")
	double TerrestrialMaxScale = 1e9;   // Large super-Earth

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Gas Giant")
	double GasGiantMinScale = 3e9;      // Sub-Neptune / ice giant

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets|Gas Giant")
	double GasGiantMaxScale = 2e10;     // Hot Jupiter class

	/** Fraction of Extent used for innermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double InnerOrbitFraction = 0.08;

	/** Fraction of Extent used for outermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double OuterOrbitFraction = 0.85;

	// --- Tier configs ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams SmallTier;

	FStarSystemParams()
	{
		//Extent = 274877906944;

		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.MaxParticlesPerSlot = 64;

		MidTier.GridDepth = 4;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 0; // Zero — unused for now

		SmallTier.GridDepth = 7;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 0; // Zero — unused for now
	}
};

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
	/** Interval in seconds between planet spawn-scan dispatches. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Scanning")
	float SpawnScanInterval = 0.1f;

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
	virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos) override;
#pragma endregion

#pragma region Planet Pooled Spawn/Despawn Hooks
	/** Maps each live octree node (planet) to its spawned placeholder actor. */
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
	// No InitializeChildPool — star systems manage individual actor spawns, not a pool.

	virtual FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const override;
#pragma endregion

#pragma region Data Generation
	StarSystemDataGenerator SystemGenerator;

	/** Analytically generated planet positions (line layout, no noise). */
	TArray<FVector>       PlanetPositions;
	TArray<float>         PlanetExtents;
	TArray<FLinearColor>  PlanetColors;
#pragma endregion

#pragma region Niagara Assets
	UPROPERTY()
	UNiagaraSystem* StarSystemLargeCloud; // Planet sprite system

	UPROPERTY()
	UNiagaraSystem* StarSystemMidCloud;   // Placeholder

	UPROPERTY()
	UNiagaraSystem* StarSystemSmallCloud; // Placeholder
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
	double LastScanDispatchTime = 0.0;
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
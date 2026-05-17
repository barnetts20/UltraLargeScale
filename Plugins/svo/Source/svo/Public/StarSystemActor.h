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
	 *  The sprite's world radius ≈ Extent * UnitScale. The bounds used for
	 *  the planet Niagara system is that * BoundsScaleMultiplier.
	 *  Tune to cover all planetary orbits comfortably. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Star")
	double BoundsScaleMultiplier = 8.0;

	/** Maximum number of planet sprites to generate (line layout). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	int32 MaxPlanets = 8;

	/** Fraction of Extent used for innermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double InnerOrbitFraction = 0.05;

	/** Fraction of Extent used for outermost orbit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planets")
	double OuterOrbitFraction = 0.65;

	// --- Tier configs ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tiers")
	FStarSystemTierParams SmallTier;

	FStarSystemParams()
	{
		// Star system extents are much smaller than galaxy; default values
		// are illustrative — the spawning galaxy actor overwrites UnitScale.
		Extent = 1 << 20; // ~1M octree units default; overridden at runtime.

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

	UPROPERTY(EditAnywhere, Category = "StarSystem Properties")
	bool bAutoInitializeOnBeginPlay = false;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
#pragma endregion

#pragma region Virtual Traversal (mirrors GalaxyActor)
	/** Virtual traversal of the player through star-system-local space.
	 *  Accumulates PlayerDelta * (SpeedScale / UnitScale) each tick.
	 *  Used to keep planet Niagara positions camera-relative. */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** VirtualTraversal at the last Niagara position push. */
	FVector LastPushedVirtualTraversal = FVector::ZeroVector;

	/** Minimum VT delta before re-pushing positions to Niagara. */
	double ParallaxPushThreshold = 0.5;
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

	// Generic aliases expected by ProximityTrackerComponent.
	// Forward to the typed planet hooks so existing call sites compile unchanged.
	inline void SpawnEntityFromPool(TSharedPtr<FOctreeNode> InNode) { SpawnPlanetFromPool(InNode); }
	inline void ReturnEntityToPool(TSharedPtr<FOctreeNode> InNode) { ReturnPlanetToPool(InNode); }
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

	virtual void ApplyParallaxOffset() override; // Stub — VT model used instead.
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
};
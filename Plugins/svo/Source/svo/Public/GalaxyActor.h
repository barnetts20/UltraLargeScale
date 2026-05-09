// GalaxyActor.h
// Full tier streaming system mirroring UniverseActor.
// Large tier: exhaustive single-pass (NeighborhoodRadius=0), always loaded.
// Mid/Small tiers: neighborhood streaming with cell cache.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyDataGenerator.h"
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

	// Virtual traversal of the player through galaxy-local space. Initialized
	// at spawn to (PlayerPos - SpawnLoc) so particles appear at the correct
	// world position from the first frame. Accumulates PlayerDelta *
	// (SpeedScale / UnitScale) each tick — shrinking toward zero as the player
	// approaches — giving full floating-point precision when nearby.
	// Used identically to AUniverseActor::VirtualTraversal.
	FVector VirtualTraversal = FVector::ZeroVector;
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
	FString NiagaraPath = FString("/svo/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge");

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
	void InitializeTier(FParticleTierConfig& Config, FParticleTierState& State);
	void UpdateTier(FParticleTierConfig& Config, FParticleTierState& State);
	void PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State);

	/// Returns true if the given grid coord's cell overlaps the galaxy volume.
	bool CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const;
#pragma endregion

#pragma region Tier System - Octree Integration
	void InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx);
	void InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 SlotIndex, int32 BufferIdx);
	void InsertParticleIntoOctree(FSlotEntry& Entry, const FVector& Position, float Extent, int32 SlotIndex, double TreeExtent);
#pragma endregion

#pragma region Tier System - Cell Cache
	void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx);
	void CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& NewCenter);
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
	int StarSystemPoolSize = 5;
	TArray<AStarSystemActor*> StarSystemPool;
#pragma endregion

#pragma region Tick
	int32 DiagTickCount = 0;
	virtual void Tick(float DeltaTime) override;
#pragma endregion
};
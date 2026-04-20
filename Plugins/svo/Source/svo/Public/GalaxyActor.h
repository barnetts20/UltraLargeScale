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
	// SpeedScale, IsDebug, InitializationState, Octree inherited from base class
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AStarSystemActor>> SpawnedStarSystems;
	void SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

protected:
#pragma region Params Accessors (implement pure virtuals from base)
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override {
		return Universe ? Universe->SpeedScale : SpeedScale;
	}
#pragma endregion

#pragma region Initialization (implement pure virtuals from base)
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void InitializeChildPool() override;  // Star system pool initialization
#pragma endregion

#pragma region Data Generation
	GalaxyDataGenerator GalaxyGenerator;
#pragma endregion

#pragma region Niagara (galaxy-specific)
	FString NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");
	// Positions, Extents, Colors, SectorGalaxyCloud, NiagaraComponent inherited from base
#pragma endregion

#pragma region Volumetric (galaxy-specific)
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_GalaxyRaymarchPseudoVolume_Inst.MT_GalaxyRaymarchPseudoVolume_Inst");
	// PseudoVolumeTexture, VolumetricComponent, VolumeMaterial inherited from base
#pragma endregion

#pragma region Star System Pool
	TSubclassOf<AStarSystemActor> StarSystemActorClass;
	int StarSystemPoolSize = 5;
	TArray<AStarSystemActor*> StarSystemPool;
#pragma endregion
};
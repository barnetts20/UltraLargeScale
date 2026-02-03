#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "StarSystemDataGenerator.h"
#include "GalaxyActor.h"
#include "StarSystemActor.generated.h"

class AUniverseActor;

UCLASS()
class SVO_API AStarSystemActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AStarSystemActor();
	~AStarSystemActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "StarSystem Properties")
	FStarSystemParams Params;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "StarSystem Parent Actor")
	AGalaxyActor* Galaxy;
	// SpeedScale, IsDebug, InitializationState, Octree inherited from base class
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AActor>> SpawnedEntities;
	void SpawnEntityFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnEntityToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

protected:
#pragma region Params Accessors (implement pure virtuals from base)
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override {
		return Galaxy && Galaxy->Universe ? Galaxy->Universe->SpeedScale : SpeedScale;
	}
#pragma endregion

#pragma region Initialization (implement pure virtuals from base)
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	// No InitializeChildPool() - star systems don't spawn child pools
#pragma endregion

#pragma region Data Generation
	StarSystemDataGenerator SystemGenerator;
#pragma endregion

#pragma region Niagara (star system-specific)
	FString NiagaraPath = FString("/svo/NG_StarSystemCloud.NG_StarSystemCloud");
	// Positions, Extents, Colors, PointCloudNiagara, NiagaraComponent inherited from base
#pragma endregion

#pragma region Volumetric (star system-specific)
	FString VolumeMaterialPath = FString("/svo/Materials/RayMarchers/MT_StarSystemRaymarchPseudoVolume_Inst.MT_StarSystemRaymarchPseudoVolume_Inst");
	// PseudoVolumeTexture, VolumetricComponent, VolumeMaterial inherited from base
#pragma endregion
};
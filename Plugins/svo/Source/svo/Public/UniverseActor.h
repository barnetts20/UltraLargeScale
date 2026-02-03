#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "UniverseActor.generated.h"

class AGalaxyActor;

UCLASS()
class SVO_API AUniverseActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AUniverseActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;
	// SpeedScale is inherited from base class
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

protected:
#pragma region Params Accessors (implement pure virtuals from base)
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override { return SpeedScale; }  // Root level - no parent
	
#pragma endregion

#pragma region Initialization (implement pure virtuals from base)
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void InitializeChildPool() override;  // Galaxy pool initialization
#pragma endregion

#pragma region Data Generation
	UniverseDataGenerator UniverseGenerator;
#pragma endregion

#pragma region Niagara (additional universe-specific members)
	TArray<FVector> Rotations;  // Universe needs rotations, base class doesn't
	// Positions, Extents, Colors, PointCloudNiagara, NiagaraComponent inherited from base
#pragma endregion

#pragma region Volumetric (universe-specific)
// PseudoVolumeTexture, VolumetricComponent inherited from base
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");
#pragma endregion

#pragma region Galaxy Pool
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;
#pragma endregion

#pragma region Overrides
	virtual void BeginPlay() override;
	// Tick() inherited from base class - handles parallax automatically
#pragma endregion
};
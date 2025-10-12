#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <GalaxyActor.h>
#include <FOctree.h>
#include "StarSystemActor.generated.h"

UCLASS()
class SVO_API AStarSystemActor : public AActor
{
	GENERATED_BODY()
	
public:
#pragma region Constructor/Destructor
	AStarSystemActor();
	~AStarSystemActor();
#pragma endregion

#pragma region Public Parameters
	UPROPERTY()
	AGalaxyActor* Galaxy;

	TSharedPtr<FOctree> Octree;
	int Seed = 133780085;
	int64 Extent = 549755813888; // roughly 2x the radius of the solar system in km
	double UnitScale = 1.0;
	double SpeedScale = 1.0;
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;
	FVector AxisRotation = FVector::ZeroVector;
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
#pragma endregion

#pragma region Lifecycle Management
	void ResetForPool();			//Call externally before returning to pool
	void ResetForSpawn();           //Call externally before calling initialize
#pragma endregion

#pragma region Initialization
	void Initialize();				//Kick of async initialization of system
#pragma endregion

protected:
#pragma region Data Initialization
	StarSystemGenerator SystemGenerator;
	void InitializeData();
#pragma endregion

#pragma region Niagara
	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;
	FString NiagaraPath = FString("/svo/NG_StarSystemCloud.NG_StarSystemCloud");
	
	UPROPERTY()
	UNiagaraSystem* PointCloudNiagara;
	
	UPROPERTY()
	UNiagaraComponent* NiagaraComponent;
	
	void InitializeNiagara();
#pragma endregion

#pragma region Volumetric
	UPROPERTY()
	UTexture2D* PseudoVolumeTexture;
	
	UPROPERTY()
	UMaterialInstanceDynamic* VolumeMaterial;
	FString VolumeMaterialPath = FString("/svo/Materials/RayMarchers/MT_StarSystemRaymarchPseudoVolume_Inst.MT_StarSystemRaymarchPseudoVolume_Inst");
	UPROPERTY()
	UStaticMeshComponent* VolumetricComponent;
	void InitializeVolumetric();
#pragma endregion

#pragma region Parallax
	FVector LastFrameOfReferenceLocation = FVector(0, 0, 0);
	FVector CurrentFrameOfReferenceLocation;
	void ApplyParallaxOffset();
	virtual void Tick(float DeltaTime) override;
#pragma endregion

};

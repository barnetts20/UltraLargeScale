#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "FOctree.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "PointCloudGenerator.h"
#include "FastNoise/FastNoise.h"
#include <UniverseActor.h>
#include "GalaxyActor.generated.h"
class AStarSystemActor;
#pragma endregion

UCLASS()
class SVO_API AGalaxyActor : public AActor
{
	GENERATED_BODY()

public:
	#pragma region Constructor/Destructor
	AGalaxyActor();
	~AGalaxyActor();
	#pragma endregion

	#pragma region Public Parameters
	UPROPERTY()
	AUniverseActor* Universe;

	TSharedPtr<FOctree> Octree;
	int Seed = 133780085;
	int64 Extent = 2147483648;
	double UnitScale = 100.0;
	double SpeedScale = 1.0;
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;
	FVector AxisRotation = FVector::ZeroVector;
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
	#pragma endregion

	#pragma region Lifecycle Management
	void ResetForPool();			//Call externally before returning to pool
	void ResetForSpawn();           //Call externally before calling initialize
	#pragma endregion
	
	#pragma region Pooled Spawn/Despawn Hooks
	void SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode);
	#pragma endregion

	#pragma region Initialization
	void Initialize();				//Kick of async initialization of system
	#pragma endregion

protected:
	#pragma region Data Initialization
	GalaxyGenerator GalaxyGenerator;
	void InitializeData();
	#pragma endregion

	#pragma region Star System Pool
	TSubclassOf<AStarSystemActor> StarSystemActorClass;
	int StarSystemPoolSize = 5;
	TArray<AStarSystemActor*> StarSystemPool;
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AStarSystemActor>> SpawnedStarSystems;
	void InitializeStarSystemPool();
	#pragma endregion

	#pragma region Niagara
	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;
	FString NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");

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
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_GalaxyRaymarchPseudoVolume_Inst.MT_GalaxyRaymarchPseudoVolume_Inst");

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

#pragma once

#pragma region Includes
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
	AUniverseActor* Universe;
	TSharedPtr<FOctree> Octree;
	int Seed = 133780085;
	int64 Extent = 2147483648;
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
	GalaxyGenerator GalaxyGenerator;
	void InitializeData();
	#pragma endregion

	#pragma region Niagara
	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;
	FString NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;
	void InitializeNiagara();
	#pragma endregion

	#pragma region Volumetric
	UTexture2D* PseudoVolumeTexture;
	UMaterialInstanceDynamic* VolumeMaterial;
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

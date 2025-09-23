#pragma once

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


UCLASS()
class SVO_API AGalaxyActor : public AActor
{
	GENERATED_BODY()

public:
	AGalaxyActor();
	~AGalaxyActor();

	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	AUniverseActor* Universe;
	TSharedPtr<FOctree> Octree;

	//Default Initialization parameters
	int Seed = 133780085;
	int64 Extent = 2147483648;
	double UnitScale = 1.0;
	double SpeedScale = 1.0;

	FVector AxisRotation = FVector::ZeroVector;
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);

	void ResetForPool();			//Call externally before returning to pool
	void ResetForSpawn();           //Call externally before calling initialize
	void Initialize();				//Kick of async initialization of system

protected:
	//Point cloud generator
	GalaxyGenerator GalaxyGenerator;

	//Niagara Data and Component
	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;

	FString NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;

	//Volume Data and Component
	UTexture2D* PseudoVolumeTexture;
	UMaterialInstanceDynamic* VolumeMaterial;
	UStaticMeshComponent* VolumetricComponent;

	//Parallax
	FVector LastFrameOfReferenceLocation = FVector(0, 0, 0);
	FVector CurrentFrameOfReferenceLocation;

	void InitializeData();			//Initialize the octree data for the galaxy
	void InitializeVolumetric();	//Fetch the volume density data, create a volume texture, initialize the volumetric component
	void InitializeNiagara();		//Initialize the particle system

	virtual void Tick(float DeltaTime) override;
};

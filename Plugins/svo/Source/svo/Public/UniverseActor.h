#pragma once

#include "CoreMinimal.h"
#include "FOctree.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "PointCloudGenerator.h"
#include "NiagaraSystem.h"
#include "UniverseActor.generated.h"

class AGalaxyActor; // Forward declaration

UCLASS()
class SVO_API AUniverseActor : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AUniverseActor();

	ELifecycleState InitializationState = ELifecycleState::Initializing;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Seed = 133780085;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 10000.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 2000000; //TODO: NIAGARA STREAMING, ASYNC POINT GENERATION IN BATCHES TO OPTIMIZE LOAD TIME/STREAMING... 2 million is max spawn burst, more than this would need chunking


	int64 Extent = 2147483648;

	TSharedPtr<FOctree> Octree;
	UniverseGenerator UniverseGenerator;

	void SpawnGalaxy(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);

protected:
	//Niagara Data and Component
	TArray<FVector> Positions;
	TArray<FVector> Rotations;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;

	//Volumetric
	TArray<uint8> TextureData;
	UTexture2D* PseudoVolumeTexture;
	UStaticMeshComponent* VolumetricComponent;

	//Managed galaxy actors
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;

	//Parallax tracking locations
	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;

	void Initialize();
	void InitializeGalaxyPool();
	void InitializeData();
	void InitializeVolumetric();
	void InitializeNiagara();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};

#pragma once

#include "CoreMinimal.h"
#include "FOctreeNode.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Seed = 133780085;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 Extent = 16777216;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 GalaxyExtent = 8388608;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 20000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 2000000;

	bool Initialized = false;

	TSharedPtr<FOctree> Octree;

	FString NiagaraPath;
	class UNiagaraSystem* PointCloudNiagara;
	class UNiagaraComponent* NiagaraComponent;

	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;

	UVolumeTexture* DensityVolumeTexture;
	void SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition);
	void DestroyGalaxy(TSharedPtr<FOctreeNode> InNode);

	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
protected:
	void Initialize();
	void InitializeNiagara(TArray<FVector> InPositions, TArray<FVector> InRotations, TArray<float> InExtents, TArray<FLinearColor> InColors);
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
};

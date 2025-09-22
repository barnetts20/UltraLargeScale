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
	int64 Extent = 2147483648;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 GalaxyExtent = 536870912;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 10000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 2000000; //TODO: NIAGARA STREAMING, ASYNC POINT GENERATION IN BATCHES TO OPTIMIZE LOAD TIME/STREAMING

	TSharedPtr<FOctree> Octree;
	UniverseGenerator UGenerator;
	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	
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
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition);
	void DestroyGalaxy(TSharedPtr<FOctreeNode> InNode);

	//Parallax tracking locations
	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;

	//Noise formulas
	TArray<const char*> EncodedTrees = {
	"DQAIAAAAAAAAQAcAAAAAAD8AAAAAAA==",
	"FwAAAAAAAACAPwAAgD8AAIC/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA",
	"FwAAAAAAmpmZPwAAAAAAAIA/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA",
	"DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==",
	"FwAAAADAAACAPwAAgD8AAIC/EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==",
	"FwAAAAAAAACAPwAAgD8AAIC/DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA=="
	};

protected:
	void Initialize();
	bool TryCleanUpComponents();
	void MarkDestroying();
	void InitializeData();
	void FetchData();
	void InitializeVolumetric();
	void InitializeNiagara();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};

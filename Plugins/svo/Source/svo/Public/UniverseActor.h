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
	int64 Extent = 2147483648;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 GalaxyExtent = 2147483648;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 10000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 2000000; //TODO: NIAGARA STREAMING, ASYNC POINT GENERATION IN BATCHES TO OPTIMIZE LOAD TIME/STREAMING

	bool Initialized = false;
	TSharedPtr<FOctree> Octree;

	//Parallax tracking locations
	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;

	//Subcomponents
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;
	UStaticMeshComponent* VolumetricComponent;
	UVolumeTexture* DensityVolumeTexture;

	//Managed galaxy actors
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition);
	void DestroyGalaxy(TSharedPtr<FOctreeNode> InNode);

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
	void InitializeNiagara(TArray<FVector> InPositions, TArray<FVector> InRotations, TArray<float> InExtents, TArray<FLinearColor> InColors);
	void InitializeVolumetric(UVolumeTexture* InVolumeTexture);
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};

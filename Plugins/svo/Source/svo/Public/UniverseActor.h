#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "FOctree.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "PointCloudGenerator.h"
#include "NiagaraSystem.h"
#include "UniverseActor.generated.h"

class AGalaxyActor;
#pragma endregion

UCLASS()
class SVO_API AUniverseActor : public AActor
{
	GENERATED_BODY()

public:
	#pragma region Constructor/Destructor
	AUniverseActor();
	#pragma endregion

	#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	int Seed = 133780085;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	double UnitScale = 1000000.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	double SpeedScale = 1.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	int Count = 2000000; //TODO: NIAGARA STREAMING, ASYNC POINT GENERATION IN BATCHES TO OPTIMIZE LOAD TIME/STREAMING... 2 million is max spawn burst, more than this would need chunking
	#pragma endregion

	#pragma region Public Parameters
	int64 Extent = 2147483648;
	ELifecycleState InitializationState = ELifecycleState::Initializing;
	TSharedPtr<FOctree> Octree;
	#pragma endregion
	
	#pragma region Pooled Spawn/Despawn Hooks
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
	#pragma endregion

	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
protected:
	#pragma region Initialization
	UniverseGenerator UniverseGenerator;
	void Initialize();
	void InitializeData();
	#pragma endregion

	#pragma region Niagara Parameters and Components 
	TArray<FVector> Positions;
	TArray<FVector> Rotations;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;
	void InitializeNiagara();
	#pragma endregion

	#pragma region Volumetric Parameters and Components
	TArray<uint8> TextureData;
	UTexture2D* PseudoVolumeTexture;
	UStaticMeshComponent* VolumetricComponent;
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");
	void InitializeVolumetric();
	#pragma endregion

	#pragma region Galaxy Pool
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;

	void InitializeGalaxyPool();
	void SampleGalaxies();
	#pragma endregion

	#pragma region Parallax
	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;
	void ApplyParallaxOffset();
	#pragma endregion

	#pragma region Overrides
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	#pragma endregion
};

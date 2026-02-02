#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "FOctree.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "PointCloudGenerator.h"
#include "NiagaraSystem.h"
#include <UniverseDataGenerator.h>
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
	FUniverseParams Params;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	double SpeedScale = 1.0;

	#pragma endregion

	#pragma region Public Parameters
	bool IsDebug = false;

	ELifecycleState InitializationState = ELifecycleState::Initializing;
	TSharedPtr<FOctree> Octree;
	#pragma endregion
	
	#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
	#pragma endregion

protected:
	#pragma region Initialization
	UniverseDataGenerator UniverseGenerator;
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
	#pragma endregion

	#pragma region Parallax
	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;
	void ApplyParallaxOffset();
	void DrawDebugBounds();
	#pragma endregion

	#pragma region Overrides
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	#pragma endregion
};

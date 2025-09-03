// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FOctreeNode.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include <UniverseActor.h>
#include "GalaxyActor.generated.h"


UCLASS()
class SVO_API AGalaxyActor : public AActor
{
	GENERATED_BODY()

public:
	AGalaxyActor() {
		PrimaryActorTick.bCanEverTick = true;
		NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");
		USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
		SetRootComponent(SceneRoot);

		FSoftObjectPath NiagaraSystemPath(NiagaraPath);
		PointCloudNiagara = Cast<UNiagaraSystem>(NiagaraSystemPath.TryLoad());
	}

	AUniverseActor* Universe;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Seed = 133780085;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 Extent = 2147483648;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 500000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	FVector AxisRotation = FVector::ZeroVector;

	TArray<const char*> EncodedTrees = {
		"DQAIAAAAAAAAQAcAAAAAAD8AAAAAAA==",
		"FwAAAAAAAACAPwAAgD8AAIC/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA",
		"FwAAAAAAmpmZPwAAAAAAAIA/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA",
		"DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==",
		"FwAAAADAAACAPwAAgD8AAIC/EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==",
		"FwAAAAAAAACAPwAAgD8AAIC/DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA=="
	};

	TSharedPtr<FOctree> Octree;

	std::atomic<int32> InitializedComponents{ 0 };
	EGalaxyState InitializationState = EGalaxyState::Initializing;

	FString NiagaraPath;
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;
	TArray<uint8> TextureData;
	UVolumeTexture* VolumeTexture;
	UStaticMeshComponent* VolumetricComponent;

	FVector LastFrameOfReferenceLocation = FVector(0,0,0);
	FVector CurrentFrameOfReferenceLocation;
	FVector GalaxyRealSpaceOrigin;

	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;

	FLinearColor ParentColor = FLinearColor(1,1,1,0);
	void Initialize();
	void MarkDestroying();
	void DebugDrawTree();

protected:

	void InitializeData();
	void FetchData();
	void InitializeNiagara();
	void InitializeVolumetric();
	bool TryCleanUpComponents();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};

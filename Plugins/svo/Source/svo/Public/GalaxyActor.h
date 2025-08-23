// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FOctreeNode.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include <UniverseActor.h>
#include "Components/InstancedStaticMeshComponent.h"
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
		InstancedStarMesh = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedStarMesh"));
		InstancedStarMesh->SetupAttachment(RootComponent);
		InstancedStarMesh->bDisableCollision = true;
		// Set default meshStarMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Plane"));
		StarMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere"));
		if (StarMesh)
		{
			
			InstancedStarMesh->SetStaticMesh(StarMesh);
		}
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


	FString NiagaraPath;
	int ChunkSize = 50000;
	
	class UNiagaraSystem* PointCloudNiagara;
	class UNiagaraComponent* NiagaraComponent;
	class UStaticMeshComponent* VolumetricComponent;

	UInstancedStaticMeshComponent* InstancedStarMesh;
	UStaticMesh* StarMesh;

	FVector LastFrameOfReferenceLocation = FVector(0,0,0);
	FVector CurrentFrameOfReferenceLocation;
	FVector GalaxyRealSpaceOrigin;

	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;

	int CurNiagaraIndex = 0;
	bool PopulatingNiagara = false;

	FLinearColor ParentColor = FLinearColor(1,1,1,0);
	void Initialize();

protected:
	void InitializeNiagara();
	void SpawnInstancedGalaxy();
	void InitializeVolumetric(UVolumeTexture* InVolumeTexture);
	void DebugDrawTree();
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
};

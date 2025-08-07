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
		NiagaraPath = FString("/svo/NG_PointCloud.NG_PointCloud");
		USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
		SetRootComponent(SceneRoot);
	}
	// Sets default values for this actor's properties
	AGalaxyActor(AUniverseActor* InUniverse) : Universe(InUniverse) {
		PrimaryActorTick.bCanEverTick = true;
		NiagaraPath = FString("/svo/NG_PointCloud.NG_PointCloud");
		USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
		SetRootComponent(SceneRoot);
	};

	AGalaxyActor(AUniverseActor* InUniverse, int InSeed, int64 InExtent, double InUnitScale, int InCount, FVector InAxisRotation) : Universe(InUniverse), Seed(Seed), Extent(InExtent), UnitScale(InUnitScale), Count(InCount), AxisRotation(InAxisRotation) {
		PrimaryActorTick.bCanEverTick = true;
		NiagaraPath = FString("/svo/NG_PointCloud.NG_PointCloud");
		USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
		SetRootComponent(SceneRoot);
	};

	AUniverseActor* Universe;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Seed = 133780085;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 Extent = 8388608;

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
	class UNiagaraSystem* PointCloudNiagara;
	class UNiagaraComponent* NiagaraComponent;

	FVector LastFrameOfReferenceLocation = FVector(0,0,0);
	FVector CurrentFrameOfReferenceLocation;
	FVector GalaxyRealSpaceOrigin;
	void Initialize();

protected:
	void InitializeNiagara(TArray<FVector> Positions, TArray<float> Extents, TArray<FLinearColor> Colors);
	void DebugDrawTree();
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
};

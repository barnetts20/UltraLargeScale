// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FOctreeNode.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "SparseVoxelActor.generated.h"

UCLASS()
class SVO_API ASparseVoxelActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASparseVoxelActor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Seed = 133780085;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int64 Extent = 1048576;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double UnitScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	double SpeedScale = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree Properties")
	int Count = 2000000;

	TSharedPtr<FOctree> Octree;

	FString NiagaraPath;
	class UNiagaraSystem* PointCloudNiagara;
	class UNiagaraComponent* NiagaraComponent;

	FVector LastFrameOfReferenceLocation;
	FVector CurrentFrameOfReferenceLocation;

protected:
	void Initialize();
	void InitializeNiagara(TArray<FVector> Positions, TArray<float> Extents, TArray<FLinearColor> Colors);
	void DebugDrawTree();
	virtual void BeginPlay() override;

public:	
	virtual void Tick(float DeltaTime) override;
};

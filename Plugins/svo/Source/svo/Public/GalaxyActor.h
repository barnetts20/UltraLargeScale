// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FOctree.h"
#include "GameFramework/Actor.h"
#include "NiagaraComponent.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "PointCloudGenerator.h"
#include "FastNoise/FastNoise.h"
#include <UniverseActor.h>
#include "GalaxyActor.generated.h"


UCLASS()
class SVO_API AGalaxyActor : public AActor
{
	GENERATED_BODY()

public:
	AGalaxyActor(); 
	ELifecycleState InitializationState = ELifecycleState::Initializing;
	AUniverseActor* Universe;	//Parent UniverseActor pointer
	TSharedPtr<FOctree> Octree; //Octree data
	GalaxyGenerator GalaxyGenerator;
	//Initialization parameters
	int Seed = 133780085;
	int64 Extent = 2147483648;
	double UnitScale = 1.0;
	double SpeedScale = 1.0;
	int Count = 500000;
	FVector AxisRotation = FVector::ZeroVector;
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
	TArray<const char*> EncodedTrees = { "DQAIAAAAAAAAQAcAAAAAAD8AAAAAAA==", "FwAAAAAAAACAPwAAgD8AAIC/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA", "FwAAAAAAmpmZPwAAAAAAAIA/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA", "DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==", "FwAAAADAAACAPwAAgD8AAIC/EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==", "FwAAAAAAAACAPwAAgD8AAIC/DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==" };

	//Niagara Data and Component
	TArray<FVector> Positions;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;

	FString NiagaraPath = FString("/svo/NG_GalaxyCloud.NG_GalaxyCloud");
	UNiagaraSystem* PointCloudNiagara;
	UNiagaraComponent* NiagaraComponent;

	//Volume Data and Component
	TArray<uint8> TextureData;
	UVolumeTexture* VolumeTexture;
	UStaticMeshComponent* VolumetricComponent;

	//Parallax
	FVector LastFrameOfReferenceLocation = FVector(0,0,0);
	FVector CurrentFrameOfReferenceLocation;

	void Initialize();				//Kick of async initialization of system
	void MarkDestroying();			//Call externally before destroying

protected:
	void InitializeData();			//Initialize the octree data for the galaxy
	void FetchData();				//Fetch the data for the particle system
	void InitializeVolumetric();	//Fetch the volume density data, create a volume texture, initialize the volumetric component
	void InitializeNiagara();		//Initialize the particle system
	bool TryCleanUpComponents();	//Check early exit condition, destroy niagara and volumetric components and return true if early exit 
	
	virtual void Tick(float DeltaTime) override;

};

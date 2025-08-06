// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <UniverseActor.h>
#include <GalaxyActor.h>
#include "ProximityTrackerComponent.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SVO_API UProximityTrackerComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UProximityTrackerComponent();
	
	// Timer
	FTimerHandle UpdateTimerHandle;

	// How often to update proximity checks (seconds)
	UPROPERTY(EditAnywhere, Category = "Proximity")
	double UpdateInterval = .05;

	// Radius around player to scan (in tree units)
	UPROPERTY(EditAnywhere, Category = "Proximity")
	int64 ScanExtent = 2000;

	// Universe reference
	UPROPERTY()
	AUniverseActor* UniverseActor;

	TSet<TSharedPtr<FOctreeNode>> SpawnedGalaxyNodes;
	// Callback to run the proximity check
	void OnProximityUpdate();

	void DebugDrawNode(TSharedPtr<FOctreeNode> InNode);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
};

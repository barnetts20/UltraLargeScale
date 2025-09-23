#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <UniverseActor.h>
#include <GalaxyActor.h>
#include "ProximityTrackerComponent.generated.h"
#pragma endregion

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SVO_API UProximityTrackerComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	#pragma region Constructor/Destructor
	UProximityTrackerComponent();
	#pragma endregion

	#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, Category = "Proximity")
	double UpdateInterval = .05;

	UPROPERTY(EditAnywhere, Category = "Proximity")
	int64 ScanExtent = 2000;

	UPROPERTY(EditAnywhere, Category = "Proximity")
	bool DebugMode = false;

	UPROPERTY()
	AUniverseActor* UniverseActor;
	#pragma endregion
	
protected:
	#pragma region Initialization
	virtual void BeginPlay() override;
	#pragma endregion
	
	#pragma region Proximity Polling
	FTimerHandle UpdateTimerHandle;
	TSet<TSharedPtr<FOctreeNode>> SpawnedGalaxyNodes;
	void OnProximityUpdate();
	#pragma endregion

	#pragma region Debug
	void DebugDrawNode(TSharedPtr<FOctreeNode> InNode);
	#pragma endregion
};

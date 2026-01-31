#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <StarSystemActor.h>
#include "ParallaxStaticMeshActor.generated.h"

UCLASS()
class SVO_API AParallaxStaticMeshActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AParallaxStaticMeshActor();
	AStarSystemActor* System;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	UStaticMeshComponent* MeshComponent;

	// Parallax parameters - set by spawner
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax")
	double UnitScale = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax")
	double SpeedScale = 1;

	#pragma region Parallax
	FVector LastFrameOfReferenceLocation = FVector(0, 0, 0);
	FVector CurrentFrameOfReferenceLocation;
	void ApplyParallaxOffset();
	virtual void Tick(float DeltaTime) override;
#pragma endregion
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
};

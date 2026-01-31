// Fill out your copyright notice in the Description page of Project Settings.


#include "ParallaxStaticMeshActor.h"
#include <Kismet/GameplayStatics.h>

// Sets default values
AParallaxStaticMeshActor::AParallaxStaticMeshActor()
{
	PrimaryActorTick.bCanEverTick = true;

	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	RootComponent = MeshComponent;

	MeshComponent->SetMobility(EComponentMobility::Movable);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);
	MeshComponent->SetRenderCustomDepth(true);

	MeshComponent->SetCustomDepthStencilValue(1);

	UnitScale = 1.0;
	SpeedScale = 1.0;
}

// Called when the game starts or when spawned
void AParallaxStaticMeshActor::BeginPlay()
{
	Super::BeginPlay();
	// Set initial frame of reference
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				LastFrameOfReferenceLocation = Pawn->GetActorLocation();
			}
		}
	}
}

#pragma region Parallax
void AParallaxStaticMeshActor::ApplyParallaxOffset()
{
	bool bHasReference = false;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				CurrentFrameOfReferenceLocation = Pawn->GetActorLocation();
				bHasReference = true;
			}
		}
	}
	if (!bHasReference)
	{
		UE_LOG(LogTemp, Warning, TEXT("Parallax: No valid reference camera or pawn found."));
		return;
	}
	FVector CurrentActorLocation = GetActorLocation();

	double ParallaxRatio = (System && System->Galaxy && System->Galaxy->Universe ? System->Galaxy->Universe->SpeedScale : SpeedScale) / UnitScale;
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	FVector NewActorLocation = CurrentActorLocation + ParallaxOffset;

	SetActorLocation(NewActorLocation);
}

void AParallaxStaticMeshActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ApplyParallaxOffset();
}
#pragma endregion

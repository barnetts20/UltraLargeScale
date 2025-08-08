// Fill out your copyright notice in the Description page of Project Settings.
#include "UniverseActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <GalaxyActor.h>
#include <Kismet/GameplayStatics.h>

// Sets default values
AUniverseActor::AUniverseActor()
{
	PrimaryActorTick.bCanEverTick = true;
	NiagaraPath = FString("/svo/NG_UniverseCloud.NG_UniverseCloud");
	USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	SetRootComponent(SceneRoot);
}

void AUniverseActor::Initialize()
{
	Async(EAsyncExecution::Thread, [this]()
		{
			//Populate Data into the tree
			Octree = MakeShared<FOctree>(Extent);
			auto Generator = new GlobularNoiseGenerator(Seed);

			Generator->Count = this->Count;
			Generator->Falloff = .5;
			Generator->Rotation = FRotator(0);
			Generator->DepthRange = 10;
			Generator->InsertDepthOffset = 0;
			Generator->WarpAmount = FVector(1);

			//Finally populate data into the tree
			Generator->GenerateData(Octree);

			//Extract data from the tree and construct niagara arrays
			TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetPopulatedNodes(); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
			TArray<FVector> Positions;
			TArray<FVector> Rotations;
			TArray<float> Extents;
			TArray<FLinearColor> Colors;
			Positions.SetNumUninitialized(Leaves.Num());
			Rotations.SetNumUninitialized(Leaves.Num());
			Extents.SetNumUninitialized(Leaves.Num());
			Colors.SetNumUninitialized(Leaves.Num());

			ParallelFor(Leaves.Num(), [&](int32 Index)
				{
					const TSharedPtr<FOctreeNode>& Leaf = Leaves[Index];
					FRandomStream RandStream(Leaf->Data.ObjectId);
					FVector ColorVector = RandStream.GetUnitVector();
					Rotations[Index] = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
					Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);
					Extents[Index] = static_cast<float>(Leaf->Extent);
					Colors[Index] = FLinearColor(ColorVector.X, ColorVector.Y, ColorVector.Z);;
				});

			//Pass the arrays back to the game thread to instantiate the particle system
			AsyncTask(ENamedThreads::GameThread, [this, Positions = MoveTemp(Positions), Rotations = MoveTemp(Rotations), Extents = MoveTemp(Extents), Colors = MoveTemp(Colors)]()
				{
					InitializeNiagara(Positions, Rotations, Extents, Colors);
				});
		});
}

void AUniverseActor::InitializeNiagara(TArray<FVector> InPositions, TArray<FVector> InRotations, TArray<float> InExtents, TArray<FLinearColor> InColors)
{
	FSoftObjectPath NiagaraSystemPath(NiagaraPath);
	PointCloudNiagara = Cast<UNiagaraSystem>(NiagaraSystemPath.TryLoad());

	if (PointCloudNiagara)
	{
		NiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
			PointCloudNiagara,
			GetRootComponent(),
			NAME_None,
			FVector::ZeroVector,
			FRotator::ZeroRotator,
			EAttachLocation::SnapToTarget,// KeepRelativeOffset,
			true
		);

		if (NiagaraComponent)
		{
			// Set system bounds
			FBox Bounds(FVector(-Extent), FVector(Extent));
			NiagaraComponent->SetSystemFixedBounds(Bounds);
			// Pass in user parameters (assuming Niagara system is setup to receive them)
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), InPositions);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Rotations"), InRotations);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), InColors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), InExtents);
			// Optionally reactivate to refresh state if needed
			NiagaraComponent->Activate(true);
		}
	}
	Initialized = true;
}

void AUniverseActor::SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode))
	{
		return;
	}

	//Scale is derived from perceived universe extent divided galaxy extent
	const double GalaxyUnitScale = (InNode->Extent * this->UnitScale) / GalaxyExtent; 
	// Compute correct parallax ratios
	const double GalaxyParallaxRatio = (SpeedScale / GalaxyUnitScale);
	const double UniverseParallaxRatio = (SpeedScale / UnitScale);
	if (UniverseParallaxRatio > 40) return;
	// 1. Node world position
	FVector NodeWorldPosition = FVector(InNode->Center.X, InNode->Center.Y, InNode->Center.Z) + GetActorLocation();
	// 2. Player offset from node
	FVector PlayerToNode = InReferencePosition - NodeWorldPosition;
	// 4. Final spawn position
	FVector GalaxySpawnPosition = InReferencePosition - PlayerToNode * (GalaxyParallaxRatio / UniverseParallaxRatio);

	AGalaxyActor* NewGalaxy = GetWorld()->SpawnActor<AGalaxyActor>(GalaxyActorClass, GalaxySpawnPosition, FRotator::ZeroRotator);
	if (NewGalaxy)
	{
		NewGalaxy->Universe = this; // We need to delay spawn till we can proceduralize bounding info
		NewGalaxy->Extent = GalaxyExtent;
		NewGalaxy->Seed = InNode->Data.ObjectId;
		NewGalaxy->SpeedScale = SpeedScale;
		NewGalaxy->UnitScale = GalaxyUnitScale;
		FRandomStream RandStream(InNode->Data.ObjectId);
		FVector ColorVector = RandStream.GetUnitVector();
		NewGalaxy->ParentColor = FLinearColor(ColorVector.X, ColorVector.Y, ColorVector.Z);
		FVector normal = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
		FMatrix RotationMatrix = FRotationMatrix::MakeFromZX(normal, FVector::ForwardVector);
		NewGalaxy->AxisRotation = normal * 360;
		NewGalaxy->Initialize();

		SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(NewGalaxy));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to spawn galaxy for node with ObjectId: %d"), InNode->Data.ObjectId);
	}
}

void AUniverseActor::DestroyGalaxy(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid())
	{
		return;
	}

	TWeakObjectPtr<AGalaxyActor> GalaxyToDestroy;

	// Find the actor in the map and remove the entry simultaneously.
	if (SpawnedGalaxies.RemoveAndCopyValue(InNode, GalaxyToDestroy))
	{
		// Ensure the actor pointer is still valid before trying to destroy it.
		if (GalaxyToDestroy.IsValid())
		{
			UE_LOG(LogTemp, Log, TEXT("Destroying galaxy for node with ObjectId: %d"),
				InNode->Data.ObjectId);
			GalaxyToDestroy->Destroy();
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Galaxy actor was already invalid for node with ObjectId: %d"),
				InNode->Data.ObjectId);
		}
	}
}

// Called when the game starts or when spawned
void AUniverseActor::BeginPlay()
{
	Super::BeginPlay();
	Initialize();
}

void AUniverseActor::Tick(float DeltaTime)
{
	bool bHasReference = false;
	if (const auto* World = GetWorld())
	{
		auto Controller = UGameplayStatics::GetPlayerController(World, 0);
		if (Controller)
		{
			CurrentFrameOfReferenceLocation = Controller->GetPawn()->GetActorLocation();
			bHasReference = true;
		}
	}

	if (!bHasReference)
	{
		UE_LOG(LogTemp, Warning, TEXT("Parallax: No valid reference camera or pawn found."));
		return;
	}

	double ParallaxRatio = SpeedScale / UnitScale;
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	SetActorLocation(GetActorLocation() + ParallaxOffset);
	Super::Tick(DeltaTime);
}


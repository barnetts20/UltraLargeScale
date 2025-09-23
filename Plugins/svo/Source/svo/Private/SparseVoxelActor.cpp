//TODO:REMOVE, NO LONGER USED/OUT OF DATE
// Fill out your copyright notice in the Description page of Project Settings.
#include "SparseVoxelActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>

// Sets default values
ASparseVoxelActor::ASparseVoxelActor()
{
	PrimaryActorTick.bCanEverTick = true;
	NiagaraPath = FString("/svo/NG_PointCloud.NG_PointCloud");
	USceneComponent* SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	SetRootComponent(SceneRoot);
}

void ASparseVoxelActor::Initialize()
{
	Async(EAsyncExecution::Thread, [this]()
	{
		//Populate Data into the tree
		Octree = MakeShared<FOctree>(Extent);
		//auto Generator = new SimpleRandomGenerator(Seed);
		//auto Generator = new SimpleRandomNoiseGenerator(Seed); 
		// 
		//auto Generator = new GlobularGenerator(Seed); 
		auto Generator = new GlobularNoiseGenerator(Seed);
		// 
		//auto Generator = new SpiralGenerator(Seed);
		//auto Generator = new SpiralNoiseGenerator(Seed);
		//
		//auto Generator = new BurstGenerator(Seed);
		//auto Generator = new BurstNoiseGenerator(Seed);
		
		Generator->Count = this->Count;
		Generator->Falloff = .5;
		Generator->Rotation = FRotator(0);
		Generator->DepthRange = 8;
		Generator->WarpAmount = FVector(1);

		//Finally populate data into the tree
		Generator->GenerateData(Octree);

		//Extract data from the tree and construct niagara arrays
		TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetLeafNodes(); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
		TArray<FVector> Positions;
		TArray<float> Extents;
		TArray<FLinearColor> Colors;
		Positions.SetNumUninitialized(Leaves.Num());
		Extents.SetNumUninitialized(Leaves.Num());
		Colors.SetNumUninitialized(Leaves.Num());

		ParallelFor(Leaves.Num(), [&](int32 Index)
		{
			const TSharedPtr<FOctreeNode>& Leaf = Leaves[Index];

			Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);
			Extents[Index] = static_cast<float>(Leaf->Extent);

			FRandomStream RandStream(Leaf->Data.ObjectId);
			float colorEffect = .3;
			FVector UnitVec = (RandStream.GetUnitVector() * RandStream.FRand() * colorEffect + (1-colorEffect)) * 1.0f;
			Colors[Index] = FLinearColor(UnitVec.X, UnitVec.Y, UnitVec.Z);
		});

		//Pass the arrays back to the game thread to instantiate the particle system
		AsyncTask(ENamedThreads::GameThread, [this, Positions = MoveTemp(Positions), Extents = MoveTemp(Extents), Colors = MoveTemp(Colors)]()
		{
			InitializeNiagara(Positions, Extents, Colors);
		});
	});
}

void ASparseVoxelActor::InitializeNiagara(TArray<FVector> Positions, TArray<float> Extents, TArray<FLinearColor> Colors) 
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
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);
			// Optionally reactivate to refresh state if needed
			NiagaraComponent->Activate(true);
		}
	}
}

void ASparseVoxelActor::DebugDrawTree()
{
	if (!Octree.IsValid()) return;
	TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetLeafNodes();

	for (auto& Node : Leaves)
	{
		FVector WorldCenter = FVector(
			Node->Center.X, Node->Center.Y, Node->Center.Z
		) + GetActorLocation();

		float BoxExtent = Node->Extent;
		
		DrawDebugBox(
			GetWorld(),
			WorldCenter,
			FVector(BoxExtent),
			FColor::Green,
			true, 
			10.0f,
			255,
			1.0f
		);
	}
}

// Called when the game starts or when spawned
void ASparseVoxelActor::BeginPlay()
{
	Super::BeginPlay();
	Initialize();

	bool debugDraw = false;

	if (debugDraw) {
		DebugDrawTree();
	}

}

void ASparseVoxelActor::Tick(float DeltaTime)
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
	FVector ActorOrigin = FVector::ZeroVector; // Replace with your origin if dynamic
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	SetActorLocation(GetActorLocation() + ParallaxOffset);
	Super::Tick(DeltaTime);
}


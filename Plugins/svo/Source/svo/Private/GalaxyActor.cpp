// Fill out your copyright notice in the Description page of Project Settings.
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>

void AGalaxyActor::Initialize()
{
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

	Async(EAsyncExecution::Thread, [this]()
		{
			//Populate Data into the tree
			Octree = MakeShared<FOctree>(Extent);
			FRandomStream Stream = FRandomStream(Seed);
			//TODO: Add some % chance for globular noise based galaxy, increased Z axis spread and noise amount
			auto SpiralGenerator = new SpiralNoiseGenerator(Seed);
			//Proceduralize
			SpiralGenerator->Count = Stream.RandRange(200000, 500000);
			SpiralGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
			SpiralGenerator->DepthRange = 4;
			SpiralGenerator->NumArms = Stream.RandRange(2, 12);
			SpiralGenerator->PitchAngle = Stream.FRandRange(5, 40);
			SpiralGenerator->ArmContrast = Stream.FRandRange(.2, .8);
			SpiralGenerator->RadialFalloff = Stream.FRandRange(2, 4);
			SpiralGenerator->CenterScale = Stream.FRandRange(.01, .02);
			SpiralGenerator->HorizontalSpreadMin = Stream.FRandRange(.01, .03);
			SpiralGenerator->HorizontalSpreadMax = Stream.FRandRange(.15, .3);
			SpiralGenerator->VerticalSpreadMin = Stream.FRandRange(.01, .03);
			SpiralGenerator->VerticalSpreadMax = Stream.FRandRange(.05, .15);

			double HorizontalWarp = Stream.FRandRange(.1, .9);
			double VerticalWarp = Stream.FRandRange(.1, .9);

			SpiralGenerator->WarpAmount = FVector(HorizontalWarp, HorizontalWarp, VerticalWarp);
			SpiralGenerator->EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];


			SpiralGenerator->GenerateData(Octree);
			//GlobularGenerator->GenerateData(Octree);
			//Extract data from the tree and construct niagara arrays
			TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetPopulatedNodes(); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
			TArray<FVector> Positions;
			TArray<float> Extents;
			TArray<FLinearColor> Colors;
			Positions.SetNumUninitialized(Leaves.Num());
			Extents.SetNumUninitialized(Leaves.Num());
			Colors.SetNumUninitialized(Leaves.Num());

			ParallelFor(Leaves.Num(), [&](int32 Index)
				{
					const TSharedPtr<FOctreeNode>& Leaf = Leaves[Index];
					FRandomStream RandStream(Leaf->Data.ObjectId);
					FVector ColorVector = RandStream.GetUnitVector();
					//ColorVector = FVector(FMath::Abs(ColorVector.X), FMath::Abs(ColorVector.Y), FMath::Abs(ColorVector.Z)).GetSafeNormal();

					Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);// Should already be in particle system local space
					Extents[Index] = static_cast<float>(Leaf->Extent);
					Colors[Index] = FLinearColor(ColorVector.X, ColorVector.Y, ColorVector.Z);
				});

			//Pass the arrays back to the game thread to instantiate the particle system
			AsyncTask(ENamedThreads::GameThread, [this, Positions = MoveTemp(Positions), Extents = MoveTemp(Extents), Colors = MoveTemp(Colors)]()
				{
					InitializeNiagara(Positions, Extents, Colors);
				});
		});
}

void AGalaxyActor::InitializeNiagara(TArray<FVector> Positions, TArray<float> Extents, TArray<FLinearColor> Colors)
{
	FSoftObjectPath NiagaraSystemPath(NiagaraPath);
	PointCloudNiagara = Cast<UNiagaraSystem>(NiagaraSystemPath.TryLoad());

	if (PointCloudNiagara)
	{
		//SYSTEM IS SPAWNING WITH EITHER AN OFFSET AT THE SYSTEM LEVEL OR AT INSERTION OF POINTS IN A WAY THAT IS UNCENTERING IT
		NiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
			PointCloudNiagara,
			GetRootComponent(),
			NAME_None,
			FVector::ZeroVector,
			FRotator::ZeroRotator,
			EAttachLocation::KeepWorldPosition,
			true
		);

		if (NiagaraComponent)
		{
			NiagaraComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
			NiagaraComponent->SetAutoActivate(false); // Must be before initialization

			// Set system bounds
			FBox Bounds(FVector(-Extent), FVector(Extent));
			NiagaraComponent->SetSystemFixedBounds(Bounds);

			// Pass in user parameters (assuming Niagara system is setup to receive them)
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);
			// Optionally reactivate to refresh state if needed
			NiagaraComponent->Activate(true);
			NiagaraComponent->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		}
	}
}

void AGalaxyActor::DebugDrawTree()
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
void AGalaxyActor::BeginPlay()
{
	Super::BeginPlay();
	GalaxyRealSpaceOrigin = GetActorLocation(); // <- Store initial spawn position
	//Initialize();

	bool debugDraw = false;

	if (debugDraw) {
		DebugDrawTree();
	}

}

void AGalaxyActor::Tick(float DeltaTime)
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

	double ParallaxRatio = (Universe ? Universe->SpeedScale : SpeedScale) / UnitScale;
	FVector ActorOrigin = FVector::ZeroVector; // Replace with your origin if dynamic
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	//DrawDebugSphere(GetWorld(), GetActorLocation(), Extent, 34, FColor::Green, false, -1, 0, 24); //THIS SPHERE IS CORRECTLY LOCATED
	SetActorLocation(GetActorLocation() + ParallaxOffset);
	Super::Tick(DeltaTime);
}


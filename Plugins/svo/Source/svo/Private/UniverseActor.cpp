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
			
			//Proceduralize
			FRandomStream Stream(Seed);
			auto Generator = new GlobularNoiseGenerator(Seed);
			Generator->Count = this->Count;
			Generator->Falloff = .5;
			Generator->Rotation = FRotator(0);
			Generator->DepthRange = 10;
			Generator->InsertDepthOffset = 4;
			Generator->WarpAmount = FVector(1);
			Generator->EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];

			//Finally populate data into the tree
			Generator->GenerateData(Octree);

			//Extract data from the tree and construct niagara arrays
			TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetPopulatedNodes(-1, -1, 1);
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
					Rotations[Index] = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
					Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);
					Extents[Index] = static_cast<float>(Leaf->Extent * 2);
					Colors[Index] = FLinearColor(Leaf->Data.Composition);
				}
			);

			//Pass the arrays back to the game thread to instantiate the particle system
			AsyncTask(ENamedThreads::GameThread, [this, Positions = MoveTemp(Positions), Rotations = MoveTemp(Rotations), Extents = MoveTemp(Extents), Colors = MoveTemp(Colors)]()
				{
					InitializeVolumetric(Octree->CreateVolumeTextureFromOctree(64));
					InitializeNiagara(Positions, Rotations, Extents, Colors);
					Initialized = true;
				}
			);
		});
}

void AUniverseActor::InitializeVolumetric(UVolumeTexture* InVolumeTexture) {
	UMaterialInterface* GasMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_VolumeRaymarch_Inst.MT_VolumeRaymarch_Inst"));
	UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(GasMaterial, this);
	
	//TODO: Configure material with the volume texture
	DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), InVolumeTexture);
	//Set up color variance etc
	//

	UStaticMesh* VolumetricMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals"));
	VolumetricComponent = NewObject<UStaticMeshComponent>(this);
	VolumetricComponent->SetStaticMesh(VolumetricMesh);
	VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
	VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	VolumetricComponent->SetMaterial(0, DynamicMaterial);
	VolumetricComponent->RegisterComponent();
}

void AUniverseActor::InitializeNiagara(TArray<FVector> InPositions, TArray<FVector> InRotations, TArray<float> InExtents, TArray<FLinearColor> InColors)
{
	PointCloudNiagara = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_UniverseCloud.NG_UniverseCloud"));
	if (PointCloudNiagara)
	{
		NiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
			PointCloudNiagara,
			GetRootComponent(),
			NAME_None,
			FVector::ZeroVector,
			FRotator::ZeroRotator,
			EAttachLocation::SnapToTarget,
			true, 
			false
		);

		if (NiagaraComponent)
		{
			NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Extent), FVector(Extent)));
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), InPositions);
			//UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Rotations"), InRotations);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), InColors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), InExtents);
			NiagaraComponent->Activate(true);
		}
	}
}

void AUniverseActor::SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || !Initialized)
	{
		return;
	}

	//Scale is derived from perceived universe extent divided galaxy extent
	const double GalaxyUnitScale = (InNode->Extent * this->UnitScale) / GalaxyExtent; 

	// Compute correct parallax ratios
	const double GalaxyParallaxRatio = (SpeedScale / GalaxyUnitScale);
	const double UniverseParallaxRatio = (SpeedScale / UnitScale);

	// Compute spawn location
	FVector NodeWorldPosition = FVector(InNode->Center.X, InNode->Center.Y, InNode->Center.Z) + GetActorLocation();
	FVector PlayerToNode = InReferencePosition - NodeWorldPosition;
	FVector GalaxySpawnPosition = InReferencePosition - PlayerToNode * (GalaxyParallaxRatio / UniverseParallaxRatio);

	AGalaxyActor* NewGalaxy = GetWorld()->SpawnActor<AGalaxyActor>(GalaxyActorClass, GalaxySpawnPosition, FRotator::ZeroRotator);
	if (NewGalaxy)
	{
		NewGalaxy->Universe = this;
		NewGalaxy->Extent = GalaxyExtent;
		NewGalaxy->Seed = InNode->Data.ObjectId;
		NewGalaxy->SpeedScale = SpeedScale;
		NewGalaxy->UnitScale = GalaxyUnitScale;
		FRandomStream RandStream(InNode->Data.ObjectId);
		NewGalaxy->ParentColor = FLinearColor(InNode->Data.Composition.GetSafeNormal());
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


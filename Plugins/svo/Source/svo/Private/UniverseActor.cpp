#pragma region Includes/ForwardDec
#include "UniverseActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <Camera/CameraComponent.h>
#include <GameFramework/SpringArmComponent.h>
#pragma endregion

#pragma region Constructor/Destructor
AUniverseActor::AUniverseActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
	PointCloudNiagara = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_UniverseCloud.NG_UniverseCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	Octree = MakeShared<FOctree>(Extent);
}
#pragma endregion

#pragma region Initialization

void AUniverseActor::BeginPlay()
{
	Super::BeginPlay();
	Initialize();
}

void AUniverseActor::Initialize()
{
	InitializationState = ELifecycleState::Initializing;

	// Set initial frame of reference position
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

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
	{
		double StartTime = FPlatformTime::Seconds();

		InitializeGalaxyPool();
		InitializeData();
		InitializeVolumetric();
		InitializeNiagara();

		InitializationState = ELifecycleState::Ready;

		double TotalDuration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Initialize total duration: %.3f seconds"), TotalDuration);
	});
}

void AUniverseActor::InitializeGalaxyPool() {
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
	{
		for(int i = 0; i < GalaxyPoolSize; i++){
			AGalaxyActor* Galaxy = GetWorld()->SpawnActor<AGalaxyActor>(GalaxyActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
			Galaxy->Universe = this;
			GalaxyPool.Add(Galaxy);
		}
		CompletionPromise.SetValue();
	});
	CompletionFuture.Wait();
}

void AUniverseActor::InitializeData() {
	double StartTime = FPlatformTime::Seconds();

	FRandomStream Stream(Seed);
	UniverseGenerator.Seed = Seed;
	UniverseParams UniverseParams;
	UniverseParams.Extent = Extent;

	//TODO: Proceduralize universe generator params / make a factory
	UniverseGenerator.UniverseParams = UniverseParams;
	UniverseGenerator.Rotation = FRotator(0);
	UniverseGenerator.DepthRange = 7; 
	UniverseGenerator.InsertDepthOffset = 5;
	UniverseGenerator.GenerateData(Octree);

	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(UniverseGenerator.GeneratedData, PointNodes, VolumeNodes);

	Positions.SetNumUninitialized(PointNodes.Num());
	Rotations.SetNumUninitialized(PointNodes.Num());
	Extents.SetNumUninitialized(PointNodes.Num());
	Colors.SetNumUninitialized(PointNodes.Num());

	ParallelFor(PointNodes.Num(), [&](int32 Index) {
		const TSharedPtr<FOctreeNode>& Node = PointNodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Rotations[Index] = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
		Positions[Index] = FVector(Node->Center.X, Node->Center.Y, Node->Center.Z);
		Extents[Index] = static_cast<float>(Node->Extent * 2) * RandStream.FRandRange(.5, 1);
		Colors[Index] = FLinearColor(Node->Data.Composition);
	}, EParallelForFlags::BackgroundPriority);

	PseudoVolumeTexture = FOctreeTextureProcessor::GeneratePseudoVolumeTextureFromMipData(FOctreeTextureProcessor::UpscalePseudoVolumeDensityData(FOctreeTextureProcessor::GenerateVolumeMipDataFromOctree(VolumeNodes, 32, Extent, Octree->DepthMaxDensity), 32));

	double GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Universe data generation took: %.3f seconds"), GenDuration);
}

void AUniverseActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
	{
		VolumetricComponent = NewObject<UStaticMeshComponent>(this); //TODO: THESE NEED TO MOVE INTO SEPERATE RUN ONCE INIT
		VolumetricComponent->SetVisibility(false);
		VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));  //TODO: THESE NEED TO MOVE INTO SEPERATE RUN ONCE INIT
		VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		VolumetricComponent->TranslucencySortPriority = 1;
		VolumetricComponent->RegisterComponent();
		VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));

		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
			LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_UniverseRaymarchPsuedoVolume_Inst.MT_UniverseRaymarchPsuedoVolume_Inst")),
			this
		);

		DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
		//TODO: Proceduralize material
		//TODO: End material proceduralizatiion

		VolumetricComponent->SetMaterial(0, DynamicMaterial);
		VolumetricComponent->SetVisibility(true);

		CompletionPromise.SetValue();
	});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void AUniverseActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
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
		NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Extent), FVector(Extent)));
		NiagaraComponent->TranslucencySortPriority = 0;

		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Rotations"), Rotations);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);

			AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
			{
				NiagaraComponent->Activate(true);
				CompletionPromise.SetValue();
			});
		});
	});
	CompletionFuture.Wait();

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void AUniverseActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready || GalaxyPool.Num() == 0)
	{
		return;
	}

	AGalaxyActor* Galaxy = nullptr;
	if (GalaxyPool.Num() > 0)
	{
		Galaxy = GalaxyPool.Pop();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Galaxy pool exhausted, consider increasing GalaxyPoolSize"));
		return;
	}

	SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(Galaxy));
	Galaxy->ResetForSpawn();

	Galaxy->UnitScale = (InNode->Extent * this->UnitScale) / Galaxy->Extent;	//Scale is derived from perceived node extent divided by galaxy extent
	Galaxy->SpeedScale = SpeedScale;											//Speed scale is the same across all parallax components in the system
	Galaxy->Seed = InNode->Data.ObjectId;
	Galaxy->ParentColor = FLinearColor(InNode->Data.Composition);

	// Compute correct parallax ratios and spawn location
	const double GalaxyParallaxRatio = (SpeedScale / Galaxy->UnitScale);
	const double UniverseParallaxRatio = (SpeedScale / UnitScale);
	FVector NodeWorldPosition = FVector(InNode->Center.X, InNode->Center.Y, InNode->Center.Z) + GetActorLocation();
	FVector PlayerToNode = CurrentFrameOfReferenceLocation - NodeWorldPosition;
	FVector GalaxySpawnPosition = CurrentFrameOfReferenceLocation - PlayerToNode * (GalaxyParallaxRatio / UniverseParallaxRatio);
	Galaxy->SetActorLocation(GalaxySpawnPosition);

	//Compute Axis Tilt >>> TODO: MOVE TO GALAXY PARAM FACTORY/GENERATOR
	FRandomStream RandStream(InNode->Data.ObjectId);
	FVector normal = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
	FMatrix RotationMatrix = FRotationMatrix::MakeFromZX(normal, FVector::ForwardVector);
	Galaxy->AxisRotation = normal * 360;
	
	Galaxy->Initialize();				
	Galaxy->SetActorHiddenInGame(false);
}

void AUniverseActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid())
	{
		return;
	}
	TWeakObjectPtr<AGalaxyActor> GalaxyToDestroy;

	// Find the actor in the map and remove the entry simultaneously.
	if (SpawnedGalaxies.RemoveAndCopyValue(InNode, GalaxyToDestroy))
	{
		AGalaxyActor* PoolGalaxy = GalaxyToDestroy.Get();
		
		// Ensure the actor pointer is still valid before trying to destroy it.
		if (PoolGalaxy)
		{
			UE_LOG(LogTemp, Log, TEXT("Resetting galaxy for node with ObjectId: %d"), InNode->Data.ObjectId);
			PoolGalaxy->ResetForPool();

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PoolGalaxy]()
			{
				double StartTime = FPlatformTime::Seconds();
				
				//Handle Octree flush here instead of in galaxy, that way we can ensure flush is done before returning it to the pool
				PoolGalaxy->Octree->bIsResetting.store(true);
				FPlatformProcess::Sleep(0.05f);
				PoolGalaxy->Octree = MakeShared<FOctree>(PoolGalaxy->Extent);
				PoolGalaxy->Octree->bIsResetting.store(false);

				double ODuration = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), ODuration);

				// Return to pool on game thread
				AsyncTask(ENamedThreads::GameThread, [this, PoolGalaxy, StartTime]()
				{
					GalaxyPool.Insert(PoolGalaxy, 0);
				});
			});
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Galaxy actor was already invalid for node with ObjectId: %d"), InNode->Data.ObjectId);
		}	
	}
}
#pragma endregion

#pragma region Parallax
void AUniverseActor::ApplyParallaxOffset()
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

	double ParallaxRatio = SpeedScale / UnitScale;
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	SetActorLocation(GetActorLocation() + ParallaxOffset);
}

void AUniverseActor::Tick(float DeltaTime)
{
	ApplyParallaxOffset();
}
#pragma endregion
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
	Octree = MakeShared<FOctree>(Params.Extent);
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

	UniverseGenerator.Params = Params;
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
		Positions[Index] = Node->Center;
		Extents[Index] = static_cast<float>(Node->Extent * (1 + Node->Data.Density));
		Colors[Index] = FLinearColor(Node->Data.Composition);
	}, EParallelForFlags::BackgroundPriority);

	PseudoVolumeTexture = FOctreeTextureProcessor::GeneratePseudoVolumeTextureFromMipData(FOctreeTextureProcessor::UpscalePseudoVolumeDensityData(FOctreeTextureProcessor::GenerateVolumeMipDataFromOctree(VolumeNodes, 32, Params.Extent, Octree->DepthMaxDensity), 32));

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
		VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
			LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath),
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
		NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Params.Extent), FVector(Params.Extent)));
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
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	if (GalaxyPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Galaxy pool exhausted, consider increasing GalaxyPoolSize"));
		return;
	}

	AGalaxyActor* Galaxy = GalaxyPool.Pop();

	SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(Galaxy));
	Galaxy->ResetForSpawn();

	Galaxy->Params.UnitScale = (InNode->Extent * this->Params.UnitScale) / Galaxy->Params.Extent;	//Scale is derived from perceived node extent divided by galaxy extent
	Galaxy->SpeedScale = SpeedScale;												//Speed scale is the same across all parallax components in the system and subsystems
	Galaxy->Params.Seed = InNode->Data.ObjectId;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);

	// Compute correct parallax ratios and spawn location
	const double GalaxyParallaxRatio = (SpeedScale / Galaxy->Params.UnitScale);
	const double UniverseParallaxRatio = (SpeedScale / Params.UnitScale);
	FVector NodeWorldPosition = InNode->Center + GetActorLocation();
	FVector PlayerToNode = CurrentFrameOfReferenceLocation - NodeWorldPosition;
	FVector GalaxySpawnPosition = CurrentFrameOfReferenceLocation - PlayerToNode * (GalaxyParallaxRatio / UniverseParallaxRatio);
	Galaxy->SetActorLocation(GalaxySpawnPosition);

	//Compute Axis Tilt >>> TODO: MOVE TO GALAXY PARAM FACTORY/GENERATOR
	FRandomStream RandStream(InNode->Data.ObjectId);
	const float u1 = RandStream.FRand();
	const float u2 = RandStream.FRand();
	const float u3 = RandStream.FRand();

	const float sqrt1MinusU1 = FMath::Sqrt(1.f - u1);
	const float sqrtU1 = FMath::Sqrt(u1);

	FQuat Quat(
		sqrt1MinusU1 * FMath::Sin(2.f * PI * u2),
		sqrt1MinusU1 * FMath::Cos(2.f * PI * u2),
		sqrtU1 * FMath::Sin(2.f * PI * u3),
		sqrtU1 * FMath::Cos(2.f * PI * u3)
	);
	Galaxy->Params.Rotation = Quat.Rotator();

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
				PoolGalaxy->Octree = MakeShared<FOctree>(PoolGalaxy->Params.Extent);
				PoolGalaxy->Octree->bIsResetting.store(false);

				double ODuration = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), ODuration);

				// Return to pool on game thread
				AsyncTask(ENamedThreads::GameThread, [this, PoolGalaxy]()
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

	double ParallaxRatio = SpeedScale / Params.UnitScale;
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	SetActorLocation(GetActorLocation() + ParallaxOffset);
}
void AUniverseActor::DrawDebugBounds()
{
	// Draw debug box for the octree root node
	if (Octree.IsValid() && InitializationState == ELifecycleState::Ready)
	{
		if (UWorld* World = GetWorld())
		{
			FVector ActorLocation = GetActorLocation();

			// Convert octree extent to world scale
			// WorldExtent = OctreeExtent * UnitScale
			double WorldExtent = Octree->Extent;

			FVector BoxExtent(WorldExtent, WorldExtent, WorldExtent);

			// Draw the box centered at the actor location
			DrawDebugBox(
				World,
				ActorLocation,
				BoxExtent,
				FColor::Purple,
				false,
				-1.0f,
				0,
				WorldExtent * 0.005f  // Thickness relative to world extent
			);

			// Draw coordinate axes at the center
			DrawDebugCoordinateSystem(
				World,
				ActorLocation,
				FRotator::ZeroRotator,
				WorldExtent * 0.1f,
				false,
				-1.0f,
				0,
				WorldExtent * 0.001f
			);
		}
	}
}
void AUniverseActor::Tick(float DeltaTime)
{
	ApplyParallaxOffset();
	if(IsDebug) DrawDebugBounds();
}
#pragma endregion
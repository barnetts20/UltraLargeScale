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
	PointCloudNiagara = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_UniverseCloud.NG_UniverseCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	SetRootComponent(SceneRoot);
}

void AUniverseActor::Initialize()
{
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

	Octree = MakeShared<FOctree>(Extent);

	Async(EAsyncExecution::Thread, [this]()
	{
		double StartTime = FPlatformTime::Seconds();

		InitializeData();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		FetchData();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		InitializeVolumetric();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		InitializeNiagara();
		if (TryCleanUpComponents()) return; //Early exit if destroying

		InitializationState = ELifecycleState::Ready;

		double TotalDuration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Initialize total duration: %.3f seconds"), TotalDuration);
	});
}

bool AUniverseActor::TryCleanUpComponents() {
	if (InitializationState != ELifecycleState::Destroying) return false;
	AsyncTask(ENamedThreads::GameThread, [this]()
	{
		if (VolumetricComponent) VolumetricComponent->DestroyComponent();
		if (NiagaraComponent) NiagaraComponent->DestroyComponent();
	});
	return true;
}

void AUniverseActor::MarkDestroying() {
	InitializationState = ELifecycleState::Destroying;
}

void AUniverseActor::InitializeData() {
	double StartTime = FPlatformTime::Seconds();
	FRandomStream Stream(Seed);
	auto Generator = new GlobularNoiseGenerator(Seed);
	Generator->Count = this->Count;
	Generator->Falloff = .5;
	Generator->Rotation = FRotator(0);
	Generator->DepthRange = 8;
	Generator->InsertDepthOffset = 5;
	Generator->WarpAmount = FVector(1);
	Generator->EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];
	Generator->GenerateData(Octree);
	double GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Universe generation took: %.3f seconds"), GenDuration);
}

void AUniverseActor::FetchData() {
	double StartTime = FPlatformTime::Seconds();
	TArray<TSharedPtr<FOctreeNode>> Nodes = Octree->GetPopulatedNodes(-1, -1, 1);
	Positions.SetNumUninitialized(Nodes.Num());
	Rotations.SetNumUninitialized(Nodes.Num());
	Extents.SetNumUninitialized(Nodes.Num());
	Colors.SetNumUninitialized(Nodes.Num());

	ParallelFor(Nodes.Num(), [&](int32 Index)
	{
		const TSharedPtr<FOctreeNode>& Node = Nodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Rotations[Index] = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
		Positions[Index] = FVector(Node->Center.X, Node->Center.Y, Node->Center.Z);
		Extents[Index] = static_cast<float>(Node->Extent * 2);
		Colors[Index] = FLinearColor(Node->Data.Composition);
	}, EParallelForFlags::BackgroundPriority);

	double FetchDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Node fetch + array population took: %.3f seconds"), FetchDuration);
}

void AUniverseActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();
	int Resolution = 64;
	TextureData = Octree->CreateVolumeDensityDataFromOctree(Resolution);
	if (TryCleanUpComponents()) return; //Early exit if destroying

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, Resolution, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
	{
		double SourceStart = FPlatformTime::Seconds();

		VolumeTexture = NewObject<UVolumeTexture>(
			GetTransientPackage(),
			NAME_None,
			RF_Transient
		);

		VolumeTexture->SRGB = false;
		VolumeTexture->CompressionSettings = TC_VectorDisplacementmap;
		VolumeTexture->CompressionNone = true;
		VolumeTexture->Filter = TF_Nearest;
		VolumeTexture->AddressMode = TA_Clamp;
		VolumeTexture->MipGenSettings = TMGS_NoMipmaps;
		VolumeTexture->LODGroup = TEXTUREGROUP_ColorLookupTable;
		VolumeTexture->NeverStream = true;
		VolumeTexture->DeferCompression = true;
		VolumeTexture->UnlinkStreaming();

		VolumeTexture->Source.Init(Resolution, Resolution, Resolution, 1, ETextureSourceFormat::TSF_BGRA8, TextureData.GetData());
		VolumeTexture->UpdateResource();
		FlushRenderingCommands();

		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
			LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_UniverseRaymarch_Inst.MT_UniverseRaymarch_Inst")),
			this
		);

		DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), VolumeTexture);
		//TODO: Proceduralize material

		VolumetricComponent = NewObject<UStaticMeshComponent>(this);
		VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
		VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
		VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		VolumetricComponent->SetMaterial(0, DynamicMaterial);
		VolumetricComponent->RegisterComponent();


		double SourceDuration = FPlatformTime::Seconds() - SourceStart;
		UE_LOG(LogTemp, Log, TEXT("AUniverseActor::VolumeTexture Source.Init & Update took: %.3f seconds"), SourceDuration);

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

void AUniverseActor::SpawnGalaxy(TSharedPtr<FOctreeNode> InNode, FVector InReferencePosition)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready)
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
		NewGalaxy->ParentColor = FLinearColor(InNode->Data.Composition);
		FVector normal = FVector(RandStream.FRand(), RandStream.FRand(), RandStream.FRand()).GetSafeNormal();
		FMatrix RotationMatrix = FRotationMatrix::MakeFromZX(normal, FVector::ForwardVector);
		NewGalaxy->AxisRotation = normal * 360;
		NewGalaxy->Count = RandStream.RandRange(150000, 250000);
		SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(NewGalaxy));
		NewGalaxy->Initialize();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to spawn galaxy for node with ObjectId: %d"), InNode->Data.ObjectId);
	}
}

void AUniverseActor::DestroyGalaxy(TSharedPtr<FOctreeNode> InNode)
{
	AsyncTask(ENamedThreads::GameThread, [this, InNode]()
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
				GalaxyToDestroy->MarkDestroying();
				GalaxyToDestroy->Destroy();
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Galaxy actor was already invalid for node with ObjectId: %d"),
					InNode->Data.ObjectId);
			}
		}
	});
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
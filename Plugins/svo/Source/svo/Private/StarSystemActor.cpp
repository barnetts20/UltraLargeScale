#pragma region Includes/ForwardDec
#include "StarSystemActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <Camera/CameraComponent.h>
#include <GameFramework/SpringArmComponent.h>
#pragma endregion

#pragma region Constructor/Destructor
AStarSystemActor::AStarSystemActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
	PointCloudNiagara = Cast<UNiagaraSystem>(FSoftObjectPath(NiagaraPath).TryLoad());
	Octree = MakeShared<FOctree>(Extent);
}

AStarSystemActor::~AStarSystemActor()
{
	//Release all populated data
	Positions.Empty();
	Extents.Empty();
	Colors.Empty();
	SystemGenerator.GeneratedData.Empty();
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region Initialization
void AStarSystemActor::Initialize()
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

			if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying
			InitializeData();
			if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying
			InitializeVolumetric();
			if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying
			InitializeNiagara();
			if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying

			InitializationState = ELifecycleState::Ready;

			double TotalDuration = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Initialize total duration: %.3f seconds"), TotalDuration);
		});
}

void AStarSystemActor::InitializeData() {
	double StartTime = FPlatformTime::Seconds();

	FRandomStream Stream(Seed);
	SystemGenerator.Seed = Seed;

	//TODO: These will need a look, we will have a more diverse selection of objects than the other generators, so we may need multiple size ranges
	//we will start off with just a placeholder though
	SystemGenerator.DepthRange = 7; //With seven levels, assuming our smallest star is say 1/2 the size of the sun, we can cover the vast majority of potential realistic star scales
	SystemGenerator.InsertDepthOffset = 8; //Controlls the depth above max depth the smallest stars will be generated in
	
	SystemGenerator.Rotation = FRotator(Stream.FRandRange(-35, 35), Stream.FRandRange(-35, 35), Stream.FRandRange(-35, 35));
	SystemGenerator.GeneratedData.SetNum(0);

	//TODO: If we want a factory approach it would happen here to generate the base params

	SystemGenerator.SystemParams = StarSystemParams();
	SystemGenerator.SystemParams.StarColor = ParentColor;
	SystemGenerator.GenerateData(Octree);

	double GenFinish = FPlatformTime::Seconds();
	double GenDuration = GenFinish - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Data Generation took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying

	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(SystemGenerator.GeneratedData, PointNodes, VolumeNodes);

	double InsertFinish = FPlatformTime::Seconds();
	GenDuration = InsertFinish - GenFinish;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Bulk Insert took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying

	Positions.SetNumUninitialized(PointNodes.Num());
	Extents.SetNumUninitialized(PointNodes.Num());
	Colors.SetNumUninitialized(PointNodes.Num());

	ParallelFor(PointNodes.Num(), [&](int32 Index) {
		const TSharedPtr<FOctreeNode>& Node = PointNodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Positions[Index] = FVector(Node->Center.X, Node->Center.Y, Node->Center.Z);
		Extents[Index] = static_cast<float>(Node->Extent * (1 + Node->Data.Density));
		Colors[Index] = FLinearColor(Node->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);

	PseudoVolumeTexture = FOctreeTextureProcessor::GeneratePseudoVolumeTextureFromMipData(FOctreeTextureProcessor::UpscalePseudoVolumeDensityData(FOctreeTextureProcessor::GenerateVolumeMipDataFromOctree(VolumeNodes, 32, Extent, Octree->DepthMaxDensity), 32));

	double RemapFinish = FPlatformTime::Seconds();
	GenDuration = RemapFinish - InsertFinish;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Data Remap took: %.3f seconds"), GenDuration);

	GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeData took: %.3f seconds"), GenDuration);
}

void AStarSystemActor::InitializeVolumetric()
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

			VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarchPsuedoVolume_Inst.MT_GalaxyRaymarchPsuedoVolume_Inst")),
				this
			);

			VolumeMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *SystemGenerator.SystemParams.VolumeNoise));
			VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), SystemGenerator.SystemParams.VolumeAmbientColor);
			VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), SystemGenerator.SystemParams.VolumeCoolShift);
			VolumeMaterial->SetVectorParameterValue(FName("HotShift"), SystemGenerator.SystemParams.VolumeHotShift);
			VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), SystemGenerator.SystemParams.VolumeHueVariance);
			VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), SystemGenerator.SystemParams.VolumeHueVarianceScale);
			VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), SystemGenerator.SystemParams.VolumeSaturationVariance);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), SystemGenerator.SystemParams.VolumeTemperatureInfluence);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), SystemGenerator.SystemParams.VolumeTemperatureScale);
			VolumeMaterial->SetScalarParameterValue(FName("Density"), SystemGenerator.SystemParams.VolumeDensity);
			VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), SystemGenerator.SystemParams.VolumeWarpAmount);
			VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), SystemGenerator.SystemParams.VolumeWarpScale);

			VolumetricComponent->SetMaterial(0, VolumeMaterial);
			VolumetricComponent->SetVisibility(true);

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void AStarSystemActor::InitializeNiagara()
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
			NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Extent); //TODO: Might not need this, check niagara component
			NiagaraComponent->TranslucencySortPriority = 1;

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable {
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);

				AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable {
					NiagaraComponent->Activate(true);
					CompletionPromise.SetValue();
					});
				});
		});
	CompletionFuture.Wait();

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Lifecycle Management
void AStarSystemActor::ResetForSpawn() {
	InitializationState = ELifecycleState::Uninitialized;
}

void AStarSystemActor::ResetForPool() {
	double StartTime = FPlatformTime::Seconds();

	InitializationState = ELifecycleState::Pooling; //Set to pooling to stop any further init operations

	if (VolumetricComponent)
	{
		VolumetricComponent->DetachFromParent();
		VolumetricComponent->DestroyComponent();
		VolumetricComponent = nullptr;
	}
	if (NiagaraComponent)
	{
		NiagaraComponent->DetachFromParent();
		NiagaraComponent->DestroyComponent();
		NiagaraComponent = nullptr;
	}

	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::ResetForPool took: %.3f seconds"), Duration);
}
#pragma endregion

#pragma region Parallax
void AStarSystemActor::ApplyParallaxOffset()
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

	double ParallaxRatio = (Galaxy && Galaxy->Universe ? Galaxy->Universe->SpeedScale : SpeedScale) / UnitScale;
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	FVector NewActorLocation = CurrentActorLocation + ParallaxOffset;

	SetActorLocation(NewActorLocation);

	float DistanceToPlayer = (NewActorLocation - CurrentFrameOfReferenceLocation).Size();

	//UE_LOG(LogTemp, Warning, TEXT("=== Star System Parallax ==="));
	//UE_LOG(LogTemp, Warning, TEXT("Player Position: %s"), *CurrentFrameOfReferenceLocation.ToString());
	//UE_LOG(LogTemp, Warning, TEXT("Actor Position (before): %s"), *CurrentActorLocation.ToString());
	//UE_LOG(LogTemp, Warning, TEXT("Actor Position (after): %s"), *NewActorLocation.ToString());
	if(ELifecycleState::Ready == this->InitializationState)
	UE_LOG(LogTemp, Warning, TEXT("Distance to Player: %f"), DistanceToPlayer);
	//UE_LOG(LogTemp, Warning, TEXT("UnitScale: %f"), UnitScale);
	//UE_LOG(LogTemp, Warning, TEXT("ParallaxRatio: %f"), ParallaxRatio);
	//UE_LOG(LogTemp, Warning, TEXT("PlayerOffset: %s"), *PlayerOffset.ToString());
	//UE_LOG(LogTemp, Warning, TEXT("ParallaxOffset: %s"), *ParallaxOffset.ToString());
}

void AStarSystemActor::Tick(float DeltaTime)
{
	ApplyParallaxOffset();
}
#pragma endregion
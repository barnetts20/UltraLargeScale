// Fill out your copyright notice in the Description page of Project Settings.
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include "Engine/VolumeTexture.h"
#include <Kismet/GameplayStatics.h>
#include <AssetRegistry/AssetRegistryModule.h>

AGalaxyActor::AGalaxyActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
	PointCloudNiagara = Cast<UNiagaraSystem>(FSoftObjectPath(NiagaraPath).TryLoad());
}

void AGalaxyActor::Initialize()
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

	//Initialize parent thread, any initialize tasks should manage their own game thread segments
	//By default, they will execute their logic on this thread
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
	{
		double StartTime = FPlatformTime::Seconds();

		//FPlatformProcess::Sleep(0.5f);
		if (TryCleanUpComponents()) return; //Early exit if destroying
		InitializeData();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		FetchData();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		InitializeVolumetric();
		if (TryCleanUpComponents()) return; //Early exit if destroying
		InitializeNiagara();
		if (TryCleanUpComponents()) return; //Early exit if destroying

		VolumetricComponent->SetVisibility(true);
		InitializationState = ELifecycleState::Ready;

		double TotalDuration = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Initialize total duration: %.3f seconds"), TotalDuration);
	});
}

bool AGalaxyActor::TryCleanUpComponents() {
	if (InitializationState != ELifecycleState::Destroying) return false;
	AsyncTask(ENamedThreads::GameThread, [this]()
	{
		if (VolumetricComponent) VolumetricComponent->DestroyComponent();
		if (NiagaraComponent) NiagaraComponent->DestroyComponent();
	});
	return true;
}

void AGalaxyActor::MarkDestroying() {
	InitializationState = ELifecycleState::Destroying;
	GalaxyGenerator.MarkDestroying();
}

void AGalaxyActor::InitializeData() {
	double StartTime = FPlatformTime::Seconds();
	FRandomStream Stream(Seed);
	auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];

	GalaxyGenerator.Seed = Seed;
	GalaxyGenerator.DepthRange = 7; //With seven levels, assuming our smallest star is say 1/2 the size of the sun, we can cover the vast majority of potential realistic star scales
	GalaxyGenerator.InsertDepthOffset = 8; //Controlls the depth above max depth the smallest stars will be generated in
	GalaxyGenerator.Rotation = FRotator(Stream.FRandRange(-45, 45), Stream.FRandRange(-45, 45), Stream.FRandRange(-45, 45));
	GalaxyParamFactory GalaxyParamGen;
	GalaxyParamGen.Seed = this->Seed;

	GalaxyGenerator.GalaxyParams = GalaxyParamGen.GenerateParams();
	GalaxyGenerator.GenerateData(Octree);

	double GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Galaxy generation took: %.3f seconds"), GenDuration);
}

void AGalaxyActor::FetchData() {	
	double StartTime = FPlatformTime::Seconds();
	TArray<TSharedPtr<FOctreeNode>> Nodes = Octree->GetPopulatedNodes(-1, -1, 1);
	Positions.SetNumUninitialized(Nodes.Num());
	Extents.SetNumUninitialized(Nodes.Num());
	Colors.SetNumUninitialized(Nodes.Num());
	ParallelFor(Nodes.Num(), [&](int32 Index)
	{
		const TSharedPtr<FOctreeNode>& Node = Nodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Positions[Index] = FVector(Node->Center.X, Node->Center.Y, Node->Center.Z);
		Extents[Index] = static_cast<float>(Node->Extent * 2) * RandStream.FRandRange(.5,1);
		Colors[Index] = FLinearColor(Node->Data.Composition);
	}, EParallelForFlags::BackgroundPriority);

	double FetchDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Node fetch + array population took: %.3f seconds"), FetchDuration);
}

void AGalaxyActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();
	
	int Resolution = 32;

	FastNoise::SmartNode<> Noise = FastNoise::NewFromEncodedNodeTree("EwDhetQ/FwAAAAAAAACAPwAAgL8AAIA/DQADAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==");
	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Noise);
	SeedOffset->SetOffset(Seed);
	auto ScaleNoise = FastNoise::New<FastNoise::DomainScale>();
	ScaleNoise->SetSource(SeedOffset);
	ScaleNoise->SetScale(1);

	PseudoVolumeTexture = FOctreeTextureProcessor::GeneratePseudoVolumeTextureFromMipData(FOctreeTextureProcessor::UpscalePseudoVolumeDensityData(FOctreeTextureProcessor::GenerateVolumeMipDataFromOctree(Octree, Resolution), Resolution, 256, ScaleNoise, 1, FVector::ZeroVector, 1.33, Seed), 256);
	if (TryCleanUpComponents()) return;

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			double SourceStart = FPlatformTime::Seconds();
			UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarchPsuedoVolume_Inst.MT_GalaxyRaymarchPsuedoVolume_Inst")),
				this
			);
			DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			DynamicMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *GalaxyGenerator.GalaxyParams.VolumeNoise));
			
			DynamicMaterial->SetVectorParameterValue(FName("AmbientColor"), GalaxyGenerator.GalaxyParams.VolumeAmbientColor);
			DynamicMaterial->SetVectorParameterValue(FName("CoolShift"), GalaxyGenerator.GalaxyParams.VolumeCoolShift);
			DynamicMaterial->SetVectorParameterValue(FName("HotShift"), GalaxyGenerator.GalaxyParams.VolumeHotShift);
			DynamicMaterial->SetScalarParameterValue(FName("HueVariance"), GalaxyGenerator.GalaxyParams.VolumeHueVariance);
			DynamicMaterial->SetScalarParameterValue(FName("HueVarianceScale"), GalaxyGenerator.GalaxyParams.VolumeHueVarianceScale);
			DynamicMaterial->SetScalarParameterValue(FName("SaturationVariance"), GalaxyGenerator.GalaxyParams.VolumeSaturationVariance);
			DynamicMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), GalaxyGenerator.GalaxyParams.VolumeTemperatureInfluence);
			DynamicMaterial->SetScalarParameterValue(FName("TemperatureScale"), GalaxyGenerator.GalaxyParams.VolumeTemperatureScale);
			DynamicMaterial->SetScalarParameterValue(FName("Density"), GalaxyGenerator.GalaxyParams.VolumeDensity);
			DynamicMaterial->SetScalarParameterValue(FName("WarpAmount"), GalaxyGenerator.GalaxyParams.VolumeWarpAmount);
			DynamicMaterial->SetScalarParameterValue(FName("WarpScale"), GalaxyGenerator.GalaxyParams.VolumeWarpScale);

			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetVisibility(false);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->SetMaterial(0, DynamicMaterial);
			VolumetricComponent->TranslucencySortPriority = 1;
			VolumetricComponent->RegisterComponent();

			double SourceDuration = FPlatformTime::Seconds() - SourceStart;
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::VolumeTexture Source.Init & Update took: %.3f seconds"), SourceDuration);

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void AGalaxyActor::InitializeNiagara()
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
		NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Extent);
		NiagaraComponent->TranslucencySortPriority = 1;

		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
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
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
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
	SetActorLocation(GetActorLocation() + ParallaxOffset);
	Super::Tick(DeltaTime);
}

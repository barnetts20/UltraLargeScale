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
}

void AGalaxyActor::InitializeData() {
	double StartTime = FPlatformTime::Seconds();
	FRandomStream Stream(Seed);
	auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];

	auto Generator = new GalaxyGenerator(Seed);
	Generator->DepthRange = 7; //With seven levels, assuming our smallest star is say 1/2 the size of the sun, we can cover the vast majority of potential realistic star scales
	Generator->InsertDepthOffset = 9; //Controlls the depth above max depth the smallest stars will be generated in

	//TODO: Configure galaxy archtypes, then later we can randomize around them
	GalaxyParams E0;
	E0.ArmNumPoints = 0;
	E0.DiscNumPoints = 0;
	E0.BulgeNumPoints = 700000;
	E0.BulgeBaseDensity = 3;
	E0.BulgeDepthBias = .2;
	E0.BackgroundBaseDensity = 20;
	E0.BulgeAxisScale = FVector(1);
	E0.GalaxyRatio = .4;
	E0.BulgeRatio = 1.5;
	E0.BackgroundNumPoints = 300000;

	GalaxyParams E3;
	E3.ArmNumPoints = 0;
	E3.DiscNumPoints = 0;
	E3.BulgeNumPoints = 700000;
	E3.BulgeBaseDensity = 3;
	E3.BulgeDepthBias = .2;
	E3.BackgroundBaseDensity = 50;
	E3.BulgeAxisScale = FVector(1,1,.7);
	E3.GalaxyRatio = .3;
	E3.BulgeRatio = 1.25;
	E3.BackgroundNumPoints = 300000;
	
	GalaxyParams E5;
	E5.ArmNumPoints = 0;
	E5.DiscNumPoints = 200000;
	E5.DiscHeightRatio = .3;
	E5.DiscDepthBias = 1.2;
	E5.DiscBaseDensity = 3;
	E5.BulgeNumPoints = 500000;
	E5.BulgeBaseDensity = 3;
	E5.BulgeDepthBias = .2;
	E5.BackgroundBaseDensity = 20;
	E5.BulgeAxisScale = FVector(1, 1, .7);
	E5.GalaxyRatio = .3;
	E5.BulgeRatio = 1.25;
	E5.BackgroundNumPoints = 300000;

	GalaxyParams E7; 
	E7.ArmNumPoints = 0;
	E7.DiscNumPoints = 300000;
	E7.DiscDepthBias = .8;
	E7.DiscHeightRatio = .2;
	E7.DiscBaseDensity = 3;
	E7.BulgeNumPoints = 400000;
	E7.BulgeBaseDensity = 3; 
	E7.BulgeDepthBias = .2;
	E7.BackgroundBaseDensity = 20;
	E7.BulgeAxisScale = FVector(1, 1, .6);
	E7.GalaxyRatio = .3;
	E7.BulgeRatio = 1.25;
	E7.BackgroundNumPoints = 300000;

	GalaxyParams S0;
	S0.ArmNumPoints = 0;
	S0.DiscNumPoints = 400000;
	S0.DiscDepthBias = .3;
	S0.DiscHeightRatio = .1;
	S0.BulgeNumPoints = 300000;
	S0.BulgeBaseDensity = 3;
	S0.BulgeDepthBias = .2;
	S0.BackgroundBaseDensity = 20;
	S0.BulgeAxisScale = FVector(1, 1, .5);
	S0.GalaxyRatio = .3;
	S0.BulgeRatio = 1.25;
	S0.BackgroundNumPoints = 300000;

	GalaxyParams Sa;
	Sa.TwistStrength = 4;
	Sa.TwistCoreStrength = 4;
	Sa.ArmNumArms = 2;
	Sa.ArmClusterRadiusMin = .15;
	Sa.ArmClusterRadiusMax = .4;
	Sa.ArmIncoherence = 3;
	Sa.ArmStartRatio = 0;

	GalaxyParams Sb;
	Sb.TwistStrength = 12;
	Sb.ArmNumArms = 2;
	Sb.ArmIncoherence = 6;
	Sb.ArmStartRatio = 0;

	GalaxyParams Sc;
	Sc.TwistStrength = 8;
	Sc.ArmNumArms = 4;
	Sc.ArmIncoherence = 8;
	Sc.ArmStartRatio = 0;

	//Sc.TwistCoreTwistExponent = .3;
	Sc.TwistCoreRadius = .02;

	GalaxyParams SBa;
	SBa.BulgeRatio = .35;
	SBa.BulgeAxisScale = FVector(1, .3, .3);
	SBa.TwistStrength = 4;
	SBa.TwistCoreStrength = 0;
	SBa.ArmNumArms = 2;
	SBa.ArmClusterRadiusMin = .05;
	SBa.ArmClusterRadiusMax = .4;
	SBa.ArmIncoherence = 2;
	SBa.ArmHeightRatio = .5;
	SBa.ArmStartRatio = 0.0;

	GalaxyParams SBb; 
	SBb.BulgeAxisScale = FVector(1, .3, .3);
	SBb.TwistStrength = 6;
	SBb.TwistCoreStrength = 0;
	SBb.ArmNumArms = 2;
	SBb.ArmClusterRadiusMin = .05;
	SBb.ArmClusterRadiusMax = .25;
	SBb.ArmIncoherence = 8;
	SBb.ArmHeightRatio = .5;
	SBb.ArmStartRatio = 0;

	GalaxyParams SBc;
	SBc.BulgeAxisScale = FVector(1, 1, 1);
	SBc.TwistStrength = 12;
	SBc.TwistCoreStrength = 0;
	SBc.ArmNumArms = 2;
	SBc.ArmClusterRadiusMin = .05;
	SBc.ArmClusterRadiusMax = .3;
	SBc.ArmIncoherence = 8;
	SBc.ArmHeightRatio = .5;
	SBc.ArmStartRatio = 0;
	
	GalaxyParams Irr; //Probably need a special case for this since it would be cluster based

	Generator->GalaxyParams = S0;
	Generator->GenerateData(Octree);

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
		Extents[Index] = static_cast<float>(Node->Extent);
		Colors[Index] = FLinearColor(Node->Data.Composition);
	}, EParallelForFlags::BackgroundPriority);

	double FetchDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Node fetch + array population took: %.3f seconds"), FetchDuration);
}

void AGalaxyActor::InitializeVolumetric()
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
				LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarch_Inst.MT_GalaxyRaymarch_Inst")),
				this
			);

			DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), VolumeTexture);
			//TODO: Proceduralize material
			//AmbientColor
			FRandomStream Stream(Seed);

			//DynamicMaterial->SetVectorParameterValue(FName("AmbientColor"), ParentColor);
			//DynamicMaterial->SetVectorParameterValue(FName("CoolShift"), FLinearColor(Stream.FRandRange(0,6), Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), 1));
			//DynamicMaterial->SetVectorParameterValue(FName("HotShift"), FLinearColor(Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), 1));
			//DynamicMaterial->SetScalarParameterValue(FName("HueVariance"), Stream.FRandRange(0,.5));
			//DynamicMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Stream.FRandRange(.5, 3));
			//DynamicMaterial->SetScalarParameterValue(FName("SaturationVariance"), Stream.FRandRange(0, .5));
			//DynamicMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Stream.FRandRange(2, 8));
			//DynamicMaterial->SetScalarParameterValue(FName("TemperatureScale"), Stream.FRandRange(1, 6));
			//DynamicMaterial->SetScalarParameterValue(FName("Density"), Stream.FRandRange(0.1, .5));
			//DynamicMaterial->SetScalarParameterValue(FName("WarpAmount"), Stream.FRandRange(0.02, .15));
			//DynamicMaterial->SetScalarParameterValue(FName("WarpScale"), Stream.FRandRange(0.5, 2));

			//NoiseDomainOffset

			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->SetRelativeLocation(FVector(0.0f, 0.0f, .02 * Extent)); //Bump up slightly to account for insertion offset
			VolumetricComponent->SetMaterial(0, DynamicMaterial);
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
// Fill out your copyright notice in the Description page of Project Settings.
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include "Engine/VolumeTexture.h"
#include <Kismet/GameplayStatics.h>
#include <AssetRegistry/AssetRegistryModule.h>

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
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			double StartTime = FPlatformTime::Seconds();

			InitializeData();
			if (TryCleanUpComponents()) return;
			FetchData();
			if (TryCleanUpComponents()) return;
			InitializeVolumetric();
			if (TryCleanUpComponents()) return;
			InitializeNiagara();
			if (TryCleanUpComponents()) return;

			InitializationState = EGalaxyState::Ready;

			double TotalDuration = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Initialize total duration: %.3f seconds"), TotalDuration);
		});
}

bool AGalaxyActor::TryCleanUpComponents() {
	if (InitializationState != EGalaxyState::Destroying) return false;
	AsyncTask(ENamedThreads::GameThread, [this]()
	{
		if (VolumetricComponent) VolumetricComponent->DestroyComponent();
		if (NiagaraComponent) NiagaraComponent->DestroyComponent();
	});
	return true;
}

void AGalaxyActor::MarkDestroying() {
	InitializationState = EGalaxyState::Destroying;
}

void AGalaxyActor::InitializeData() {
	// ---------------------
	// Galaxy generation
	// ---------------------
	double StartTime = FPlatformTime::Seconds();
	FRandomStream Stream(Seed);
	auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];
	int DepthRange = 3;
	int InsertOffset = 5;
	double GlobularChance = .3;

	if (Stream.FRand() < GlobularChance)
	{
		auto GlobularGenerator = new GlobularNoiseGenerator(Seed);
		GlobularGenerator->Count = Count;
		GlobularGenerator->Falloff = Stream.FRandRange(.5, 1.5);
		GlobularGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
		GlobularGenerator->EncodedTree = EncodedTree;
		GlobularGenerator->DepthRange = DepthRange;
		GlobularGenerator->HorizontalExtent = .8;
		GlobularGenerator->VerticalExtent = Stream.FRandRange(.4, .8);
		GlobularGenerator->WarpAmount = FVector(Stream.FRandRange(.0, 1));
		GlobularGenerator->InsertDepthOffset = InsertOffset;
		GlobularGenerator->GenerateData(Octree);
	}
	else
	{
		auto SpiralGenerator = new SpiralNoiseGenerator(Seed);
		SpiralGenerator->Count = Count;
		SpiralGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
		SpiralGenerator->DepthRange = DepthRange;
		SpiralGenerator->NumArms = Stream.RandRange(2, 8);
		SpiralGenerator->PitchAngle = Stream.FRandRange(5, 40);
		SpiralGenerator->ArmContrast = Stream.FRandRange(.2, .8);
		SpiralGenerator->RadialFalloff = Stream.FRandRange(1.5, 3);
		SpiralGenerator->CenterScale = Stream.FRandRange(.01, .02);

		double SpreadMin = Stream.FRandRange(.01, .03);
		double SpreadMax = Stream.FRandRange(.15, .3);
		SpiralGenerator->HorizontalSpreadMin = SpreadMin;
		SpiralGenerator->HorizontalSpreadMax = SpreadMax;
		SpiralGenerator->VerticalSpreadMin = SpreadMin;
		SpiralGenerator->VerticalSpreadMax = SpreadMax;

		double HorizontalWarp = Stream.FRandRange(.1, .7);
		double VerticalWarp = Stream.FRandRange(.1, .7);

		SpiralGenerator->WarpAmount = FVector(HorizontalWarp, HorizontalWarp, VerticalWarp);
		SpiralGenerator->EncodedTree = EncodedTree;
		SpiralGenerator->InsertDepthOffset = InsertOffset;
		SpiralGenerator->GenerateData(Octree);
	}
	double GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("Galaxy generation took: %.3f seconds"), GenDuration);
}

void AGalaxyActor::FetchData() {	
	// ---------------------
	// Fetch populated nodes
	// ---------------------
	double StartTime = FPlatformTime::Seconds();
	TArray<TSharedPtr<FOctreeNode>> Nodes = Octree->GetPopulatedNodes(-1, -1, 1);
	Positions.SetNumUninitialized(Nodes.Num());
	Extents.SetNumUninitialized(Nodes.Num());
	Colors.SetNumUninitialized(Nodes.Num());
	ParallelFor(Nodes.Num(), [&](int32 Index)
		{
			const TSharedPtr<FOctreeNode>& Leaf = Nodes[Index];
			FRandomStream RandStream(Leaf->Data.ObjectId);
			Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);
			Extents[Index] = static_cast<float>(Leaf->Extent);
			Colors[Index] = FLinearColor(Leaf->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);

	double FetchDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("Node fetch + array population took: %.3f seconds"), FetchDuration);
}

void AGalaxyActor::InitializeNiagara()
{
	// ---------------------
	// Initialize particle system
	// ---------------------
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
		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Extent), FVector(Extent)));
			NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Extent);
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

void AGalaxyActor::InitializeVolumetric()
{
	// ---------------------
	// Initialize volumetric data
	// ---------------------
	double StartTime = FPlatformTime::Seconds();
	int Resolution = 32;
	TextureData = Octree->CreateVolumeDensityDataFromOctree(Resolution);
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

			UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarch_Inst.MT_GalaxyRaymarch_Inst")),
				this
			);
			DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), VolumeTexture);

			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->SetMaterial(0, DynamicMaterial);
			VolumetricComponent->RegisterComponent();
			FlushRenderingCommands();

			double SourceDuration = FPlatformTime::Seconds() - SourceStart;
			UE_LOG(LogTemp, Log, TEXT("VolumeTexture Source.Init & Update took: %.3f seconds"), SourceDuration);

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("Volumetric initialization took: %.3f seconds"), VolumetricDuration);
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

void AGalaxyActor::BeginPlay()
{
	Super::BeginPlay();
	GalaxyRealSpaceOrigin = GetActorLocation(); // <- Store initial spawn position

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


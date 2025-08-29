// Fill out your copyright notice in the Description page of Project Settings.
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include "Engine/VolumeTexture.h"
#include <Kismet/GameplayStatics.h>
#include <AssetRegistry/AssetRegistryModule.h>

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

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
	{
		//Populate Data into the tree
		Octree = MakeShared<FOctree>(Extent);
		FRandomStream Stream = FRandomStream(Seed);
		auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];
		int DepthRange = 6;
		int InsertOffset = 2;
		double GlobularChance = .3;
		if (Stream.FRand() < GlobularChance) {
			auto GlobularGenerator = new GlobularNoiseGenerator(Seed);
			GlobularGenerator->Count = Count;
			GlobularGenerator->Falloff = Stream.FRandRange(.5, 1.5);
			GlobularGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
			GlobularGenerator->EncodedTree = EncodedTree;
			GlobularGenerator->DepthRange = DepthRange;
			GlobularGenerator->HorizontalExtent = .8;/// *Extent;
			GlobularGenerator->VerticalExtent = Stream.FRandRange(.4, .8);// *Extent;
			GlobularGenerator->WarpAmount = FVector(Stream.FRandRange(.0, 1));
			GlobularGenerator->InsertDepthOffset = InsertOffset;
			GlobularGenerator->GenerateData(Octree);
		}
		else {
			auto SpiralGenerator = new SpiralNoiseGenerator(Seed);
			//Proceduralize
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

		TArray<TSharedPtr<FOctreeNode>> Nodes = Octree->GetPopulatedNodes(-1, -1, 1); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
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
		});

		//Pass the arrays back to the game thread to instantiate the particle system
		InitializeVolumetric();
		InitializeNiagara();
	});
}

void AGalaxyActor::InitializeNiagara()
{
	AsyncTask(ENamedThreads::GameThread, [this]()
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
			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
			{
				NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Extent), FVector(Extent)));
				NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Extent);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);
				AsyncTask(ENamedThreads::GameThread, [this]()
				{
					NiagaraComponent->Activate(true);
				});
			});
	});
}

void AGalaxyActor::InitializeVolumetric() {
	int Resolution = 32;
	TextureData = Octree->CreateVolumeDensityDataFromOctree(Resolution);

	AsyncTask(ENamedThreads::GameThread, [this, Resolution]()
	{
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
		//AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, Resolution]()
		//{
			VolumeTexture->Source.Init(Resolution, Resolution, Resolution, 1, ETextureSourceFormat::TSF_BGRA8, TextureData.GetData());	
			//AsyncTask(ENamedThreads::GameThread, [this, Resolution]()
			//{
				VolumeTexture->UpdateResource();
				FlushRenderingCommands();
				UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarch_Inst.MT_GalaxyRaymarch_Inst")), this);
				DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), VolumeTexture);

				VolumetricComponent = NewObject<UStaticMeshComponent>(this);
				VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
				VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
				VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
				VolumetricComponent->SetMaterial(0, DynamicMaterial);
				VolumetricComponent->RegisterComponent();
			//});
		//});
	});
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


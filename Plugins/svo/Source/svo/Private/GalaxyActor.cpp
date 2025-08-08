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

	Async(EAsyncExecution::Thread, [this]()
		{
			//Populate Data into the tree
			Octree = MakeShared<FOctree>(Extent);
			FRandomStream Stream = FRandomStream(Seed);
			this->Count = Stream.RandRange(200000, 500000);
			auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];
			int DepthRange = 4;

			if (Stream.FRand() < .3) {
				auto GlobularGenerator = new GlobularNoiseGenerator(Seed);
				GlobularGenerator->Count = Count;
				GlobularGenerator->Falloff = Stream.FRandRange(.5, 1.5);
				GlobularGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
				GlobularGenerator->EncodedTree = EncodedTree;
				GlobularGenerator->DepthRange = DepthRange;
				GlobularGenerator->HorizontalExtent = .9;/// *Extent;
				GlobularGenerator->VerticalExtent = Stream.FRandRange(.1, .9);// *Extent;
				GlobularGenerator->WarpAmount = FVector(Stream.FRandRange(.0, 1.1));
				GlobularGenerator->InsertDepthOffset = 4;
				GlobularGenerator->GenerateData(Octree);
			}
			else {
				auto SpiralGenerator = new SpiralNoiseGenerator(Seed);
				//Proceduralize
				SpiralGenerator->Count = Count;
				SpiralGenerator->Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
				SpiralGenerator->DepthRange = DepthRange;
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
				SpiralGenerator->EncodedTree = EncodedTree;
				SpiralGenerator->InsertDepthOffset = 4;
				SpiralGenerator->GenerateData(Octree);
			}

			//GlobularGenerator->GenerateData(Octree);
			//Extract data from the tree and construct niagara arrays
			TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetPopulatedNodes(); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
			Positions.SetNumUninitialized(Leaves.Num());
			Extents.SetNumUninitialized(Leaves.Num());
			Colors.SetNumUninitialized(Leaves.Num());

			ParallelFor(Leaves.Num(), [&](int32 Index)
				{
					const TSharedPtr<FOctreeNode>& Leaf = Leaves[Index];
					FRandomStream RandStream(Leaf->Data.ObjectId);
					FVector ColorVector = RandStream.GetUnitVector();
					Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);// Should already be in particle system local space
					Extents[Index] = static_cast<float>(Leaf->Extent);
					Colors[Index] = FLinearColor(ColorVector);
				});

			//Pass the arrays back to the game thread to instantiate the particle system
			//InitializeNiagara(Positions, Extents, Colors);
			AsyncTask(ENamedThreads::GameThread, [this]()
				{
					InitializeNiagara();
				});
		});
}

void AGalaxyActor::InitializeNiagara()
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

			//TODO::Proceduralize Dust cloud material
			FRandomStream Stream = FRandomStream(Seed);
			UMaterialInterface* GasMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/GasSpriteMaterial_Inst.GasSpriteMaterial_Inst"));
			UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(GasMaterial, this);

			FLinearColor SecondaryColor = FLinearColor(Stream.GetUnitVector());
			DynamicMaterial->SetScalarParameterValue(FName("Extent"), Extent);

			FLinearColor HsvParent = ParentColor.LinearRGBToHSV();

			float Offset = Stream.FRandRange(20.f, 60.f);

			FLinearColor Color1 = FLinearColor(HsvParent.R - Offset + 360.f, HsvParent.G, HsvParent.B);
			FLinearColor Color2 = FLinearColor(HsvParent.R + Offset + 360.f, HsvParent.G, HsvParent.B);
			//dm->SetScalarParameterValue(FName("RadialFalloff"), 4);

			DynamicMaterial->SetVectorParameterValue(FName("Color1"), Color1.HSVToLinearRGB());
			DynamicMaterial->SetVectorParameterValue(FName("Color2"), Color2.HSVToLinearRGB());
			DynamicMaterial->SetVectorParameterValue(FName("PositionScaleOffset"), FVector4(Stream.FRandRange(-1,1), Stream.FRandRange(-1, 1), Stream.FRandRange(-1, 1), Stream.FRandRange(.75, 1.25)));
			DynamicMaterial->SetScalarParameterValue(FName("BaseNoiseScale"), Stream.FRandRange(.5, 1.5));
			DynamicMaterial->SetScalarParameterValue(FName("DistortionNoiseScale"), Stream.FRandRange(.1, .5));
			DynamicMaterial->SetScalarParameterValue(FName("DistortionAmount"), Stream.FRandRange(.02, .5));
			DynamicMaterial->SetScalarParameterValue(FName("ColorBalance"), Stream.FRandRange(1.5, 2.5));
			DynamicMaterial->SetScalarParameterValue(FName("ParticleColorInfluence"), Stream.FRandRange(0, .6));
			DynamicMaterial->SetScalarParameterValue(FName("OpacityMultiplier"), Stream.FRandRange(.005, .02));

			//Select random volume textures here
			TArray<UVolumeTexture*> NoiseTextures;
			FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

			FARFilter Filter;
			Filter.ClassPaths.Add(UVolumeTexture::StaticClass()->GetClassPathName());
			Filter.PackagePaths.Add("/SVO/VolumeTextures");
			Filter.bRecursivePaths = true;

			TArray<FAssetData> AssetList;
			AssetRegistryModule.Get().GetAssets(Filter, AssetList);

			for (const FAssetData& Asset : AssetList)
			{
				UVolumeTexture* Tex = Cast<UVolumeTexture>(Asset.GetAsset());
				if (Tex)
				{
					NoiseTextures.Add(Tex);
				}
			}
			DynamicMaterial->SetTextureParameterValue(FName("BaseNoise"), NoiseTextures[Stream.RandRange(0, NoiseTextures.Num() - 1)]);
			DynamicMaterial->SetTextureParameterValue(FName("DistortionNoise"), NoiseTextures[Stream.RandRange(0, NoiseTextures.Num() - 1)]);
			NiagaraComponent->SetVariableMaterial(FName("User.GasMaterial"), DynamicMaterial);
			//
			// 
			//Async(EAsyncExecution::Thread, [this]()
			//{
					// Pass in user parameters (assuming Niagara system is setup to receive them)
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), Positions);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), Colors);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), Extents);

					//AsyncTask(ENamedThreads::GameThread, [this, Positions = MoveTemp(Positions), Extents = MoveTemp(Extents), Colors = MoveTemp(Colors)]()
					//{
			NiagaraComponent->Activate(true);
			NiagaraComponent->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
					//});
			//});

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


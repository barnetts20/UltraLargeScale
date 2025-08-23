// Fill out your copyright notice in the Description page of Project Settings.
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include "Engine/VolumeTexture.h"
#include <Kismet/GameplayStatics.h>
#include "NiagaraSystemInstance.h"
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
			auto EncodedTree = EncodedTrees[Stream.RandRange(0, 5)];
			int DepthRange = 4;
			int InsertOffset = 3;
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

			//GlobularGenerator->GenerateData(Octree);
			//Extract data from the tree and construct niagara arrays
			TArray<TSharedPtr<FOctreeNode>> Leaves = Octree->GetPopulatedNodes(-1, -1, 1); // Replace this to pick up nodes with density instead of leaves, can store gas/stars in same tree at different depths
			Positions.SetNumUninitialized(Leaves.Num());
			Extents.SetNumUninitialized(Leaves.Num());
			Colors.SetNumUninitialized(Leaves.Num());

			ParallelFor(Leaves.Num(), [&](int32 Index)
			{
				const TSharedPtr<FOctreeNode>& Leaf = Leaves[Index];
				FRandomStream RandStream(Leaf->Data.ObjectId);
				Positions[Index] = FVector(Leaf->Center.X, Leaf->Center.Y, Leaf->Center.Z);
				Extents[Index] = static_cast<float>(Leaf->Extent);
				Colors[Index] = FLinearColor(Leaf->Data.Composition);
			});

			//Pass the arrays back to the game thread to instantiate the particle system
			//InitializeNiagara(Positions, Extents, Colors);
			SpawnInstancedGalaxy();
			//InitializeNiagara();
			AsyncTask(ENamedThreads::GameThread, [this]()
				{
					InitializeVolumetric(Octree->CreateVolumeTextureFromOctree(64));
				});
		});
}

void AGalaxyActor::SpawnInstancedGalaxy()
{
	if (!InstancedStarMesh || Positions.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Cannot spawn galaxy - missing component or no position data"));
		return;
	}

	// Create transforms array
	TArray<FTransform> InstanceTransforms;
	TArray<float> CustomDataArray;
	InstanceTransforms.Reserve(Positions.Num());
	CustomDataArray.Reserve(Positions.Num() * 3);
	// Convert your data to transforms (in local space)
	ParallelFor(Positions.Num(), [&](int32 Index)
	{
		//Need a struct to pack this up with color
		FTransform InstanceTransform;
		InstanceTransform.SetLocation(Positions[Index]);        // Local position relative to actor
		InstanceTransform.SetRotation(FQuat::Identity);     // No rotation for now
		InstanceTransform.SetScale3D(FVector(Extents[Index]));  // Uniform scale from extent
		InstanceTransforms.Add(InstanceTransform);
		CustomDataArray.Add(Colors[Index].R);
		CustomDataArray.Add(Colors[Index].G);
		CustomDataArray.Add(Colors[Index].B);
	});

	// Spawn all instances at once
	AsyncTask(ENamedThreads::GameThread, [this, InstanceTransforms, CustomDataArray]()
	{
		// CRITICAL: Disable per-instance culling
		InstancedStarMesh->bNeverDistanceCull = true;
		UMaterialInterface* GasMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/MT_EmissiveColor.MT_EmissiveColor"));
		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(GasMaterial, this);
		InstancedStarMesh->SetMaterial(0, DynamicMaterial);
		//InstancedStarMesh->SetCustomData() Use this to encode color??
		InstancedStarMesh->AddInstances(InstanceTransforms, false, false, false);
		//InstancedStarMesh->SetCustomData(0, CustomDataArray);
	});

	UE_LOG(LogTemp, Log, TEXT("Spawned %d instanced stars"), Positions.Num());
}

void AGalaxyActor::InitializeNiagara()
{
	AsyncTask(ENamedThreads::GameThread, [this]()
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
				true,
				false
			);

			if (NiagaraComponent)
			{
				NiagaraComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
				FBox Bounds(FVector(-Extent), FVector(Extent));
				NiagaraComponent->SetSystemFixedBounds(Bounds);
				NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Extent);
				PopulatingNiagara = true;
			}
		}
	});
}

void AGalaxyActor::InitializeVolumetric(UVolumeTexture* InVolumeTexture) {
	AsyncTask(ENamedThreads::GameThread, [this, InVolumeTexture]()
	{
		UMaterialInterface* GasMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/svo/Materials/RayMarchers/MT_GalaxyRaymarch_Inst.MT_GalaxyRaymarch_Inst"));
		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(GasMaterial, this);

		//TODO: Configure material with the volume texture
		FRandomStream RandomStream(Seed);
		DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), InVolumeTexture);
		//DynamicMaterial->SetScalarParameterValue(FName("Density"), 20);
		DynamicMaterial->SetScalarParameterValue(FName("MaxSteps"), 64);
		//DynamicMaterial->SetScalarParameterValue(FName("WarpAmount"), .2);
		//DynamicMaterial->SetScalarParameterValue(FName("WarpFrequency"), .2);
		// Convert parent color to HSV for better color manipulation
		FVector ColorVector(ParentColor.R, ParentColor.G, ParentColor.B);
		FLinearColor ParentHSV = ParentColor.LinearRGBToHSV();

		// Randomize hue shift (±30 degrees)
		float HueShift = RandomStream.FRandRange(-30.0f, 30.0f);
		float NewHue = FMath::Fmod(ParentHSV.R + HueShift + 360.0f, 360.0f);

		// Randomize saturation (0.7 to 1.3 of original)
		float SaturationMultiplier = RandomStream.FRandRange(0.7f, 1.3f);
		float NewSaturation = FMath::Clamp(ParentHSV.G * SaturationMultiplier, 0.0f, 1.0f);

		// Randomize brightness (0.8 to 1.5 of original for lighter variants)
		float BrightnessMultiplier = RandomStream.FRandRange(0.8f, 1.5f);
		float NewBrightness = FMath::Clamp(ParentHSV.B * BrightnessMultiplier, 0.0f, 1.0f);

		FLinearColor RandomizedHSV(NewHue, NewSaturation, NewBrightness, ParentColor.A);
		FLinearColor LightColor = RandomizedHSV.HSVToLinearRGB();

		DynamicMaterial->SetVectorParameterValue(FName("LightColor"), RandomStream.GetUnitVector().GetAbs());
		DynamicMaterial->SetVectorParameterValue(FName("AmbientColor"), ParentColor);
		//Set up color variance etc
		//

		UStaticMesh* VolumetricMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals"));
		VolumetricComponent = NewObject<UStaticMeshComponent>(this);
		VolumetricComponent->SetStaticMesh(VolumetricMesh);
		VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));
		VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		VolumetricComponent->SetMaterial(0, DynamicMaterial);
		VolumetricComponent->RegisterComponent();
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
	FVector PlayerOffset = CurrentFrameOfReferenceLocation - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	FVector ParallaxOffset = PlayerOffset * (1.0 - ParallaxRatio);
	SetActorLocation(GetActorLocation() + ParallaxOffset);

	//TODO:: PROCESS NIAGARA CHUNKS UNTIL EMPTY
	//if (PopulatingNiagara) {
	//	if (Positions.Num() == 0)
	//	{
	//		PopulatingNiagara = false;
	//		NiagaraComponent->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	//		return;
	//	}

	//	// Determine how many elements to take this frame (end boundary)
	//	int32 NumToProcess = FMath::Min(ChunkSize, Positions.Num());

	//	// Clear previous chunk contents (optional, depends on processing needs)
	//	TArray<FVector> PositionChunk;
	//	TArray<float> ExtentChunk;
	//	TArray<FLinearColor> ColorChunk;

	//	PositionChunk.AddZeroed(NumToProcess);
	//	ExtentChunk.AddZeroed(NumToProcess);
	//	ColorChunk.AddZeroed(NumToProcess);
	//	// Copy chunk from MasterArray into ChunkArray
	//	ParallelFor(NumToProcess, [&](int32 Index)
	//	{
	//		PositionChunk[Index] = Positions[Index];
	//		ExtentChunk[Index] = Extents[Index];
	//		ColorChunk[Index] = Colors[Index];
	//	});
	//	//for (int32 i = 0; i < NumToProcess; ++i)
	//	//{
	//	//	PositionChunk[i] = Positions[i];
	//	//	ExtentChunk[i] = Extents[i];
	//	//	ColorChunk[i] = Colors[i];
	//	//}
	//	// Remove that chunk from MasterArray
	//	Positions.RemoveAt(0, NumToProcess);
	//	Extents.RemoveAt(0, NumToProcess);
	//	Colors.RemoveAt(0, NumToProcess);

	//	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), PositionChunk);
	//	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("User.Colors"), ColorChunk);
	//	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(NiagaraComponent, FName("User.Extents"), ExtentChunk);
	//	
	//	NiagaraComponent->ResetSystem();
	//	NiagaraComponent->Activate(true);
	//}
	Super::Tick(DeltaTime);
}


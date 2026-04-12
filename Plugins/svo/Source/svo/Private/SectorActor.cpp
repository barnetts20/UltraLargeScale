#pragma region Includes/ForwardDec
#include "SectorActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor
ASectorActor::ASectorActor()
{
	PointCloudNiagara = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_SectorParallaxCloud.NG_SectorParallaxCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent);
}
#pragma endregion

#pragma region Initialization
void ASectorActor::BeginPlay()
{
	Super::BeginPlay();

	// Initialize last frame location to player position to avoid first-frame jump
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				LastFrameOfReferenceLocation = Pawn->GetActorLocation();
				CurrentFrameOfReferenceLocation = LastFrameOfReferenceLocation;
			}
		}
	}

	Initialize();
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
	// Galaxy pool disabled for parallax prototype testing
	/*
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			for (int i = 0; i < GalaxyPoolSize; i++) {
				AGalaxyActor* Galaxy = GetWorld()->SpawnActor<AGalaxyActor>(GalaxyActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				Galaxy->Universe = this;
				GalaxyPool.Add(Galaxy);
			}
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
	*/
}

void ASectorActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	// --- Phase 1: Gas density from noise (low-res) ---
	int noiseResolution = 64;
	auto DensityNoise = FastNoise::NewFromEncodedNodeTree(Params.EncodedTree);
	TArray<uint8> LowResData = FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		noiseResolution,
		Params.Extent,
		Octree,
		FMath::FloorLog2(noiseResolution),
		1.0f,
		3      // write to alpha channel (density)
	);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Upscale noise to 256^3 ---
	TArray<uint8> VolumeData = FVolumeTextureUtils::UpscaleVolumeData(LowResData, noiseResolution);
	LowResData.Empty(); // Free low-res buffer

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Phase 2: Object generation (VBOs) ---
	UniverseGenerator.Params = Params;
	UniverseGenerator.GenerateData(Octree);
	TArray<TSharedPtr<FOctreeNode>> VolumeChunks;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(UniverseGenerator.GeneratedData, PointNodes, VolumeChunks);

	// --- Phase 2b: Splat VBO density into 256^3 volume ---
	// Each galaxy VBO contributes a spherical density kernel at depth-8 resolution.
	// Where galaxies cluster, splats overlap and accumulate — clusters emerge
	// from the density overlap, no dedicated detection needed.
	FVolumeTextureUtils::SplatVBOsToVolume(
		VolumeData,
		256,            // Always splat at full volume resolution
		Params.Extent,
		PointNodes,
		2.0f,           // RadiusScale — fatten splats so clusters read at 256^3
		2.0f,           // FalloffPower — quadratic smooth falloff
		8.0f,           // Intensity — leave headroom for overlap accumulation
		-1              // Channel — all channels (grayscale density for now)
	);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Build Niagara arrays from point nodes ---
	Positions.SetNumUninitialized(PointNodes.Num());
	Rotations.SetNumUninitialized(PointNodes.Num());
	Extents.SetNumUninitialized(PointNodes.Num());
	Colors.SetNumUninitialized(PointNodes.Num());

	ParallelFor(PointNodes.Num(), [&](int32 Index) {
		const TSharedPtr<FOctreeNode>& Node = PointNodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Rotations[Index] = RandStream.GetUnitVector();
		Positions[Index] = Node->Center;
		Extents[Index] = static_cast<float>(Node->Extent * (1 + Node->Data.ScaleFactor));
		Colors[Index] = FLinearColor(Node->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);

	double ObjectDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::Object generation took: %.3f seconds"), ObjectDuration);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Pack and create texture (volume data already at 256^3) ---
	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(
		FVolumeTextureUtils::PackToPseudoVolumeLayout(VolumeData)
		//, "/svo/Generated/BakedTest"
	);

	double GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeData total took: %.3f seconds"), GenDuration);
}

void ASectorActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetVisibility(false);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->TranslucencySortPriority = 1;
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

			UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath),
				this
			);

			DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumetricComponent->SetMaterial(0, DynamicMaterial);
			VolumetricComponent->SetVisibility(true);

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void ASectorActor::InitializeNiagara()
{
	//return; //
	double StartTime = FPlatformTime::Seconds();

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			// Capture player position at the moment we set up Niagara
			FVector PlayerPos = FVector::ZeroVector;
			if (auto* Controller = UGameplayStatics::GetPlayerController(GetWorld(), 0))
			{
				if (APawn* Pawn = Controller->GetPawn())
				{
					PlayerPos = Pawn->GetActorLocation();
				}
			}

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

			// Center the Niagara system on the player immediately
			NiagaraComponent->SetWorldLocation(PlayerPos);

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
				{
					// Compute player-relative positions for Niagara
					// Octree positions (Positions array) stay as absolute ground truth
					TArray<FVector> RelativePositions;
					RelativePositions.SetNumUninitialized(Positions.Num());
					ParallelFor(Positions.Num(), [&](int32 i) {
						RelativePositions[i] = Positions[i] - PlayerPos;
						}, EParallelForFlags::BackgroundPriority);

					UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NiagaraComponent, FName("User.Positions"), RelativePositions);
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
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Player-Centered Parallax
void ASectorActor::ApplyParallaxOffset()
{
	
	if (!NiagaraComponent || InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	// Get player position
	bool bHasReference = false;
	FVector CurrentPlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				CurrentPlayerPos = Pawn->GetActorLocation();
				bHasReference = true;
			}
		}
	}

	if (!bHasReference)
	{
		return;
	}

	// Compute parallax offset for this frame
	FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;

	// ParallaxRatio = SpeedScale / UnitScale
	ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
	FVector ParallaxOffset = -PlayerDelta * ParallaxRatio;

	// Push offset to Niagara — all particles shift by this amount
	NiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);

	SetActorLocation(GetActorLocation() + ParallaxOffset);
	// Keep the Niagara system centered on the player
	NiagaraComponent->SetWorldLocation(CurrentPlayerPos);

}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void ASectorActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
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

	Galaxy->Params.UnitScale = (InNode->Extent * this->Params.UnitScale) / Galaxy->Params.Extent;
	Galaxy->SpeedScale = SpeedScale;
	Galaxy->Params.Seed = InNode->Data.ObjectId;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	// Compute axis tilt
	FRandomStream RandStream(InNode->Data.ObjectId);
	Galaxy->Params.Rotation = RandStream.GetUnitVector().Rotation();
	// Compute correct parallax ratios and spawn location
	Galaxy->SetActorLocation(ComputeChildSpawnLocation(InNode->Center, Galaxy->Params.UnitScale));
	Galaxy->Initialize();
	Galaxy->SetActorHiddenInGame(false);
}

void ASectorActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid())
	{
		return;
	}

	TWeakObjectPtr<AGalaxyActor> GalaxyToDestroy;
	if (SpawnedGalaxies.RemoveAndCopyValue(InNode, GalaxyToDestroy))
	{
		AGalaxyActor* PoolGalaxy = GalaxyToDestroy.Get();
		if (PoolGalaxy)
		{
			UE_LOG(LogTemp, Log, TEXT("Resetting galaxy for node with ObjectId: %d"), InNode->Data.ObjectId);
			PoolGalaxy->ResetForPool();

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PoolGalaxy]()
				{
					double StartTime = FPlatformTime::Seconds();

					// Flush octree
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
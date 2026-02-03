#pragma region Includes/ForwardDec
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor/Destructor
AGalaxyActor::AGalaxyActor()
{
	PointCloudNiagara = Cast<UNiagaraSystem>(FSoftObjectPath(NiagaraPath).TryLoad());
	StarSystemActorClass = AStarSystemActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent);
}

AGalaxyActor::~AGalaxyActor()
{
	Positions.Empty();
	Extents.Empty();
	Colors.Empty();
	GalaxyGenerator.GeneratedData.Empty();
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region Initialization
void AGalaxyActor::InitializeChildPool()
{
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			for (int i = 0; i < StarSystemPoolSize; i++) {
				AStarSystemActor* System = GetWorld()->SpawnActor<AStarSystemActor>(StarSystemActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				System->Galaxy = this;
				StarSystemPool.Add(System);
			}
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
}

void AGalaxyActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	GalaxyGenerator.Params = Params;
	GalaxyGenerator.GeneratedData.SetNum(0);

	GalaxyParamFactory GalaxyParamGen;
	GalaxyParamGen.Seed = Params.Seed;
	GalaxyGenerator.Params = GalaxyParamGen.GenerateParams();
	GalaxyGenerator.GenerateData(Octree);

	double GenFinish = FPlatformTime::Seconds();
	double GenDuration = GenFinish - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Data Generation took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return;

	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(GalaxyGenerator.GeneratedData, PointNodes, VolumeNodes);

	double InsertFinish = FPlatformTime::Seconds();
	GenDuration = InsertFinish - GenFinish;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Bulk Insert took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return;

	Positions.SetNumUninitialized(PointNodes.Num());
	Extents.SetNumUninitialized(PointNodes.Num());
	Colors.SetNumUninitialized(PointNodes.Num());

	ParallelFor(PointNodes.Num(), [&](int32 Index) {
		const TSharedPtr<FOctreeNode>& Node = PointNodes[Index];
		FRandomStream RandStream(Node->Data.ObjectId);
		Positions[Index] = Node->Center;
		Extents[Index] = static_cast<float>(Node->Extent * (1 + Node->Data.Density));
		Colors[Index] = FLinearColor(Node->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);

	PseudoVolumeTexture = FOctreeTextureProcessor::GeneratePseudoVolumeTextureFromMipData(
		FOctreeTextureProcessor::UpscalePseudoVolumeDensityData(
			FOctreeTextureProcessor::GenerateVolumeMipDataFromOctree(VolumeNodes, 32, Params.Extent, Octree->DepthMaxDensity),
			32
		)
	);

	double RemapFinish = FPlatformTime::Seconds();
	GenDuration = RemapFinish - InsertFinish;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Data Remap took: %.3f seconds"), GenDuration);

	GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeData took: %.3f seconds"), GenDuration);
}

void AGalaxyActor::InitializeVolumetric()
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
			VolumetricComponent->DepthPriorityGroup = ESceneDepthPriorityGroup::SDPG_MAX;
			VolumetricComponent->bRenderInDepthPass = false;
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

			VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath),
				this
			);

			VolumeMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *GalaxyGenerator.Params.VolumeNoise));
			VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), GalaxyGenerator.Params.VolumeAmbientColor);
			VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), GalaxyGenerator.Params.VolumeCoolShift);
			VolumeMaterial->SetVectorParameterValue(FName("HotShift"), GalaxyGenerator.Params.VolumeHotShift);
			VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), GalaxyGenerator.Params.VolumeHueVariance);
			VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), GalaxyGenerator.Params.VolumeHueVarianceScale);
			VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), GalaxyGenerator.Params.VolumeSaturationVariance);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), GalaxyGenerator.Params.VolumeTemperatureInfluence);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), GalaxyGenerator.Params.VolumeTemperatureScale);
			VolumeMaterial->SetScalarParameterValue(FName("Density"), GalaxyGenerator.Params.VolumeDensity);
			VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), GalaxyGenerator.Params.VolumeWarpAmount);
			VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), GalaxyGenerator.Params.VolumeWarpScale);

			VolumetricComponent->SetMaterial(0, VolumeMaterial);
			VolumetricComponent->SetVisibility(true);

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
			NiagaraComponent->SetSystemFixedBounds(FBox(FVector(-Params.Extent), FVector(Params.Extent)));
			NiagaraComponent->SetVariableFloat(FName("MaxExtent"), Params.Extent);
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

#pragma region Star System Pooled Spawn Hooks
void AGalaxyActor::SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !StarSystemActorClass || SpawnedStarSystems.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	if (StarSystemPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Star System pool exhausted, consider increasing StarSystemPoolSize"));
		return;
	}

	AStarSystemActor* System = StarSystemPool.Pop();
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));
	System->ResetForSpawn();

	System->Params.UnitScale = (InNode->Extent * Params.UnitScale) / System->Params.Extent;
	System->SpeedScale = Universe->SpeedScale;
	System->Params.Seed = InNode->Data.ObjectId;
	System->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	System->Params.Rotation = FRandomStream(InNode->Data.ObjectId).GetUnitVector().Rotation();

	// Compute spawn location using base class helper
	System->SetActorLocation(ComputeChildSpawnLocation(InNode->Center, System->Params.UnitScale));

	System->Initialize();
	System->SetActorHiddenInGame(false);
}

void AGalaxyActor::ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid())
	{
		return;
	}

	TWeakObjectPtr<AStarSystemActor> SystemToDestroy;
	if (SpawnedStarSystems.RemoveAndCopyValue(InNode, SystemToDestroy))
	{
		AStarSystemActor* PoolSystem = SystemToDestroy.Get();
		if (PoolSystem)
		{
			UE_LOG(LogTemp, Log, TEXT("Resetting star system for node with ObjectId: %d"), InNode->Data.ObjectId);
			PoolSystem->ResetForPool();

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PoolSystem]()
				{
					double StartTime = FPlatformTime::Seconds();

					// Flush octree
					PoolSystem->Octree->bIsResetting.store(true);
					FPlatformProcess::Sleep(0.05f);
					PoolSystem->Octree = MakeShared<FOctree>(PoolSystem->Params.Extent);
					PoolSystem->Octree->bIsResetting.store(false);

					double ODuration = FPlatformTime::Seconds() - StartTime;
					UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Flushing Octree took: %.3f seconds"), ODuration);

					// Return to pool on game thread
					AsyncTask(ENamedThreads::GameThread, [this, PoolSystem]()
						{
							StarSystemPool.Insert(PoolSystem, 0);
						});
				});
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Star system was already invalid for node with ObjectId: %d"), InNode->Data.ObjectId);
		}
	}
}
#pragma endregion
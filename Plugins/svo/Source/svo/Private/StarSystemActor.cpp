#pragma region Includes/ForwardDec
#include "StarSystemActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Engine/StaticMeshActor.h>
#include <Kismet/GameplayStatics.h>
#include <ParallaxStaticMeshActor.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor/Destructor
AStarSystemActor::AStarSystemActor()
{
	PointCloudNiagara = Cast<UNiagaraSystem>(FSoftObjectPath(NiagaraPath).TryLoad());
	Octree = MakeShared<FOctree>(Params.Extent);
}

AStarSystemActor::~AStarSystemActor()
{
	Positions.Empty();
	Extents.Empty();
	Colors.Empty();
	SystemGenerator.GeneratedData.Empty();
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region Initialization
void AStarSystemActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	SystemGenerator.Seed = Params.Seed;
	SystemGenerator.UnitScale = Params.UnitScale;
	SystemGenerator.Extent = Params.Extent;
	SystemGenerator.Rotation = Params.Rotation;
	SystemGenerator.GeneratedData.SetNum(0);

	SystemGenerator.SystemParams = Params;
	SystemGenerator.SystemParams.StarColor = Params.ParentColor;
	SystemGenerator.GenerateData(Octree);

	double GenFinish = FPlatformTime::Seconds();
	double GenDuration = GenFinish - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Data Generation took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return;

	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(SystemGenerator.GeneratedData, PointNodes, VolumeNodes);

	double InsertFinish = FPlatformTime::Seconds();
	GenDuration = InsertFinish - GenFinish;
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Bulk Insert took: %.3f seconds"), GenDuration);

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
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Data Remap took: %.3f seconds"), GenDuration);

	GenDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::InitializeData took: %.3f seconds"), GenDuration);
}

void AStarSystemActor::InitializeVolumetric()
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

			VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumeMaterialPath),
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
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
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
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Entity Spawn Hooks
void AStarSystemActor::SpawnEntityFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || SpawnedEntities.Contains(InNode)) return;

	double MeshScale = InNode->Extent * (1.0 + InNode->Data.Density) * Params.UnitScale;

	AsyncTask(ENamedThreads::GameThread, [this, InNode, MeshScale]()
		{
			if (UWorld* World = GetWorld())
			{
				FActorSpawnParameters SpawnParams;
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

				// Use base class helper for spawn location calculation
				FVector EntitySpawnPosition = ComputeChildSpawnLocation(InNode->Center, 1.0);

				AParallaxStaticMeshActor* MeshActor = World->SpawnActor<AParallaxStaticMeshActor>(
					AParallaxStaticMeshActor::StaticClass(),
					EntitySpawnPosition,
					FRotator::ZeroRotator,
					SpawnParams
				);

				if (MeshActor)
				{
					MeshActor->System = this;
					UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitSphere.UnitSphere"));

					if (SphereMesh)
					{
						UStaticMeshComponent* MeshComponent = MeshActor->MeshComponent;
						if (MeshComponent)
						{
							MeshComponent->SetMobility(EComponentMobility::Movable);
							MeshComponent->SetStaticMesh(SphereMesh);
							MeshComponent->SetWorldScale3D(FVector(MeshScale));
							MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
							MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);

							UMaterialInterface* BaseMaterial = MeshComponent->GetMaterial(0);
							if (BaseMaterial)
							{
								UMaterialInstanceDynamic* DynMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, MeshComponent);
								DynMaterial->SetVectorParameterValue(FName("BaseColor"), FLinearColor(InNode->Data.Composition));
								MeshComponent->SetMaterial(0, DynMaterial);
							}

							UE_LOG(LogTemp, Log, TEXT("Successfully spawned entity at %s with scale %.2f"),
								*EntitySpawnPosition.ToString(), MeshScale);
						}
					}

					SpawnedEntities.Add(InNode, TWeakObjectPtr<AActor>(MeshActor));
				}
			}
		});
}

void AStarSystemActor::ReturnEntityToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AActor>* FoundActor = SpawnedEntities.Find(InNode);
	if (FoundActor && FoundActor->IsValid())
	{
		AActor* ActorToDestroy = FoundActor->Get();
		AsyncTask(ENamedThreads::GameThread, [ActorToDestroy]()
			{
				if (ActorToDestroy && IsValid(ActorToDestroy))
				{
					ActorToDestroy->Destroy();
					UE_LOG(LogTemp, Log, TEXT("Successfully destroyed entity actor"));
				}
			});
		SpawnedEntities.Remove(InNode);
	}
}
#pragma endregion
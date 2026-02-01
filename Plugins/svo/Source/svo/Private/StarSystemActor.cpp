#pragma region Includes/ForwardDec
#include "StarSystemActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <PointCloudGenerator.h>
#include <Engine/StaticMeshActor.h>
#include <Kismet/GameplayStatics.h>
#include <Camera/CameraComponent.h>
#include <GameFramework/SpringArmComponent.h>
#include <ParallaxStaticMeshActor.h>
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
			UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Initialize total duration: %.3f seconds"), TotalDuration);
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
	SystemGenerator.UnitScale = UnitScale;
	SystemGenerator.Extent = Extent;
	SystemGenerator.Rotation = FRotator(AxisRotation.X, AxisRotation.Y, AxisRotation.Z);
	SystemGenerator.GeneratedData.SetNum(0);

	//TODO: If we want a factory approach it would happen here to generate the base params

	SystemGenerator.SystemParams = StarSystemParams();
	SystemGenerator.SystemParams.StarColor = ParentColor;
	SystemGenerator.GenerateData(Octree);

	double GenFinish = FPlatformTime::Seconds();
	double GenDuration = GenFinish - StartTime;
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Data Generation took: %.3f seconds"), GenDuration);

	if (InitializationState == ELifecycleState::Pooling) return; //Early exit if destroying

	TArray<TSharedPtr<FOctreeNode>> VolumeNodes;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(SystemGenerator.GeneratedData, PointNodes, VolumeNodes);

	double InsertFinish = FPlatformTime::Seconds();
	GenDuration = InsertFinish - GenFinish;
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::Bulk Insert took: %.3f seconds"), GenDuration);

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
			VolumetricComponent = NewObject<UStaticMeshComponent>(this); //TODO: THESE NEED TO MOVE INTO SEPERATE RUN ONCE INIT
			VolumetricComponent->SetVisibility(false);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));  //TODO: THESE NEED TO MOVE INTO SEPERATE RUN ONCE INIT
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->TranslucencySortPriority = 1;
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Extent));

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
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Lifecycle Management
void AStarSystemActor::ResetForSpawn() {
	InitializationState = ELifecycleState::Uninitialized;
}

void AStarSystemActor::SpawnEntityFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	// Don't spawn duplicates
	if (SpawnedEntities.Contains(InNode)) return;

	UE_LOG(LogTemp, Log, TEXT("=== Star System Entity Spawn Request ==="));

	// Get player position for parallax calculation
	FVector PlayerPosition = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPosition = Pawn->GetActorLocation();
			}
		}
	}

	// Now we need to "reverse" the parallax to get the real world position
	// The star system has parallax ratio: (SpeedScale / UnitScale)
	// Compute correct parallax ratios and spawn location
	double MeshScale = InNode->Extent * (1.0 + InNode->Data.Density) * UnitScale;

	// Spawn the static mesh actor on game thread
	AsyncTask(ENamedThreads::GameThread, [this, InNode, MeshScale]()
		{
			const double EntityParallaxRatio = Galaxy->Universe->SpeedScale;
			const double SystemParallaxRatio = Galaxy->Universe->SpeedScale / UnitScale;

			FVector NodeWorldPosition = FVector(InNode->Center) + GetActorLocation();
			FVector PlayerToNode = CurrentFrameOfReferenceLocation - NodeWorldPosition;
			FVector EntitySpawnPosition = CurrentFrameOfReferenceLocation - PlayerToNode * (EntityParallaxRatio / SystemParallaxRatio);

			if (UWorld* World = GetWorld())
			{
				FActorSpawnParameters SpawnParams;
				SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

				AParallaxStaticMeshActor* MeshActor = World->SpawnActor<AParallaxStaticMeshActor>(
					AParallaxStaticMeshActor::StaticClass(),
					EntitySpawnPosition,
					FRotator::ZeroRotator,
					SpawnParams
				);
				MeshActor->System = this;
				if (MeshActor)
				{
					// Load and set the unit sphere mesh
					UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitSphere.UnitSphere"));
					if (SphereMesh)
					{
						UStaticMeshComponent* MeshComponent = MeshActor->MeshComponent;
						if (MeshComponent)
						{
							MeshComponent->SetMobility(EComponentMobility::Movable);
							MeshComponent->SetStaticMesh(SphereMesh);
							MeshComponent->SetWorldScale3D(FVector(MeshScale));

							// Enable collision
							MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
							MeshComponent->SetCollisionResponseToAllChannels(ECR_Block);

							// Create dynamic material to set color based on composition
							UMaterialInterface* BaseMaterial = MeshComponent->GetMaterial(0);
							if (BaseMaterial)
							{
								UMaterialInstanceDynamic* DynMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, MeshComponent);
								FLinearColor NodeColor = FLinearColor(InNode->Data.Composition);

								// Try to set base color parameter (common parameter name)
								DynMaterial->SetVectorParameterValue(FName("BaseColor"), NodeColor);
								MeshComponent->SetMaterial(0, DynMaterial);
							}

							UE_LOG(LogTemp, Log, TEXT("Successfully spawned entity mesh at %s with scale %.2f"),
								*EntitySpawnPosition.ToString(), MeshScale);
						}
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Failed to load UnitSphere mesh"));
					}

					// Store the spawned actor as AActor (upcast)
					SpawnedEntities.Add(InNode, TWeakObjectPtr<AActor>(MeshActor));
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("Failed to spawn static mesh actor"));
				}
			}
		});
}

void AStarSystemActor::ReturnEntityToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	UE_LOG(LogTemp, Log, TEXT("=== Star System Entity Despawn Request ==="));
	UE_LOG(LogTemp, Log, TEXT("Object Type: %d"), static_cast<int32>(InNode->Data.TypeId));

	// Find and destroy the spawned actor
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
	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::ResetForPool took: %.3f seconds"), Duration);
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
	//if(ELifecycleState::Ready == this->InitializationState)
	//UE_LOG(LogTemp, Warning, TEXT("Distance to Player: %f"), DistanceToPlayer);
	//UE_LOG(LogTemp, Warning, TEXT("UnitScale: %f"), UnitScale);
	//UE_LOG(LogTemp, Warning, TEXT("ParallaxRatio: %f"), ParallaxRatio);
	//UE_LOG(LogTemp, Warning, TEXT("PlayerOffset: %s"), *PlayerOffset.ToString());
	//UE_LOG(LogTemp, Warning, TEXT("ParallaxOffset: %s"), *ParallaxOffset.ToString());
}

void AStarSystemActor::Tick(float DeltaTime)
{
	ApplyParallaxOffset();
	if(IsDebug) DrawDebugBounds();
}
#pragma endregion
#pragma region Debug
void AStarSystemActor::DrawDebugBounds()
{
	// Draw debug box for the octree root node
	if (Octree.IsValid() && InitializationState == ELifecycleState::Ready)
	{
		if (UWorld* World = GetWorld())
		{
			FVector ActorLocation = GetActorLocation();

			// Convert octree extent to world scale
			// WorldExtent = OctreeExtent * UnitScale
			double WorldExtent = Octree->Extent;

			FVector BoxExtent(WorldExtent, WorldExtent, WorldExtent);

			// Draw the box centered at the actor location
			DrawDebugBox(
				World,
				ActorLocation,
				BoxExtent,
				FColor::Green,
				false,
				-1.0f,
				0,
				WorldExtent * 0.005f  // Thickness relative to world extent
			);

			// Draw coordinate axes at the center
			//DrawDebugCoordinateSystem(
			//	World,
			//	ActorLocation,
			//	FRotator::ZeroRotator,
			//	WorldExtent * 0.1f,
			//	false,
			//	-1.0f,
			//	0,
			//	WorldExtent * 0.001f
			//);
		}
	}
}
#pragma endregion
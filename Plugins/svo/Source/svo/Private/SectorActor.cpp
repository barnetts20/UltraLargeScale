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
}

void ASectorActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	// --- Phase 1: Gas density from noise (low-res, alpha channel only) ---
	int noiseResolution = 128;
	auto DensityNoise = FastNoise::NewFromEncodedNodeTree(Params.EncodedTree);
	TArray<uint8> LowResData = FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		noiseResolution,
		Params.Extent,
		Octree,
		FMath::FloorLog2(noiseResolution),
		1.0f,
		3      // Alpha only (gas density)
	);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Upscale noise to 256^3 ---
	TArray<uint8> VolumeData = FVolumeTextureUtils::UpscaleVolumeData(LowResData, noiseResolution);
	LowResData.Empty();

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Phase 2: Object generation (VBOs) ---
	UniverseGenerator.Params = Params;
	UniverseGenerator.GenerateData(Octree);
	TArray<TSharedPtr<FOctreeNode>> VolumeChunks;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(UniverseGenerator.GeneratedData, PointNodes, VolumeChunks);

	// --- Phase 2b: Splat VBO density into R channel + octree at depth 8 ---
	FVolumeTextureUtils::SplatVBOsToVolume(
		VolumeData,
		256,
		Params.Extent,
		PointNodes,
		FVector(6, 6, 6),
		FVector(12, 12, 12),
		2.0f,
		4.0f,
		0.5f,
		1.5f,
		2,              // R channel
		Octree,
		8
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

	// --- Pack and create texture (already at 256^3, no second upscale) ---
	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(
		FVolumeTextureUtils::PackToPseudoVolumeLayout(VolumeData),
		"/svo/Generated/BakedTest"
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
	double StartTime = FPlatformTime::Seconds();

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
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
			NiagaraComponent->SetWorldLocation(PlayerPos);

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
				{
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

	// Initialize proximity system after VBO Niagara is set up
	InitializeProximitySystem();

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

	FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;

	ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
	FVector ParallaxOffset = -PlayerDelta * ParallaxRatio;

	NiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);

	// Apply parallax to proximity system too
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		ProximityNiagaraComponent->SetWorldLocation(CurrentPlayerPos);
	}

	SetActorLocation(GetActorLocation() + ParallaxOffset);
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
	FRandomStream RandStream(InNode->Data.ObjectId);
	Galaxy->Params.Rotation = RandStream.GetUnitVector().Rotation();
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

					PoolGalaxy->Octree->bIsResetting.store(true);
					FPlatformProcess::Sleep(0.05f);
					PoolGalaxy->Octree = MakeShared<FOctree>(PoolGalaxy->Params.Extent);
					PoolGalaxy->Octree->bIsResetting.store(false);

					double ODuration = FPlatformTime::Seconds() - StartTime;
					UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), ODuration);

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

#pragma region Proximity Galaxy Streaming

void ASectorActor::InitializeProximitySystem()
{
	double StartTime = FPlatformTime::Seconds();

	const int32 TotalSlots = 27;
	const int32 TotalParticles = TotalSlots * MaxParticlesPerNode;

	// --- Allocate flat buffers ---
	ProximityPositions.SetNumZeroed(TotalParticles);
	ProximityExtents.SetNumZeroed(TotalParticles);
	ProximityColors.SetNumZeroed(TotalParticles);
	SlotParticleCounts.SetNumZeroed(TotalSlots);

	// --- Initialize free slot pool ---
	FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		FreeSlots.Add(i);
	}
	ActiveNodeSlots.Empty();
	CurrentScanCoord = FIntVector(INT32_MIN);

	// --- Get player position ---
	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	// --- Determine initial scan coord and populate all 27 nodes ---
	FVector LocalPos = PlayerPos - GetActorLocation();
	CurrentScanCoord = PositionToScanCoord(LocalPos);

	int32 NodesPerSide = 1 << ScanDepth;
	for (int32 dz = -1; dz <= 1; ++dz)
	{
		for (int32 dy = -1; dy <= 1; ++dy)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				FIntVector NeighborCoord = CurrentScanCoord + FIntVector(dx, dy, dz);

				if (NeighborCoord.X < 0 || NeighborCoord.X >= NodesPerSide ||
					NeighborCoord.Y < 0 || NeighborCoord.Y >= NodesPerSide ||
					NeighborCoord.Z < 0 || NeighborCoord.Z >= NodesPerSide)
				{
					continue;
				}

				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(NeighborCoord, SlotIndex);
				GenerateNodeGalaxies(NeighborCoord, SlotIndex);
			}
		}
	}

	// --- Create proximity Niagara and push data (all on game thread) ---
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			ProximityNiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
				PointCloudNiagara,
				GetRootComponent(),
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				true,
				false
			);

			if (ProximityNiagaraComponent)
			{
				ProximityNiagaraComponent->SetSystemFixedBounds(
					FBox(FVector(-Params.Extent), FVector(Params.Extent)));
				ProximityNiagaraComponent->SetWorldLocation(PlayerPos);

				PushProximityToNiagara();
				ProximityNiagaraComponent->Activate(true);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeProximitySystem - Failed to create proximity Niagara component"));
			}

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeProximitySystem took %.3f sec (%d nodes, %d total particles)"),
		Duration, ActiveNodeSlots.Num(), TotalParticles);
}

void ASectorActor::UpdateProximityNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!ProximityNiagaraComponent) return;

	// --- Get player position in octree-local space ---
	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	FVector LocalPos = PlayerPos - GetActorLocation();
	FIntVector NewScanCoord = PositionToScanCoord(LocalPos);

	if (NewScanCoord == CurrentScanCoord) return;

	double StartTime = FPlatformTime::Seconds();

	int32 NodesPerSide = 1 << ScanDepth;
	TSet<FIntVector> OldSet;
	TSet<FIntVector> NewSet;

	for (int32 dz = -1; dz <= 1; ++dz)
	{
		for (int32 dy = -1; dy <= 1; ++dy)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				FIntVector OldNeighbor = CurrentScanCoord + FIntVector(dx, dy, dz);
				FIntVector NewNeighbor = NewScanCoord + FIntVector(dx, dy, dz);

				if (OldNeighbor.X >= 0 && OldNeighbor.X < NodesPerSide &&
					OldNeighbor.Y >= 0 && OldNeighbor.Y < NodesPerSide &&
					OldNeighbor.Z >= 0 && OldNeighbor.Z < NodesPerSide)
				{
					OldSet.Add(OldNeighbor);
				}

				if (NewNeighbor.X >= 0 && NewNeighbor.X < NodesPerSide &&
					NewNeighbor.Y >= 0 && NewNeighbor.Y < NodesPerSide &&
					NewNeighbor.Z >= 0 && NewNeighbor.Z < NodesPerSide)
				{
					NewSet.Add(NewNeighbor);
				}
			}
		}
	}

	TArray<FIntVector> ExitingNodes;
	TArray<FIntVector> EnteringNodes;

	for (const FIntVector& Coord : OldSet)
	{
		if (!NewSet.Contains(Coord))
		{
			ExitingNodes.Add(Coord);
		}
	}

	for (const FIntVector& Coord : NewSet)
	{
		if (!OldSet.Contains(Coord))
		{
			EnteringNodes.Add(Coord);
		}
	}

	for (const FIntVector& Coord : ExitingNodes)
	{
		int32* SlotPtr = ActiveNodeSlots.Find(Coord);
		if (SlotPtr)
		{
			FreeSlots.Add(*SlotPtr);
			ActiveNodeSlots.Remove(Coord);
		}
	}

	for (const FIntVector& Coord : EnteringNodes)
	{
		if (FreeSlots.Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateProximityNodes - No free slots available!"));
			break;
		}

		int32 SlotIndex = FreeSlots.Pop();
		ActiveNodeSlots.Add(Coord, SlotIndex);
		GenerateNodeGalaxies(Coord, SlotIndex);
	}

	CurrentScanCoord = NewScanCoord;

	// Push on game thread (Tick is already game thread, but be explicit)
	PushProximityToNiagara();
	ProximityNiagaraComponent->ReinitializeSystem();
	double Duration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateProximityNodes crossing (%d exiting, %d entering) took %.3f sec"),
		ExitingNodes.Num(), EnteringNodes.Num(), Duration);
}

FIntVector ASectorActor::PositionToScanCoord(const FVector& InLocalPos) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	int32 X = FMath::Clamp(FMath::FloorToInt((InLocalPos.X + Params.Extent) / NodeSize), 0, NodesPerSide - 1);
	int32 Y = FMath::Clamp(FMath::FloorToInt((InLocalPos.Y + Params.Extent) / NodeSize), 0, NodesPerSide - 1);
	int32 Z = FMath::Clamp(FMath::FloorToInt((InLocalPos.Z + Params.Extent) / NodeSize), 0, NodesPerSide - 1);

	return FIntVector(X, Y, Z);
}

FVector ASectorActor::ScanCoordToCenter(const FIntVector& InCoord) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	return FVector(
		-Params.Extent + (InCoord.X + 0.5) * NodeSize,
		-Params.Extent + (InCoord.Y + 0.5) * NodeSize,
		-Params.Extent + (InCoord.Z + 0.5) * NodeSize
	);
}

double ASectorActor::GetScanNodeExtent() const
{
	return Params.Extent / (double)(1 << ScanDepth);
}

void ASectorActor::GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex)
{
	const int32 BufferStart = InSlotIndex * MaxParticlesPerNode;
	const FVector NodeCenter = ScanCoordToCenter(InNodeCoord);
	const double NodeExtent = GetScanNodeExtent();

	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InNodeCoord.X), GetTypeHash(InNodeCoord.Y)),
		GetTypeHash(InNodeCoord.Z)
	);
	int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	int32 ActualCount = 0;
	int32 MaxAttempts = MaxParticlesPerNode * RejectionOversampleFactor;

	for (int32 i = 0; i < MaxAttempts; ++i)
	{
		if (ActualCount >= MaxParticlesPerNode) break;

		FVector Candidate(
			Stream.FRandRange(-NodeExtent, NodeExtent),
			Stream.FRandRange(-NodeExtent, NodeExtent),
			Stream.FRandRange(-NodeExtent, NodeExtent)
		);
		Candidate += NodeCenter;

		if (FMath::Abs(Candidate.X) > Params.Extent ||
			FMath::Abs(Candidate.Y) > Params.Extent ||
			FMath::Abs(Candidate.Z) > Params.Extent)
		{
			continue;
		}

		float Density = Octree->SampleDensityAtPosition(Candidate);
		if (Stream.FRand() > Density) continue;
		FVector CompVec = Stream.GetUnitVector();

		// Galaxy scale from distribution curve
		float ScaleSample = Stream.FRand();
		double DepthDivisor = 1000000.0; // 2^5
		double Scale = FPointData::SampleScaleFromDistribution(
			Params.MinGalaxyScale / DepthDivisor,  // 3e23 / 32 ≈ 9.4e21
			Params.MaxGalaxyScale / DepthDivisor,  // 3e26 / 32 ≈ 9.4e24
			ScaleSample, Params.ScaleDistributionCurve);

		// Convert to octree-local extent using same logic as VBOs
		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		double NodeExtentAtDepth = Params.Extent / (double)(1 << PointData.InsertDepth);
		float FinalExtent = static_cast<float>(NodeExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		// Write into slot
		int32 Idx = BufferStart + ActualCount;
		ProximityPositions[Idx] = Candidate;
		ProximityExtents[Idx] = FinalExtent;
		ProximityColors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		if (ActualCount < 3)
		{
			UE_LOG(LogTemp, Log, TEXT("  Galaxy %d: Scale=%.2e, LocalSize=%.2e, depth=%d, SF=%.4f, extentAtDepth=%.0f, finalExtent=%.0f"),
				ActualCount, Scale, Scale / Params.UnitScale, PointData.InsertDepth, PointData.Data.ScaleFactor, NodeExtentAtDepth, FinalExtent);
		}

		ActualCount++;
	}

	for (int32 i = ActualCount; i < MaxParticlesPerNode; ++i)
	{
		int32 Idx = BufferStart + i;
		ProximityPositions[Idx] = FVector::ZeroVector;
		ProximityExtents[Idx] = 0.0f;
		ProximityColors[Idx] = FLinearColor::Black;
	}

	SlotParticleCounts[InSlotIndex] = ActualCount;
	UE_LOG(LogTemp, Log, TEXT("GenerateNodeGalaxies node (%d,%d,%d) slot %d: %d galaxies from %d attempts"),
		InNodeCoord.X, InNodeCoord.Y, InNodeCoord.Z, InSlotIndex, ActualCount, MaxAttempts);
}

void ASectorActor::PushProximityToNiagara()
{
	if (!ProximityNiagaraComponent) return;

	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	FVector ActorPos = GetActorLocation();

	TArray<FVector> RelativePositions;
	RelativePositions.SetNumUninitialized(ProximityPositions.Num());

	ParallelFor(ProximityPositions.Num(), [&](int32 i) {
		if (ProximityExtents[i] > 0.0f)
		{
			RelativePositions[i] = (ProximityPositions[i] + ActorPos) - PlayerPos;
		}
		else
		{
			RelativePositions[i] = FVector::ZeroVector;
		}
		}, EParallelForFlags::BackgroundPriority);

	int32 ActiveCount = 0;
	for (float E : ProximityExtents) { if (E > 0.0f) ActiveCount++; }
	UE_LOG(LogTemp, Log, TEXT("PushProximityToNiagara: %d active particles, NiagaraValid=%d"),
		ActiveCount, ProximityNiagaraComponent != nullptr);

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
		ProximityNiagaraComponent, FName("User.Positions"), RelativePositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
		ProximityNiagaraComponent, FName("User.Extents"), ProximityExtents);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
		ProximityNiagaraComponent, FName("User.Colors"), ProximityColors);

	ProximityNiagaraComponent->SetWorldLocation(PlayerPos);
}

void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateProximityNodes();
}

#pragma endregion
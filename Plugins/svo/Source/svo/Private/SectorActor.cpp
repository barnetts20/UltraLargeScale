#pragma region Includes/ForwardDec
#include "SectorActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <NiagaraFunctionLibrary.h>
#include <DrawDebugHelpers.h>
#include <TimerManager.h>
#pragma endregion

#pragma region Constructor
ASectorActor::ASectorActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));

	SectorGalaxyCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_SectorParallaxCloud.NG_SectorParallaxCloud"));
	SectorClusterCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorClusterCloud.NG_SectorClusterCloud"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGasCloud.NG_SectorGasCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();

	// Corner-align the tree so depth-2 cells line up with sector coarse cells.
	// Center = (SE, SE, SE), extent = 4*SE → depth-2 grid centers at
	// {-2*SE, 0, +2*SE, +4*SE} along each axis. The first three match
	// coarse cell coords {-1, 0, +1}; the fourth is unused buffer.
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}
#pragma endregion

#pragma region Lifecycle
void ASectorActor::Initialize()
{
	InitializationState = ELifecycleState::Initializing;

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

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			double StartTime = FPlatformTime::Seconds();

			InitializeChildPool();
			if (InitializationState == ELifecycleState::Pooling) return;

			InitializeData();
			if (InitializationState == ELifecycleState::Pooling) return;

			InitializeVolumetric();
			if (InitializationState == ELifecycleState::Pooling) return;

			InitializeNiagara();
			if (InitializationState == ELifecycleState::Pooling) return;

			InitializationState = ELifecycleState::Ready;

			double TotalDuration = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTemp, Log, TEXT("%s::Initialize total duration: %.3f seconds"),
				*GetClass()->GetName(), TotalDuration);

			// TimerManager must be touched on the game thread.
			AsyncTask(ENamedThreads::GameThread, [this]()
				{
					if (IsValid(this))
					{
						StartSpawnScanTimer();
					}
				});
		});
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

	if (bAutoInitializeOnBeginPlay)
	{
		Initialize();
	}
}

void ASectorActor::ConfigureCell(FIntVector InCellCoord)
{
	CellCoord = InCellCoord;
	CellOrigin = FVector(
		static_cast<double>(CellCoord.X) * 2.0 * Params.Extent,
		static_cast<double>(CellCoord.Y) * 2.0 * Params.Extent,
		static_cast<double>(CellCoord.Z) * 2.0 * Params.Extent);
	SetActorLocation(CellOrigin);

	// Rebuild the octree against the actual extent in case Params were
	// overridden after construction (see constructor comment for sizing).
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
}

FastNoise::SmartNode<> ASectorActor::BuildNoise(int InSeed) {
	auto Voronoi = FastNoise::New<FastNoise::CellularDistance>();
	Voronoi->SetDistanceFunction(FastNoise::DistanceFunction::EuclideanSquared);
	Voronoi->SetReturnType(FastNoise::CellularDistance::ReturnType::Index0);
	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Voronoi);
	SeedOffset->SetOffset(InSeed);
	auto DomainScale = FastNoise::New<FastNoise::DomainScale>();
	DomainScale->SetSource(SeedOffset);
	DomainScale->SetScale(Params.NoiseParams.MasterScale);
	auto Fbm0 = FastNoise::New<FastNoise::FractalFBm>();
	Fbm0->SetSource(DomainScale);
	Fbm0->SetOctaveCount(3);
	auto Remap0 = FastNoise::New<FastNoise::Remap>();
	Remap0->SetSource(Fbm0);
	Remap0->SetRemap(0, 1, Params.NoiseParams.ClusterRemapMax, Params.NoiseParams.ClusterRemapMin);
	auto Pow0 = FastNoise::New<FastNoise::PowInt>();
	Pow0->SetValue(Remap0);
	Pow0->SetPow(Params.NoiseParams.ClusterFalloff);
	auto Scale0 = FastNoise::New<FastNoise::DomainScale>();
	Scale0->SetSource(Pow0);
	Scale0->SetScale(Params.NoiseParams.ClusterScale);
	auto Pow1 = FastNoise::New<FastNoise::PowInt>();
	Pow1->SetValue(Fbm0);
	Pow1->SetPow(Params.NoiseParams.WebFalloff);
	auto Mul0 = FastNoise::New<FastNoise::Multiply>();
	Mul0->SetLHS(Scale0);
	Mul0->SetRHS(Pow1);
	auto Mul1 = FastNoise::New<FastNoise::Multiply>();
	Mul1->SetLHS(Mul0);
	Mul1->SetRHS(Params.NoiseParams.ClusterMulti);
	auto Remap1 = FastNoise::New<FastNoise::Remap>();
	Remap1->SetSource(Pow1);
	Remap1->SetRemap(0, 1, Params.NoiseParams.WebRemapMin, Params.NoiseParams.WebRemapMax);
	auto Add0 = FastNoise::New<FastNoise::Add>();
	Add0->SetLHS(Remap1);
	Add0->SetRHS(Mul1);
	auto Warp0 = FastNoise::New<FastNoise::DomainWarpGradient>();
	Warp0->SetSource(Add0);
	Warp0->SetWarpAmplitude(Params.NoiseParams.WarpAmp);
	Warp0->SetWarpFrequency(Params.NoiseParams.WarpFreq);
	return Warp0;
}

void ASectorActor::InitializeData()
{
	double TotalStart = FPlatformTime::Seconds();
	double StepStart;

	// With the coarse and proximity streaming tiers both sampling noise
	// directly per-candidate, nothing in the particle pipeline reads the
	// sector-wide DensityVolume. The only consumer is the debug volumetric
	// raymarcher, which samples the PseudoVolumeTexture thousands of times
	// per ray. When bEnableVolumetric is false (the default), skip the entire
	// noise-to-volume pipeline — it was a ~120ms unconditional cost per init.
	if (!bEnableVolumetric)
	{
		UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeData - volumetric disabled, skipping density volume build (%.3f sec)"),
			FPlatformTime::Seconds() - TotalStart);
		return;
	}

	StepStart = FPlatformTime::Seconds();
	int noiseResolution = 128;

	auto DensityNoise = BuildNoise(69);

	// Cell-coord WorldOffset for the noise sampler. Each cell extends
	// 2 units in normalized noise-space, so adjacent cells offset by
	// (2 * CellCoord) to produce a continuous noise field.
	const FVector NoiseOffset(
		static_cast<double>(CellCoord.X) * 2.0,
		static_cast<double>(CellCoord.Y) * 2.0,
		static_cast<double>(CellCoord.Z) * 2.0);

	TArray<uint8> LowResData = FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		noiseResolution,
		Params.Extent,
		nullptr,
		-1,
		1.0f,
		3,        // alpha only (gas density)
		NoiseOffset
	);
	UE_LOG(LogTemp, Log, TEXT("  [InitData] Noise sampling (%d^3): %.3f sec"), noiseResolution, FPlatformTime::Seconds() - StepStart);
	if (InitializationState == ELifecycleState::Pooling) return;

	if (noiseResolution == 256) {
		DensityBuffer = MoveTemp(LowResData);
	}
	else {
		StepStart = FPlatformTime::Seconds();
		DensityBuffer = FVolumeTextureUtils::UpscaleVolumeData(LowResData, noiseResolution);
		LowResData.Empty();
		UE_LOG(LogTemp, Log, TEXT("  [InitData] Upscale (%d^3 -> 256^3): %.3f sec"), noiseResolution, FPlatformTime::Seconds() - StepStart);
	}

	DensityVolume = FDensityVolume(
		DensityBuffer,
		FVector::ZeroVector,
		FVector(Params.Extent, Params.Extent, Params.Extent),
		256);

	if (InitializationState == ELifecycleState::Pooling) return;

	StepStart = FPlatformTime::Seconds();
	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(FVolumeTextureUtils::PackToPseudoVolumeLayout(DensityBuffer));
	UE_LOG(LogTemp, Log, TEXT("  [InitData] CreatePseudoVolumeTexture: %.3f sec"), FPlatformTime::Seconds() - StepStart);

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeData total: %.3f sec"), FPlatformTime::Seconds() - TotalStart);
}

void ASectorActor::InitializeVolumetric()
{
	if (!bEnableVolumetric)
	{
		UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeVolumetric skipped (bEnableVolumetric=false)"));
		return;
	}

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

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::Volumetric initialization took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void ASectorActor::InitializeNiagara()
{
	// Dispatches to the two streaming subsystem initializers. Both own their
	// own Niagara components, double-buffered particle state, and push/activate
	// sequence. Ordering between them is not significant.
	double StartTime = FPlatformTime::Seconds();

	InitializeProximitySystem();
	InitializeCoarseSystem();

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Player-Centered Parallax
// VirtualTraversal-based pegged-actor parallax model:
//   1. Actor is pegged to the player every tick (SetActorLocation).
//   2. VirtualTraversal accumulates Ratio * PlayerDelta each tick, encoding
//      how far the player has "virtually" moved through sector-grid space.
//   3. Per-frame scratch pad broadcast User.ParallaxOffset = -Ratio *
//      PlayerDelta drifts stored particle positions to stay aligned.
//   4. Push math: Relative = LocalPos - VirtualTraversal.
//   5. Streaming: boundary-cross checks use VirtualTraversal directly.
void ASectorActor::ApplyParallaxOffset()
{
	if (InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	FVector CurrentPlayerPos = FVector::ZeroVector;
	bool bHasReference = false;
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

	const FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;

	const double Ratio = (Params.UnitScale > 0.0) ? (SpeedScale / Params.UnitScale) : 0.0;

	VirtualTraversal += PlayerDelta * Ratio;

	// Scratch-pad broadcast. Stored particle positions satisfy
	// stored = LocalPos - VirtualTraversal at every tick. Each frame
	// VirtualTraversal grows by Ratio * PlayerDelta, so stored needs to
	// shrink by the same amount.
	const FVector ParallaxOffset = -PlayerDelta * Ratio;
	if (CoarseClusterNiagara)
	{
		CoarseClusterNiagara->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
	}
	if (CoarseGasNiagara)
	{
		CoarseGasNiagara->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
	}
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
	}

	// Peg the actor. Components follow via attachment.
	SetActorLocation(CurrentPlayerPos);
}
#pragma endregion

#pragma region Shutdown
void ASectorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopSpawnScanTimer();

	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->Deactivate();
		ProximityNiagaraComponent->DestroyComponent();
		ProximityNiagaraComponent = nullptr;
	}

	if (CoarseClusterNiagara)
	{
		CoarseClusterNiagara->Deactivate();
		CoarseClusterNiagara->DestroyComponent();
		CoarseClusterNiagara = nullptr;
	}

	if (CoarseGasNiagara)
	{
		CoarseGasNiagara->Deactivate();
		CoarseGasNiagara->DestroyComponent();
		CoarseGasNiagara = nullptr;
	}

	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Child Spawn Location
FVector ASectorActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
	const double ThisUnitScale = Params.UnitScale;
	const double Ratio = (ChildUnitScale > 0.0) ? (ThisUnitScale / ChildUnitScale) : 1.0;
	const FVector CellLocalOffset = NodeCenter - CellOrigin;
	return CurrentFrameOfReferenceLocation + CellLocalOffset * Ratio;
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

					UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);

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

#pragma region Coarse Cluster Streaming

// Coarse cells are 2*Params.Extent on a side, centered at integer-coord multiples.
// Cell (N, M, K) occupies [(N-0.5)*2*Extent .. (N+0.5)*2*Extent]. Coord (0,0,0)
// is centered at the sector's CellOrigin.
FIntVector ASectorActor::PositionToCoarseCoord(const FVector& InWorldPos) const
{
	const double CellSize = 2.0 * Params.Extent;
	return FIntVector(
		FMath::FloorToInt32(InWorldPos.X / CellSize + 0.5),
		FMath::FloorToInt32(InWorldPos.Y / CellSize + 0.5),
		FMath::FloorToInt32(InWorldPos.Z / CellSize + 0.5));
}

FVector ASectorActor::CoarseCoordToCenter(const FIntVector& InCoord) const
{
	const double CellSize = 2.0 * Params.Extent;
	return FVector(
		static_cast<double>(InCoord.X) * CellSize,
		static_cast<double>(InCoord.Y) * CellSize,
		static_cast<double>(InCoord.Z) * CellSize);
}

void ASectorActor::InitializeCoarseSystem()
{
	double StartTime = FPlatformTime::Seconds();

	// Slot pool sized to 3x3x3 = 27 at radius 1. Wider radii multiply.
	const int32 Side = 2 * CoarseNeighborhoodRadius + 1;
	const int32 TotalSlots = Side * Side * Side;

	CoarseClusterBuffers[0].Allocate(TotalSlots, MaxClusterPerCoarseNode, /*bWantRotations=*/true);
	CoarseClusterBuffers[1].Allocate(TotalSlots, MaxClusterPerCoarseNode, /*bWantRotations=*/true);
	CoarseGasBuffers[0].Allocate(TotalSlots, MaxClusterPerCoarseNode);
	CoarseGasBuffers[1].Allocate(TotalSlots, MaxClusterPerCoarseNode);
	CoarseFrontIdx.store(0);

	CoarseSlotCounts.SetNumZeroed(TotalSlots);
	CoarseFreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		CoarseFreeSlots.Add(i);
	}
	ActiveCoarseNodes.Empty();

	// VirtualTraversal is 0 at init — player starts at virtual coord (0,0,0).
	const FVector LocalPlayerPos = VirtualTraversal;
	CoarseCenterCell = PositionToCoarseCoord(LocalPlayerPos);

	// Two-phase pipeline:
	//   (1) Serial slot allocation — CoarseFreeSlots / ActiveCoarseNodes aren't threadsafe.
	//   (2) Parallel generation — each worker writes to a disjoint slot-indexed slice.
	//   (3) Serial octree insert — InsertPosition mutex-guards internally.
	TArray<TPair<FIntVector, int32>> ToGenerate;
	ToGenerate.Reserve(TotalSlots);
	for (int32 dz = -CoarseNeighborhoodRadius; dz <= CoarseNeighborhoodRadius; ++dz)
	{
		for (int32 dy = -CoarseNeighborhoodRadius; dy <= CoarseNeighborhoodRadius; ++dy)
		{
			for (int32 dx = -CoarseNeighborhoodRadius; dx <= CoarseNeighborhoodRadius; ++dx)
			{
				const FIntVector NeighborCoord = CoarseCenterCell + FIntVector(dx, dy, dz);
				const int32 SlotIndex = CoarseFreeSlots.Pop();
				ActiveCoarseNodes.Add(NeighborCoord, FCoarseSlotEntry{ SlotIndex, {} });
				ToGenerate.Emplace(NeighborCoord, SlotIndex);
			}
		}
	}

	ParallelFor(ToGenerate.Num(), [this, &ToGenerate](int32 i)
		{
			const TPair<FIntVector, int32>& Pair = ToGenerate[i];
			GenerateCoarseNode(Pair.Key, Pair.Value, CoarseClusterBuffers[0], CoarseGasBuffers[0]);
		}, EParallelForFlags::BackgroundPriority);

	for (const TPair<FIntVector, int32>& Pair : ToGenerate)
	{
		FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Pair.Key);
		if (Entry)
		{
			InsertCoarseCellIntoOctree(Pair.Key, Pair.Value, CoarseClusterBuffers[0], Entry->InsertedNodes);
		}
	}

	// Mirror front → back so either buffer is a valid starting state.
	CoarseClusterBuffers[1].CopyFrom(CoarseClusterBuffers[0]);
	CoarseGasBuffers[1].CopyFrom(CoarseGasBuffers[0]);

	// Spawn Niagara components on game thread, push real data, activate.
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

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			UNiagaraSystem* ClusterTemplate = SectorClusterCloud ? SectorClusterCloud : SectorGalaxyCloud;
			UNiagaraSystem* GasTemplate = SectorGasCloud ? SectorGasCloud : SectorGalaxyCloud;

			if (!SectorClusterCloud)
			{
				UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeCoarseSystem - SectorClusterCloud not assigned; falling back to SectorGalaxyCloud."));
			}
			if (!SectorGasCloud)
			{
				UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeCoarseSystem - SectorGasCloud not assigned; falling back to SectorGalaxyCloud."));
			}

			CoarseClusterNiagara = UNiagaraFunctionLibrary::SpawnSystemAttached(
				ClusterTemplate,
				GetRootComponent(),
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				/*bAutoDestroy=*/ false,
				/*bAutoActivate=*/ true);

			CoarseGasNiagara = UNiagaraFunctionLibrary::SpawnSystemAttached(
				GasTemplate,
				GetRootComponent(),
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				/*bAutoDestroy=*/ false,
				/*bAutoActivate=*/ true);

			// Fixed bounds cover the full active neighborhood.
			const double BoundsExtent = (2 * CoarseNeighborhoodRadius + 1) * Params.Extent;
			const FBox CoarseBounds(FVector(-BoundsExtent), FVector(BoundsExtent));

			if (CoarseClusterNiagara)
			{
				CoarseClusterNiagara->SetSystemFixedBounds(CoarseBounds);
				CoarseClusterNiagara->TranslucencySortPriority = 0;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeCoarseSystem - Failed to create CoarseClusterNiagara"));
			}

			if (CoarseGasNiagara)
			{
				CoarseGasNiagara->SetSystemFixedBounds(CoarseBounds);
				CoarseGasNiagara->TranslucencySortPriority = 0;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeCoarseSystem - Failed to create CoarseGasNiagara"));
			}

			PushCoarseToNiagara();
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeCoarseSystem took %.3f sec (%d slots, %d max particles/slot, center %d,%d,%d)"),
		FPlatformTime::Seconds() - StartTime, TotalSlots, MaxClusterPerCoarseNode,
		CoarseCenterCell.X, CoarseCenterCell.Y, CoarseCenterCell.Z);
}

void ASectorActor::UpdateCoarseNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!CoarseClusterNiagara && !CoarseGasNiagara) return;

	// If async generation completed a swap, push the new front buffer.
	// ReinitializeSystem() forces Niagara to re-spawn with the new user arrays.
	if (bCoarseNeedsPush.load())
	{
		PushCoarseToNiagara();
		if (CoarseClusterNiagara) CoarseClusterNiagara->ReinitializeSystem();
		if (CoarseGasNiagara) CoarseGasNiagara->ReinitializeSystem();
		bCoarseNeedsPush.store(false);
	}

	if (bCoarseUpdateInProgress.load()) return;

	const FVector LocalPlayerPos = VirtualTraversal;
	const FIntVector NewCoarseCoord = PositionToCoarseCoord(LocalPlayerPos);

	if (NewCoarseCoord == CoarseCenterCell) return;

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateCoarseNodes - boundary cross: (%d,%d,%d) -> (%d,%d,%d)"),
		CoarseCenterCell.X, CoarseCenterCell.Y, CoarseCenterCell.Z,
		NewCoarseCoord.X, NewCoarseCoord.Y, NewCoarseCoord.Z);

	bCoarseUpdateInProgress.store(true);
	const FIntVector OldCenter = CoarseCenterCell;
	CoarseCenterCell = NewCoarseCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, OldCenter, NewCoarseCoord]()
		{
			double StartTime = FPlatformTime::Seconds();

			// Copy front → back; unchanged slots carry over untouched.
			const int32 FrontIdx = CoarseFrontIdx.load();
			const int32 BackIdx = 1 - FrontIdx;
			CoarseClusterBuffers[BackIdx].CopyFrom(CoarseClusterBuffers[FrontIdx]);
			CoarseGasBuffers[BackIdx].CopyFrom(CoarseGasBuffers[FrontIdx]);

			// Coarse coords are universe-wide integer space with no bounds.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			for (int32 dz = -CoarseNeighborhoodRadius; dz <= CoarseNeighborhoodRadius; ++dz)
			{
				for (int32 dy = -CoarseNeighborhoodRadius; dy <= CoarseNeighborhoodRadius; ++dy)
				{
					for (int32 dx = -CoarseNeighborhoodRadius; dx <= CoarseNeighborhoodRadius; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						if (OldCenter.X != INT32_MIN)
						{
							OldSet.Add(OldCenter + Offset);
						}
						NewSet.Add(NewCoarseCoord + Offset);
					}
				}
			}

			TArray<FIntVector> ExitingNodes;
			TArray<FIntVector> EnteringNodes;
			for (const FIntVector& Coord : OldSet)
			{
				if (!NewSet.Contains(Coord)) ExitingNodes.Add(Coord);
			}
			for (const FIntVector& Coord : NewSet)
			{
				if (!OldSet.Contains(Coord)) EnteringNodes.Add(Coord);
			}

			// Free exiting slots: dead-stub their data, retire from octree, return slot.
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Coord);
				if (Entry)
				{
					CoarseClusterBuffers[BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
					CoarseGasBuffers[BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
					CoarseFreeSlots.Add(Entry->SlotIndex);

					for (const TSharedPtr<FOctreeNode>& Node : Entry->InsertedNodes)
					{
						if (Octree.IsValid())
						{
							Octree->RemoveObjectIdFromNode(Node, Entry->SlotIndex);
						}
					}
					ActiveCoarseNodes.Remove(Coord);
				}
			}

			// Generate entering nodes:
			//   (1) Serial slot allocation.
			//   (2) Parallel generation into back buffer slices.
			//   (3) Serial octree insert.
			TArray<TPair<FIntVector, int32>> ToGenerate;
			ToGenerate.Reserve(EnteringNodes.Num());
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (CoarseFreeSlots.Num() == 0)
				{
					UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateCoarseNodes - no free slots; dropping cell (%d,%d,%d)"),
						Coord.X, Coord.Y, Coord.Z);
					continue;
				}
				const int32 SlotIndex = CoarseFreeSlots.Pop();
				ActiveCoarseNodes.Add(Coord, FCoarseSlotEntry{ SlotIndex, {} });
				ToGenerate.Emplace(Coord, SlotIndex);
			}

			ParallelFor(ToGenerate.Num(), [this, &ToGenerate, BackIdx](int32 i)
				{
					const TPair<FIntVector, int32>& Pair = ToGenerate[i];
					GenerateCoarseNode(Pair.Key, Pair.Value, CoarseClusterBuffers[BackIdx], CoarseGasBuffers[BackIdx]);
				}, EParallelForFlags::BackgroundPriority);

			for (const TPair<FIntVector, int32>& Pair : ToGenerate)
			{
				FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Pair.Key);
				if (Entry)
				{
					InsertCoarseCellIntoOctree(Pair.Key, Pair.Value, CoarseClusterBuffers[BackIdx], Entry->InsertedNodes);
				}
			}

			CoarseFrontIdx.store(BackIdx);
			bCoarseNeedsPush.store(true);
			bCoarseUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateCoarseNodes - %d entering, %d exiting in %.3f sec"),
				EnteringNodes.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

void ASectorActor::GenerateCoarseNode(const FIntVector& InCoarseCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InClusterBuffer, FNiagaraParticleBuffer& InGasBuffer)
{
	// Batched noise sampling — three phases:
	//   1. Generate candidate positions + normalized noise coords.
	//   2. One GenPositionArray3D call covering all candidates.
	//   3. Walk results, rejection-gate, write accepted to slot buffers.
	//
	// Cluster and gas share 1:1 positions and slot indexing; accepted points
	// write a cluster entry and a gas entry at the same slot index.
	// Gas extent is lerped by the same density value used for rejection.

	const int32 BufferStart = InSlotIndex * InClusterBuffer.SlotCapacity;
	const FVector NodeCenter = CoarseCoordToCenter(InCoarseCoord);

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoarseCoord.X), GetTypeHash(InCoarseCoord.Y)),
		GetTypeHash(InCoarseCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const float ExtentRange = GasMaxExtent - GasMinExtent;
	const int32 NumCandidates = MaxClusterPerCoarseNode;
	const double InvExtent = 1.0 / (double)Params.Extent;

	// Every candidate in this cell shares the same coord-derived noise offset.
	const double NoiseOffsetX = (double)InCoarseCoord.X * 2.0;
	const double NoiseOffsetY = (double)InCoarseCoord.Y * 2.0;
	const double NoiseOffsetZ = (double)InCoarseCoord.Z * 2.0;

	// --- Phase 1: generate candidates + normalized noise coords ---
	TArray<FVector> CandidatePositions;
	TArray<float> NoiseX, NoiseY, NoiseZ;
	CandidatePositions.SetNumUninitialized(NumCandidates);
	NoiseX.SetNumUninitialized(NumCandidates);
	NoiseY.SetNumUninitialized(NumCandidates);
	NoiseZ.SetNumUninitialized(NumCandidates);

	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const FVector Candidate(
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent));
		CandidatePositions[i] = Candidate;

		NoiseX[i] = (float)(Candidate.X * InvExtent + NoiseOffsetX);
		NoiseY[i] = (float)(Candidate.Y * InvExtent + NoiseOffsetY);
		NoiseZ[i] = (float)(Candidate.Z * InvExtent + NoiseOffsetZ);
	}

	// --- Phase 2: batch noise evaluation ---
	auto DensityNoise = BuildNoise(69);
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	DensityNoise->GenPositionArray3D(
		NoiseOut.GetData(),
		NumCandidates,
		NoiseX.GetData(),
		NoiseY.GetData(),
		NoiseZ.GetData(),
		0.0f, 0.0f, 0.0f,
		Params.Seed);

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// --- Phase 3: accept/reject + write to slot ---
	int32 ActualCount = 0;
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float Density = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.MinClusterScale,
			Params.MaxClusterScale,
			ScaleSample, Params.ScaleDistributionCurve);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		const float ClusterExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		const FVector CompVec = Stream.GetUnitVector();
		const FVector Rotation = Stream.GetUnitVector();
		const float GasExtent = GasMinExtent + ExtentRange * Density;
		const FVector LocalPos = CandidatePositions[i] + NodeCenter;

		const int32 Idx = BufferStart + ActualCount;
		InClusterBuffer.Positions[Idx] = LocalPos;
		InClusterBuffer.Rotations[Idx] = Rotation;
		InClusterBuffer.Extents[Idx] = ClusterExtent;
		InClusterBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		InGasBuffer.Positions[Idx] = LocalPos;
		InGasBuffer.Extents[Idx] = GasExtent;
		InGasBuffer.Colors[Idx] = FLinearColor(1.0f, 1.0f, 1.0f, Density);

		ActualCount++;
	}

	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	InClusterBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);
	InGasBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);

	CoarseSlotCounts[InSlotIndex] = ActualCount;
}

void ASectorActor::InsertCoarseCellIntoOctree(const FIntVector& InCoarseCoord, int32 InSlotIndex, const FNiagaraParticleBuffer& InClusterBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const
{
	// Walk this cell's cluster slot; insert each live particle into the octree.
	// Only cluster particles are inserted — gas shares 1:1 positions and slot
	// indexing, so a cluster insert spatially represents both.
	// ObjectId carries the slot index; TypeId tags nodes as galaxy content.
	if (!Octree.IsValid())
	{
		return;
	}

	const double TreeExtent = Octree->Extent;
	const int32 BufferStart = InSlotIndex * InClusterBuffer.SlotCapacity;
	OutInsertedNodes.Reserve(InClusterBuffer.SlotCapacity);

	for (int32 i = 0; i < InClusterBuffer.SlotCapacity; ++i)
	{
		const int32 Idx = BufferStart + i;
		const float Extent = InClusterBuffer.Extents[Idx];
		if (Extent <= 0.0f)
		{
			continue;
		}

		FPointData PointData = FPointData::MakePointDataFromWorldScale(
			static_cast<double>(Extent),
			/*InUnitScale=*/ 1.0,
			static_cast<int64>(TreeExtent));
		PointData.SetPosition(InClusterBuffer.Positions[Idx]);
		PointData.Data.ObjectId = InSlotIndex;
		PointData.Data.TypeId = GalaxyTypeId;

		TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
			PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
		if (Node.IsValid())
		{
			OutInsertedNodes.Add(Node);
		}
	}
}

void ASectorActor::PushCoarseToNiagara()
{
	const int32 FrontIdx = CoarseFrontIdx.load();
	CoarseClusterBuffers[FrontIdx].PushToNiagara(CoarseClusterNiagara, VirtualTraversal);
	CoarseGasBuffers[FrontIdx].PushToNiagara(CoarseGasNiagara, VirtualTraversal);
}

#pragma endregion

#pragma region Proximity Galaxy Streaming

void ASectorActor::InitializeProximitySystem()
{
	double StartTime = FPlatformTime::Seconds();

	const int32 TotalSlots = 27;

	ProximityBuffers[0].Allocate(TotalSlots, MaxParticlesPerNode);
	ProximityBuffers[1].Allocate(TotalSlots, MaxParticlesPerNode);
	FrontBufferIndex.store(0);

	SlotParticleCounts.SetNumZeroed(TotalSlots);
	FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		FreeSlots.Add(i);
	}
	ActiveNodeSlots.Empty();
	CurrentScanCoord = FIntVector(INT32_MIN);

	// VirtualTraversal is 0 at init — player starts at virtual coord (0,0,0).
	const FVector LocalPos = VirtualTraversal;
	CurrentScanCoord = PositionToScanCoord(LocalPos);

	// Two-phase pipeline: serial slot allocation → parallel generate → serial octree insert.
	TArray<TPair<FIntVector, int32>> ToGenerate;
	ToGenerate.Reserve(27);
	for (int32 dz = -1; dz <= 1; ++dz)
	{
		for (int32 dy = -1; dy <= 1; ++dy)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				FIntVector NeighborCoord = CurrentScanCoord + FIntVector(dx, dy, dz);
				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(NeighborCoord, FProximitySlotEntry{ SlotIndex, {} });
				ToGenerate.Emplace(NeighborCoord, SlotIndex);
			}
		}
	}

	ParallelFor(ToGenerate.Num(), [this, &ToGenerate](int32 i)
		{
			const TPair<FIntVector, int32>& Pair = ToGenerate[i];
			GenerateNodeGalaxies(Pair.Key, Pair.Value, ProximityBuffers[0]);
		}, EParallelForFlags::BackgroundPriority);

	for (const TPair<FIntVector, int32>& Pair : ToGenerate)
	{
		FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Pair.Key);
		if (Entry)
		{
			InsertProximityCellIntoOctree(Pair.Key, Pair.Value, ProximityBuffers[0], Entry->InsertedNodes);
		}
	}

	// Mirror front → back so either buffer is a valid starting state.
	ProximityBuffers[1].CopyFrom(ProximityBuffers[0]);

	// Create proximity Niagara component and push data on game thread.
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			if (!SectorGalaxyCloud)
			{
				UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeProximitySystem - SectorGalaxyCloud not assigned."));
			}

			ProximityNiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
				SectorGalaxyCloud,
				GetRootComponent(),
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				false,
				true
			);

			if (ProximityNiagaraComponent)
			{
				ProximityNiagaraComponent->SetSystemFixedBounds(
					FBox(FVector(-Params.Extent), FVector(Params.Extent)));

				PushProximityToNiagara();
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeProximitySystem - Failed to create proximity Niagara component"));
			}

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeProximitySystem took %.3f sec (%d slots, %d max particles/slot)"),
		FPlatformTime::Seconds() - StartTime, TotalSlots, MaxParticlesPerNode);
}

void ASectorActor::UpdateProximityNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!ProximityNiagaraComponent) return;

	if (bProximityNeedsPush.load())
	{
		PushProximityToNiagara();
		ProximityNiagaraComponent->ReinitializeSystem();
		bProximityNeedsPush.store(false);
	}

	if (bProximityUpdateInProgress.load()) return;

	const FVector LocalPos = VirtualTraversal;
	FIntVector NewScanCoord = PositionToScanCoord(LocalPos);

	if (NewScanCoord == CurrentScanCoord) return;

	bProximityUpdateInProgress.store(true);
	FIntVector OldScanCoord = CurrentScanCoord;
	CurrentScanCoord = NewScanCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, OldScanCoord, NewScanCoord]()
		{
			double StartTime = FPlatformTime::Seconds();

			// Copy front → back; unchanged slots carry over.
			const int32 FrontIdx = FrontBufferIndex.load();
			const int32 BackIdx = 1 - FrontIdx;
			ProximityBuffers[BackIdx].CopyFrom(ProximityBuffers[FrontIdx]);

			// Scan coords are universe-wide — no NodesPerSide bounds culling.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			for (int32 dz = -1; dz <= 1; ++dz)
			{
				for (int32 dy = -1; dy <= 1; ++dy)
				{
					for (int32 dx = -1; dx <= 1; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						if (OldScanCoord.X != INT32_MIN)
						{
							OldSet.Add(OldScanCoord + Offset);
						}
						NewSet.Add(NewScanCoord + Offset);
					}
				}
			}

			TArray<FIntVector> ExitingNodes;
			TArray<FIntVector> EnteringNodes;
			for (const FIntVector& Coord : OldSet)
			{
				if (!NewSet.Contains(Coord)) ExitingNodes.Add(Coord);
			}
			for (const FIntVector& Coord : NewSet)
			{
				if (!OldSet.Contains(Coord)) EnteringNodes.Add(Coord);
			}

			// Free exiting slots: dead-stub their data, retire from octree, return slot.
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Coord);
				if (Entry)
				{
					ProximityBuffers[BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
					FreeSlots.Add(Entry->SlotIndex);

					for (const TSharedPtr<FOctreeNode>& Node : Entry->InsertedNodes)
					{
						if (Octree.IsValid())
						{
							Octree->RemoveObjectIdFromNode(Node, Entry->SlotIndex);
						}
					}
					ActiveNodeSlots.Remove(Coord);
				}
			}

			// Generate entering nodes:
			//   (1) Serial slot allocation.
			//   (2) Parallel generation into back buffer slices.
			//   (3) Serial octree insert.
			TArray<TPair<FIntVector, int32>> ToGenerate;
			ToGenerate.Reserve(EnteringNodes.Num());
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (FreeSlots.Num() == 0)
				{
					break;
				}
				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(Coord, FProximitySlotEntry{ SlotIndex, {} });
				ToGenerate.Emplace(Coord, SlotIndex);
			}

			ParallelFor(ToGenerate.Num(), [this, &ToGenerate, BackIdx](int32 i)
				{
					const TPair<FIntVector, int32>& Pair = ToGenerate[i];
					GenerateNodeGalaxies(Pair.Key, Pair.Value, ProximityBuffers[BackIdx]);
				}, EParallelForFlags::BackgroundPriority);

			for (const TPair<FIntVector, int32>& Pair : ToGenerate)
			{
				FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Pair.Key);
				if (Entry)
				{
					InsertProximityCellIntoOctree(Pair.Key, Pair.Value, ProximityBuffers[BackIdx], Entry->InsertedNodes);
				}
			}

			FrontBufferIndex.store(BackIdx);
			bProximityNeedsPush.store(true);
			bProximityUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateProximityNodes - %d entering, %d exiting in %.3f sec"),
				EnteringNodes.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

FIntVector ASectorActor::PositionToScanCoord(const FVector& InLocalPos) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	// Scan coords are a universe-wide integer lattice — NOT clamped to [0, NodesPerSide).
	// GenerateNodeGalaxies builds a coord-derived transient density field so there
	// is no sector-wide volume to fall off the edge of.
	const int32 X = FMath::FloorToInt((InLocalPos.X + Params.Extent) / NodeSize);
	const int32 Y = FMath::FloorToInt((InLocalPos.Y + Params.Extent) / NodeSize);
	const int32 Z = FMath::FloorToInt((InLocalPos.Z + Params.Extent) / NodeSize);

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

void ASectorActor::GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InBuffer)
{
	// Batched noise sampling — three phases matching GenerateCoarseNode.
	// Candidates are scan-node-local rather than cell-local; each candidate
	// may straddle coarse cell boundaries so noise offset is computed
	// per-candidate rather than shared across the node.

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;
	const FVector NodeCenter = ScanCoordToCenter(InNodeCoord);
	const double NodeExt = GetScanNodeExtent();

	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InNodeCoord.X), GetTypeHash(InNodeCoord.Y)),
		GetTypeHash(InNodeCoord.Z)
	);
	int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = MaxParticlesPerNode;
	const double InvExtent = 1.0 / (double)Params.Extent;
	const double TwoExtent = 2.0 * (double)Params.Extent;

	// --- Phase 1: generate candidates + normalized noise coords ---
	TArray<FVector> CandidatePositions;
	TArray<float> NoiseX, NoiseY, NoiseZ;
	CandidatePositions.SetNumUninitialized(NumCandidates);
	NoiseX.SetNumUninitialized(NumCandidates);
	NoiseY.SetNumUninitialized(NumCandidates);
	NoiseZ.SetNumUninitialized(NumCandidates);

	for (int32 i = 0; i < NumCandidates; ++i)
	{
		FVector Candidate(
			Stream.FRandRange(-NodeExt, NodeExt),
			Stream.FRandRange(-NodeExt, NodeExt),
			Stream.FRandRange(-NodeExt, NodeExt)
		);
		Candidate += NodeCenter;
		CandidatePositions[i] = Candidate;

		// Determine which coarse cell this candidate falls into and compute
		// its cell-local normalized coord + coord-derived offset.
		const int32 CX = FMath::FloorToInt32(Candidate.X / TwoExtent + 0.5);
		const int32 CY = FMath::FloorToInt32(Candidate.Y / TwoExtent + 0.5);
		const int32 CZ = FMath::FloorToInt32(Candidate.Z / TwoExtent + 0.5);

		const double CenterX = (double)CX * TwoExtent;
		const double CenterY = (double)CY * TwoExtent;
		const double CenterZ = (double)CZ * TwoExtent;

		NoiseX[i] = (float)((Candidate.X - CenterX) * InvExtent + (double)CX * 2.0);
		NoiseY[i] = (float)((Candidate.Y - CenterY) * InvExtent + (double)CY * 2.0);
		NoiseZ[i] = (float)((Candidate.Z - CenterZ) * InvExtent + (double)CZ * 2.0);
	}

	// --- Phase 2: batch noise evaluation ---
	auto DensityNoise = BuildNoise(69);
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	DensityNoise->GenPositionArray3D(
		NoiseOut.GetData(),
		NumCandidates,
		NoiseX.GetData(),
		NoiseY.GetData(),
		NoiseZ.GetData(),
		0.0f, 0.0f, 0.0f,
		Params.Seed
	);

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// --- Phase 3: accept/reject + write to slot ---
	int32 ActualCount = 0;
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float Density = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		FVector CompVec = Stream.GetUnitVector();

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.MinGalaxyScale,
			Params.MaxGalaxyScale,
			ScaleSample, Params.ScaleDistributionCurve);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		const float FinalExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Extents[Idx] = FinalExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	InBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);

	SlotParticleCounts[InSlotIndex] = ActualCount;
}

void ASectorActor::InsertProximityCellIntoOctree(const FIntVector& InNodeCoord, int32 InSlotIndex, const FNiagaraParticleBuffer& InBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const
{
	if (!Octree.IsValid())
	{
		return;
	}

	const double TreeExtent = Octree->Extent;
	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;
	OutInsertedNodes.Reserve(InBuffer.SlotCapacity);

	for (int32 i = 0; i < InBuffer.SlotCapacity; ++i)
	{
		const int32 Idx = BufferStart + i;
		const float Extent = InBuffer.Extents[Idx];
		if (Extent <= 0.0f)
		{
			continue;
		}

		FPointData PointData = FPointData::MakePointDataFromWorldScale(
			static_cast<double>(Extent),
			/*InUnitScale=*/ 1.0,
			static_cast<int64>(TreeExtent));
		PointData.SetPosition(InBuffer.Positions[Idx]);
		PointData.Data.ObjectId = InSlotIndex;
		PointData.Data.TypeId = GalaxyTypeId;

		TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
			PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
		if (Node.IsValid())
		{
			OutInsertedNodes.Add(Node);
		}
	}
}

void ASectorActor::PushProximityToNiagara()
{
	ProximityBuffers[FrontBufferIndex.load()].PushToNiagara(ProximityNiagaraComponent, VirtualTraversal);
}

#pragma endregion

#pragma region Tick
void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	ApplyParallaxOffset();
	UpdateCoarseNodes();
	UpdateProximityNodes();
}
#pragma endregion

#pragma region Public Octree Queries
TArray<TSharedPtr<FOctreeNode>> ASectorActor::GetNodesInRange(const FVector& InCenter, double InRadius, int32 InTypeId) const
{
	if (!Octree.IsValid())
	{
		return {};
	}
	return Octree->GetNodesInRange(InCenter, InRadius, /*MinDepth=*/ -1, /*MaxDepth=*/ -1, InTypeId);
}

TArray<TSharedPtr<FOctreeNode>> ASectorActor::GetNodesByScreenSpace(const FVector& InCenter, double InExtent, double InScreenSpaceThreshold, int32 InTypeId) const
{
	if (!Octree.IsValid())
	{
		return {};
	}
	return Octree->GetNodesByScreenSpace(InCenter, InExtent, InScreenSpaceThreshold,
		/*MinDepth=*/ -1, /*MaxDepth=*/ -1, InTypeId);
}
#pragma endregion

#pragma region Spawn Range Scanning
void ASectorActor::StartSpawnScanTimer()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(SpawnScanTimerHandle);
		World->GetTimerManager().SetTimer(
			SpawnScanTimerHandle,
			this,
			&ASectorActor::UpdateSpawnRangeNodes,
			SpawnScanInterval,
			/*bLoop=*/ true);
	}
}

void ASectorActor::StopSpawnScanTimer()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(SpawnScanTimerHandle);
	}
	TrackedSpawnNodes.Empty();
}

void ASectorActor::UpdateSpawnRangeNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;

	const FVector LocalPlayerPos = VirtualTraversal;

	const TArray<TSharedPtr<FOctreeNode>> NearbyArray =
		GetNodesByScreenSpace(LocalPlayerPos, SpawnScanExtent, SpawnScreenSpaceThreshold, GalaxyTypeId);

	TSet<TSharedPtr<FOctreeNode>> NearbySet(NearbyArray);

	for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
	{
		if (!TrackedSpawnNodes.Contains(Node))
		{
			LogSpawnNodeEnter(Node);
		}
		if (bDebugDrawSpawnNodes)
		{
			DebugDrawSpawnNode(Node);
		}
	}

	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedSpawnNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
	}

	TrackedSpawnNodes = MoveTemp(NearbySet);
}

void ASectorActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;

	const int32 SlotId = InNode->Data.ObjectId;
	const int32 ExtraCount = InNode->Data.AdditionalObjectIds.Num();

	UE_LOG(LogTemp, Log,
		TEXT("ASectorActor::SpawnScan ENTER — node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d slot=%d extras=%d scale=%.3f"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Depth, SlotId, ExtraCount, InNode->Data.ScaleFactor);

	if (bLogSpawnEnterExitBuffers)
	{
		// Identify tier by checking whether the slot index appears in ActiveCoarseNodes.
		bool bIsCoarse = false;
		for (const auto& Pair : ActiveCoarseNodes)
		{
			if (Pair.Value.SlotIndex == SlotId)
			{
				bIsCoarse = true;
				break;
			}
		}

		if (bIsCoarse)
		{
			const FNiagaraParticleBuffer& Front = CoarseClusterBuffers[CoarseFrontIdx.load()];
			const int32 Start = SlotId * MaxClusterPerCoarseNode;
			int32 LiveCount = 0;
			for (int32 i = 0; i < MaxClusterPerCoarseNode; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log,
				TEXT("  coarse slot %d: %d live cluster particles of %d capacity"),
				SlotId, LiveCount, MaxClusterPerCoarseNode);
		}
		else
		{
			const FNiagaraParticleBuffer& Front = ProximityBuffers[FrontBufferIndex.load()];
			const int32 Start = SlotId * MaxParticlesPerNode;
			int32 LiveCount = 0;
			for (int32 i = 0; i < MaxParticlesPerNode; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log,
				TEXT("  proximity slot %d: %d live particles of %d capacity"),
				SlotId, LiveCount, MaxParticlesPerNode);
		}
	}
}

void ASectorActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;

	UE_LOG(LogTemp, Log,
		TEXT("ASectorActor::SpawnScan EXIT  — node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d slot=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Depth, InNode->Data.ObjectId);
}

void ASectorActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;

	// Particle rendered world position = PlayerPos + LocalPos - VirtualTraversal.
	// Node->Center is the octree's quantization of LocalPos, so:
	//   NodeCenterWorld = GetActorLocation() + Node->Center - VirtualTraversal
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	const FVector BoxExtent(InNode->Extent);

	DrawDebugBox(
		World,
		NodeCenterWorld,
		BoxExtent,
		FColor::Green,
		/*bPersistent=*/ false,
		/*Lifetime=*/ SpawnScanInterval,
		/*DepthPriority=*/ 0,
		/*Thickness=*/ 10.0f);
}
#pragma endregion
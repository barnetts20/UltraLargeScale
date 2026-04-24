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
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));

	SectorGalaxyCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_SectorParallaxCloud.NG_SectorParallaxCloud"));
	SectorClusterCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorClusterCloud.NG_SectorClusterCloud"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGasCloud.NG_SectorGasCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	// Corner-align the tree so depth-2 cells line up with sector coarse
	// cells. Center = (SE, SE, SE), extent = 4*SE → depth-2 grid centers
	// at {-2*SE, 0, +2*SE, +4*SE} along each axis. The first three match
	// coarse cell coords {-1, 0, +1}; the fourth is unused buffer.
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}
#pragma endregion

#pragma region Lifecycle
// Copied locally from the former AProceduralSpaceActor base. Same async-init
// pattern: InitializeChildPool → InitializeData → InitializeVolumetric →
// InitializeNiagara, checking InitializationState == Pooling between each step
// so a ResetForPool mid-init short-circuits cleanly.
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

	// Skip auto-Initialize when something else (e.g. AUniverseActor) is
	// driving the lifecycle and needs to ConfigureCell() first. Manually-
	// placed sectors in a level still init themselves on play.
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

	// The sector's constructor created an Octree with default Params.Extent.
	// If the universe has overridden Params (via SpawnSectorForCell setting
	// Sector->Params before ConfigureCell), the octree may now be sized
	// wrong. Rebuild against the actual extent (sized 4× and corner-aligned
	// — see constructor comment).
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
}

FastNoise::SmartNode<> ASectorActor::BuildNoise(int InSeed) {
	//Param staging TODO: Wrap in noise param struct
	float masterScale = 1;
	float clusterFalloff = 32;
	float clusterScale = 3;
	float clusterMulti = 50;
	float clusterRemapMax = 1.001;
	float clusterRemapMin = 0;

	float webFalloff = 3;
	float webRemapMin = -.1;
	float webRemapMax = 1.000;

	float warpAmp = .25;
	float warpFreq = 1;
	//end param

	auto Voronoi = FastNoise::New<FastNoise::CellularDistance>();
	Voronoi->SetDistanceFunction(FastNoise::DistanceFunction::EuclideanSquared);
	Voronoi->SetReturnType(FastNoise::CellularDistance::ReturnType::Index0);

	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Voronoi);
	SeedOffset->SetOffset(InSeed);

	auto DomainScale = FastNoise::New<FastNoise::DomainScale>();
	DomainScale->SetSource(SeedOffset);
	DomainScale->SetScale(masterScale);

	auto Fbm0 = FastNoise::New<FastNoise::FractalFBm>();
	Fbm0->SetSource(DomainScale);
	Fbm0->SetOctaveCount(3); //Octave count will probs stay hardcoded

	auto Remap0 = FastNoise::New<FastNoise::Remap>();
	Remap0->SetSource(Fbm0);
	Remap0->SetRemap(0, 1, clusterRemapMax, clusterRemapMin);

	auto Pow0 = FastNoise::New<FastNoise::PowInt>();
	Pow0->SetValue(Remap0);
	Pow0->SetPow(clusterFalloff);

	auto Scale0 = FastNoise::New<FastNoise::DomainScale>();
	Scale0->SetSource(Pow0);
	Scale0->SetScale(clusterScale);

	auto Pow1 = FastNoise::New<FastNoise::PowInt>();
	Pow1->SetValue(Fbm0);
	Pow1->SetPow(webFalloff);

	auto Mul0 = FastNoise::New<FastNoise::Multiply>();
	Mul0->SetLHS(Scale0);
	Mul0->SetRHS(Pow1);

	auto Mul1 = FastNoise::New<FastNoise::Multiply>();
	Mul1->SetLHS(Mul0);
	Mul1->SetRHS(clusterMulti);

	auto Remap1 = FastNoise::New<FastNoise::Remap>();
	Remap1->SetSource(Pow1);
	Remap1->SetRemap(0, 1, webRemapMin, webRemapMax);

	auto Add0 = FastNoise::New<FastNoise::Add>();
	Add0->SetLHS(Remap1);
	Add0->SetRHS(Mul1);

	auto Warp0 = FastNoise::New<FastNoise::DomainWarpGradient>();
	Warp0->SetSource(Add0);
	Warp0->SetWarpAmplitude(warpAmp);
	Warp0->SetWarpFrequency(warpFreq);

	return Warp0;
}

void ASectorActor::InitializeData()
{
	double TotalStart = FPlatformTime::Seconds();
	double StepStart;

	// With the coarse streaming tier and fine-tier proximity system both now
	// sampling noise directly per-candidate (GenerateCoarseNode +
	// GenerateNodeGalaxies use GenPositionArray3D batches), nothing in the
	// particle/data pipeline reads the sector-wide DensityVolume anymore.
	// The only consumer is the debug volumetric raymarcher, which samples the
	// PseudoVolumeTexture thousands of times per ray and genuinely needs the
	// GPU-sampled volume texture form. When bEnableVolumetric is false (the
	// default), skip the entire noise-to-volume + upscale + pseudo-volume
	// pipeline — it was a ~120ms unconditional cost on every sector init.
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
	// 2 units in normalized noise-space (because a cell's local sampling
	// covers [-1, +1] after normalization), so adjacent cells offset by
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

	// Wrap DensityBuffer in the sampler view. Sector noise spans
	// [-Params.Extent, +Params.Extent] centered at sector local origin, so the
	// view's source-space matches sector local space directly.
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
	// Per-sector raymarched volumetric is too expensive to run on a 27-sector
	// grid. Default disabled; only directly-placed sectors with the flag
	// flipped on (e.g. for volumetric debugging) actually spawn the cube.
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

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void ASectorActor::InitializeNiagara()
{
	// With FSectorNiagaraLayerData / LayerComponents gone, InitializeNiagara
	// is now just a dispatcher for the two streaming subsystems. Both own
	// their own Niagara components, their own double-buffered particle state,
	// and their own initial-population + spawn-component + push + activate
	// sequence. Ordering between them is not significant — neither reads the
	// other's state.
	double StartTime = FPlatformTime::Seconds();

	InitializeProximitySystem();
	InitializeCoarseSystem();

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Player-Centered Parallax
// Parallax model (restored from pre-refactor working build):
//   1. Single ratio per actor: Ratio = SpeedScale / Params.UnitScale.
//   2. Per-frame ParallaxOffset = -PlayerDelta * Ratio.
//   3. Actor drifts by ParallaxOffset, so GetActorLocation() is a stable
//      slow-drifting reference frame the streaming pipeline can query
//      for coord-space boundary crossings.
//   4. Niagara components SetWorldLocation(PlayerPos) each tick so they
//      sit at the camera; User.ParallaxOffset is broadcast to their
//      scratch pad as the per-frame delta.
//   5. Push math: Relative = (Local + ActorPos) - PlayerPos. Component
//      at PlayerPos renders this as (Local + ActorPos) in world space —
//      a world-pinned slow-drifting position.
//
// Task 1's per-tier ratio model (CoarseUnitScale / FineUnitScale with
// scratch-pad-only parallax + actor pegged to player) broke streaming
// because non-center cells rendered at player-relative positions that
// drifted off-screen instead of staying world-pinned. Multi-tier
// differentiation is deferred — see design doc.
void ASectorActor::ApplyParallaxOffset()
{
	// Mirrors the original AProceduralSpaceActor::ApplyParallaxOffset /
	// ASectorActor::ApplyParallaxOffset pattern: single parallax ratio,
	// actor drifts by -PlayerDelta*Ratio, Niagara components pinned to
	// player via SetWorldLocation, scratch pad also broadcast the same
	// ParallaxOffset. This restores the exact rendering semantics of the
	// working pre-refactor build. Per-tier ratio differentiation is out
	// of scope for Task 1 — see design doc for the deferred multi-ratio
	// plan.
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

	// Single-ratio parallax (same as pre-refactor). Ratio = SpeedScale /
	// Params.UnitScale; ParallaxOffset is per-frame delta applied to both
	// the actor (as slow-drift) and every Niagara component's scratch
	// pad (as per-particle delta).
	const double Ratio = (Params.UnitScale > 0.0) ? (SpeedScale / Params.UnitScale) : 0.0;
	const FVector ParallaxOffset = -PlayerDelta * Ratio;

	if (CoarseClusterNiagara)
	{
		CoarseClusterNiagara->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		CoarseClusterNiagara->SetWorldLocation(CurrentPlayerPos);
	}
	if (CoarseGasNiagara)
	{
		CoarseGasNiagara->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		CoarseGasNiagara->SetWorldLocation(CurrentPlayerPos);
	}
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		ProximityNiagaraComponent->SetWorldLocation(CurrentPlayerPos);
	}

	// Actor drifts by the same per-frame offset. This is the mechanism
	// that makes (PlayerPos - GetActorLocation()) a stable slow-drift
	// reference for coord-space tracking in UpdateCoarseNodes /
	// UpdateProximityNodes, and makes (LocalPos + ActorPos - PlayerPos)
	// produce correct world-pinned render positions in the Push* calls.
	SetActorLocation(GetActorLocation() + ParallaxOffset);
}
#pragma endregion

#pragma region Shutdown
void ASectorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Explicitly tear down managed Niagara components before the engine's GC
	// pass runs. Without this, the stale-reference detector at PIE end can
	// observe partially-destroyed components still referenced through our
	// UPROPERTY chain and the printer itself can break on the walk.
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
	// In the player-pegged frame, NodeCenter is in sector-local coords (offset
	// from CellOrigin), not world-space. Recover the cell-local offset and
	// scale by the tier unit-scale ratio so nearer-tier children (smaller
	// ChildUnitScale) sit proportionally closer to the player.
	//
	// With ThisUnitScale == ChildUnitScale this collapses to
	// CurrentFrameOfReferenceLocation + (NodeCenter - CellOrigin), which
	// matches the "treat sector-local as player-relative" convention used by
	// the Niagara push path.
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

#pragma region Coarse Cluster Streaming

// Coordinate helpers. Coarse cells are 2*Params.Extent on a side, centered at
// integer-coord multiples — i.e. cell (N, M, K) occupies world space
// [(N-0.5) * 2*Extent .. (N+0.5) * 2*Extent]. Coord (0,0,0) is centered at
// the sector's CellOrigin in the sector-local frame. Same convention the old
// AUniverseActor used for its sector grid, just now the grid lives inside a
// single sector actor.
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
	const int32 TotalParticles = TotalSlots * MaxClusterPerCoarseNode;

	CoarseBuffers[0].Allocate(TotalParticles);
	CoarseBuffers[1].Allocate(TotalParticles);
	CoarseFrontIdx.store(0);

	CoarseSlotCounts.SetNumZeroed(TotalSlots);
	CoarseFreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		CoarseFreeSlots.Add(i);
	}
	ActiveCoarseNodes.Empty();

	// --- Determine initial coarse coord from player position and populate
	// all neighborhood cells into the front buffer synchronously. Matches
	// the pattern in InitializeProximitySystem — having real data in the
	// buffer before the component spawns means the first push has something
	// to draw, rather than dead particles until the first async regen lands.
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

	// Coarse coords are sector-actor-local. With ApplyParallaxOffset
	// restored to drift the actor by -PlayerDelta*Ratio each tick,
	// GetActorLocation() is again the slow-drifting reference frame the
	// streaming pipeline was built around. PlayerPos - GetActorLocation()
	// is the player's position in that frame.
	const FVector LocalPlayerPos = PlayerPos - GetActorLocation();
	CoarseCenterCell = PositionToCoarseCoord(LocalPlayerPos);

	// Two-phase pipeline (Task 2 — registry model):
	//   (1) Serial slot allocation. CoarseFreeSlots / ActiveCoarseNodes
	//       aren't threadsafe; pop slots and build the (Coord, Slot) work
	//       list under single-threaded control.
	//   (2) Parallel generation. Each worker writes to a disjoint slot-
	//       indexed slice of InitBuffer — no contention.
	//   (3) Per-cell octree insert. After generation completes, walk each
	//       cell's particle slice and bulk-insert each non-dead particle
	//       at a depth derived from its extent. Record touched nodes per
	//       cell entry for cell-exit cleanup.
	FCoarseBuffer& InitBuffer = CoarseBuffers[0];
	TArray<TPair<FIntVector, int32>> ToGenerate;
	ToGenerate.Reserve(Side * Side * Side);
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

	ParallelFor(ToGenerate.Num(), [this, &ToGenerate, &InitBuffer](int32 i)
		{
			const TPair<FIntVector, int32>& Pair = ToGenerate[i];
			GenerateCoarseNode(Pair.Key, Pair.Value, InitBuffer);
		}, EParallelForFlags::BackgroundPriority);

	// Octree registry pass. InsertCoarseCellIntoOctree calls FOctree::
	// InsertPosition per particle, which mutex-guards. Running the inserts
	// in parallel would serialize on that mutex anyway and add scheduler
	// overhead, so do them serially in slot-iteration order.
	for (const TPair<FIntVector, int32>& Pair : ToGenerate)
	{
		FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Pair.Key);
		if (Entry)
		{
			InsertCoarseCellIntoOctree(Pair.Key, Pair.Value, InitBuffer, Entry->InsertedNodes);
		}
	}

	// Mirror front → back so either buffer is a valid starting state for
	// the first boundary-cross update.
	CoarseBuffers[1].ClusterPositions = CoarseBuffers[0].ClusterPositions;
	CoarseBuffers[1].ClusterRotations = CoarseBuffers[0].ClusterRotations;
	CoarseBuffers[1].ClusterExtents = CoarseBuffers[0].ClusterExtents;
	CoarseBuffers[1].ClusterColors = CoarseBuffers[0].ClusterColors;
	CoarseBuffers[1].GasPositions = CoarseBuffers[0].GasPositions;
	CoarseBuffers[1].GasExtents = CoarseBuffers[0].GasExtents;
	CoarseBuffers[1].GasColors = CoarseBuffers[0].GasColors;

	// --- Spawn Niagara components on game thread, push real data, activate.
	// Same pattern as InitializeProximitySystem.
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

			// Fixed bounds cover the full active neighborhood, not just one
			// cell. Component sits at the actor (pegged to player); particles
			// reach CoarseNeighborhoodRadius cells away in every direction.
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

			// Push the initial (real) data into both components, then activate.
			// No SetWorldLocation here — the components are attached to the
			// actor which ApplyParallaxOffset will peg to the player.
			PushCoarseToNiagara();

			if (CoarseClusterNiagara) CoarseClusterNiagara->Activate(true);
			if (CoarseGasNiagara) CoarseGasNiagara->Activate(true);

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
	// ReinitializeSystem() forces Niagara to re-spawn with the new user
	// arrays — without it, already-live particles hold onto whatever
	// positions/extents they were spawned with, so the buffer swap updates
	// the component's user parameters but no visible change occurs. Same
	// pattern as UpdateProximityNodes.
	if (bCoarseNeedsPush.load())
	{
		PushCoarseToNiagara();
		if (CoarseClusterNiagara) CoarseClusterNiagara->ReinitializeSystem();
		if (CoarseGasNiagara) CoarseGasNiagara->ReinitializeSystem();
		bCoarseNeedsPush.store(false);
	}

	// Don't start a new update if one is in flight.
	if (bCoarseUpdateInProgress.load()) return;

	// Player pos → coarse coord, in sector-actor-local space. With
	// ApplyParallaxOffset restored to drift the actor by -PlayerDelta*Ratio,
	// GetActorLocation() is the slow-drifting reference the streaming
	// pipeline was built around.
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
	const FVector LocalPlayerPos = PlayerPos - GetActorLocation();
	const FIntVector NewCoarseCoord = PositionToCoarseCoord(LocalPlayerPos);

	if (NewCoarseCoord == CoarseCenterCell) return;

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateCoarseNodes - boundary cross: (%d,%d,%d) -> (%d,%d,%d)"),
		CoarseCenterCell.X, CoarseCenterCell.Y, CoarseCenterCell.Z,
		NewCoarseCoord.X, NewCoarseCoord.Y, NewCoarseCoord.Z);

	// Boundary crossed — async regen of entering slots into back buffer.
	bCoarseUpdateInProgress.store(true);
	const FIntVector OldCenter = CoarseCenterCell;
	CoarseCenterCell = NewCoarseCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, OldCenter, NewCoarseCoord]()
		{
			double StartTime = FPlatformTime::Seconds();

			// Copy front → back as the starting state; unchanged slots carry
			// over untouched, only entering/exiting slots get modified.
			const int32 FrontIdx = CoarseFrontIdx.load();
			const int32 BackIdx = 1 - FrontIdx;
			FCoarseBuffer& BackBuffer = CoarseBuffers[BackIdx];
			const FCoarseBuffer& FrontBuffer = CoarseBuffers[FrontIdx];

			BackBuffer.ClusterPositions = FrontBuffer.ClusterPositions;
			BackBuffer.ClusterRotations = FrontBuffer.ClusterRotations;
			BackBuffer.ClusterExtents = FrontBuffer.ClusterExtents;
			BackBuffer.ClusterColors = FrontBuffer.ClusterColors;
			BackBuffer.GasPositions = FrontBuffer.GasPositions;
			BackBuffer.GasExtents = FrontBuffer.GasExtents;
			BackBuffer.GasColors = FrontBuffer.GasColors;

			// Entering / exiting coord sets. Unlike the proximity system
			// (which clamps to [0, NodesPerSide) inside the sector), coarse
			// coords are universe-wide integer space with no bounds — the
			// sliding window can be anywhere.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			for (int32 dz = -CoarseNeighborhoodRadius; dz <= CoarseNeighborhoodRadius; ++dz)
			{
				for (int32 dy = -CoarseNeighborhoodRadius; dy <= CoarseNeighborhoodRadius; ++dy)
				{
					for (int32 dx = -CoarseNeighborhoodRadius; dx <= CoarseNeighborhoodRadius; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						// Only register OldSet cells if the OldCenter was a
						// real center (not the INT32_MIN cold-start sentinel).
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

			// Free exiting slots: dead-stub the cell's particles in the back
			// buffer, return the slot to the free pool, and walk every octree
			// node this cell touched to retire its slot index. Other cells
			// may have inserted into the same node (collision overflow), so
			// we use RemoveObjectIdFromNode rather than blanket-clearing.
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Coord);
				if (Entry)
				{
					const int32 SlotStart = Entry->SlotIndex * MaxClusterPerCoarseNode;
					for (int32 i = 0; i < MaxClusterPerCoarseNode; ++i)
					{
						const int32 Idx = SlotStart + i;
						BackBuffer.ClusterPositions[Idx] = DeadPos;
						BackBuffer.ClusterRotations[Idx] = FVector::ZeroVector;
						BackBuffer.ClusterExtents[Idx] = 0.0f;
						BackBuffer.ClusterColors[Idx] = FLinearColor::Black;
						BackBuffer.GasPositions[Idx] = DeadPos;
						BackBuffer.GasExtents[Idx] = 0.0f;
						BackBuffer.GasColors[Idx] = FLinearColor::Black;
					}
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

			// Generate entering nodes into the back buffer (registry model):
			//   (1) Serial: pop slots, build (Coord, Slot) work list,
			//       register placeholder ActiveCoarseNodes entries with
			//       empty InsertedNodes lists.
			//   (2) Parallel: GenerateCoarseNode writes to disjoint slot
			//       slices of BackBuffer. Same as pre-Task 2.
			//   (3) Serial: per cell, walk the slot's particles and
			//       register each into the octree (per-particle insert at
			//       MakePointDataFromWorldScale-derived depth). Records
			//       touched nodes back into the cell entry.
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

			ParallelFor(ToGenerate.Num(), [this, &ToGenerate, &BackBuffer](int32 i)
				{
					const TPair<FIntVector, int32>& Pair = ToGenerate[i];
					GenerateCoarseNode(Pair.Key, Pair.Value, BackBuffer);
				}, EParallelForFlags::BackgroundPriority);

			// Octree registry pass. Serial — InsertPosition mutex-guards
			// internally so parallel calls would just contend on that lock.
			for (const TPair<FIntVector, int32>& Pair : ToGenerate)
			{
				FCoarseSlotEntry* Entry = ActiveCoarseNodes.Find(Pair.Key);
				if (Entry)
				{
					InsertCoarseCellIntoOctree(Pair.Key, Pair.Value, BackBuffer, Entry->InsertedNodes);
				}
			}

			// Atomic buffer swap.
			CoarseFrontIdx.store(BackIdx);
			bCoarseNeedsPush.store(true);
			bCoarseUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateCoarseNodes - %d entering, %d exiting in %.3f sec"),
				EnteringNodes.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

void ASectorActor::GenerateCoarseNode(const FIntVector& InCoarseCoord, int32 InSlotIndex, FCoarseBuffer& InBuffer)
{
	// Direct batched noise sampling — no transient density volume. Same
	// pattern as GenerateNodeGalaxies:
	//   Phase 1: generate all candidate positions + their normalized noise
	//            coords (with coord-derived offset baked in).
	//   Phase 2: one GenPositionArray3D call covering all candidates.
	//   Phase 3: walk results, rejection-gate, write accepted to slot buffer.
	//
	// Why no volume: rejection sampling reads density exactly MaxClusterPer-
	// CoarseNode times (500 currently). Building a 128³ = 2M voxel volume as
	// a lookup structure to serve 500 reads was pure waste — ~8MB alloc per
	// cell, serialized into the stream, and most voxels never sampled. The
	// FastNoise batch path evaluates noise at exactly the points we actually
	// care about. Volume textures stay relevant only for raymarching consumers
	// (see InitializeData + bEnableVolumetric), which sample the same field
	// thousands of times per frame at arbitrary positions.
	//
	// Differences from GenerateNodeGalaxies:
	//   - Candidates are cell-local [-Extent, +Extent] instead of scan-node-
	//     local — coarse cells are the outer unit here, not a child of one.
	//   - Noise offset is InCoarseCoord * 2.0 directly; no per-candidate
	//     coarse-cell lookup since every candidate shares this node's coord.
	//   - Per accepted point: writes both a cluster and a gas entry (1:1
	//     matched), gas extent lerped by the same density value we rejection-
	//     gated against.
	//
	// Position uses CoarseCoordToCenter directly (the cell's logical center).
	// The octree's quantization doesn't influence rendering — the spatial
	// index is populated as a separate post-generation pass and only stores
	// slot indices, not authoritative positions.

	const int32 BufferStart = InSlotIndex * MaxClusterPerCoarseNode;
	const FVector NodeCenter = CoarseCoordToCenter(InCoarseCoord);

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoarseCoord.X), GetTypeHash(InCoarseCoord.Y)),
		GetTypeHash(InCoarseCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const float ExtentRange = GasMaxExtent - GasMinExtent;
	const int32 NumCandidates = MaxClusterPerCoarseNode;
	const double InvExtent = 1.0 / (double)Params.Extent;

	// Coord-derived noise offset — every candidate in this cell gets the same
	// offset applied, unlike GenerateNodeGalaxies where a scan node can
	// straddle coarse cell boundaries.
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
		// Cell-local candidate in [-Extent, +Extent].
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

		// Cluster scale pick — stable curve-driven sample per-object.
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

		// Gas extent lerped by the same density we gated against.
		const float GasExtent = GasMinExtent + ExtentRange * Density;

		// Sector-actor-local position. The push pass converts to player-
		// relative via (Local + ActorPos) - PlayerPos; world render lands
		// at (Local + ActorPos), which tracks the actor's slow drift.
		const FVector LocalPos = CandidatePositions[i] + NodeCenter;

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.ClusterPositions[Idx] = LocalPos;
		InBuffer.ClusterRotations[Idx] = Rotation;
		InBuffer.ClusterExtents[Idx] = ClusterExtent;
		InBuffer.ClusterColors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		InBuffer.GasPositions[Idx] = LocalPos;
		InBuffer.GasExtents[Idx] = GasExtent;
		InBuffer.GasColors[Idx] = FLinearColor(1.0f, 1.0f, 1.0f, Density);

		ActualCount++;
	}

	// Dead particles — offscreen.
	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	for (int32 i = ActualCount; i < MaxClusterPerCoarseNode; ++i)
	{
		const int32 Idx = BufferStart + i;
		InBuffer.ClusterPositions[Idx] = DeadPos;
		InBuffer.ClusterRotations[Idx] = FVector::ZeroVector;
		InBuffer.ClusterExtents[Idx] = 0.0f;
		InBuffer.ClusterColors[Idx] = FLinearColor::Black;
		InBuffer.GasPositions[Idx] = DeadPos;
		InBuffer.GasExtents[Idx] = 0.0f;
		InBuffer.GasColors[Idx] = FLinearColor::Black;
	}

	CoarseSlotCounts[InSlotIndex] = ActualCount;
}

void ASectorActor::InsertCoarseCellIntoOctree(const FIntVector& InCoarseCoord, int32 InSlotIndex, const FCoarseBuffer& InBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const
{
	// Walk this cell's slot in the buffer; insert each non-dead cluster
	// particle into the octree. Per-particle depth comes from
	// MakePointDataFromWorldScale, which picks the deepest depth at which
	// the node is still big enough to contain the particle's extent.
	// ObjectId carries the slot index so queries can dereference into the
	// buffer; TypeId tags the node as galaxy content so non-galaxy queries
	// can filter it out.
	//
	// We only insert cluster particles, not gas — cluster and gas share
	// 1:1 positions and slot indexing, so a cluster insert spatially
	// represents both. Gas-only queries (if ever needed) can use the same
	// slot index to look up the gas data.
	if (!Octree.IsValid())
	{
		return;
	}

	const double TreeExtent = Octree->Extent;
	const int32 BufferStart = InSlotIndex * MaxClusterPerCoarseNode;
	OutInsertedNodes.Reserve(MaxClusterPerCoarseNode);

	// Apply the actor's slow-drift offset so the octree position matches
	// the cell's stable world frame at insert time. (The tree is in
	// sector-local space; ApplyParallaxOffset shifts the actor by
	// -PlayerDelta*Ratio per tick, so GetActorLocation() represents the
	// drift origin. NOT applied to particle positions in InBuffer — those
	// stay in pristine sector-local space; this offset is only for
	// deciding which octree node to bucket into.)
	for (int32 i = 0; i < MaxClusterPerCoarseNode; ++i)
	{
		const int32 Idx = BufferStart + i;
		const float Extent = InBuffer.ClusterExtents[Idx];
		if (Extent <= 0.0f)
		{
			continue;
		}

		// MakePointDataFromWorldScale returns an FPointData with InsertDepth
		// + a Data with ScaleFactor encoding sub-node precision. We override
		// position/ObjectId/TypeId on the result to carry our slot tag.
		FPointData PointData = FPointData::MakePointDataFromWorldScale(
			static_cast<double>(Extent),
			/*InUnitScale=*/ 1.0,
			static_cast<int64>(TreeExtent));
		PointData.SetPosition(InBuffer.ClusterPositions[Idx]);
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
	// Positions in CoarseBuffers are stored in sector-actor-local space
	// (offset from actor's stable-drift position). Convert to player-
	// relative at push time: Local + ActorPos - PlayerPos. Component sits
	// at the player via SetWorldLocation below, so that relative offset
	// renders at (PlayerPos + Local + ActorPos - PlayerPos) = Local +
	// ActorPos in world space — a slow-drifting world-pinned position.
	const int32 FrontIdx = CoarseFrontIdx.load();
	const FCoarseBuffer& Front = CoarseBuffers[FrontIdx];

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

	const FVector ActorPos = GetActorLocation();
	const int32 Num = Front.ClusterPositions.Num();

	TArray<FVector> RelativeClusterPositions;
	TArray<FVector> RelativeGasPositions;
	RelativeClusterPositions.SetNumUninitialized(Num);
	RelativeGasPositions.SetNumUninitialized(Num);

	// Keep dead particles at ZeroVector so the extent==0 gate culls them;
	// for live particles, convert local → world → player-relative.
	for (int32 i = 0; i < Num; ++i)
	{
		RelativeClusterPositions[i] = (Front.ClusterExtents[i] > 0.0f)
			? (Front.ClusterPositions[i] + ActorPos) - PlayerPos
			: FVector::ZeroVector;

		RelativeGasPositions[i] = (Front.GasExtents[i] > 0.0f)
			? (Front.GasPositions[i] + ActorPos) - PlayerPos
			: FVector::ZeroVector;
	}

	if (CoarseClusterNiagara)
	{
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
			CoarseClusterNiagara, FName("User.Positions"), RelativeClusterPositions);
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
			CoarseClusterNiagara, FName("User.Rotations"), Front.ClusterRotations);
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
			CoarseClusterNiagara, FName("User.Extents"), Front.ClusterExtents);
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
			CoarseClusterNiagara, FName("User.Colors"), Front.ClusterColors);
		CoarseClusterNiagara->SetWorldLocation(PlayerPos);
	}

	if (CoarseGasNiagara)
	{
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
			CoarseGasNiagara, FName("User.Positions"), RelativeGasPositions);
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
			CoarseGasNiagara, FName("User.Extents"), Front.GasExtents);
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
			CoarseGasNiagara, FName("User.Colors"), Front.GasColors);
		CoarseGasNiagara->SetWorldLocation(PlayerPos);
	}
}

#pragma endregion

#pragma region Proximity Galaxy Streaming

void ASectorActor::InitializeProximitySystem()
{
	double StartTime = FPlatformTime::Seconds();

	const int32 TotalSlots = 27;
	const int32 TotalParticles = TotalSlots * MaxParticlesPerNode;

	// Allocate both buffers
	ProximityBuffers[0].Allocate(TotalParticles);
	ProximityBuffers[1].Allocate(TotalParticles);
	FrontBufferIndex.store(0);

	SlotParticleCounts.SetNumZeroed(TotalSlots);

	// Initialize free slot pool
	FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		FreeSlots.Add(i);
	}
	ActiveNodeSlots.Empty();
	CurrentScanCoord = FIntVector(INT32_MIN);

	// Get player position
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

	// Determine initial scan coord. PlayerPos - GetActorLocation() uses the
	// slow-drifting actor frame (ApplyParallaxOffset drifts the actor by
	// -PlayerDelta*Ratio each tick).
	const FVector LocalPos = PlayerPos - GetActorLocation();
	CurrentScanCoord = PositionToScanCoord(LocalPos);

	FProximityBuffer& InitBuffer = ProximityBuffers[0];

	// Two-phase pipeline (registry model — same as InitializeCoarseSystem):
	// serial slot allocation → parallel generate → serial per-cell octree
	// insert pass.
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

	ParallelFor(ToGenerate.Num(), [this, &ToGenerate, &InitBuffer](int32 i)
		{
			const TPair<FIntVector, int32>& Pair = ToGenerate[i];
			GenerateNodeGalaxies(Pair.Key, Pair.Value, InitBuffer);
		}, EParallelForFlags::BackgroundPriority);

	for (const TPair<FIntVector, int32>& Pair : ToGenerate)
	{
		FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Pair.Key);
		if (Entry)
		{
			InsertProximityCellIntoOctree(Pair.Key, Pair.Value, InitBuffer, Entry->InsertedNodes);
		}
	}

	// Copy front to back so both start identical
	ProximityBuffers[1].Positions = ProximityBuffers[0].Positions;
	ProximityBuffers[1].Extents = ProximityBuffers[0].Extents;
	ProximityBuffers[1].Colors = ProximityBuffers[0].Colors;

	// Create proximity Niagara and push data on game thread
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			UNiagaraSystem* GalaxyTemplate = SectorGalaxyCloud ? SectorGalaxyCloud : SectorGalaxyCloud;
			if (!SectorGalaxyCloud)
			{
				UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeProximitySystem - SectorGalaxyCloud not assigned; falling back to SectorGalaxyCloud. The nearby/galaxy-scale layer needs its own system asset to carry the correct material and fade range."));
			}

			ProximityNiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
				GalaxyTemplate,
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

				// No SetWorldLocation — component is attached to the actor
				// which ApplyParallaxOffset pegs to the player every tick.
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
}

void ASectorActor::UpdateProximityNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!ProximityNiagaraComponent) return;

	// If async generation completed a swap, push the new front buffer to Niagara
	if (bProximityNeedsPush.load())
	{
		PushProximityToNiagara();
		ProximityNiagaraComponent->ReinitializeSystem();
		bProximityNeedsPush.store(false);
	}

	// Don't start a new update if one is already in flight
	if (bProximityUpdateInProgress.load()) return;

	// Get player position in sector-actor-local space. PlayerPos -
	// GetActorLocation() uses the slow-drifting actor frame.
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

	const FVector LocalPos = PlayerPos - GetActorLocation();
	FIntVector NewScanCoord = PositionToScanCoord(LocalPos);

	if (NewScanCoord == CurrentScanCoord) return;

	// Boundary crossed — kick off async generation into back buffer
	bProximityUpdateInProgress.store(true);
	FIntVector OldScanCoord = CurrentScanCoord;
	CurrentScanCoord = NewScanCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, OldScanCoord, NewScanCoord]()
		{
			double StartTime = FPlatformTime::Seconds();

			// Copy front buffer to back buffer as starting point.
			// Unchanged slots carry over; only entering slots get regenerated.
			int32 FrontIdx = FrontBufferIndex.load();
			int32 BackIdx = 1 - FrontIdx;
			FProximityBuffer& BackBuffer = ProximityBuffers[BackIdx];

			BackBuffer.Positions = ProximityBuffers[FrontIdx].Positions;
			BackBuffer.Extents = ProximityBuffers[FrontIdx].Extents;
			BackBuffer.Colors = ProximityBuffers[FrontIdx].Colors;

			// Compute entering/exiting sets. Scan coords are now universe-wide
			// (see PositionToScanCoord), so no NodesPerSide bounds culling —
			// the 3x3x3 neighborhood around the player is always 27 cells.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;

			for (int32 dz = -1; dz <= 1; ++dz)
			{
				for (int32 dy = -1; dy <= 1; ++dy)
				{
					for (int32 dx = -1; dx <= 1; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						// OldSet only meaningful if OldScanCoord was a real center,
						// not the INT32_MIN cold-start sentinel.
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
			// Free exiting slots, dead-stub their data, retire each cell's
			// slot from every octree node it touched.
			for (const FIntVector& Coord : ExitingNodes)
			{
				FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Coord);
				if (Entry)
				{
					int32 SlotStart = Entry->SlotIndex * MaxParticlesPerNode;
					for (int32 i = 0; i < MaxParticlesPerNode; ++i)
					{
						BackBuffer.Positions[SlotStart + i] = FVector(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
						BackBuffer.Extents[SlotStart + i] = 0.0f;
						BackBuffer.Colors[SlotStart + i] = FLinearColor::Black;
					}

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

			// Generate entering nodes into back buffer (registry model):
			// serial slot allocation → parallel generate → serial per-cell
			// octree insert. Same shape as UpdateCoarseNodes.
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

			ParallelFor(ToGenerate.Num(), [this, &ToGenerate, &BackBuffer](int32 i)
				{
					const TPair<FIntVector, int32>& Pair = ToGenerate[i];
					GenerateNodeGalaxies(Pair.Key, Pair.Value, BackBuffer);
				}, EParallelForFlags::BackgroundPriority);

			for (const TPair<FIntVector, int32>& Pair : ToGenerate)
			{
				FProximitySlotEntry* Entry = ActiveNodeSlots.Find(Pair.Key);
				if (Entry)
				{
					InsertProximityCellIntoOctree(Pair.Key, Pair.Value, BackBuffer, Entry->InsertedNodes);
				}
			}

			// Atomic swap: back becomes front
			FrontBufferIndex.store(BackIdx);

			// Signal game thread to push the new front buffer on next Tick
			bProximityNeedsPush.store(true);

			bProximityUpdateInProgress.store(false);
		});
}

FIntVector ASectorActor::PositionToScanCoord(const FVector& InLocalPos) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	// Scan coords are a universe-wide integer lattice with step size NodeSize,
	// NOT clamped to [0, NodesPerSide). The old clamp pinned the scan window
	// to the sector's own extent and prevented the boundary-cross branch from
	// firing when the player walked into a neighbor coarse cell. Coords outside
	// [0, NodesPerSide) are fully legal now — GenerateNodeGalaxies builds a
	// coord-derived transient density field, so there's no sector-wide volume
	// to fall off the edge of.
	//
	// Uses FloorToInt (signed-safe) so negative positions map to negative
	// coords symmetrically — coord 0 covers [-Params.Extent, -Params.Extent + NodeSize].
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

void ASectorActor::GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FProximityBuffer& InBuffer)
{
	const int32 BufferStart = InSlotIndex * MaxParticlesPerNode;
	// Position/extent use the cell's logical center — the octree's
	// quantization doesn't influence rendering. Octree insert is a separate
	// post-generation pass.
	const FVector NodeCenter = ScanCoordToCenter(InNodeCoord);
	const double NodeExt = GetScanNodeExtent();

	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InNodeCoord.X), GetTypeHash(InNodeCoord.Y)),
		GetTypeHash(InNodeCoord.Z)
	);
	int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	// --- Batched direct noise sampling ---
	// Generate the full candidate set up front, batch-evaluate the noise
	// function via GenPositionArray3D (SIMD path, same API SampleNoiseToVolume
	// uses internally), then walk the results to do rejection + acceptance
	// into the output slot. Replaces the old per-candidate GenSingle3D loop;
	// same math, same noise, ~8× SIMD-width speedup on the noise evaluation.
	//
	// Candidate count equals slot capacity — no rejection oversampling. The
	// actual accepted count is whatever density lets through, which is the
	// point: particle count directly reflects the density field instead of
	// being flattened to the slot cap in dense cells. If you want more
	// particles, raise MaxParticlesPerNode.
	//
	// Coord-derived noise offset: each candidate falls into one coarse cell,
	// and the noise field is offset by CoarseCoord * 2.0 for that cell so
	// adjacent cells produce a continuous field (same convention as
	// InitializeData and GenerateCoarseNode). The offset is baked into the
	// per-candidate normalized coord before the batch call — FastNoise has no
	// per-call setup to reconfigure, so mixing offsets within one batch is
	// fine: the math just evaluates noise at whatever coord you pass.
	//
	// Power semantics: GenPositionArray3D returns the raw noise output; the
	// coarse-tier pre-bake applies pow(clamp(raw, 0, 1), InNoisePower) with
	// power=1.0 in InitializeData. With power=1 that's equivalent to a bare
	// clamp, which is what we do below.

	const int32 NumCandidates = MaxParticlesPerNode;
	const double InvExtent = 1.0 / (double)Params.Extent;
	const double TwoExtent = 2.0 * (double)Params.Extent;

	// --- Phase 1: generate candidates + normalized noise coords ---
	// Positions are stored as doubles (needed for the output buffer), noise
	// coords are floats (GenPositionArray3D takes float arrays). Keep a
	// parallel copy of the double position to avoid a float→double round-trip
	// at accept time.
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

		// Which coarse cell this candidate falls into, and its position within
		// that cell's local frame. Inlined rather than calling
		// PositionToCoarseCoord + CoarseCoordToCenter to avoid the double
		// division — we already have TwoExtent and InvExtent handy.
		const int32 CX = FMath::FloorToInt32(Candidate.X / TwoExtent + 0.5);
		const int32 CY = FMath::FloorToInt32(Candidate.Y / TwoExtent + 0.5);
		const int32 CZ = FMath::FloorToInt32(Candidate.Z / TwoExtent + 0.5);

		// coarse-local = sector-local - coarse-center, normalized by Extent,
		// then add the coord-derived offset.
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

	// Noise coord arrays no longer needed — free now rather than holding
	// through the accept loop.
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

	// Dead particles — offscreen
	for (int32 i = ActualCount; i < MaxParticlesPerNode; ++i)
	{
		const int32 Idx = BufferStart + i;
		InBuffer.Positions[Idx] = FVector(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
		InBuffer.Extents[Idx] = 0.0f;
		InBuffer.Colors[Idx] = FLinearColor::Black;
	}

	SlotParticleCounts[InSlotIndex] = ActualCount;
}

void ASectorActor::InsertProximityCellIntoOctree(const FIntVector& InNodeCoord, int32 InSlotIndex, const FProximityBuffer& InBuffer, TArray<TSharedPtr<FOctreeNode>>& OutInsertedNodes) const
{
	// Same shape as InsertCoarseCellIntoOctree. Walks this cell's slot in
	// the proximity buffer; per non-dead galaxy point, inserts at the
	// depth picked by MakePointDataFromWorldScale based on point extent.
	if (!Octree.IsValid())
	{
		return;
	}

	const double TreeExtent = Octree->Extent;
	const int32 BufferStart = InSlotIndex * MaxParticlesPerNode;
	OutInsertedNodes.Reserve(MaxParticlesPerNode);

	for (int32 i = 0; i < MaxParticlesPerNode; ++i)
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
	if (!ProximityNiagaraComponent) return;

	// Positions in ProximityBuffers are stored in sector-actor-local space.
	// Same push semantics as PushCoarseToNiagara: convert Local →
	// player-relative so that (component-at-player + relative) renders as
	// (Local + ActorPos) in world space — a slow-drifting world-pinned
	// position that tracks the actor drift applied by ApplyParallaxOffset.
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

	const FVector ActorPos = GetActorLocation();
	const FProximityBuffer& Front = ProximityBuffers[FrontBufferIndex.load()];
	const int32 Num = Front.Positions.Num();

	TArray<FVector> RelativePositions;
	RelativePositions.SetNumUninitialized(Num);

	for (int32 i = 0; i < Num; ++i)
	{
		RelativePositions[i] = (Front.Extents[i] > 0.0f)
			? (Front.Positions[i] + ActorPos) - PlayerPos
			: FVector::ZeroVector;
	}

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
		ProximityNiagaraComponent, FName("User.Positions"), RelativePositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
		ProximityNiagaraComponent, FName("User.Extents"), Front.Extents);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
		ProximityNiagaraComponent, FName("User.Colors"), Front.Colors);

	ProximityNiagaraComponent->SetWorldLocation(PlayerPos);
}

void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// With the inheritance break, Super::Tick is AActor::Tick (which does
	// nothing for our purposes). ApplyParallaxOffset used to be driven from
	// AProceduralSpaceActor::Tick; now we call it directly here.
	ApplyParallaxOffset();

	UpdateCoarseNodes();
	UpdateProximityNodes();
}

#pragma endregion

#pragma region Public Octree Queries
TArray<TSharedPtr<FOctreeNode>> ASectorActor::GetNearbyGalaxyNodes(const FVector& InCenter, double InRadius) const
{
	// Spatial range query into the unified per-sector octree. Depths are
	// per-particle (set by MakePointDataFromWorldScale based on extent), so
	// we don't depth-restrict the query — only TypeId-filter for galaxy
	// content. Caller dereferences each node's ObjectId (and
	// AdditionalObjectIds for collision-bucket nodes) into the appropriate
	// flat buffer.
	if (!Octree.IsValid())
	{
		return {};
	}
	return Octree->GetNodesInRange(InCenter, InRadius, /*MinDepth=*/ -1, /*MaxDepth=*/ -1, GalaxyTypeId);
}
#pragma endregion
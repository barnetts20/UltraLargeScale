#pragma region Includes/ForwardDec
#include "SectorActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region FSectorNiagaraLayerData
void FSectorNiagaraLayerData::PopulateFromPointNodes(
	const TArray<TSharedPtr<FOctreeNode>>& InPointNodes,
	UNiagaraSystem* InSystemAsset,
	FName InLayerName,
	FVector InWorldOffset)
{
	SystemAsset = InSystemAsset;
	LayerName = InLayerName;

	const int32 Num = InPointNodes.Num();
	Positions.SetNumUninitialized(Num);
	Rotations.SetNumUninitialized(Num);
	Extents.SetNumUninitialized(Num);
	Colors.SetNumUninitialized(Num);

	// Mirror of the old per-node array build that ran inline in
	// ASectorActor::InitializeData before layer extraction. Per-point
	// rotation is derived from a random stream seeded on ObjectId so the
	// rotation is stable across runs of the same sector. WorldOffset shifts
	// every position by the sector's cell origin so multi-sector grids
	// render at the correct world location.
	ParallelFor(Num, [&](int32 Index)
		{
			const TSharedPtr<FOctreeNode>& Node = InPointNodes[Index];
			FRandomStream RandStream(Node->Data.ObjectId);

			Positions[Index] = Node->Center + InWorldOffset;
			Rotations[Index] = RandStream.GetUnitVector();
			Extents[Index] = static_cast<float>(Node->Extent * (1.0 + Node->Data.ScaleFactor));
			Colors[Index] = FLinearColor(Node->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);
}
#pragma endregion

#pragma region FSectorNiagaraLayerData::PopulateFromDensityVolume
void FSectorNiagaraLayerData::PopulateFromDensityVolume(
	const FDensityVolume& InDensityVolume,
	double InSectorExtent,
	int32 InSampleCount,
	float InMinExtent,
	float InMaxExtent,
	int32 InSeed,
	UNiagaraSystem* InSystemAsset,
	FName InLayerName,
	FVector InWorldOffset)
{
	SystemAsset = InSystemAsset;
	LayerName = InLayerName;

	if (InSampleCount <= 0 || !InDensityVolume.IsValid())
	{
		Positions.Empty();
		Rotations.Empty();
		Extents.Empty();
		Colors.Empty();
		return;
	}

	// Allocate worst-case (every candidate accepted). We trim to the actual
	// accepted count after the parallel pass via SetNum.
	Positions.SetNumUninitialized(InSampleCount);
	Rotations.SetNumUninitialized(InSampleCount);
	Extents.SetNumUninitialized(InSampleCount);
	Colors.SetNumUninitialized(InSampleCount);

	std::atomic<int32> WriteIdx{ 0 };
	const float ExtentRange = InMaxExtent - InMinExtent;

	ParallelFor(InSampleCount, [&](int32 i)
		{
			// Per-candidate deterministic stream — same sector seed + candidate
			// index always produces the same accept/reject + position. Lets
			// re-init reproduce the same gas pattern.
			FRandomStream Stream(HashCombine(InSeed, GetTypeHash(i)));

			const FVector Candidate(
				Stream.FRandRange(-InSectorExtent, InSectorExtent),
				Stream.FRandRange(-InSectorExtent, InSectorExtent),
				Stream.FRandRange(-InSectorExtent, InSectorExtent)
			);

			// Density is sampled in sector-local space (the candidate position
			// pre-offset). World offset is added to the output only.
			const float Density = InDensityVolume.SampleDensityAtLocalPos(Candidate, 3);
			if (Stream.FRand() > Density) return; // rejected

			const int32 Slot = WriteIdx.fetch_add(1, std::memory_order_relaxed);
			Positions[Slot] = Candidate + InWorldOffset;
			Rotations[Slot] = FVector::ZeroVector; // gas is billboard / radially symmetric
			Extents[Slot] = InMinExtent + ExtentRange * Density;
			Colors[Slot] = FLinearColor(1.0f, 1.0f, 1.0f, Density);
		}, EParallelForFlags::BackgroundPriority);

	const int32 FinalCount = WriteIdx.load(std::memory_order_relaxed);
	Positions.SetNum(FinalCount, /*bAllowShrinking=*/ false);
	Rotations.SetNum(FinalCount, /*bAllowShrinking=*/ false);
	Extents.SetNum(FinalCount,   /*bAllowShrinking=*/ false);
	Colors.SetNum(FinalCount,    /*bAllowShrinking=*/ false);
}
#pragma endregion

#pragma region FSectorNiagaraLayerData::PopulateGasFromPointNodes
void FSectorNiagaraLayerData::PopulateGasFromPointNodes(
	const TArray<TSharedPtr<FOctreeNode>>& InPointNodes,
	const FDensityVolume& InDensityVolume,
	float InMinExtent,
	float InMaxExtent,
	UNiagaraSystem* InSystemAsset,
	FName InLayerName,
	FVector InWorldOffset)
{
	SystemAsset = InSystemAsset;
	LayerName = InLayerName;

	const int32 Num = InPointNodes.Num();
	Positions.SetNumUninitialized(Num);
	Rotations.SetNumUninitialized(Num);
	Extents.SetNumUninitialized(Num);
	Colors.SetNumUninitialized(Num);

	const float ExtentRange = InMaxExtent - InMinExtent;
	const bool bHasDensity = InDensityVolume.IsValid();

	// Co-locate gas sprites with the cluster-scale points. Density is
	// resampled at each node's sector-local center so the gas sprite's extent
	// scales with how "thick" the region it sits in is — the cluster path
	// already gated acceptance on density, so samples here should generally
	// be non-zero, but we still clamp to handle any edge cases at the volume
	// boundary.
	ParallelFor(Num, [&](int32 Index)
		{
			const TSharedPtr<FOctreeNode>& Node = InPointNodes[Index];
			const FVector LocalPos = Node->Center;

			const float Density = bHasDensity
				? FMath::Clamp(InDensityVolume.SampleDensityAtLocalPos(LocalPos, 3), 0.0f, 1.0f)
				: 1.0f;

			Positions[Index] = LocalPos + InWorldOffset;
			Rotations[Index] = FVector::ZeroVector;
			Extents[Index] = InMinExtent + ExtentRange * Density;
			Colors[Index] = FLinearColor(1.0f, 1.0f, 1.0f, Density);
		}, EParallelForFlags::BackgroundPriority);
}
#pragma endregion

#pragma region Constructor
ASectorActor::ASectorActor()
{
	SectorGalaxyCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_SectorParallaxCloud.NG_SectorParallaxCloud"));
	SectorClusterCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorClusterCloud.NG_SectorClusterCloud"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGasCloud.NG_SectorGasCloud"));
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
	// wrong. Rebuild against the actual extent.
	Octree = MakeShared<FOctree>(Params.Extent);
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
	float webRemapMin = -.005;
	float webRemapMax = 1.005;

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

	// Cluster + gas sprite generation has moved out of init into the coarse
	// streaming tier (see InitializeCoarseSystem / UpdateCoarseNodes). Each
	// active coarse node builds its own transient density volume during
	// generation and drops it afterwards.
	//
	// HOWEVER: the fine-tier proximity system (GenerateNodeGalaxies) still
	// reads the sector's member DensityVolume during its rejection sampling,
	// and the volumetric raymarcher consumes DensityBuffer when enabled. So
	// we still build the sector-wide density volume here — we just skip the
	// point-generation / octree-insert / LayerData-populate steps that used
	// to follow it.
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

	// Pseudo-volume texture — only needed for the debug volumetric raymarcher.
	if (bEnableVolumetric)
	{
		StepStart = FPlatformTime::Seconds();
		PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(FVolumeTextureUtils::PackToPseudoVolumeLayout(DensityBuffer));
		UE_LOG(LogTemp, Log, TEXT("  [InitData] CreatePseudoVolumeTexture: %.3f sec"), FPlatformTime::Seconds() - StepStart);
	}

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
	double StartTime = FPlatformTime::Seconds();

	// Resolve the player's current position once — used as the initial
	// origin for relative-space particle data across all layers. Running on
	// whatever thread we're already on; GetPlayerController is thread-safe
	// enough for this read-only usage (layers re-sync on first tick anyway).
	FVector PlayerPos = FVector::ZeroVector;
	if (auto* Controller = UGameplayStatics::GetPlayerController(GetWorld(), 0))
	{
		if (APawn* Pawn = Controller->GetPawn())
		{
			PlayerPos = Pawn->GetActorLocation();
		}
	}

	// LayerComponents stays index-parallel with LayerData. Reserve up front;
	// entries get written from the game thread during the spawn phase.
	LayerComponents.SetNum(LayerData.Num());

	// ---- Phase 1 (game thread): spawn one UNiagaraComponent per layer. ----
	// Components must be spawned on the game thread and attached to the
	// sector root. SetSystemFixedBounds / location are set here too so the
	// engine has a valid bounds immediately.
	TPromise<void> SpawnPromise;
	TFuture<void>  SpawnFuture = SpawnPromise.GetFuture();
	TWeakObjectPtr<ASectorActor> WeakSelf(this);
	AsyncTask(ENamedThreads::GameThread,
		[WeakSelf, PlayerPos, SpawnPromise = MoveTemp(SpawnPromise)]() mutable
		{
			ASectorActor* Self = WeakSelf.Get();
			if (!Self)
			{
				SpawnPromise.SetValue();
				return;
			}

			for (int32 i = 0; i < Self->LayerData.Num(); ++i)
			{
				const FSectorNiagaraLayerData& Layer = Self->LayerData[i];
				if (!Layer.SystemAsset)
				{
					UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeNiagara: layer %d (%s) has no SystemAsset, skipping"),
						i, *Layer.LayerName.ToString());
					continue;
				}

				UNiagaraComponent* Comp = UNiagaraFunctionLibrary::SpawnSystemAttached(
					Layer.SystemAsset,
					Self->GetRootComponent(),
					NAME_None,
					FVector::ZeroVector,
					FRotator::ZeroRotator,
					EAttachLocation::SnapToTarget,
					/*bAutoDestroy=*/ true,
					/*bAutoActivate=*/ false);

				if (!Comp) continue;

				Comp->SetSystemFixedBounds(FBox(FVector(-Self->Params.Extent), FVector(Self->Params.Extent)));
				Comp->TranslucencySortPriority = 0;
				Comp->SetWorldLocation(PlayerPos);

				Self->LayerComponents[i] = Comp;
			}

			SpawnPromise.SetValue();
		});
	SpawnFuture.Wait();

	// ---- Phase 2 (background): build relative positions and push User.* arrays. ----
	// Particles live in sector-relative space so positions get offset by the
	// initial player position before being pushed to Niagara.
	TPromise<void> PushPromise;
	TFuture<void>  PushFuture = PushPromise.GetFuture();
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask,
		[WeakSelf, PlayerPos, PushPromise = MoveTemp(PushPromise)]() mutable
		{
			ASectorActor* Self = WeakSelf.Get();
			if (!Self)
			{
				PushPromise.SetValue();
				return;
			}

			for (int32 i = 0; i < Self->LayerData.Num(); ++i)
			{
				UNiagaraComponent* Comp = Self->LayerComponents[i].Get();
				if (!Comp) continue;

				const FSectorNiagaraLayerData& Layer = Self->LayerData[i];

				TArray<FVector> RelativePositions;
				const int32 Num = Layer.Positions.Num();
				RelativePositions.SetNumUninitialized(Num);
				ParallelFor(Num, [&](int32 j)
					{
						RelativePositions[j] = Layer.Positions[j] - PlayerPos;
					}, EParallelForFlags::BackgroundPriority);

				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
					Comp, FName("User.Positions"), RelativePositions);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
					Comp, FName("User.Rotations"), Layer.Rotations);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
					Comp, FName("User.Colors"), Layer.Colors);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
					Comp, FName("User.Extents"), Layer.Extents);
			}

			PushPromise.SetValue();
		});
	PushFuture.Wait();

	// ---- Phase 3 (game thread): activate all components. ----
	TPromise<void> ActivatePromise;
	TFuture<void>  ActivateFuture = ActivatePromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread,
		[WeakSelf, ActivatePromise = MoveTemp(ActivatePromise)]() mutable
		{
			ASectorActor* Self = WeakSelf.Get();
			if (Self)
			{
				for (const TObjectPtr<UNiagaraComponent>& Comp : Self->LayerComponents)
				{
					if (Comp) Comp->Activate(true);
				}
			}
			ActivatePromise.SetValue();
		});
	ActivateFuture.Wait();

	// Proximity system is still driven inline — its streaming lifecycle is
	// separate from the one-shot layer systems and will be migrated later.
	InitializeProximitySystem();

	// Coarse cluster/gas streaming system. LayerData/LayerComponents above
	// is now vestigial for these two layers — the coarse tier owns its own
	// Niagara components (CoarseClusterNiagara / CoarseGasNiagara) because
	// its data is streamed per-node and double-buffered, not captured once
	// at init time.
	InitializeCoarseSystem();

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds (%d layers)"),
		TotalDuration, LayerData.Num());
}
#pragma endregion

#pragma region Player-Centered Parallax
void ASectorActor::ApplyParallaxOffset()
{
	if (InitializationState != ELifecycleState::Ready)
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

	// Broadcast parallax to every managed layer component. All layers use
	// the same User.ParallaxOffset / reseat-at-player pattern; per-layer
	// behavior differences are expressed in the Niagara asset, not here.
	for (const TObjectPtr<UNiagaraComponent>& Comp : LayerComponents)
	{
		if (Comp)
		{
			Comp->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
			Comp->SetWorldLocation(CurrentPlayerPos);
		}
	}

	// Coarse cluster + gas components get the same parallax broadcast. They
	// live outside LayerComponents because their data is streamed per-node
	// and double-buffered rather than captured at init.
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

	// Proximity system is not yet migrated — still driven inline. Will move
	// into LayerComponents when its streaming lifecycle is unified.
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		ProximityNiagaraComponent->SetWorldLocation(CurrentPlayerPos);
	}

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
	for (const TObjectPtr<UNiagaraComponent>& Comp : LayerComponents)
	{
		if (Comp)
		{
			Comp->Deactivate();
			Comp->DestroyComponent();
		}
	}
	LayerComponents.Empty();
	LayerData.Empty();

	// Proximity component is not yet part of LayerComponents; tear it down
	// the same way.
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->Deactivate();
		ProximityNiagaraComponent->DestroyComponent();
		ProximityNiagaraComponent = nullptr;
	}

	// Coarse streaming components — same pattern as proximity.
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
// world origin. Same convention the old AUniverseActor used for its sector
// grid, just now the grid lives inside a single sector actor.
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

	// Coarse coords are sector-actor-local. Subtract ActorLocation so a
	// player placed at the same spot as the sector starts in cell (0,0,0).
	const FVector LocalPlayerPos = PlayerPos - GetActorLocation();
	CoarseCenterCell = PositionToCoarseCoord(LocalPlayerPos);

	FCoarseBuffer& InitBuffer = CoarseBuffers[0];
	for (int32 dz = -CoarseNeighborhoodRadius; dz <= CoarseNeighborhoodRadius; ++dz)
	{
		for (int32 dy = -CoarseNeighborhoodRadius; dy <= CoarseNeighborhoodRadius; ++dy)
		{
			for (int32 dx = -CoarseNeighborhoodRadius; dx <= CoarseNeighborhoodRadius; ++dx)
			{
				const FIntVector NeighborCoord = CoarseCenterCell + FIntVector(dx, dy, dz);
				const int32 SlotIndex = CoarseFreeSlots.Pop();
				ActiveCoarseNodes.Add(NeighborCoord, SlotIndex);
				GenerateCoarseNode(NeighborCoord, SlotIndex, InitBuffer);
			}
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
			// cell. Component sits at the player; particles reach
			// CoarseNeighborhoodRadius cells away in every direction.
			const double BoundsExtent = (2 * CoarseNeighborhoodRadius + 1) * Params.Extent;
			const FBox CoarseBounds(FVector(-BoundsExtent), FVector(BoundsExtent));

			if (CoarseClusterNiagara)
			{
				CoarseClusterNiagara->SetSystemFixedBounds(CoarseBounds);
				CoarseClusterNiagara->TranslucencySortPriority = 0;
				CoarseClusterNiagara->SetWorldLocation(PlayerPos);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeCoarseSystem - Failed to create CoarseClusterNiagara"));
			}

			if (CoarseGasNiagara)
			{
				CoarseGasNiagara->SetSystemFixedBounds(CoarseBounds);
				CoarseGasNiagara->TranslucencySortPriority = 0;
				CoarseGasNiagara->SetWorldLocation(PlayerPos);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeCoarseSystem - Failed to create CoarseGasNiagara"));
			}

			// Push the initial (real) data into both components, then activate.
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
	// pattern as UpdateProximityNodes (line just below).
	if (bCoarseNeedsPush.load())
	{
		PushCoarseToNiagara();
		if (CoarseClusterNiagara) CoarseClusterNiagara->ReinitializeSystem();
		if (CoarseGasNiagara) CoarseGasNiagara->ReinitializeSystem();
		bCoarseNeedsPush.store(false);
	}

	// Don't start a new update if one is in flight.
	if (bCoarseUpdateInProgress.load()) return;

	// Player pos → coarse coord. Coarse coords live in sector-actor-local space
	// (same convention as InitializeCoarseSystem and UpdateProximityNodes), so
	// subtract ActorLocation before binning. Critical because ApplyParallaxOffset
	// drifts the actor's world position every tick — using raw PlayerPos here
	// would compare against a CoarseCenterCell that was computed in a different
	// (and constantly receding) coordinate frame at init time, and the
	// boundary-cross branch would either fire wrongly on frame 1 or never fire.
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

			// Free exiting slots; dead-stub their data in the back buffer.
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				int32* SlotPtr = ActiveCoarseNodes.Find(Coord);
				if (SlotPtr)
				{
					const int32 SlotStart = *SlotPtr * MaxClusterPerCoarseNode;
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
					CoarseFreeSlots.Add(*SlotPtr);
					ActiveCoarseNodes.Remove(Coord);
				}
			}

			// Generate entering nodes into the back buffer.
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (CoarseFreeSlots.Num() == 0)
				{
					UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateCoarseNodes - no free slots; dropping cell (%d,%d,%d)"),
						Coord.X, Coord.Y, Coord.Z);
					continue;
				}
				const int32 SlotIndex = CoarseFreeSlots.Pop();
				ActiveCoarseNodes.Add(Coord, SlotIndex);
				GenerateCoarseNode(Coord, SlotIndex, BackBuffer);
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
	// Mirrors GenerateNodeGalaxies exactly — same inline accept/reject loop,
	// same slot layout, same dead-stub tail. The only differences:
	//   - Operates on a fresh density volume scoped to this coarse cell,
	//     built with a coord-derived noise offset so adjacent cells produce
	//     a continuous field.
	//   - Writes both cluster and gas entries per accepted point (1:1).
	//   - Uses cluster scale range (MinClusterScale/MaxClusterScale) instead
	//     of galaxy scale range.

	const int32 BufferStart = InSlotIndex * MaxClusterPerCoarseNode;
	const FVector NodeCenter = CoarseCoordToCenter(InCoarseCoord);

	// --- Build transient density volume for this coarse cell ---
	// NoiseOffset shifts the sampling so cell N aligns with cell N+1 in
	// continuous noise space (each cell covers [-1,+1] normalized → offset
	// by 2*Coord). Same pattern InitializeData uses for a single sector.
	auto DensityNoise = BuildNoise(69);
	const FVector NoiseOffset(
		static_cast<double>(InCoarseCoord.X) * 2.0,
		static_cast<double>(InCoarseCoord.Y) * 2.0,
		static_cast<double>(InCoarseCoord.Z) * 2.0);

	TArray<uint8> NodeDensityBuffer = FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		CoarseDensityResolution,
		Params.Extent,
		nullptr,
		-1,
		1.0f,
		3,
		NoiseOffset);

	FDensityVolume NodeDensityVolume(
		NodeDensityBuffer,
		FVector::ZeroVector,
		FVector(Params.Extent, Params.Extent, Params.Extent),
		CoarseDensityResolution);

	// --- Seeded stream for this cell ---
	// Coord-hashed so the same coord always produces the same points even
	// across re-entries; pattern matches GenerateNodeGalaxies.
	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoarseCoord.X), GetTypeHash(InCoarseCoord.Y)),
		GetTypeHash(InCoarseCoord.Z));
	int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const float ExtentRange = GasMaxExtent - GasMinExtent;

	// Candidate count equals slot capacity. The actual accepted count is
	// whatever density lets through — that's the point: particle count
	// directly reflects the density field. If a cell wants more particles,
	// increase MaxClusterPerCoarseNode, not an oversample factor.
	int32 ActualCount = 0;
	const int32 NumCandidates = MaxClusterPerCoarseNode;

	for (int32 i = 0; i < NumCandidates; ++i)
	{
		// Candidate in cell-local space [-Extent, +Extent].
		FVector Candidate(
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent));

		// Density rejection, same pattern as GenerateNodeGalaxies. Candidate
		// is cell-local; density volume is also cell-local (noise offset was
		// applied at sample time, not at query time).
		float Density = NodeDensityVolume.SampleDensityAtLocalPos(Candidate, 3);
		if (Stream.FRand() > Density) continue;

		// Cluster scale pick, same curve-driven sample as
		// UniverseDataGenerator::GenerateDataInternal used.
		float ScaleSample = Stream.FRand();
		double Scale = FPointData::SampleScaleFromDistribution(
			Params.MinClusterScale,
			Params.MaxClusterScale,
			ScaleSample, Params.ScaleDistributionCurve);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		float ClusterExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		// Composition / rotation — stable random per-object for rotation so
		// cluster sprites don't flicker on re-entry.
		FVector CompVec = Stream.GetUnitVector();
		FVector Rotation = Stream.GetUnitVector();

		// Gas extent lerped by density at the accepted position.
		float GasExtent = GasMinExtent + ExtentRange * Density;

		// Sector-actor-local position for the sprite (cell-local candidate
		// offset + cell center, both in sector-local space). The push pass
		// converts to player-relative: Local + ActorPos - PlayerPos.
		const FVector LocalPos = Candidate + NodeCenter;

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

	// Dead particles — offscreen — same pattern as GenerateNodeGalaxies.
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

void ASectorActor::PushCoarseToNiagara()
{
	// Positions in CoarseBuffers are stored in sector-actor-local space (same
	// convention as ProximityBuffers). Convert to player-relative at push
	// time: Local + ActorPos = World, then World - PlayerPos = Relative to
	// Niagara component (which sits at the player).
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

	// Keep dead particles at their parked offscreen pos so Niagara culls
	// them; for live particles, convert local → world → player-relative.
	// Extent == 0 is the dead-particle signal in both subsystems.
	for (int32 i = 0; i < Num; ++i)
	{
		if (Front.ClusterExtents[i] > 0.0f)
		{
			RelativeClusterPositions[i] = (Front.ClusterPositions[i] + ActorPos) - PlayerPos;
		}
		else
		{
			RelativeClusterPositions[i] = FVector::ZeroVector;
		}

		if (Front.GasExtents[i] > 0.0f)
		{
			RelativeGasPositions[i] = (Front.GasPositions[i] + ActorPos) - PlayerPos;
		}
		else
		{
			RelativeGasPositions[i] = FVector::ZeroVector;
		}
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

	// Determine initial scan coord and populate all 27 nodes into front buffer
	FVector LocalPos = PlayerPos - GetActorLocation();
	CurrentScanCoord = PositionToScanCoord(LocalPos);

	FProximityBuffer& InitBuffer = ProximityBuffers[0];

	for (int32 dz = -1; dz <= 1; ++dz)
	{
		for (int32 dy = -1; dy <= 1; ++dy)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				// Scan coords are universe-wide now — no NodesPerSide clamp.
				// Every neighbor in the 3x3x3 window gets a slot.
				FIntVector NeighborCoord = CurrentScanCoord + FIntVector(dx, dy, dz);

				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(NeighborCoord, SlotIndex);
				GenerateNodeGalaxies(NeighborCoord, SlotIndex, InitBuffer);
			}
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
				ProximityNiagaraComponent->SetWorldLocation(PlayerPos);

				PushProximityToNiagara();
				//ProximityNiagaraComponent->ReinitializeSystem(); 
				ProximityNiagaraComponent->Activate(true);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeProximitySystem - Failed to create proximity Niagara component"));
			}

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	//double Duration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeProximitySystem took %.3f sec (%d nodes, %d total particles)"),
	//	Duration, ActiveNodeSlots.Num(), TotalParticles);
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

	// Get player position in octree-local space
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
			// Free exiting slots and clear their data in the back buffer
			for (const FIntVector& Coord : ExitingNodes)
			{
				int32* SlotPtr = ActiveNodeSlots.Find(Coord);
				if (SlotPtr)
				{
					//UE_LOG(LogTemp, Log, TEXT("Clearing exiting slot %d in back buffer %d"), *SlotPtr, 1 - FrontBufferIndex.load());
					int32 SlotStart = *SlotPtr * MaxParticlesPerNode;
					for (int32 i = 0; i < MaxParticlesPerNode; ++i)
					{
						BackBuffer.Positions[SlotStart + i] = FVector(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
						BackBuffer.Extents[SlotStart + i] = 0.0f;
						BackBuffer.Colors[SlotStart + i] = FLinearColor::Black;
					}

					FreeSlots.Add(*SlotPtr);
					ActiveNodeSlots.Remove(Coord);
				}
			}

			// Generate entering nodes into back buffer
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (FreeSlots.Num() == 0)
				{
					//UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateProximityNodes - No free slots available!"));
					break;
				}

				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(Coord, SlotIndex);
				GenerateNodeGalaxies(Coord, SlotIndex, BackBuffer);
			}

			// Atomic swap: back becomes front
			FrontBufferIndex.store(BackIdx);

			// Signal game thread to push the new front buffer on next Tick
			bProximityNeedsPush.store(true);

			//double Duration = FPlatformTime::Seconds() - StartTime;
			//UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateProximityNodes (%d exiting, %d entering) took %.3f sec"),
			//	ExitingNodes.Num(), EnteringNodes.Num(), Duration);

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

void ASectorActor::PushProximityToNiagara()
{
	if (!ProximityNiagaraComponent) return;

	int32 ActiveCount = 0;
	const FProximityBuffer& Front1 = ProximityBuffers[FrontBufferIndex.load()];
	for (int32 i = 0; i < Front1.Extents.Num(); ++i) { if (Front1.Extents[i] > 0.0f) ActiveCount++; }
	//UE_LOG(LogTemp, Log, TEXT("PushProximityToNiagara: FrontIdx=%d, bufferSize=%d, active=%d"),
	//	FrontBufferIndex.load(), Front1.Positions.Num(), ActiveCount);

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
	const FProximityBuffer& Front = ProximityBuffers[FrontBufferIndex.load()];

	TArray<FVector> RelativePositions;
	RelativePositions.SetNumUninitialized(Front.Positions.Num());

	for (int32 i = 0; i < Front.Positions.Num(); ++i)
	{
		if (Front.Extents[i] > 0.0f)
		{
			RelativePositions[i] = (Front.Positions[i] + ActorPos) - PlayerPos;
		}
		else
		{
			RelativePositions[i] = FVector::ZeroVector;
		}
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
	UpdateCoarseNodes();
	UpdateProximityNodes();
}

#pragma endregion
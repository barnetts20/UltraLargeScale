// GalaxyActor.cpp
// Full tier streaming system mirroring UniverseActor pattern.

#pragma region Includes
#include "GalaxyActor.h"
#include "svo.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
#include "FVolumeTextureUtils.h"
#include <DrawDebugHelpers.h>
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>

#pragma endregion

#pragma region Constructor/Destructor
AGalaxyActor::AGalaxyActor()
{
	// Galaxies are driven by their parent universe via TickFromParent.
	// UE tick is only enabled for level-placed standalone galaxies
	// (bAutoInitializeOnBeginPlay = true) in BeginPlay.
	PrimaryActorTick.bCanEverTick = true;
	SetActorTickEnabled(false);

	GalaxyLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge"));
	GalaxyMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxyMid.NG_GalaxyMid"));
	GalaxySmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxySmall.NG_GalaxySmall"));

	StarSystemActorClass = AStarSystemActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent);
}

AGalaxyActor::~AGalaxyActor()
{
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region BeginPlay
void AGalaxyActor::BeginPlay()
{
	Super::BeginPlay();
	if (bAutoInitializeOnBeginPlay)
	{
		SetActorTickEnabled(true);
		InitializationState = ELifecycleState::Initializing;
		TWeakObjectPtr<AGalaxyActor> WeakThis(this);
		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
			{
				if (AGalaxyActor* Self = WeakThis.Get())
					Self->Initialize();
			});
	}
}
#pragma endregion

#pragma region EndPlay
void AGalaxyActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	// Signal any in-flight star system initializations to abort, then clear tracking.
	for (auto& Pair : SpawnedStarSystems)
	{
		if (AStarSystemActor* System = Pair.Value.Get())
			System->InitializationState = ELifecycleState::Pooling;
	}
	SpawnedStarSystems.Empty();
	StarSystemPool.Empty();

	// Drain any in-flight pushes before destroying the components they may touch.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		FTierStreamingSystem::BeginShutdownDrain(*Tier);

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
	}
	TierNiagaraComponents.Empty();
	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Pool Lifecycle
void AGalaxyActor::ResetForPool()
{
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	// DRAIN BEFORE FREE: an async boundary-cross task may still own this
	// tier's buffers (it writes Buffers/SlotCounts/SlotEntries and its tail
	// commit touches NiagaraComponents under PushCS). Bar new pushes and wait
	// out any in-flight push (BeginShutdownDrain), then wait for the task
	// itself to clear bUpdateInProgress — its commit bails on bShuttingDown,
	// so the wait is bounded by the remaining generation work. Only EndPlay
	// did this previously; the pool path freed live buffers under a worker.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		FTierStreamingSystem::BeginShutdownDrain(*Tier);
		while (Tier->bUpdateInProgress.load())
		{
			FPlatformProcess::Sleep(0.0005f);
		}
	}

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
		Tier->NiagaraComponents.Empty();
		Tier->Buffers.Empty();
		Tier->SlotEntries.Empty();
		Tier->SlotCounts.Empty();
		Tier->CellCache.Empty();
		Tier->CenterCoord = FIntVector(INT32_MIN);
		Tier->StampedCenter = FIntVector(INT32_MIN);
		Tier->StampedNCenter = FVector::ZeroVector;
		Tier->AppliedBoundsPad = -1.0;
		Tier->bUpdateInProgress.store(false);
		// Pooled actors are re-initialized — clear the shutdown bar so pushes
		// work again on the next spawn (EndPlay leaves it set; we must not).
		Tier->bShuttingDown.store(false);
	}
	TierNiagaraComponents.Empty();
	DiagTickCount = 0;

	Super::ResetForPool();
}

void AGalaxyActor::ResetForSpawn()
{
	Super::ResetForSpawn();
	VirtualTraversal = FVector::ZeroVector;
	LastPushedVirtualTraversal = FVector::ZeroVector;
	LastFrameOfReferenceLocation = FVector::ZeroVector;
	CurrentFrameOfReferenceLocation = FVector::ZeroVector;
	DiagTickCount = 0;
}
#pragma endregion

#pragma region Initialization
void AGalaxyActor::InitializeChildPool()
{
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self) { CompletionPromise.SetValue(); return; }
			for (int i = 0; i < Self->StarSystemPoolSize; i++) {
				AStarSystemActor* System = Self->GetWorld()->SpawnActor<AStarSystemActor>(
					Self->StarSystemActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				System->Galaxy = Self;
				System->SetActorHiddenInGame(true);
				Self->StarSystemPool.Add(System);
			}
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
}

void AGalaxyActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	GalaxyGenerator.Params = Params;
	// MaxEntityScale is a fixed absolute world-cm value on FGalaxyParams.
	// DeriveScaleRanges cascades it through the tier depth sequence to
	// set MinScale/MaxScale per tier. No per-instance derivation needed.
	GalaxyGenerator.Params.DeriveScaleRanges();
	GalaxyGenerator.Initialize();

	if (InitializationState == ELifecycleState::Pooling) return;

	TArray<uint8> VolumeData = GalaxyGenerator.SampleNoiseVolume(Params.MaterialParams.DensityVolumeResolution);

	if (InitializationState == ELifecycleState::Pooling) return;

	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(
		FVolumeTextureUtils::PackToPseudoVolumeLayout(
			FVolumeTextureUtils::UpscaleVolumeData(VolumeData, Params.MaterialParams.DensityVolumeResolution)));

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeData took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self || Self->InitializationState == ELifecycleState::Pooling)
			{
				CompletionPromise.SetValue();
				return;
			}

			Self->VolumetricComponent = NewObject<UStaticMeshComponent>(Self);
			Self->VolumetricComponent->SetVisibility(false);
			Self->VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			Self->VolumetricComponent->AttachToComponent(Self->RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			Self->VolumetricComponent->SetAbsolute(true, false, false);
			Self->VolumetricComponent->TranslucencySortPriority = 1;
			Self->VolumetricComponent->DepthPriorityGroup = ESceneDepthPriorityGroup::SDPG_MAX;
			Self->VolumetricComponent->bRenderInDepthPass = false;
			Self->VolumetricComponent->RegisterComponent();
			Self->VolumetricComponent->SetWorldScale3D(FVector(2 * Self->Params.Extent));

			Self->VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *Self->VolumetricMaterialPath), Self);

			Self->VolumeMaterial->SetTextureParameterValue(FName("VolumeTexture"), Self->PseudoVolumeTexture);
			Self->VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *Self->Params.MaterialParams.VolumeNoise));
			Self->VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), Self->Params.MaterialParams.VolumeAmbientColor);
			Self->VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), Self->Params.MaterialParams.VolumeCoolShift);
			Self->VolumeMaterial->SetVectorParameterValue(FName("HotShift"), Self->Params.MaterialParams.VolumeHotShift);
			Self->VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), Self->Params.MaterialParams.VolumeHueVariance);
			Self->VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Self->Params.MaterialParams.VolumeHueVarianceScale);
			Self->VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), Self->Params.MaterialParams.VolumeSaturationVariance);
			Self->VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Self->Params.MaterialParams.VolumeTemperatureInfluence);
			Self->VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), Self->Params.MaterialParams.VolumeTemperatureScale);
			Self->VolumeMaterial->SetScalarParameterValue(FName("ScaleFactor"), Self->Params.MaterialParams.VolumeDensity);
			Self->VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), Self->Params.MaterialParams.VolumeWarpAmount);
			Self->VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), Self->Params.MaterialParams.VolumeWarpScale);

			Self->VolumetricComponent->SetMaterial(0, Self->VolumeMaterial);
			Self->VolumetricComponent->SetVisibility(true);
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeVolumetric took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::InitializeTier(Ctx, LargeTierConfig, LargeTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;
	FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;
	FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);

	// No timer start needed — Universe::DetermineAndDispatchScan drives scans.

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeNiagara total: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Grid Coord Helpers
FIntVector AGalaxyActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	return FTierStreamingSystem::PositionToGridCoord(InPos, InGridDepth, Params.Extent, GridExtentMultiplier);
}

FVector AGalaxyActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	return FTierStreamingSystem::GridCoordToCenter(InCoord, InGridDepth, Params.Extent, GridExtentMultiplier);
}

double AGalaxyActor::GetGridCellExtent(int32 InGridDepth) const
{
	return FTierStreamingSystem::GetGridCellExtent(InGridDepth, Params.Extent, GridExtentMultiplier);
}

bool AGalaxyActor::CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const
{
	const FVector Center = GridCoordToCenter(Coord, GridDepth);
	const double HalfCell = GetGridCellExtent(GridDepth);
	const double Ext = Params.Extent;
	return (Center.X + HalfCell > -Ext && Center.X - HalfCell < Ext) &&
		(Center.Y + HalfCell > -Ext && Center.Y - HalfCell < Ext) &&
		(Center.Z + HalfCell > -Ext && Center.Z - HalfCell < Ext);
}
#pragma endregion

#pragma region Tier System - BuildTierConfigs
void AGalaxyActor::BuildTierConfigs()
{
	Params.DeriveScaleRanges();

	// --- Large tier: exhaustive single cell, no streaming ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = 0;
	LargeTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	LargeTierConfig.NiagaraAssets = { GalaxyLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		GalaxyGenerator.GenerateLargeTierSlot(SlotIndex, *Buffers[0], LargeTierState.SlotCounts[SlotIndex]);
		};

	// --- Mid tier: neighborhood streaming ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.MaxParticlesPerSlot;
	MidTierConfig.NiagaraAssets = { GalaxyMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.TierIndex = 1;
	MidTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) {
		return !CellOverlapsVolume(Coord, MidTierConfig.GridDepth);
		};
	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);
		GalaxyGenerator.GenerateTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter,
			CellExt, Params.MidTier, 7, MidTierState.SlotCounts[SlotIndex]);
		};

	// --- Small tier: neighborhood streaming ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.MaxParticlesPerSlot;
	SmallTierConfig.NiagaraAssets = { GalaxySmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) {
		return !CellOverlapsVolume(Coord, SmallTierConfig.GridDepth);
		};
	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, SmallTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(SmallTierConfig.GridDepth);
		GalaxyGenerator.GenerateTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter,
			CellExt, Params.SmallTier, 23, SmallTierState.SlotCounts[SlotIndex]);
		};

	// Shared bounds convention — see the derivation in
	// AUniverseActor::BuildTierConfigs. Tight half-bound is
	// (2R+2) * CellHalfExtent (+ particle radius via ApplyPendingBounds);
	// we provision 2 * (2R+1) to match the universe tiers. The previous
	// (2R+1) * CellHalfExtent under-bounded by up to one half-cell when VT
	// sat near a cell boundary, risking edge-cell culling pops.
	auto MakeBounds = [this](const FParticleTierConfig& Config) {
		const double HalfExt = GetGridCellExtent(Config.GridDepth) * (2 * Config.NeighborhoodRadius + 1) * 2.0;
		return FBox(FVector(-HalfExt), FVector(HalfExt));
		};

	LargeTierConfig.ComputeBounds = [this, MakeBounds]() { return FBox(FVector(-Params.Extent), FVector(Params.Extent)); };
	MidTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(MidTierConfig); };
	SmallTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(SmallTierConfig); };
}
#pragma endregion

#pragma region Tier System - BuildStreamingContext
FTierStreamingContext AGalaxyActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = Params.Extent;
	Ctx.UnitScale = Params.UnitScale;
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = false;
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = GetName();
	Ctx.ParentSeed = Params.Seed;
	Ctx.GetLatestVT = [this] { return ReadLatestVT(); };
	Ctx.GetLiveState = [this] { return InitializationState; };
	return Ctx;
}
#pragma endregion

#pragma region Tick
void AGalaxyActor::Tick(float DeltaTime)
{
	AActor::Tick(DeltaTime);
	if (InitializationState != ELifecycleState::Ready) return;

	FVector CurrentPlayerPos;
	if (!GetPlayerLocation(GetWorld(), CurrentPlayerPos)) return;

	TickFromParent(DeltaTime, CurrentPlayerPos);
}

void AGalaxyActor::ApplyParallaxOffset(const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Galaxy::ApplyParallaxOffset");
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double ActiveSpeedScale = GetParentSpeedScale();
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(InPlayerPos);

	if (VolumetricComponent)
		VolumetricComponent->SetWorldLocation(InPlayerPos - VirtualTraversal);

	// Publish EVERY frame so a late-completing full push still uses current VT.
	PublishLatestVT(VirtualTraversal);

	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		LastPushedVirtualTraversal = VirtualTraversal;
		SchedulePush();
	}
}

// Coalesced, single-flight per-frame push (see AUniverseActor::SchedulePush).
void AGalaxyActor::SchedulePush()
{
	bPushDirty.store(true, std::memory_order_release);
	if (bPushWorkerLive.exchange(true)) return;
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			for (;;)
			{
				bPushDirty.store(false, std::memory_order_relaxed);
				FTierStreamingSystem::PushTierPositions(
					{ &LargeTierState, &MidTierState, &SmallTierState },
					[this] { return ReadLatestVT(); });
				if (!bPushDirty.load(std::memory_order_acquire))
				{
					bPushWorkerLive.store(false, std::memory_order_release);
					if (!bPushDirty.load(std::memory_order_acquire)) return;
					if (bPushWorkerLive.exchange(true)) return;
				}
			}
		});
}

void AGalaxyActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Galaxy::TickFromParent");
	if (InitializationState != ELifecycleState::Ready) return;

	// --- VirtualTraversal accumulation ---
	ApplyParallaxOffset(InPlayerPos);

	// --- Process pending spawn-scan results ---
	// VirtualTraversal is resolved for this frame, so SpawnStarSystemFromPool
	// sees the correct parallax state. Mirrors UniverseActor::Tick ordering.
	ProcessPendingScanResults();

	// --- Drive active star systems and handle deferred placement ---
	for (auto& Pair : SpawnedStarSystems)
	{
		AStarSystemActor* System = Pair.Value.Get();
		if (!System) continue;

		if (System->InitializationState == ELifecycleState::Ready)
		{
			if (System->bPendingPlacement)
				FinalizeStarSystemPlacement(System);

			System->TickFromParent(DeltaTime, InPlayerPos);
		}
	}

	// --- Tier streaming ---
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	// Apply any Niagara fixed bounds deferred from a boundary-cross push (GT only).
	FTierStreamingSystem::ApplyPendingBounds(LargeTierConfig, LargeTierState);
	FTierStreamingSystem::ApplyPendingBounds(MidTierConfig, MidTierState);
	FTierStreamingSystem::ApplyPendingBounds(SmallTierConfig, SmallTierState);

	if (IsDebug) DrawDebugBounds();

	if (IsDebug && ++DiagTickCount % 60 == 0)
	{
		const FIntVector MidCoord = PositionToGridCoord(VirtualTraversal, MidTierConfig.GridDepth);
		const FIntVector SmallCoord = PositionToGridCoord(VirtualTraversal, SmallTierConfig.GridDepth);
		UE_LOG(LogTemp, Verbose, TEXT("Galaxy [%s] VT=(%.0f,%.0f,%.0f) midGrid=(%d,%d,%d)->(%d,%d,%d) smallGrid=(%d,%d,%d)->(%d,%d,%d) updates=%d/%d"),
			*GetName(),
			VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z,
			MidCoord.X, MidCoord.Y, MidCoord.Z,
			MidTierState.CenterCoord.X, MidTierState.CenterCoord.Y, MidTierState.CenterCoord.Z,
			SmallCoord.X, SmallCoord.Y, SmallCoord.Z,
			SmallTierState.CenterCoord.X, SmallTierState.CenterCoord.Y, SmallTierState.CenterCoord.Z,
			MidTierState.bUpdateInProgress.load() ? 1 : 0,
			SmallTierState.bUpdateInProgress.load() ? 1 : 0);
	}
}
#pragma endregion

#pragma region Child Spawn Location
FVector AGalaxyActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
	const double SizeRatio = Params.UnitScale / ChildUnitScale;
	const FVector RenderedPos = GetActorLocation() + NodeCenter - VirtualTraversal;
	const FVector CameraToNode = RenderedPos - CurrentFrameOfReferenceLocation;
	return CurrentFrameOfReferenceLocation + CameraToNode * SizeRatio;
}
#pragma endregion

#pragma region Spawn Range Scanning
void AGalaxyActor::RequestScan()
{
	SVO_GT_SCOPE("Galaxy::RequestScan");
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;

	// GT snapshot of the tree ref (one atomic ref-count bump) —
	// ReturnGalaxyToPool's flush task reassigns Octree off-thread; reading
	// the member from this worker would be a torn read of a TSharedPtr.
	TSharedPtr<FOctree> TreeSnapshot = Octree;
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos, TreeSnapshot]()
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self || !TreeSnapshot.IsValid()) return;

			const TArray<TSharedPtr<FOctreeNode>> NearbyArray =
				TreeSnapshot->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray]()
				{
					AGalaxyActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
					InnerSelf->bSpawnScanInProgress.store(false);
				});
		});
}

bool AGalaxyActor::IsPlayerInsideBounds() const
{
	if (!Octree.IsValid()) return false;
	const double E = Octree->Extent;
	return FMath::Abs(VirtualTraversal.X) <= E
		&& FMath::Abs(VirtualTraversal.Y) <= E
		&& FMath::Abs(VirtualTraversal.Z) <= E;
}

void AGalaxyActor::ProcessPendingScanResults()
{
	SVO_GT_SCOPE("Galaxy::ProcessPendingScanResults");
	if (!bHasPendingScanResults) return;
	bHasPendingScanResults = false;

	TSet<TSharedPtr<FOctreeNode>> NearbySet(PendingScanResults);
	PendingScanResults.Empty();

	for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
	{
		if (!TrackedSpawnNodes.Contains(Node))
		{
			LogSpawnNodeEnter(Node);
			SpawnStarSystemFromPool(Node);
		}
		if (bDebugDrawSpawnNodes) DebugDrawSpawnNode(Node);
	}

	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedSpawnNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
		ReturnStarSystemToPool(Node);
	}

	TrackedSpawnNodes = MoveTemp(NearbySet);
}

void AGalaxyActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log,
		TEXT("AGalaxyActor::SpawnScan ENTER — center=(%.1f,%.1f,%.1f) extent=%.2f depth=%d seed=%d tier=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Depth, InNode->Data.ObjectId, InNode->Data.TypeId);
}

void AGalaxyActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log,
		TEXT("AGalaxyActor::SpawnScan EXIT  — center=(%.1f,%.1f,%.1f) extent=%.2f depth=%d seed=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Depth, InNode->Data.ObjectId);
}

void AGalaxyActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	// Rendered world position = PlayerPos + NodeCenter - VirtualTraversal
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	DrawDebugBox(World, NodeCenterWorld, FVector(InNode->Extent),
		FColor::Cyan, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion

#pragma region Star System Pooled Spawn Hooks
void AGalaxyActor::SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Galaxy::SpawnStarSystemFromPool");
	if (!InNode.IsValid() || !StarSystemActorClass || SpawnedStarSystems.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	if (StarSystemPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AGalaxyActor: Star System pool exhausted"));
		return;
	}

	// --- Resolve the actual particle position from the tier buffer ---
	// The octree node center is a quantized approximation; the real rendered
	// position lives in the Niagara buffer. Mirrors UniverseActor::SpawnGalaxyFromPool.
	const int32 TierIndex = FMath::Clamp(InNode->Data.TypeId, 0, 2);
	FParticleTierState* TierStates[] = { &LargeTierState,  &MidTierState,  &SmallTierState };
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	FVector  ParticlePos = InNode->Center;  // fallback
	float    ParticleExtent = static_cast<float>(InNode->Extent);

	// ParticleIndex is the absolute buffer index. Direct lookup.
	// SINGLE-BUFFER READ GUARD: the transition task overwrites entering
	// slots in place, so only read the CPU arrays while no task is in
	// flight. On the GT this check is race-free (the flag is set on the GT
	// before the task spawns), and if a transition IS in flight the matched
	// node's slot may be mid-rewrite anyway — the octree fallback above
	// (node center/extent) is the correct answer in that case.
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	if (AbsIdx >= 0 && MatchedState.Buffers.Num() > 0 &&
		!MatchedState.bUpdateInProgress.load())
	{
		const FNiagaraParticleBuffer& Buf = MatchedState.Buffers[0];
		ParticlePos = Buf.Positions[AbsIdx];
		ParticleExtent = Buf.Extents[AbsIdx];
	}

	AStarSystemActor* System = StarSystemPool.Pop();
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));
	System->ResetForSpawn();

	// INVERTED DERIVATION: UnitScale is the per-layer design constant
	// (FGalaxyParams::StarSystemUnitScale, flowing from the universe
	// template through every galaxy); the system's LOCAL Extent is what
	// varies. The system's real span is the star sprite's real size times
	// BoundsScaleMultiplier (clearing the star glyph — MaxEntityScale
	// already authors the full orbital diameter); dividing by the constant
	// UnitScale converts that to local units. Because star sprite sizes
	// are themselves invariant (authored real sizes through the constant
	// galaxy UnitScale), system extents — and therefore planet local sizes
	// and the planet-sprite precision budget — are a single global range
	// rather than per-instance quantities.
	System->Params.UnitScale = Params.StarSystemUnitScale;
	{
		const double DerivedExtent =
			(static_cast<double>(ParticleExtent) * Params.UnitScale
				* System->Params.BoundsScaleMultiplier)
			/ System->Params.UnitScale;
		System->Params.Extent = FMath::Clamp(DerivedExtent,
			System->Params.MinDerivedExtent, System->Params.MaxDerivedExtent);
		if (System->Params.Extent != DerivedExtent)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("AGalaxyActor::SpawnStarSystemFromPool — derived extent %.3e clamped to %.3e; ")
				TEXT("retune FGalaxyParams::StarSystemUnitScale or the clamp bounds."),
				DerivedExtent, System->Params.Extent);
		}
	}
	System->SpeedScale = Universe ? Universe->SpeedScale : SpeedScale;
	// ObjectId is the deterministic hierarchical seed composed from
	// (GalaxySeed, GridCoord, GenerationIndex) during octree insertion.
	System->Params.Seed = InNode->Data.ObjectId;
	System->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	System->Params.Rotation = FRandomStream(InNode->Data.ObjectId).GetUnitVector().Rotation();

	// Deferred placement — FinalizeStarSystemPlacement uses this frame's VT.
	System->PendingNodeCenter = ParticlePos;
	System->bPendingPlacement = true;

	UE_LOG(LogTemp, Log,
		TEXT("AGalaxyActor::SpawnStarSystemFromPool — particle=(%.1f,%.1f,%.1f) extent=%.2f unitScale(const)=%.4e derivedExtent=%.4e seed=%d (deferred)"),
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z,
		ParticleExtent, System->Params.UnitScale, System->Params.Extent, System->Params.Seed);

	System->Initialize();
}

void AGalaxyActor::FinalizeStarSystemPlacement(AStarSystemActor* System)
{
	SVO_GT_SCOPE("Galaxy::FinalizeStarSystemPlacement");
	// Mirrors AUniverseActor::FinalizeGalaxyPlacement exactly.
	// Called on the first tick after async init completes, so VirtualTraversal
	// and CurrentFrameOfReferenceLocation are resolved for this frame.

	const FVector SpawnLoc = ComputeChildSpawnLocation(System->PendingNodeCenter, System->Params.UnitScale);
	System->SetActorLocation(SpawnLoc);
	System->LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	System->CurrentFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;

	// VT_initial = PlayerPos - SpawnLoc, so that:
	//   Rendered pos = PlayerPos + (LocalPos - VT) = SpawnLoc + LocalPos
	// matches the galaxy's particle sprite position.
	System->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;
	// Seed the push threshold baseline too, mirroring FinalizeGalaxyPlacement,
	// so the first tick doesn't fire a spurious full-delta push.
	System->LastPushedVirtualTraversal = System->VirtualTraversal;

	System->SetActorHiddenInGame(false);
	System->bPendingPlacement = false;

	UE_LOG(LogTemp, Log,
		TEXT("AGalaxyActor::FinalizeStarSystemPlacement — spawnLoc=(%.1f,%.1f,%.1f) VT=(%.1f,%.1f,%.1f)"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		System->VirtualTraversal.X, System->VirtualTraversal.Y, System->VirtualTraversal.Z);
}

void AGalaxyActor::ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Galaxy::ReturnStarSystemToPool");
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AStarSystemActor> WeakSystem;
	if (SpawnedStarSystems.RemoveAndCopyValue(InNode, WeakSystem))
	{
		AStarSystemActor* PoolSystem = WeakSystem.Get();
		if (PoolSystem)
		{
			PoolSystem->ResetForPool();
			TWeakObjectPtr<AGalaxyActor> WeakThis(this);
			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakSystem]()
				{
					AStarSystemActor* AsyncSystem = WeakSystem.Get();
					if (!AsyncSystem) return;
					AsyncSystem->Octree->bIsResetting.store(true);
					FPlatformProcess::Yield();
					AsyncSystem->Octree = MakeShared<FOctree>(AsyncSystem->Params.Extent);
					AsyncSystem->Octree->bIsResetting.store(false);
					AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakSystem]()
						{
							AGalaxyActor* Self = WeakThis.Get();
							AStarSystemActor* InnerSystem = WeakSystem.Get();
							if (Self && InnerSystem) Self->StarSystemPool.Insert(InnerSystem, 0);
						});
				});
		}
	}
}
#pragma endregion
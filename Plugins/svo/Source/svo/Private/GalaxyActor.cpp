// GalaxyActor.cpp
// Full tier streaming system mirroring UniverseActor pattern.

#pragma region Includes
#include "GalaxyActor.h"
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

#pragma region Parallax
void AGalaxyActor::ApplyParallaxOffset()
{
	// VirtualTraversal accumulation and Niagara pushes are handled inline in
	// TickFromParent. This stub satisfies the pure virtual.
}
#pragma endregion

#pragma region Pool Lifecycle
void AGalaxyActor::ResetForPool()
{
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
		Tier->NiagaraComponents.Empty();
		Tier->Buffers.Empty();
		Tier->ActiveSlots.Empty();
		Tier->FreeSlots.Empty();
		Tier->SlotCounts.Empty();
		Tier->CellCache.Empty();
		Tier->CenterCoord = FIntVector(INT32_MIN);
		Tier->FrontIdx.store(0);
		Tier->bUpdateInProgress.store(false);
		Tier->bNeedsPush.store(false);
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
	GalaxyGenerator.Params.MaxEntityScale = Params.MaxEntityScaleFraction
		* static_cast<double>(Params.Extent) * Params.UnitScale;
	GalaxyGenerator.Params.DeriveScaleRanges();
	GalaxyGenerator.Initialize();

	if (InitializationState == ELifecycleState::Pooling) return;

	TArray<uint8> VolumeData = GalaxyGenerator.SampleNoiseVolume(Params.DensityVolumeResolution);

	if (InitializationState == ELifecycleState::Pooling) return;

	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(
		FVolumeTextureUtils::PackToPseudoVolumeLayout(
			FVolumeTextureUtils::UpscaleVolumeData(VolumeData, Params.DensityVolumeResolution)));

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
			Self->VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *Self->Params.VolumeNoise));
			Self->VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), Self->Params.VolumeAmbientColor);
			Self->VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), Self->Params.VolumeCoolShift);
			Self->VolumeMaterial->SetVectorParameterValue(FName("HotShift"), Self->Params.VolumeHotShift);
			Self->VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), Self->Params.VolumeHueVariance);
			Self->VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Self->Params.VolumeHueVarianceScale);
			Self->VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), Self->Params.VolumeSaturationVariance);
			Self->VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Self->Params.VolumeTemperatureInfluence);
			Self->VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), Self->Params.VolumeTemperatureScale);
			Self->VolumeMaterial->SetScalarParameterValue(FName("ScaleFactor"), Self->Params.VolumeDensity);
			Self->VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), Self->Params.VolumeWarpAmount);
			Self->VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), Self->Params.VolumeWarpScale);

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

	auto MakeBounds = [this](const FParticleTierConfig& Config) {
		const double HalfExt = GetGridCellExtent(Config.GridDepth) * (2 * Config.NeighborhoodRadius + 1);
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

void AGalaxyActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	if (InitializationState != ELifecycleState::Ready) return;

	// --- VirtualTraversal accumulation ---
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;
	const double ActiveSpeedScale = (Universe ? Universe->SpeedScale : SpeedScale);
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(InPlayerPos);

	if (VolumetricComponent)
		VolumetricComponent->SetWorldLocation(InPlayerPos - VirtualTraversal);

	// --- Niagara position push ---
	// Galaxy Niagara components are attached (not absolute-positioned),
	// so they follow the actor via SetActorLocation above. Only enter
	// the per-component loop when positions actually need re-pushing.
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	const bool bNeedsPush = (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold);
	if (bNeedsPush)
	{
		for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		{
			const int32 FrontIdx = Tier->FrontIdx.load();
			for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
			{
				UNiagaraComponent* NC = Tier->NiagaraComponents[b];
				if (!NC || b >= Tier->Buffers.Num()) continue;
				const TArray<FVector>& RelPos = Tier->Buffers[b][FrontIdx].MakeRelativePositions(VirtualTraversal);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NC, NiagaraBufferParams::Positions, RelPos);
			}
		}
		LastPushedVirtualTraversal = VirtualTraversal;
	}

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
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;

	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos]()
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self) return;

			const TArray<TSharedPtr<FOctreeNode>> NearbyArray =
				Self->Octree->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);

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
	FParticleTierConfig* TierConfigs[] = { &LargeTierConfig, &MidTierConfig, &SmallTierConfig };
	FParticleTierState* TierStates[] = { &LargeTierState,  &MidTierState,  &SmallTierState };
	FParticleTierConfig& MatchedConfig = *TierConfigs[TierIndex];
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	FVector  ParticlePos = InNode->Center;  // fallback
	float    ParticleExtent = static_cast<float>(InNode->Extent);

	// ParticleIndex is the absolute buffer index. Direct lookup.
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	const int32 FrontIdx = MatchedState.FrontIdx.load();
	if (AbsIdx >= 0 && MatchedState.Buffers.Num() > 0)
	{
		const FNiagaraParticleBuffer& Front = MatchedState.Buffers[0][FrontIdx];
		ParticlePos = Front.Positions[AbsIdx];
		ParticleExtent = Front.Extents[AbsIdx];
	}

	AStarSystemActor* System = StarSystemPool.Pop();
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));
	System->ResetForSpawn();

	// UnitScale: scale the system so its virtual space is BoundsScaleMultiplier
	// times larger than the star sprite's world radius.
	//
	// Base (no multiplier): UnitScale = (ParticleExtent * Galaxy.UnitScale) / Extent
	//   → Extent octree units == one star sprite radius in world space.
	//   → All planets orbit within the star glyph itself. Wrong.
	//
	// With multiplier:  UnitScale = (ParticleExtent * Galaxy.UnitScale * BoundsScaleMultiplier) / Extent
	//   → Extent octree units == BoundsScaleMultiplier star radii in world space.
	//   → OuterOrbitFraction * Extent octree units == a comfortable orbital distance.
	System->Params.UnitScale = (static_cast<double>(ParticleExtent) * Params.UnitScale
		* System->Params.BoundsScaleMultiplier) / System->Params.Extent;
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
		TEXT("AGalaxyActor::SpawnStarSystemFromPool — particle=(%.1f,%.1f,%.1f) extent=%.2f unitScale=%.4e seed=%d (deferred)"),
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z,
		ParticleExtent, System->Params.UnitScale, System->Params.Seed);

	System->Initialize();
}

void AGalaxyActor::FinalizeStarSystemPlacement(AStarSystemActor* System)
{
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

	System->SetActorHiddenInGame(false);
	System->bPendingPlacement = false;

	UE_LOG(LogTemp, Log,
		TEXT("AGalaxyActor::FinalizeStarSystemPlacement — spawnLoc=(%.1f,%.1f,%.1f) VT=(%.1f,%.1f,%.1f)"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		System->VirtualTraversal.X, System->VirtualTraversal.Y, System->VirtualTraversal.Z);
}

void AGalaxyActor::ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode)
{
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
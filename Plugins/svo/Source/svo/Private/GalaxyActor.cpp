// GalaxyActor.cpp
// Full tier streaming system mirroring UniverseActor pattern.

#pragma region Includes
#include "GalaxyActor.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
#include "FVolumeTextureUtils.h"
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor/Destructor
AGalaxyActor::AGalaxyActor()
{
	PrimaryActorTick.bCanEverTick = true;

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

#pragma region Pool Lifecycle
void AGalaxyActor::ResetForPool()
{
	// Tear down tier Niagara components before base class teardown
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC)
			{
				NC->Deactivate();
				NC->DestroyComponent();
				NC = nullptr;
			}
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

	// Base class handles VolumetricComponent and legacy NiagaraComponent
	Super::ResetForPool();
}

void AGalaxyActor::ResetForSpawn()
{
	Super::ResetForSpawn();
	VirtualTraversal = FVector::ZeroVector;
	LastPushedVirtualTraversal = FVector::ZeroVector;
	LastFrameOfReferenceLocation = FVector::ZeroVector;
	CurrentFrameOfReferenceLocation = FVector::ZeroVector;
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
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetVisibility(false);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			// Absolute world transform: Tick sets VolumetricComponent->SetWorldLocation
			// to (PlayerPos - VirtualTraversal) each frame so the box stays at the
			// galaxy's virtual origin regardless of where the actor is pegged.
			VolumetricComponent->SetAbsolute(true, false, false);
			VolumetricComponent->TranslucencySortPriority = 1;
			VolumetricComponent->DepthPriorityGroup = ESceneDepthPriorityGroup::SDPG_MAX;
			VolumetricComponent->bRenderInDepthPass = false;
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

			VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath), this);

			VolumeMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *Params.VolumeNoise));
			VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), Params.VolumeAmbientColor);
			VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), Params.VolumeCoolShift);
			VolumeMaterial->SetVectorParameterValue(FName("HotShift"), Params.VolumeHotShift);
			VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), Params.VolumeHueVariance);
			VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Params.VolumeHueVarianceScale);
			VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), Params.VolumeSaturationVariance);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Params.VolumeTemperatureInfluence);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), Params.VolumeTemperatureScale);
			VolumeMaterial->SetScalarParameterValue(FName("ScaleFactor"), Params.VolumeDensity);
			VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), Params.VolumeWarpAmount);
			VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), Params.VolumeWarpScale);

			VolumetricComponent->SetMaterial(0, VolumeMaterial);
			VolumetricComponent->SetVisibility(true);
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
	LargeTierConfig.NeighborhoodRadius = 0;  // Single cell, always loaded
	LargeTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	LargeTierConfig.NiagaraAssets = { GalaxyLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, LargeTierConfig.GridDepth);
		GalaxyGenerator.GenerateLargeTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, LargeTierState.SlotCounts[SlotIndex]);
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
		GalaxyGenerator.GenerateMidTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, MidTierState.SlotCounts[SlotIndex]);
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
		GalaxyGenerator.GenerateSmallTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, SmallTierState.SlotCounts[SlotIndex]);
		};

	// ComputeBounds for mid/small tiers.
	// Positions pushed to Niagara are camera-relative (pos - VT), centered
	// near zero. Bounds are centered on the origin.
	auto MakeBounds = [this](const FParticleTierConfig& Config) {
		const double HalfExt = GetGridCellExtent(Config.GridDepth) * (2 * Config.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfExt), FVector(HalfExt));
		};

	LargeTierConfig.ComputeBounds = [this, MakeBounds]() {
		return FBox(FVector(-Params.Extent), FVector(Params.Extent));
		};
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
	Ctx.bRebaseInProgress = false;  // Galaxies never rebase.
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = true;
	Ctx.OwnerName = GetName();
	return Ctx;
}
#pragma endregion

#pragma region Tick
void AGalaxyActor::Tick(float DeltaTime)
{
	// Pool-managed galaxies have UE tick disabled and are driven exclusively by
	// AUniverseActor::Tick via TickFromParent. This path only executes for
	// level-placed galaxies (bAutoInitializeOnBeginPlay = true).
	AActor::Tick(DeltaTime);
	if (InitializationState != ELifecycleState::Ready) return;

	FVector CurrentPlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
			if (APawn* Pawn = Controller->GetPawn())
				CurrentPlayerPos = Pawn->GetActorLocation();

	TickFromParent(DeltaTime, CurrentPlayerPos);
}

void AGalaxyActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	if (InitializationState != ELifecycleState::Ready) return;

	// --- VirtualTraversal accumulation ---
	// Mirrors AUniverseActor::ApplyParallaxOffset. ratio = SpeedScale / UnitScale
	// shrinks as the player approaches, giving full float precision when nearby.
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;
	const double ActiveSpeedScale = (Universe ? Universe->SpeedScale : SpeedScale);
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	// Peg the actor to the player so UE's scene stays in a clean numerical range.
	SetActorLocation(InPlayerPos);

	// Reposition the volumetric box to the galaxy's virtual world origin.
	// Galaxy-origin in world space = PlayerPos - VirtualTraversal.
	if (VolumetricComponent)
		VolumetricComponent->SetWorldLocation(InPlayerPos - VirtualTraversal);

	// Peg Niagara components to the player. SetWorldLocation must happen every
	// tick since the actor is pegged to the moving player. The position array
	// push (the expensive part) is gated by the same sub-pixel threshold as
	// UniverseActor to avoid redundant full-array copies.
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	const bool bNeedsPush = (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold);
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		const int32 FrontIdx = Tier->FrontIdx.load();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;
			NC->SetWorldLocation(InPlayerPos);
			if (bNeedsPush)
			{
				const TArray<FVector>& RelPos = Tier->Buffers[b][FrontIdx].MakeRelativePositions(VirtualTraversal);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NC, NiagaraBufferParams::Positions, RelPos);
			}
		}
	}
	if (bNeedsPush) LastPushedVirtualTraversal = VirtualTraversal;

	// --- Tier streaming ---
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	// --- Drive active star systems with the already-resolved player position ---
	// Star systems have UE tick disabled; this is their only per-frame entry point.
	for (auto& Pair : SpawnedStarSystems)
	{
		if (AStarSystemActor* System = Pair.Value.Get())
			System->TickFromParent(DeltaTime, InPlayerPos);
	}

	if (IsDebug) DrawDebugBounds();

	// --- Per-instance diagnostics (only when debug enabled, fires every 60 frames) ---
	if (IsDebug && ++DiagTickCount % 60 == 0)
	{
		const FVector ActorLoc = GetActorLocation();
		FVector FirstParticlePos = FVector::ZeroVector;
		if (LargeTierState.Buffers.Num() > 0 && LargeTierState.Buffers[0].Num() > 0)
		{
			const auto& Buf = LargeTierState.Buffers[0][LargeTierState.FrontIdx.load()];
			for (int32 i = 0; i < Buf.Positions.Num(); ++i)
				if (Buf.Extents[i] > 0.0f) { FirstParticlePos = Buf.Positions[i]; break; }
		}
		FVector NiagaraCompLoc = FVector::ZeroVector;
		if (LargeTierState.NiagaraComponents.Num() > 0 && LargeTierState.NiagaraComponents[0])
			NiagaraCompLoc = LargeTierState.NiagaraComponents[0]->GetComponentLocation();

		UE_LOG(LogTemp, Verbose, TEXT("Galaxy Diag: ActorLoc=(%.0f,%.0f,%.0f) NiagaraComp=(%.0f,%.0f,%.0f) FirstParticle=(%.1f,%.1f,%.1f) VT=(%.0f,%.0f,%.0f) PlayerLocal=(%.0f,%.0f,%.0f)"),
			ActorLoc.X, ActorLoc.Y, ActorLoc.Z,
			NiagaraCompLoc.X, NiagaraCompLoc.Y, NiagaraCompLoc.Z,
			FirstParticlePos.X, FirstParticlePos.Y, FirstParticlePos.Z,
			VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z,
			CurrentFrameOfReferenceLocation.X, CurrentFrameOfReferenceLocation.Y, CurrentFrameOfReferenceLocation.Z);

		FVector VolWorldPos = FVector::ZeroVector;
		FVector VolRelPos = FVector::ZeroVector;
		FVector NCRelPos = FVector::ZeroVector;
		if (VolumetricComponent)
		{
			VolWorldPos = VolumetricComponent->GetComponentLocation();
			VolRelPos = VolumetricComponent->GetRelativeLocation();
		}
		if (LargeTierState.NiagaraComponents.Num() > 0 && LargeTierState.NiagaraComponents[0])
			NCRelPos = LargeTierState.NiagaraComponents[0]->GetRelativeLocation();

		UE_LOG(LogTemp, Verbose, TEXT("Galaxy Attach: VolWorld=(%.0f,%.0f,%.0f) VolRel=(%.0f,%.0f,%.0f) NCRel=(%.0f,%.0f,%.0f) RenderedParticle=(%.0f,%.0f,%.0f)"),
			VolWorldPos.X, VolWorldPos.Y, VolWorldPos.Z,
			VolRelPos.X, VolRelPos.Y, VolRelPos.Z,
			NCRelPos.X, NCRelPos.Y, NCRelPos.Z,
			NiagaraCompLoc.X + FirstParticlePos.X, NiagaraCompLoc.Y + FirstParticlePos.Y, NiagaraCompLoc.Z + FirstParticlePos.Z);

		FVector MidFirstPos = FVector::ZeroVector;
		int32 MidLiveCount = 0;
		if (MidTierState.Buffers.Num() > 0 && MidTierState.Buffers[0].Num() > 0)
		{
			const auto& MidBuf = MidTierState.Buffers[0][MidTierState.FrontIdx.load()];
			for (int32 i = 0; i < MidBuf.Positions.Num(); ++i)
				if (MidBuf.Extents[i] > 0.0f) { if (MidLiveCount == 0) MidFirstPos = MidBuf.Positions[i]; ++MidLiveCount; }
		}
		const FBox MidBounds = MidTierConfig.ComputeBounds();
		const FIntVector MidCoord = PositionToGridCoord(VirtualTraversal, MidTierConfig.GridDepth);
		const FIntVector SmallCoord = PositionToGridCoord(VirtualTraversal, SmallTierConfig.GridDepth);

		UE_LOG(LogTemp, Verbose, TEXT("Galaxy MidDiag: firstPos=(%.0f,%.0f,%.0f) live=%d bounds=[(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)] GridExtMult=%.1f"),
			MidFirstPos.X, MidFirstPos.Y, MidFirstPos.Z, MidLiveCount,
			MidBounds.Min.X, MidBounds.Min.Y, MidBounds.Min.Z,
			MidBounds.Max.X, MidBounds.Max.Y, MidBounds.Max.Z,
			GridExtentMultiplier);

		UE_LOG(LogTemp, Verbose, TEXT("Galaxy Tick: localPos=(%.0f,%.0f,%.0f) midGrid=(%d,%d,%d) midCenter=(%d,%d,%d) smallGrid=(%d,%d,%d) smallCenter=(%d,%d,%d) midUpdate=%d smallUpdate=%d"),
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

#pragma region Star System Pooled Spawn Hooks
void AGalaxyActor::SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !StarSystemActorClass || SpawnedStarSystems.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	if (StarSystemPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Star System pool exhausted"));
		return;
	}

	AStarSystemActor* System = StarSystemPool.Pop();
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));
	System->ResetForSpawn();
	System->Params.UnitScale = (InNode->Extent * Params.UnitScale) / System->Params.Extent;
	System->SpeedScale = Universe ? Universe->SpeedScale : SpeedScale;
	System->Params.Seed = InNode->Data.ObjectId;
	System->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	System->Params.Rotation = FRandomStream(InNode->Data.ObjectId).GetUnitVector().Rotation();
	System->SetActorLocation(ComputeChildSpawnLocation(InNode->Center, System->Params.UnitScale));
	System->Initialize();
	System->SetActorHiddenInGame(false);
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
					FPlatformProcess::Yield();  // Let in-flight octree ops see the flag and bail.
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
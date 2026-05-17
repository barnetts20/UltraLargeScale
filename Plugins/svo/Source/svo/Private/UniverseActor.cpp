#pragma region Includes/ForwardDec
#include "UniverseActor.h"
#include "FTierStreamingSystem.h"
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
AUniverseActor::AUniverseActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent")));
	SectorLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorLarge.NG_SectorLarge"));
	SectorMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorMid.NG_SectorMid"));
	SectorSmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorSmall.NG_SectorSmall"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGas.NG_SectorGas"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
}
#pragma endregion

#pragma region Lifecycle
void AUniverseActor::Initialize()
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

	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self) return;

			double StartTime = FPlatformTime::Seconds();

			Self->InitializeChildPool();
			if (Self->InitializationState == ELifecycleState::Pooling) return;

			Self->InitializeData();
			if (Self->InitializationState == ELifecycleState::Pooling) return;

			Self->InitializeNiagara();
			if (Self->InitializationState == ELifecycleState::Pooling) return;

			Self->InitializationState = ELifecycleState::Ready;

			UE_LOG(LogTemp, Log, TEXT("AUniverseActor::Initialize total duration: %.3f seconds"),
				FPlatformTime::Seconds() - StartTime);

			AsyncTask(ENamedThreads::GameThread, [WeakThis]()
				{
					if (AUniverseActor* InnerSelf = WeakThis.Get())
					{
						InnerSelf->StartSpawnScanTimer();
					}
				});
		});
}
#pragma endregion

#pragma region Initialization
void AUniverseActor::BeginPlay()
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
	if (bAutoInitializeOnBeginPlay) Initialize();
}

void AUniverseActor::ConfigureCell(FIntVector InCellCoord)
{
	CellCoord = InCellCoord;
	CellOrigin = FVector(static_cast<double>(CellCoord.X) * 2.0 * Params.Extent, static_cast<double>(CellCoord.Y) * 2.0 * Params.Extent, static_cast<double>(CellCoord.Z) * 2.0 * Params.Extent);
	SetActorLocation(CellOrigin);
	Octree = MakeShared<FOctree>(Params.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
}

void AUniverseActor::InitializeChildPool()
{
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self) { CompletionPromise.SetValue(); return; }
			for (int32 i = 0; i < Self->GalaxyPoolSize; i++)
			{
				AGalaxyActor* Galaxy = Self->GetWorld()->SpawnActor<AGalaxyActor>(Self->GalaxyActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				Galaxy->bAutoInitializeOnBeginPlay = false;
				Galaxy->Universe = Self;
				Galaxy->SetActorHiddenInGame(true);
				Self->GalaxyPool.Add(Galaxy);
			}
			UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeChildPool — pre-warmed %d galaxy actors"), Self->GalaxyPoolSize);
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
}

void AUniverseActor::InitializeData()
{
	double TotalStart = FPlatformTime::Seconds();
	UniverseGenerator.Params = Params;
	UniverseGenerator.Initialize();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeData total: %.3f sec"), FPlatformTime::Seconds() - TotalStart);
}

void AUniverseActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::InitializeTier(Ctx, CoarseTierConfig, CoarseTierState, TierNiagaraComponents);
	FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Unified Particle Tier System
FIntVector AUniverseActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	return FTierStreamingSystem::PositionToGridCoord(InPos, InGridDepth, Params.Extent, GridExtentMultiplier);
}

FVector AUniverseActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	return FTierStreamingSystem::GridCoordToCenter(InCoord, InGridDepth, Params.Extent, GridExtentMultiplier);
}

double AUniverseActor::GetGridCellExtent(int32 InGridDepth) const
{
	return FTierStreamingSystem::GetGridCellExtent(InGridDepth, Params.Extent, GridExtentMultiplier);
}

void AUniverseActor::BuildTierConfigs()
{
	// Derive MinScale/MaxScale for all tiers from MaxEntityScale + depth spacing.
	// Must be called before any generate callback reads scale ranges.
	Params.DeriveScaleRanges();

	// --- Large tier (was "Coarse") ---
	CoarseTierConfig.TierName = TEXT("Large");
	CoarseTierConfig.TierIndex = 0;
	CoarseTierConfig.GridDepth = Params.LargeTier.GridDepth;
	CoarseTierConfig.NeighborhoodRadius = Params.LargeTier.NeighborhoodRadius;
	CoarseTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	CoarseTierConfig.NiagaraAssets = { SectorLargeCloud, SectorGasCloud };
	CoarseTierConfig.bWantRotations = { true, false };
	CoarseTierConfig.OctreeInsertBufferIndex = 0;
	CoarseTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, CoarseTierConfig.GridDepth);
		UniverseGenerator.GenerateLargeTierNode(Coord, SlotIndex, *Buffers[0], *Buffers[1], NodeCenter, CoarseTierState.SlotCounts[SlotIndex]);
		};

	// --- Mid tier ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.TierIndex = 1;
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.MaxParticlesPerSlot;
	MidTierConfig.NiagaraAssets = { SectorMidCloud };
	MidTierConfig.bWantRotations = { true };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);
		UniverseGenerator.GenerateMidTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, MidTierState.SlotCounts[SlotIndex]);
		};

	// --- Small tier (was "Proximity") ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.MaxParticlesPerSlot;
	SmallTierConfig.NiagaraAssets = { SectorSmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, SmallTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(SmallTierConfig.GridDepth);
		UniverseGenerator.GenerateSmallTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, SmallTierState.SlotCounts[SlotIndex]);
		};

	// Applied to each tier after its GridDepth/NeighborhoodRadius are set.
	// Captures Config by ref — safe since Config outlives all lambda calls.
	auto MakeBoundsLambda = [this](const FParticleTierConfig& Cfg) -> TFunction<FBox()>
		{
			return [this, &Cfg]() -> FBox
				{
					const double BoundsExtent = (2 * Cfg.NeighborhoodRadius + 1) * GetGridCellExtent(Cfg.GridDepth) * 2.0;
					return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
				};
		};
	CoarseTierConfig.ComputeBounds = MakeBoundsLambda(CoarseTierConfig);
	MidTierConfig.ComputeBounds = MakeBoundsLambda(MidTierConfig);
	SmallTierConfig.ComputeBounds = MakeBoundsLambda(SmallTierConfig);
}

FTierStreamingContext AUniverseActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = Params.Extent;
	Ctx.UnitScale = 1.0;  // Universe extents are already in octree-local units.
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = bRebaseInProgress.load();
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = TEXT("Universe");
	return Ctx;
}

void AUniverseActor::CheckOctreeBounds()
{
	if (!Octree.IsValid()) return;
	if (bRebaseInProgress.load()) return;
	const FVector TreeCenter = Octree->Root->Center;
	const double TreeExtent = Octree->Extent;
	const double Margin = TreeExtent * 0.75;
	const FVector Delta = VirtualTraversal - TreeCenter;
	if (FMath::Abs(Delta.X) <= Margin && FMath::Abs(Delta.Y) <= Margin && FMath::Abs(Delta.Z) <= Margin) return;
	// Only rebase if no tier is mid-update.
	const bool bAnyUpdating = CoarseTierState.bUpdateInProgress.load() || MidTierState.bUpdateInProgress.load() || SmallTierState.bUpdateInProgress.load();
	if (bAnyUpdating) return;
	bRebaseInProgress.store(true);
	const FVector RebaseOrigin = VirtualTraversal;
	const double RebaseTreeExtent = Params.Extent * PersistentTreeMultiplier;

	// Snapshot context values now, on the game thread, so the async task
	// doesn't read VirtualTraversal / Octree / etc. while Tick mutates them.
	// BuildStreamingContext is safe here because no tier update is in progress
	// (checked above) and we're on the game thread.
	FTierStreamingContext RebaseCtx = BuildStreamingContext();

	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, RebaseOrigin, RebaseTreeExtent, RebaseCtx]() {
		AUniverseActor* Self = WeakThis.Get();
		if (!Self) return;
		UE_LOG(LogTemp, Log, TEXT("AUniverseActor::RebaseOctree -- rebasing to (%.1f, %.1f, %.1f)"), RebaseOrigin.X, RebaseOrigin.Y, RebaseOrigin.Z);
		Self->Octree = MakeShared<FOctree>(RebaseTreeExtent, RebaseOrigin);

		// Build an insert context that uses the new octree but the snapshotted VT.
		FTierStreamingContext InsertCtx = RebaseCtx;
		InsertCtx.Octree = Self->Octree;

		FTierStreamingSystem::InsertTierIntoOctree(InsertCtx, Self->CoarseTierConfig, Self->CoarseTierState,
			Self->CoarseTierState.FrontIdx.load());
		FTierStreamingSystem::InsertTierIntoOctree(InsertCtx, Self->MidTierConfig, Self->MidTierState,
			Self->MidTierState.FrontIdx.load());
		FTierStreamingSystem::InsertTierIntoOctree(InsertCtx, Self->SmallTierConfig, Self->SmallTierState,
			Self->SmallTierState.FrontIdx.load());
		Self->bRebaseInProgress.store(false);
		});
}
#pragma endregion

#pragma region Player-Centered Parallax
void AUniverseActor::ApplyParallaxOffset()
{
	if (InitializationState != ELifecycleState::Ready) return;
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
	if (!bHasReference) return;
	const FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;
	const double Ratio = (Params.UnitScale > 0.0) ? (SpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;
	// Peg the actor. Components follow via attachment.
	SetActorLocation(CurrentPlayerPos);
	// Push updated relative positions to all tier buffers only when VirtualTraversal
	// has changed enough that the nearest particle would shift by at least one
	// sub-pixel. Sub-pixel changes are invisible so the push cost (full array
	// copy + Niagara DI write per tier per buffer) can be skipped.
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		for (FParticleTierState* Tier : { &CoarseTierState, &MidTierState, &SmallTierState })
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
}
#pragma endregion

#pragma region Tick
void AUniverseActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ApplyParallaxOffset();  // Resolves player pos once into CurrentFrameOfReferenceLocation

	// Process any pending spawn-scan results now that VirtualTraversal and
	// CurrentFrameOfReferenceLocation are resolved for this frame. This
	// guarantees SpawnGalaxyFromPool/ReturnGalaxyToPool always see the
	// current frame's player position, eliminating the 1-frame parallax
	// offset that occurred when the timer callback landed at an arbitrary
	// point in the frame.
	ProcessPendingScanResults();

	// Drive all active galaxies with the already-resolved player position.
	// Galaxies have UE tick disabled; this is their only per-frame entry point.
	// Each galaxy cascades down to its own star systems via TickFromParent.
	//
	// Deferred placement: galaxies spawned by SpawnGalaxyFromPool start hidden
	// with bPendingPlacement = true. On the first frame after async init
	// completes (InitializationState == Ready), FinalizeGalaxyPlacement
	// computes the spawn position using THIS frame's parallax state, sets
	// VirtualTraversal, makes it visible, and clears the flag. The first
	// TickFromParent runs immediately after in the same frame — zero drift.
	for (auto& Pair : SpawnedGalaxies)
	{
		AGalaxyActor* Galaxy = Pair.Value.Get();
		if (!Galaxy) continue;

		if (Galaxy->InitializationState == ELifecycleState::Ready)
		{
			if (Galaxy->bPendingPlacement)
			{
				FinalizeGalaxyPlacement(Galaxy);
			}
			Galaxy->TickFromParent(DeltaTime, CurrentFrameOfReferenceLocation);
		}
	}

	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, CoarseTierConfig, CoarseTierState);
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);
	CheckOctreeBounds();
}
#pragma endregion

#pragma region Shutdown
void AUniverseActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;
	StopSpawnScanTimer();
	for (FParticleTierState* Tier : { &CoarseTierState, &MidTierState, &SmallTierState })
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
	}
	TierNiagaraComponents.Empty();
	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Child Spawn Location
FVector AUniverseActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
	// The debug box renders at: ActorLocation + NodeCenter - VirtualTraversal
	// That's where the node's particle sprite appears in world space.
	//
	// The galaxy actor is physically larger than the node (Params.Extent vs
	// InNode->Extent). To subtend the same angular size from the camera,
	// it must be placed proportionally further along the same view vector.
	//
	// distance_galaxy / size_galaxy = distance_node / size_node
	// distance_galaxy = distance_node * (size_galaxy / size_node)
	//
	// ChildUnitScale encodes the node-to-galaxy size relationship:
	//   ChildUnitScale = (InNode->Extent * ParentUnitScale) / Galaxy->Params.Extent
	// So: Galaxy->Params.Extent / InNode->Extent = ParentUnitScale / ChildUnitScale
	const double SizeRatio = Params.UnitScale / ChildUnitScale;

	const FVector DebugBoxCenter = GetActorLocation() + NodeCenter - VirtualTraversal;
	const FVector CameraToNode = DebugBoxCenter - CurrentFrameOfReferenceLocation;

	return CurrentFrameOfReferenceLocation + CameraToNode * SizeRatio;
}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void AUniverseActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready) return;
	if (InNode->Data.ObjectId < 0) return;
	if (GalaxyPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Galaxy pool exhausted, consider increasing GalaxyPoolSize"));
		return;
	}
	AGalaxyActor* Galaxy = GalaxyPool.Pop();
	SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(Galaxy));
	Galaxy->ResetForSpawn();
	Galaxy->Universe = this;
	Galaxy->bAutoInitializeOnBeginPlay = false;

	// TypeId carries the tier index (0=Large, 1=Mid, 2=Small),
	// written during InsertParticleIntoOctree.
	const int32 TierIndex = FMath::Clamp(InNode->Data.TypeId, 0, 2);
	FParticleTierConfig* TierConfigs[] = { &CoarseTierConfig, &MidTierConfig, &SmallTierConfig };
	FParticleTierState* TierStates[] = { &CoarseTierState, &MidTierState, &SmallTierState };
	FParticleTierConfig& MatchedConfig = *TierConfigs[TierIndex];
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	// ObjectId is the slot index within that tier's buffer.
	const int32 SlotId = InNode->Data.ObjectId;
	const int32 FrontIdx = MatchedState.FrontIdx.load();
	const FNiagaraParticleBuffer& Front = MatchedState.Buffers[0][FrontIdx];
	const int32 SlotStart = SlotId * MatchedConfig.SlotCapacity;
	const int32 SlotCount = MatchedState.SlotCounts[SlotId];

	// Find the particle closest to the node center (node is octree-quantized).
	FVector ParticlePos = InNode->Center;
	float ParticleExtent = static_cast<float>(InNode->Extent);
	double BestDistSq = TNumericLimits<double>::Max();

	for (int32 i = 0; i < SlotCount; ++i)
	{
		const double DistSq = FVector::DistSquared(Front.Positions[SlotStart + i], InNode->Center);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			ParticlePos = Front.Positions[SlotStart + i];
			ParticleExtent = Front.Extents[SlotStart + i];
		}
	}

	// Start with the universe-level galaxy params (editor-tunable template),
	// then override per-instance fields (seed, color, rotation, scale).
	Galaxy->Params = this->GalaxyParams;

	// Derive UnitScale from the particle extent instead of the node extent.
	Galaxy->Params.UnitScale = (static_cast<double>(ParticleExtent) * this->Params.UnitScale) / Galaxy->Params.Extent;
	// MaxEntityScale is derived from MaxEntityScaleFraction in GalaxyActor::InitializeData.
	// No need to set it here — DeriveScaleRanges handles the cascade.

	Galaxy->SpeedScale = SpeedScale;
	Galaxy->Params.Seed = InNode->Data.ObjectId;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	FRandomStream RandStream(InNode->Data.ObjectId);
	Galaxy->Params.Rotation = RandStream.GetUnitVector().Rotation();

	// --- Deferred placement ---
	// Do NOT compute spawn location or set VirtualTraversal here. Parallax
	// will accumulate at the universe level during the N frames it takes for
	// async init to complete. FinalizeGalaxyPlacement (called from Tick on
	// the first frame after Ready) will compute the spawn position using that
	// frame's resolved VirtualTraversal and player position, ensuring zero
	// drift between the particle sprite and the galaxy actor.
	//
	// Cache the particle position so FinalizeGalaxyPlacement can re-derive
	// the spawn location later.
	Galaxy->PendingNodeCenter = ParticlePos;
	Galaxy->bPendingPlacement = true;

	// Keep the galaxy hidden until placement finalizes.
	// (Pool pre-warms with SetActorHiddenInGame(true) already.)

	UE_LOG(LogTemp, Log, TEXT("=== SpawnGalaxyFromPool (deferred placement) ==="));
	UE_LOG(LogTemp, Log, TEXT("  Node: center=(%.1f, %.1f, %.1f) extent=%.2f objId=%d tier=%s"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Data.ObjectId, *MatchedConfig.TierName);
	UE_LOG(LogTemp, Log, TEXT("  Particle: pos=(%.1f, %.1f, %.1f) extent=%.2f"),
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent);
	UE_LOG(LogTemp, Log, TEXT("  Galaxy: unitScale=%.2e extent=%.0f sizeRatio=%.2f seed=%d (placement deferred)"),
		Galaxy->Params.UnitScale, Galaxy->Params.Extent,
		Params.UnitScale / Galaxy->Params.UnitScale,
		Galaxy->Params.Seed);

	Galaxy->Initialize();
}

void AUniverseActor::FinalizeGalaxyPlacement(AGalaxyActor* Galaxy)
{
	// Called from Tick on the first frame after the galaxy's async init
	// completes. At this point:
	//   - ApplyParallaxOffset has already run, so VirtualTraversal and
	//     CurrentFrameOfReferenceLocation are resolved for this frame.
	//   - The galaxy has not been ticked yet, so no stale VT has accumulated.
	//
	// We compute the spawn position exactly as SpawnGalaxyFromPool used to,
	// but using the current frame's parallax state instead of the stale
	// state from N frames ago when the spawn was initiated.

	const FVector SpawnLoc = ComputeChildSpawnLocation(Galaxy->PendingNodeCenter, Galaxy->Params.UnitScale);
	Galaxy->SetActorLocation(SpawnLoc);
	Galaxy->LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	Galaxy->CurrentFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;

	// Initialize VirtualTraversal so that at spawn time the galaxy's particles
	// appear at exactly SpawnLoc in world space, matching where the Universe's
	// particle sprite is rendered.
	//
	// Rendered position = PlayerPos + (LocalPos - VT).
	// We want:           PlayerPos + (LocalPos - VT) = SpawnLoc + LocalPos
	// Solving:           VT_initial = PlayerPos - SpawnLoc
	Galaxy->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;

	// Make visible and clear the pending flag.
	Galaxy->SetActorHiddenInGame(false);
	Galaxy->bPendingPlacement = false;

	UE_LOG(LogTemp, Log, TEXT("=== FinalizeGalaxyPlacement ==="));
	UE_LOG(LogTemp, Log, TEXT("  Galaxy: spawnLoc=(%.1f, %.1f, %.1f) VT=(%.1f, %.1f, %.1f) playerPos=(%.1f, %.1f, %.1f)"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		Galaxy->VirtualTraversal.X, Galaxy->VirtualTraversal.Y, Galaxy->VirtualTraversal.Z,
		CurrentFrameOfReferenceLocation.X, CurrentFrameOfReferenceLocation.Y, CurrentFrameOfReferenceLocation.Z);
}

void AUniverseActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	TWeakObjectPtr<AGalaxyActor> WeakGalaxy;
	if (!SpawnedGalaxies.RemoveAndCopyValue(InNode, WeakGalaxy)) return;
	AGalaxyActor* Galaxy = WeakGalaxy.Get();
	if (!Galaxy) return;
	UE_LOG(LogTemp, Log, TEXT("Returning galaxy to pool for node ObjectId: %d"), InNode->Data.ObjectId);
	Galaxy->ResetForPool();
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakGalaxy]()
		{
			AGalaxyActor* AsyncGalaxy = WeakGalaxy.Get();
			if (!AsyncGalaxy) return;
			double StartTime = FPlatformTime::Seconds();
			AsyncGalaxy->Octree->bIsResetting.store(true);
			FPlatformProcess::Yield();  // Let in-flight octree ops see the flag and bail.
			AsyncGalaxy->Octree = MakeShared<FOctree>(AsyncGalaxy->Params.Extent);
			AsyncGalaxy->Octree->bIsResetting.store(false);
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakGalaxy]()
				{
					AUniverseActor* Self = WeakThis.Get();
					AGalaxyActor* InnerGalaxy = WeakGalaxy.Get();
					if (Self && InnerGalaxy) Self->GalaxyPool.Insert(InnerGalaxy, 0);
				});
		});
}
#pragma endregion

#pragma region Public Octree Queries
TArray<TSharedPtr<FOctreeNode>> AUniverseActor::GetNodesByScreenSpace(const FVector& InCenter, double InScreenSpaceThreshold, int32 InTypeId) const
{
	if (!Octree.IsValid()) return {};
	return Octree->GetNodesByScreenSpace(InCenter, InScreenSpaceThreshold, -1, -1, InTypeId);
}
#pragma endregion

#pragma region Spawn Range Scanning
void AUniverseActor::StartSpawnScanTimer()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(SpawnScanTimerHandle);
		World->GetTimerManager().SetTimer(SpawnScanTimerHandle, this, &AUniverseActor::UpdateSpawnRangeNodes, SpawnScanInterval, true);
	}
}

void AUniverseActor::StopSpawnScanTimer()
{
	if (UWorld* World = GetWorld()) World->GetTimerManager().ClearTimer(SpawnScanTimerHandle);
	TrackedSpawnNodes.Empty();
}

void AUniverseActor::UpdateSpawnRangeNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;
	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self) return;
			const TArray<TSharedPtr<FOctreeNode>> NearbyArray = Self->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold, -1);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray]()
				{
					AUniverseActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					// Store results for deferred processing in Tick, after
					// ApplyParallaxOffset has resolved the current frame's
					// player position and VirtualTraversal.
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
					InnerSelf->bSpawnScanInProgress.store(false);
				});
		});
}

void AUniverseActor::ProcessPendingScanResults()
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
			SpawnGalaxyFromPool(Node);
		}
		if (bDebugDrawSpawnNodes) DebugDrawSpawnNode(Node);
	}
	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedSpawnNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
		ReturnGalaxyToPool(Node);
	}
	TrackedSpawnNodes = MoveTemp(NearbySet);
}

void AUniverseActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const int32 SlotId = InNode->Data.ObjectId;
	const int32 ExtraCount = InNode->Data.AdditionalObjectIds.Num();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan ENTER — node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d slot=%d extras=%d scale=%.3f tier=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, SlotId, ExtraCount, InNode->Data.ScaleFactor, InNode->Data.TypeId);
	if (bLogSpawnEnterExitBuffers)
	{
		// Use TypeId (set by InsertParticleIntoOctree) to identify the source tier.
		const FParticleTierConfig* Config = nullptr;
		const FParticleTierState* State = nullptr;
		const TCHAR* TierLabel = TEXT("unknown");
		switch (InNode->Data.TypeId)
		{
		case 0: Config = &CoarseTierConfig; State = &CoarseTierState; TierLabel = TEXT("coarse"); break;
		case 1: Config = &MidTierConfig;    State = &MidTierState;    TierLabel = TEXT("mid");    break;
		case 2: Config = &SmallTierConfig;  State = &SmallTierState;  TierLabel = TEXT("small");  break;
		default: break;
		}
		if (Config && State && State->Buffers.Num() > 0)
		{
			const FNiagaraParticleBuffer& Front = State->Buffers[0][State->FrontIdx.load()];
			const int32 Start = SlotId * Config->SlotCapacity;
			int32 LiveCount = 0;
			for (int32 i = 0; i < Config->SlotCapacity; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log, TEXT("  %s slot %d: %d live particles of %d capacity"), TierLabel, SlotId, LiveCount, Config->SlotCapacity);
		}
	}
}

void AUniverseActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan EXIT  — node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d slot=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, InNode->Data.ObjectId);
}

void AUniverseActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	// Particle rendered world position = PlayerPos + LocalPos - VirtualTraversal.
	// Node->Center is the octree's quantization of LocalPos, so:
	//   NodeCenterWorld = GetActorLocation() + Node->Center - VirtualTraversal
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	const FVector BoxExtent(InNode->Extent);
	DrawDebugBox(World, NodeCenterWorld, BoxExtent, FColor::Green, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion
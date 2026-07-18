#include "UniverseActor.h"
#include "svo.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <StarSystemActor.h>
#include <NiagaraFunctionLibrary.h>
#include <DrawDebugHelpers.h>

#pragma region Constructor
AUniverseActor::AUniverseActor()
{
	bAutoInitializeOnBeginPlay = true;
	this->UniverseParams = FUniverseParamBounds::Generate(UniverseParamBounds, 666);
	SectorLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorLarge.NG_SectorLarge"));
	SectorMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorMid.NG_SectorMid"));
	SectorSmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorSmall.NG_SectorSmall"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGas.NG_SectorGas"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	Octree = MakeShared<FOctree>(UniverseParams.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
}
#pragma endregion

#pragma region Lifecycle
void AUniverseActor::Initialize()
{
	Super::Initialize();
}
#pragma endregion

#pragma region Initialization
void AUniverseActor::BeginPlay()
{
	Super::BeginPlay();
	if (bAutoInitializeOnBeginPlay) Initialize();
}

void AUniverseActor::ConfigureCell(FIntVector InCellCoord)
{
	CellCoord = InCellCoord;
	const double FullCellSize = 2.0 * UniverseParams.Extent;
	CellOrigin = FVector(
		CellCoord.X * FullCellSize,
		CellCoord.Y * FullCellSize,
		CellCoord.Z * FullCellSize
	);
	SetActorLocation(CellOrigin);
	Octree = MakeShared<FOctree>(UniverseParams.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
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
			for (int i = 0; i < Self->GalaxyPoolSize; i++) {
				AGalaxyActor* Galaxy = Self->GetWorld()->SpawnActor<AGalaxyActor>(
					Self->GalaxyActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				Galaxy->Universe = Self;
				Galaxy->SetActorHiddenInGame(true);
				Self->GalaxyPool.Add(Galaxy);
			}
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
}

void AUniverseActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();
	UniverseGenerator.Params = UniverseParams;
	UniverseGenerator.Initialize();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeData took: %.3f seconds"),
		FPlatformTime::Seconds() - StartTime);
}

void AUniverseActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::InitializeTier(Ctx, LargeTierConfig, LargeTierState, TierNiagaraComponents);
	FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Tier System - BuildTierConfigs
void AUniverseActor::BuildTierConfigs()
{
	// Derive MinScale/MaxScale for all tiers from MaxEntityScale + depth spacing.
	// Must be called before any generate callback reads scale ranges.
	UniverseParams.DeriveScaleRanges();

	// Large tier
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GridDepth = UniverseParams.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = UniverseParams.LargeTier.NeighborhoodRadius;
	LargeTierConfig.SlotCapacity = UniverseParams.LargeTier.SlotCapacity;
	LargeTierConfig.NiagaraAssets = { SectorLargeCloud, SectorGasCloud };
	LargeTierConfig.bWantRotations = { true, false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, LargeTierConfig.GridDepth);
		UniverseGenerator.GenerateLargeTierNode(Coord, SlotIndex, *Buffers[0], *Buffers[1], NodeCenter, LargeTierState.SlotCounts[SlotIndex]);
		};

	// Mid tier
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.TierIndex = 1;
	MidTierConfig.GridDepth = UniverseParams.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = UniverseParams.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = UniverseParams.MidTier.SlotCapacity;
	MidTierConfig.NiagaraAssets = { SectorMidCloud };
	MidTierConfig.bWantRotations = { true };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);
		UniverseGenerator.GenerateMidTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, MidTierState.SlotCounts[SlotIndex]);
		};

	// Small tier
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.GridDepth = UniverseParams.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = UniverseParams.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = UniverseParams.SmallTier.SlotCapacity;
	SmallTierConfig.NiagaraAssets = { SectorSmallCloud };
	SmallTierConfig.bWantRotations = { true };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, SmallTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(SmallTierConfig.GridDepth);
		UniverseGenerator.GenerateSmallTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, SmallTierState.SlotCounts[SlotIndex]);
		};

	// Applied to each tier after its GridDepth/NeighborhoodRadius are set.
	// Captures Config by ref - safe since Config outlives all lambda calls.
	//
	// Bounds derivation (actor-relative; the actor is pegged to the player).
	// Rendered offset = cellLocal + lattice + (NCenter - VT), where
	//   |cellLocal| + |lattice| <= (2R+1) * CellHalfExtent   (neighborhood half-span)
	//   |NCenter - VT|          <= CellHalfExtent            (VT resides in the center cell)
	// so the tight half-bound is (2R+2) * CellHalfExtent plus one particle
	// radius. We provision 2 * (2R+1),
	// which exceeds the tight bound for all R >= 0 and leaves slack for
	// transition frames. This is the SHARED convention - GalaxyActor's
	// MakeBounds must match it; a bare (2R+1) bound clips edge cells by up to
	// one half-cell when VT sits near a cell boundary.
	auto MakeBoundsLambda = [this](const FParticleTierConfig& Cfg) -> TFunction<FBox()>
		{
			return [this, &Cfg]() -> FBox
				{
					const double BoundsExtent = (2 * Cfg.NeighborhoodRadius + 1) * GetGridCellExtent(Cfg.GridDepth) * 2.0;
					return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
				};
		};
	LargeTierConfig.ComputeBounds = MakeBoundsLambda(LargeTierConfig);
	MidTierConfig.ComputeBounds = MakeBoundsLambda(MidTierConfig);
	SmallTierConfig.ComputeBounds = MakeBoundsLambda(SmallTierConfig);
}
#pragma endregion

#pragma region Tier System - BuildStreamingContext
FTierStreamingContext AUniverseActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = UniverseParams.Extent;
	Ctx.UnitScale = 1.0;  // Universe extents are already in octree-local units.
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = bRebaseInProgress.load();
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = TEXT("Universe");
	Ctx.ParentSeed = UniverseParams.Seed;
	Ctx.GetLatestVT = [this] { return ReadLatestVT(); };
	Ctx.GetLiveState = [this] { return InitializationState; };
	return Ctx;
}
#pragma endregion

#pragma region Parallax
void AUniverseActor::ApplyParallaxOffset(const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Universe::ApplyParallaxOffset");
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double Ratio = (UniverseParams.UnitScale > 0.0) ? (SpeedScale / UniverseParams.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(InPlayerPos);

	// Publish EVERY frame -- even below the push threshold -- so an in-flight full
	// push that completes this frame still composites against current VT.
	PublishLatestVT(VirtualTraversal);

	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		LastPushedVirtualTraversal = VirtualTraversal;
		// GAME THREAD, deliberately. This is one FVector uniform per component --
		// trivially cheap -- and SetVariable* writes the Niagara parameter store
		// with no internal lock, so racing the GT's own Niagara tick trips
		// FMTAccessDetector. The FScopeTryLock inside still skips any tier whose
		// transition commit holds PushCS, so the GT never stalls behind a
		// worker's array upload.
		FTierStreamingSystem::PushTierVT({ &LargeTierState, &MidTierState, &SmallTierState }, [this] { return ReadLatestVT(); });
	}
}

// Coalesced, single-flight per-frame push. Producers raise bPushDirty; at most one
// worker drains, re-reading the freshest VT under each tier's PushCS every pass, so
// out-of-order scheduling is harmless -- the last write always wins with current VT.
void AUniverseActor::SchedulePush()
{
	bPushDirty.store(true, std::memory_order_release);
	if (bPushWorkerLive.exchange(true)) return;
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			for (;;)
			{
				bPushDirty.store(false, std::memory_order_relaxed);
				FTierStreamingSystem::PushTierVT(
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
#pragma endregion

#pragma region Tick
void AUniverseActor::Tick(float DeltaTime)
{
	SVO_GT_SCOPE("Universe::Tick");
	{
		SVO_GT_SCOPE("Universe::SuperTick");
		Super::Tick(DeltaTime);
	}
	FVector CurrentPlayerPos;
	if (InitializationState == ELifecycleState::Ready && GetPlayerLocation(GetWorld(), CurrentPlayerPos))
	{
		ApplyParallaxOffset(CurrentPlayerPos);
	}

	// Process any pending spawn-scan results now that VirtualTraversal and
	// CurrentFrameOfReferenceLocation are resolved for this frame.
	ProcessPendingScanResults();

	// Drive all active galaxies with the already-resolved player position.
	// Galaxies have UE tick disabled; this is their only per-frame entry point.
	// Each galaxy cascades down to its own star systems via TickFromParent.
	{
		SVO_GT_SCOPE("Universe::GalaxyCascade");
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
	}

	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, LargeTierConfig, LargeTierState);
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	CheckOctreeBounds();

	// Single hierarchical scan; no per-level timers.
	// Must run after the full tick cascade so all VTs are resolved.
	DetermineAndDispatchScan();

	// Emit the once-per-second game-thread profile summary. Root tick only.
	SVO_GT_FLUSH();
}
#pragma endregion

#pragma region Shutdown
void AUniverseActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;

	// Signal any in-flight galaxy initializations to abort, then clear tracking.
	for (auto& Pair : SpawnedGalaxies)
	{
		if (AGalaxyActor* Galaxy = Pair.Value.Get())
			Galaxy->InitializationState = ELifecycleState::Pooling;
	}
	SpawnedGalaxies.Empty();
	GalaxyPool.Empty();

	// Drain any in-flight pushes before destroying the components they may touch.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		FTierStreamingSystem::BeginShutdownDrain(*Tier);

	// Mirror ResetForPool: also wait out an in-flight boundary-cross task -
	// it writes Buffers/SlotEntries/CellCache (actor members) and would keep
	// doing so through teardown. Safe on the GT: the transition task has no
	// game-thread rendezvous. The async INIT chain is deliberately NOT
	// waited here - it rendezvouses with the GT (would deadlock a GT wait);
	// it aborts on the Pooling state set above, and actor memory outlives
	// EndPlay until GC.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		while (Tier->bUpdateInProgress.load())
			FPlatformProcess::Sleep(0.0005f);
	}

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
	}
	TierNiagaraComponents.Empty();

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void AUniverseActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Universe::SpawnGalaxyFromPool");
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready) return;
	if (InNode->Data.ParticleIndex < 0) return;
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
	FParticleTierState* TierStates[] = { &LargeTierState, &MidTierState, &SmallTierState };
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	// ParticleIndex is the absolute buffer index. Direct lookup - no slot math needed.
	// SINGLE-BUFFER READ GUARD: the transition task overwrites entering
	// slots in place, so only read the CPU arrays while no task is in
	// flight. On the GT this check is race-free (the flag is set on the GT
	// before the task spawns), and if a transition IS in flight the matched
	// node's slot may be mid-rewrite anyway - the octree fallback (node
	// center/extent) is the correct answer in that case.
	FVector ParticlePos = InNode->Center;
	float ParticleExtent = static_cast<float>(InNode->Extent);
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	if (AbsIdx >= 0 && MatchedState.Buffers.Num() > 0 &&
		!MatchedState.bUpdateInProgress.load())
	{
		const FNiagaraParticleBuffer& Buf = MatchedState.Buffers[0];
		ParticlePos = Buf.Positions[AbsIdx];
		ParticleExtent = Buf.Extents[AbsIdx];
	}

	// Start with the universe-level galaxy params (editor-tunable template),
	// then override per-instance fields (seed, color, rotation, scale).
	Galaxy->Params = FGalaxyParamBounds::Generate(GalaxyParamBounds, InNode->Data.Seed);

	// INVERTED DERIVATION: UnitScale is the per-layer design constant
	// (carried in from the GalaxyParams template copy above); the galaxy's
	// LOCAL Extent is what varies - derived from the particle's real size.
	// Real galaxy size therefore expresses itself as model size and star
	// COUNT, while every authored real-unit content size converts through
	// the same constant to identical local (= perceived) sizes in every
	// galaxy. (The inverse - a constant Extent with derived UnitScale -
	// would make star sprites scale inversely with host galaxy size.)
	{
		const double DerivedExtent =
			(static_cast<double>(ParticleExtent) * this->UniverseParams.UnitScale)
			/ Galaxy->Params.UnitScale;
		Galaxy->Params.Extent = FMath::Clamp(DerivedExtent,
			Galaxy->Params.MinDerivedExtent, Galaxy->Params.MaxDerivedExtent);
		if (Galaxy->Params.Extent != DerivedExtent)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("AUniverseActor::SpawnGalaxy - derived extent %.3e clamped to %.3e; ")
				TEXT("retune GalaxyParams.UnitScale (layer constant) or the clamp bounds."),
				DerivedExtent, Galaxy->Params.Extent);
		}
	}
	// MaxEntityScale is derived from MaxEntityScaleFraction in GalaxyActor::InitializeData.
	// No need to set it here - DeriveScaleRanges handles the cascade.

	Galaxy->SpeedScale = SpeedScale;
	// Seed is the deterministic hierarchical seed composed from
	// (UniverseSeed, GridCoord, GenerationIndex) during octree insertion.
	Galaxy->Params.Seed = InNode->Data.Seed;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	FRandomStream RandStream(InNode->Data.Seed);
	Galaxy->Params.Rotation = FRotator(
		RandStream.FRandRange(-180.0f, 180.0f),
		RandStream.FRandRange(-180.0f, 180.0f),
		RandStream.FRandRange(-180.0f, 180.0f));

	Galaxy->bPendingPlacement = true;
	Galaxy->PendingNodeCenter = ParticlePos;
	Galaxy->SetActorHiddenInGame(true);
	Galaxy->Initialize();

	UE_LOG(LogTemp, Log,
		TEXT("AUniverseActor::SpawnGalaxyFromPool - pool=%d node=(%.1f,%.1f,%.1f) extent=%.1f "
			"particlePos=(%.1f,%.1f,%.1f) particleExtent=%.3f unitScale(const)=%.6e derivedExtent=%.6e seed=%d"),
		GalaxyPool.Num(),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent,
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent,
		Galaxy->Params.UnitScale, Galaxy->Params.Extent, Galaxy->Params.Seed);
}

void AUniverseActor::FinalizeGalaxyPlacement(AGalaxyActor* Galaxy)
{
	SVO_GT_SCOPE("Universe::FinalizeGalaxyPlacement");
	if (!Galaxy || !Galaxy->bPendingPlacement) return;

	const FVector SpawnLoc = ComputeChildSpawnLocation(Galaxy->PendingNodeCenter, Galaxy->Params.UnitScale);
	Galaxy->SetActorLocation(SpawnLoc);
	Galaxy->SetActorHiddenInGame(false);

	Galaxy->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;
	Galaxy->LastPushedVirtualTraversal = Galaxy->VirtualTraversal;
	Galaxy->LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	Galaxy->CurrentFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;

	Galaxy->bPendingPlacement = false;

	UE_LOG(LogTemp, Log, TEXT("=== FinalizeGalaxyPlacement ==="));
	UE_LOG(LogTemp, Log, TEXT("  Galaxy: spawnLoc=(%.1f, %.1f, %.1f) VT=(%.1f, %.1f, %.1f) playerPos=(%.1f, %.1f, %.1f)"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		Galaxy->VirtualTraversal.X, Galaxy->VirtualTraversal.Y, Galaxy->VirtualTraversal.Z,
		CurrentFrameOfReferenceLocation.X, CurrentFrameOfReferenceLocation.Y, CurrentFrameOfReferenceLocation.Z);
}

void AUniverseActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Universe::ReturnGalaxyToPool");
	if (!InNode.IsValid()) return;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	TWeakObjectPtr<AGalaxyActor> WeakGalaxy;
	if (!SpawnedGalaxies.RemoveAndCopyValue(InNode, WeakGalaxy)) return;
	AGalaxyActor* Galaxy = WeakGalaxy.Get();
	if (!Galaxy) return;
	UE_LOG(LogTemp, Log, TEXT("Returning galaxy to pool for node seed: %d"), InNode->Data.Seed);

	// Abort signal for an in-flight async init chain - checked between
	// phases and live (via GetLiveState) inside InitializeTier.
	Galaxy->InitializationState = ELifecycleState::Pooling;

	if (!Galaxy->bInitInProgress.load())
	{
		// FAST PATH: no init chain in flight (the normal case - the galaxy
		// finished initializing long before it left the spawn threshold).
		// Race-free on the GT: the flag is raised on the GT in Initialize()
		// before the chain dispatches, so a false read here means the chain
		// fully exited or never ran.
		Galaxy->ResetForPool();
		FinishGalaxyPoolReturn(WeakGalaxy);
		return;
	}

	// DEFERRED RETURN: the init chain still owns the tier buffers - its
	// generation ParallelFors write Buffers/SlotCounts and never raise
	// bUpdateInProgress, so ResetForPool's transition-drain would free
	// live arrays under the workers (use-after-free). We also cannot wait
	// for the chain on the GAME THREAD: it rendezvouses with the GT
	// (Niagara component spawns), so a GT spin here deadlocks. Park a
	// worker to wait it out, then finish teardown through the normal GT
	// path. The galaxy cannot be re-spawned meanwhile - it re-enters
	// GalaxyPool only at the end of FinishGalaxyPoolReturn.
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakGalaxy]()
		{
			while (true)
			{
				AGalaxyActor* G = WeakGalaxy.Get();
				if (!G) return;
				if (!G->bInitInProgress.load()) break;
				FPlatformProcess::Sleep(0.001f);
			}
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakGalaxy]()
				{
					AUniverseActor* Self = WeakThis.Get();
					AGalaxyActor* G = WeakGalaxy.Get();
					if (!Self || !G) return;
					G->ResetForPool();
					Self->FinishGalaxyPoolReturn(WeakGalaxy);
				});
		});
}

void AUniverseActor::FinishGalaxyPoolReturn(TWeakObjectPtr<AGalaxyActor> WeakGalaxy)
{
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakGalaxy]()
		{
			AGalaxyActor* AsyncGalaxy = WeakGalaxy.Get();
			if (!AsyncGalaxy) return;
			double StartTime = FPlatformTime::Seconds();
			AsyncGalaxy->Octree->bIsResetting.store(true);
			FPlatformProcess::Yield();  // Let in-flight octree ops see the flag and bail.
			// Build the fresh tree in a LOCAL; the MEMBER swap happens on the
			// game thread below. The old tree keeps bIsResetting raised and
			// dies when its last shared ref drops.
			TSharedPtr<FOctree> FreshTree = MakeShared<FOctree>(AsyncGalaxy->Params.Extent);
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakGalaxy, FreshTree]()
				{
					AUniverseActor* Self = WeakThis.Get();
					AGalaxyActor* InnerGalaxy = WeakGalaxy.Get();
					if (!InnerGalaxy) return;
					InnerGalaxy->Octree = FreshTree;   // swap on game thread
					if (Self) Self->GalaxyPool.Insert(InnerGalaxy, 0);
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
void AUniverseActor::RequestScan()
{
	SVO_GT_SCOPE("Universe::RequestScan");
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;
	// Snapshot the tree ref ON THE GAME THREAD (one atomic ref-count bump).
	// The rebase completion swaps this->Octree on the GT; reading the member
	// from the worker below would be a torn read of a TSharedPtr. The snapshot
	// also keeps the pre-rebase tree alive until this scan finishes.
	TSharedPtr<FOctree> TreeSnapshot = Octree;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos, TreeSnapshot]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self || !TreeSnapshot.IsValid()) return;
			const TArray<TSharedPtr<FOctreeNode>> NearbyArray = TreeSnapshot->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray, TreeSnapshot]()
				{
					AUniverseActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->bSpawnScanInProgress.store(false);
					// A rebase swapped the tree while this scan was in
					// flight: these nodes belong to the retired tree and
					// would diff against the remapped tracking as all-new /
					// all-exited. Drop them; the next interval rescans the
					// live tree.
					if (InnerSelf->Octree != TreeSnapshot) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
				});
		});
}

void AUniverseActor::DetermineAndDispatchScan()
{
	SVO_GT_SCOPE("Universe::DetermineAndDispatchScan");
	if (InitializationState != ELifecycleState::Ready) return;

	// Walk deepest-first: star systems -> galaxies -> universe.
	// Only one level dispatches a scan per tick.
	for (auto& GalaxyPair : SpawnedGalaxies)
	{
		AGalaxyActor* Galaxy = GalaxyPair.Value.Get();
		if (!Galaxy || Galaxy->InitializationState != ELifecycleState::Ready)
			continue;

		for (auto& SystemPair : Galaxy->SpawnedStarSystems)
		{
			AStarSystemActor* System = SystemPair.Value.Get();
			if (!System || System->InitializationState != ELifecycleState::Ready)
				continue;

			if (System->IsPlayerInsideBounds())
			{
				System->RequestScan();
				return;  // Short-circuit: deepest level found
			}
		}

		if (Galaxy->IsPlayerInsideBounds())
		{
			Galaxy->RequestScan();
			return;  // Short-circuit: galaxy level
		}
	}

	// Player isn't inside any child - scan at universe level
	RequestScan();
}

void AUniverseActor::ProcessPendingScanResults()
{
	SVO_GT_SCOPE("Universe::ProcessPendingScanResults");
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
	const int32 Seed = InNode->Data.Seed;
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan ENTER - node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d seed=%d bufIdx=%d scale=%.3f tier=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, Seed, AbsIdx, InNode->Data.ScaleFactor, InNode->Data.TypeId);
	if (bLogSpawnEnterExitBuffers)
	{
		// Use TypeId (set by InsertParticleIntoOctree) to identify the source tier.
		const FParticleTierConfig* Config = nullptr;
		const FParticleTierState* State = nullptr;
		const TCHAR* TierLabel = TEXT("unknown");
		switch (InNode->Data.TypeId)
		{
		case 0: Config = &LargeTierConfig; State = &LargeTierState; TierLabel = TEXT("coarse"); break;
		case 1: Config = &MidTierConfig;    State = &MidTierState;    TierLabel = TEXT("mid");    break;
		case 2: Config = &SmallTierConfig;  State = &SmallTierState;  TierLabel = TEXT("small");  break;
		default: break;
		}
		if (Config && State && State->Buffers.Num() > 0 && AbsIdx >= 0 &&
			!State->bUpdateInProgress.load()) // single-buffer read guard (GT)
		{
			const int32 SlotId = AbsIdx / Config->SlotCapacity;
			const FNiagaraParticleBuffer& Buf = State->Buffers[0];
			const int32 Start = SlotId * Config->SlotCapacity;
			int32 LiveCount = 0;
			for (int32 i = 0; i < Config->SlotCapacity; ++i)
			{
				if (Buf.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log, TEXT("  %s slot %d: %d live particles of %d capacity"), TierLabel, SlotId, LiveCount, Config->SlotCapacity);
		}
	}
}

void AUniverseActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan EXIT  - node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d seed=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, InNode->Data.Seed);
}

void AUniverseActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	const FVector BoxExtent(InNode->Extent);
	DrawDebugBox(World, NodeCenterWorld, BoxExtent, FColor::Green, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion

#pragma region Octree Bounds Check
void AUniverseActor::CheckOctreeBounds()
{
	SVO_GT_SCOPE("Universe::CheckOctreeBounds");
	if (!Octree.IsValid() || bRebaseInProgress.load()) return;

	// Measure against the tree's ACTUAL center, not origin - after a rebase the
	// root is centered on the player, so an origin-relative test would re-fire
	// every tick.
	const FVector TreeCenter = Octree->Root.IsValid() ? Octree->Root->Center : FVector::ZeroVector;
	const double  Margin = Octree->Extent * 0.75;
	const FVector Delta = VirtualTraversal - TreeCenter;

	if (FMath::Abs(Delta.X) <= Margin &&
		FMath::Abs(Delta.Y) <= Margin &&
		FMath::Abs(Delta.Z) <= Margin)
		return;

	// Don't rebase while any tier owns its back buffer / state.
	if (LargeTierState.bUpdateInProgress.load() ||
		MidTierState.bUpdateInProgress.load() ||
		SmallTierState.bUpdateInProgress.load())
		return;

	bRebaseInProgress.store(true);
	const FVector RebaseOrigin = VirtualTraversal;
	const double  TreeExtent = UniverseParams.Extent * PersistentTreeMultiplier;

	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, RebaseOrigin, TreeExtent]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self) return;

			UE_LOG(LogTemp, Warning, TEXT("AUniverseActor::RebaseOctree -> (%.1f, %.1f, %.1f)"),
				RebaseOrigin.X, RebaseOrigin.Y, RebaseOrigin.Z);

			// Build + populate the new tree in a LOCAL ptr off the game thread, then
			// hand the finished tree back for the actual swap. This removes the
			// torn-read race the original had (it assigned Self->Octree directly on
			// the background thread while game-thread readers could be mid-copy).
			TSharedPtr<FOctree> NewTree = MakeShared<FOctree>(TreeExtent, RebaseOrigin);

			FTierStreamingContext Ctx = Self->BuildStreamingContext();
			Ctx.Octree = NewTree;   // insert into the new tree, not the live one

			// Tiers are guaranteed idle for this whole task: CheckOctreeBounds
			// refused to start the rebase while any tier had a transition in
			// flight (checked on the GT), and UpdateTier gates on
			// bRebaseInProgress, so no new transition can start until the
			// swap below completes. Safe to read every tier buffer wholesale.
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->LargeTierConfig,
				Self->LargeTierState);
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->MidTierConfig,
				Self->MidTierState);
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->SmallTierConfig,
				Self->SmallTierState);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NewTree]()
				{
					AUniverseActor* S = WeakThis.Get();
					if (!S) return;

					// REMAP live spawn bookkeeping onto the new tree BEFORE
					// clearing bRebaseInProgress. SpawnedGalaxies and
					// TrackedSpawnNodes are keyed on node IDENTITY, and every
					// node instance just changed - without this remap the
					// first post-rebase scan sees all-new pointers, despawns
					// EVERY live galaxy and respawns it from the pool (full
					// re-init hitch + 2x pool headroom on the worst frame).
					// Seed survives the rebase (ComposeSeed is
					// deterministic), so counterparts are matched by the
					// node-captured particle position + seed.
					auto FindCounterpart = [&NewTree](const TSharedPtr<FOctreeNode>& Old) -> TSharedPtr<FOctreeNode>
						{
							if (!Old.IsValid()) return nullptr;
							// Descend by the CAPTURED particle position (set
							// at insert, immutable - no tier-buffer read
							// needed), not the old node center: the new
							// lattice is offset by the rebase origin, so the
							// old center and the particle it quantized can
							// straddle a new-cell boundary and land in
							// different nodes. Center fallback only for
							// legacy nodes without captured data.
							const FVector P = (Old->Data.ParticleExtent > 0.0f)
								? Old->Data.ParticlePosition
								: Old->Center;
							TSharedPtr<FOctreeNode> Candidate = NewTree->FindNodeAtPosition(P, Old->Depth);
							return (Candidate.IsValid() && Candidate->Data.Seed == Old->Data.Seed)
								? Candidate : nullptr;
						};

					TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> RemappedGalaxies;
					RemappedGalaxies.Reserve(S->SpawnedGalaxies.Num());
					for (auto& Pair : S->SpawnedGalaxies)
					{
						TSharedPtr<FOctreeNode> NewNode = FindCounterpart(Pair.Key);
						// Fall back to the old key on a miss: that one galaxy
						// despawns on the next scan (pre-remap behavior for a
						// single entry), nothing worse.
						RemappedGalaxies.Add(NewNode.IsValid() ? NewNode : Pair.Key, Pair.Value);
					}

					TSet<TSharedPtr<FOctreeNode>> RemappedTracked;
					RemappedTracked.Reserve(S->TrackedSpawnNodes.Num());
					for (const TSharedPtr<FOctreeNode>& Old : S->TrackedSpawnNodes)
					{
						TSharedPtr<FOctreeNode> NewNode = FindCounterpart(Old);
						RemappedTracked.Add(NewNode.IsValid() ? NewNode : Old);
					}

					S->SpawnedGalaxies = MoveTemp(RemappedGalaxies);
					S->TrackedSpawnNodes = MoveTemp(RemappedTracked);

					// Drop results from any scan snapshotted against the old
					// tree - diffing old-tree nodes against the remapped set
					// would re-introduce the churn this remap removes.
					// (RequestScan's completion also drops stale-tree results
					// for a scan still in flight.)
					S->bHasPendingScanResults = false;
					S->PendingScanResults.Empty();

					S->Octree = NewTree;                  // swap on game thread
					S->bRebaseInProgress.store(false);
				});
		});
}
#pragma endregion
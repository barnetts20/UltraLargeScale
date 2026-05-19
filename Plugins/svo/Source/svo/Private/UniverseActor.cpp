#pragma region Includes/ForwardDec
#include "UniverseActor.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <StarSystemActor.h>
#include <NiagaraFunctionLibrary.h>
#include <DrawDebugHelpers.h>
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
	FVector PlayerPos;
	if (AProceduralSpaceActor::GetPlayerLocation(GetWorld(), PlayerPos))
	{
		LastFrameOfReferenceLocation = PlayerPos;
		CurrentFrameOfReferenceLocation = PlayerPos;
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

			// No timer start needed — DetermineAndDispatchScan runs from Tick.
		});
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
	const double FullCellSize = 2.0 * Params.Extent;
	CellOrigin = FVector(
		CellCoord.X * FullCellSize,
		CellCoord.Y * FullCellSize,
		CellCoord.Z * FullCellSize
	);
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
	UniverseGenerator.Params = Params;
	UniverseGenerator.Initialize();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeData took: %.3f seconds"),
		FPlatformTime::Seconds() - StartTime);
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

#pragma region Tier System - BuildTierConfigs
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
#pragma endregion

#pragma region Tier System - BuildStreamingContext
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
#pragma endregion

#pragma region Parallax
void AUniverseActor::ApplyParallaxOffset()
{
	if (InitializationState != ELifecycleState::Ready) return;

	FVector CurrentPlayerPos;
	if (!AProceduralSpaceActor::GetPlayerLocation(GetWorld(), CurrentPlayerPos)) return;

	const FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;

	const double Ratio = (Params.UnitScale > 0.0) ? (SpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(CurrentPlayerPos);

	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	const bool bNeedsPush = (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold);

	// Universe Niagara components are attached (not absolute-positioned),
	// so they follow the actor via SetActorLocation above. Only enter
	// the per-component loop when positions actually need re-pushing.
	if (bNeedsPush)
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
	ApplyParallaxOffset();

	// Process any pending spawn-scan results now that VirtualTraversal and
	// CurrentFrameOfReferenceLocation are resolved for this frame.
	ProcessPendingScanResults();

	// Drive all active galaxies with the already-resolved player position.
	// Galaxies have UE tick disabled; this is their only per-frame entry point.
	// Each galaxy cascades down to its own star systems via TickFromParent.
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

	// Single hierarchical scan — replaces all per-level timers.
	// Must run after the full tick cascade so all VTs are resolved.
	DetermineAndDispatchScan();
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

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

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
	// So size_galaxy / size_node = ParentUnitScale / ChildUnitScale.
	const double SizeRatio = Params.UnitScale / ChildUnitScale;

	// World-space position of the node's sprite:
	const FVector RenderedPos = GetActorLocation() + NodeCenter - VirtualTraversal;

	// Vector from camera to the sprite:
	const FVector CameraToNode = RenderedPos - CurrentFrameOfReferenceLocation;

	// Scale by the size ratio → galaxy spawn position.
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
	Galaxy->Params.Rotation = FRotator(
		RandStream.FRandRange(-180.0f, 180.0f),
		RandStream.FRandRange(-180.0f, 180.0f),
		RandStream.FRandRange(-180.0f, 180.0f));

	Galaxy->bPendingPlacement = true;
	Galaxy->PendingNodeCenter = ParticlePos;
	Galaxy->SetActorHiddenInGame(true);
	Galaxy->Initialize();

	UE_LOG(LogTemp, Log,
		TEXT("AUniverseActor::SpawnGalaxyFromPool — pool=%d node=(%.1f,%.1f,%.1f) extent=%.1f "
			"particlePos=(%.1f,%.1f,%.1f) particleExtent=%.3f derivedUnitScale=%.6e seed=%d"),
		GalaxyPool.Num(),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent,
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent,
		Galaxy->Params.UnitScale, Galaxy->Params.Seed);
}

void AUniverseActor::FinalizeGalaxyPlacement(AGalaxyActor* Galaxy)
{
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
void AUniverseActor::RequestScan()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

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
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
					InnerSelf->bSpawnScanInProgress.store(false);
				});
		});
}

void AUniverseActor::DetermineAndDispatchScan()
{
	if (InitializationState != ELifecycleState::Ready) return;

	// Walk deepest-first: star systems → galaxies → universe.
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

	// Player isn't inside any child — scan at universe level
	RequestScan();
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
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	const FVector BoxExtent(InNode->Extent);
	DrawDebugBox(World, NodeCenterWorld, BoxExtent, FColor::Green, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion

#pragma region Grid Coord Helpers
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
#pragma endregion

#pragma region Octree Bounds Check
void AUniverseActor::CheckOctreeBounds()
{
	if (!Octree.IsValid() || bRebaseInProgress.load()) return;

	const double OctreeExtent = Octree->Extent;
	const double Threshold = OctreeExtent * 0.25;

	if (FMath::Abs(VirtualTraversal.X) > OctreeExtent - Threshold ||
		FMath::Abs(VirtualTraversal.Y) > OctreeExtent - Threshold ||
		FMath::Abs(VirtualTraversal.Z) > OctreeExtent - Threshold)
	{
		UE_LOG(LogTemp, Warning, TEXT("AUniverseActor::CheckOctreeBounds — VT=(%.0f,%.0f,%.0f) approaching octree bounds (extent=%.0f), rebase needed"),
			VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z, OctreeExtent);
	}
}
#pragma endregion
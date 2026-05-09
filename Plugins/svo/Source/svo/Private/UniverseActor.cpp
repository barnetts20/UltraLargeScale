#pragma region Includes/ForwardDec
#include "UniverseActor.h"
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

			double TotalDuration = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTemp, Log, TEXT("%s::Initialize total duration: %.3f seconds"), *Self->GetClass()->GetName(), TotalDuration);

			// TimerManager must be touched on the game thread.
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
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			for (int32 i = 0; i < GalaxyPoolSize; i++)
			{
				AGalaxyActor* Galaxy = GetWorld()->SpawnActor<AGalaxyActor>(GalaxyActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				Galaxy->bAutoInitializeOnBeginPlay = false;
				Galaxy->Universe = this;
				Galaxy->SetActorHiddenInGame(true);
				GalaxyPool.Add(Galaxy);
			}
			UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeChildPool — pre-warmed %d galaxy actors"), GalaxyPoolSize);
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
	InitializeTier(CoarseTierConfig, CoarseTierState);
	InitializeTier(MidTierConfig, MidTierState);
	InitializeTier(SmallTierConfig, SmallTierState);
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Unified Particle Tier System
FIntVector AUniverseActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FIntVector(FMath::FloorToInt32(InPos.X / CellSize + 0.5), FMath::FloorToInt32(InPos.Y / CellSize + 0.5), FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
}

FVector AUniverseActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FVector(static_cast<double>(InCoord.X) * CellSize, static_cast<double>(InCoord.Y) * CellSize, static_cast<double>(InCoord.Z) * CellSize);
}

double AUniverseActor::GetGridCellExtent(int32 InGridDepth) const
{
	return (Params.Extent * GridExtentMultiplier) / (1 << (InGridDepth + 1));
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

void AUniverseActor::InitializeTier(FParticleTierConfig& Config, FParticleTierState& State)
{
	double StartTime = FPlatformTime::Seconds();
	const int32 Side = 2 * Config.NeighborhoodRadius + 1;
	const int32 TotalSlots = Side * Side * Side;
	const int32 NumBuffers = Config.NiagaraAssets.Num();
	// Allocate double-buffered particle data — one pair per Niagara asset.
	State.Buffers.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b].SetNum(2);
		State.Buffers[b][0].Allocate(TotalSlots, Config.SlotCapacity, Config.bWantRotations[b]);
		State.Buffers[b][1].Allocate(TotalSlots, Config.SlotCapacity, Config.bWantRotations[b]);
	}
	State.FrontIdx.store(0);
	State.SlotCounts.SetNumZeroed(TotalSlots);
	State.FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		State.FreeSlots.Add(i);
	}
	State.ActiveSlots.Empty();
	// VirtualTraversal is 0 at init — player starts at virtual coord (0,0,0).
	const FVector LocalPlayerPos = VirtualTraversal;
	State.CenterCoord = PositionToGridCoord(LocalPlayerPos, Config.GridDepth);
	// Serial slot allocation → parallel generation → serial octree insert.
	TArray<TPair<FIntVector, int32>> ToGenerate;
	ToGenerate.Reserve(TotalSlots);
	for (int32 dz = -Config.NeighborhoodRadius; dz <= Config.NeighborhoodRadius; ++dz)
	{
		for (int32 dy = -Config.NeighborhoodRadius; dy <= Config.NeighborhoodRadius; ++dy)
		{
			for (int32 dx = -Config.NeighborhoodRadius; dx <= Config.NeighborhoodRadius; ++dx)
			{
				const FIntVector NeighborCoord = State.CenterCoord + FIntVector(dx, dy, dz);
				const int32 SlotIndex = State.FreeSlots.Pop();
				State.ActiveSlots.Add(NeighborCoord, FSlotEntry{ SlotIndex, {} });
				ToGenerate.Emplace(NeighborCoord, SlotIndex);
			}
		}
	}
	// Build buffer pointer arrays for the front buffer (index 0 at init).
	TArray<TArray<FNiagaraParticleBuffer*>> PerSlotBufferPtrs;
	PerSlotBufferPtrs.SetNum(ToGenerate.Num());
	for (int32 i = 0; i < ToGenerate.Num(); ++i)
	{
		PerSlotBufferPtrs[i].SetNum(NumBuffers);
		for (int32 b = 0; b < NumBuffers; ++b)
		{
			PerSlotBufferPtrs[i][b] = &State.Buffers[b][0];
		}
	}
	ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i) {
		Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
		}, EParallelForFlags::BackgroundPriority);
	// Insert this tier's generated particles into the octree.
	InsertTierIntoOctree(Config, State, 0);
	// Cache generated data for each cell so re-entry can skip procgen.
	for (const auto& Pair : ToGenerate)
	{
		CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, 0);
	}
	// Mirror front → back so either buffer is a valid starting state.
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b][1].CopyFrom(State.Buffers[b][0]);
	}
	// Spawn Niagara components on game thread. Push the full dead buffer first
	// so Niagara sees the correct particle count, then activate exactly once.
	// Particle IDs are stable from this point forward — no further
	// Activate/ReinitializeSystem calls are made for the lifetime of the tier.
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, &Config, &State, NumBuffers, CompletionPromise = MoveTemp(CompletionPromise)]() mutable {
		AUniverseActor* Self = WeakThis.Get();
		if (!Self) return;
		const FBox Bounds = Config.ComputeBounds();
		State.NiagaraComponents.SetNum(NumBuffers);
		for (int32 b = 0; b < NumBuffers; ++b)
		{
			UNiagaraSystem* Template = Config.NiagaraAssets[b];
			if (!Template) UE_LOG(LogTemp, Warning, TEXT("AUniverseActor::InitializeTier [%s] - NiagaraAssets[%d] not assigned."), *Config.TierName, b);
			UNiagaraComponent* NC = UNiagaraFunctionLibrary::SpawnSystemAttached(Template, Self->GetRootComponent(), NAME_None, FVector::ZeroVector, FRotator::ZeroRotator, EAttachLocation::SnapToTarget, false, false);
			if (NC)
			{
				NC->SetSystemFixedBounds(Bounds);
				NC->TranslucencySortPriority = 0;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("AUniverseActor::InitializeTier [%s] - Failed to create NiagaraComponent[%d]"), *Config.TierName, b);
			}
			State.NiagaraComponents[b] = NC;
			Self->TierNiagaraComponents.Add(NC);
		}
		// Push initial data (includes real particle data already generated
		// above) and activate each component exactly once.
		const int32 FrontIdx = State.FrontIdx.load();
		for (int32 b = 0; b < NumBuffers; ++b)
		{
			UNiagaraComponent* NC = State.NiagaraComponents[b];
			if (NC)
			{
				NC->SetSystemFixedBounds(Bounds);
				State.Buffers[b][FrontIdx].ActivateOnce(NC, Self->VirtualTraversal);
			}
		}
		CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
	const int32 Side_Log = 2 * Config.NeighborhoodRadius + 1;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeTier [%s] took %.3f sec (%d slots, %d max particles/slot, center %d,%d,%d)"), *Config.TierName, FPlatformTime::Seconds() - StartTime, Side_Log * Side_Log * Side_Log, Config.SlotCapacity, State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z);
}

void AUniverseActor::UpdateTier(FParticleTierConfig& Config, FParticleTierState& State)
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (bRebaseInProgress.load()) return;
	// Check that at least one component exists.
	bool bHasComponent = false;
	for (UNiagaraComponent* NC : State.NiagaraComponents)
	{
		if (NC) { bHasComponent = true; break; }
	}
	if (!bHasComponent) return;
	// If async generation completed a swap, push the new front buffer.
	if (State.bNeedsPush.load())
	{
		PushTierToNiagara(Config, State);
		State.bNeedsPush.store(false);
	}
	if (State.bUpdateInProgress.load()) return;
	const FVector LocalPlayerPos = VirtualTraversal;
	const FIntVector NewCoord = PositionToGridCoord(LocalPlayerPos, Config.GridDepth);
	if (NewCoord == State.CenterCoord) return;
	UE_LOG(LogTemp, Verbose, TEXT("AUniverseActor::UpdateTier [%s] - boundary cross: (%d,%d,%d) -> (%d,%d,%d)"), *Config.TierName, State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z, NewCoord.X, NewCoord.Y, NewCoord.Z);
	State.bUpdateInProgress.store(true);
	const FIntVector OldCenter = State.CenterCoord;
	State.CenterCoord = NewCoord;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, &Config, &State, OldCenter, NewCoord]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self)
			{
				// Actor destroyed mid-flight — release the flag so nothing deadlocks.
				State.bUpdateInProgress.store(false);
				return;
			}
			double StartTime = FPlatformTime::Seconds();
			const int32 NumBuffers = Config.NiagaraAssets.Num();
			const int32 FrontIdx = State.FrontIdx.load();
			const int32 BackIdx = 1 - FrontIdx;
			// Copy front → back; unchanged slots carry over untouched.
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				State.Buffers[b][BackIdx].CopyFrom(State.Buffers[b][FrontIdx]);
			}
			// Build old and new neighborhood coord sets.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			for (int32 dz = -Config.NeighborhoodRadius; dz <= Config.NeighborhoodRadius; ++dz)
			{
				for (int32 dy = -Config.NeighborhoodRadius; dy <= Config.NeighborhoodRadius; ++dy)
				{
					for (int32 dx = -Config.NeighborhoodRadius; dx <= Config.NeighborhoodRadius; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						if (OldCenter.X != INT32_MIN)
						{
							OldSet.Add(OldCenter + Offset);
						}
						NewSet.Add(NewCoord + Offset);
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
			// Free exiting slots: dead-stub their data, return slot index.
			// Octree nodes from exiting cells are left in place — they
			// persist as part of the spatial cache. The slot index is
			// recycled for reuse by entering cells.
			const FVector DeadPos(Self->Params.Extent * 10.0, Self->Params.Extent * 10.0, Self->Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FSlotEntry* Entry = State.ActiveSlots.Find(Coord);
				if (Entry)
				{
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						State.Buffers[b][BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
					}
					State.FreeSlots.Add(Entry->SlotIndex);
					State.ActiveSlots.Remove(Coord);
				}
			}
			// Fire optional boundary-cross hook (e.g. streaming volumetric).
			if (Config.OnBoundaryCross) Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);
			// Generate entering nodes: serial slot alloc → cache-hit restore
			// or cache-miss parallel gen → incremental octree insert.
			TArray<TPair<FIntVector, int32>> ToGenerate;  // Cache misses.
			TArray<int32> AllEnteringSlots;  // All entering slots for octree insert.
			ToGenerate.Reserve(EnteringNodes.Num());
			AllEnteringSlots.Reserve(EnteringNodes.Num());
			int32 CacheHitCount = 0;
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (State.FreeSlots.Num() == 0)
				{
					UE_LOG(LogTemp, Warning, TEXT("AUniverseActor::UpdateTier [%s] - no free slots; dropping cell (%d,%d,%d)"), *Config.TierName, Coord.X, Coord.Y, Coord.Z);
					continue;
				}
				const int32 SlotIndex = State.FreeSlots.Pop();
				State.ActiveSlots.Add(Coord, FSlotEntry{ SlotIndex, {} });
				AllEnteringSlots.Add(SlotIndex);
				// --- Cache-hit path: restore from CellCache ---
				const FCachedCellData* Cached = State.CellCache.Find(Coord);
				if (Cached && Cached->ParticleCount > 0)
				{
					const int32 LiveCount = Cached->ParticleCount;
					State.SlotCounts[SlotIndex] = LiveCount;
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						FNiagaraParticleBuffer& Buf = State.Buffers[b][BackIdx];
						const int32 Start = SlotIndex * Buf.SlotCapacity;
						const TArray<FVector>& CPos = Cached->PerBufferPositions[b];
						const TArray<float>& CExt = Cached->PerBufferExtents[b];
						const TArray<FLinearColor>& CCol = Cached->PerBufferColors[b];
						const TArray<FVector>& CRot = Cached->PerBufferRotations[b];
						for (int32 i = 0; i < LiveCount; ++i)
						{
							const int32 Idx = Start + i;
							Buf.Positions[Idx] = CPos[i];
							Buf.Extents[Idx] = CExt[i];
							Buf.Colors[Idx] = CCol[i];
							if (Buf.Rotations.Num() > 0 && CRot.Num() > 0)
							{
								Buf.Rotations[Idx] = CRot[i];
							}
						}
						// Dead-pad the remainder of the slot.
						Buf.PadSlotDead(SlotIndex, LiveCount, DeadPos);
					}
					++CacheHitCount;
				}
				else
				{
					// --- Cache-miss path: queue for generation ---
					ToGenerate.Emplace(Coord, SlotIndex);
				}
			}
			// Build buffer pointer arrays for the back buffer (cache-miss cells only).
			TArray<TArray<FNiagaraParticleBuffer*>> PerSlotBufferPtrs;
			PerSlotBufferPtrs.SetNum(ToGenerate.Num());
			for (int32 i = 0; i < ToGenerate.Num(); ++i)
			{
				PerSlotBufferPtrs[i].SetNum(NumBuffers);
				for (int32 b = 0; b < NumBuffers; ++b)
				{
					PerSlotBufferPtrs[i][b] = &State.Buffers[b][BackIdx];
				}
			}
			ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i) {
				Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
				}, EParallelForFlags::BackgroundPriority);
			// Cache generated data for each cache-miss cell.
			for (const auto& Pair : ToGenerate)
			{
				Self->CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, BackIdx);
			}
			// Incremental octree insert for all entering cells (hits + misses).
			for (int32 Slot : AllEnteringSlots)
			{
				Self->InsertSlotIntoOctree(Config, State, Slot, BackIdx);
			}
			// Evict cache entries outside the proximity window.
			State.FrontIdx.store(BackIdx);
			State.bNeedsPush.store(true);
			Self->CullTierCache(Config, State, NewCoord);
			State.bUpdateInProgress.store(false);
			UE_LOG(LogTemp, Verbose, TEXT("AUniverseActor::UpdateTier [%s] - %d entering (%d cached, %d generated), %d exiting in %.3f sec"), *Config.TierName, EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

void AUniverseActor::PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State)
{
	const int32 FrontIdx = State.FrontIdx.load();
	// Recompute bounds relative to VirtualTraversal. The positions pushed
	// to Niagara are (LocalPos - VirtualTraversal), so the bounds box must
	// encompass that range, not the absolute neighborhood extent. Without
	// this, particles at high SpeedScale can end up far outside the fixed
	// bounds, causing Lumen/radiance-cache crashes on the render thread.
	const FBox Bounds = Config.ComputeBounds();
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		UNiagaraComponent* NC = State.NiagaraComponents[b];
		if (NC) NC->SetSystemFixedBounds(Bounds);
		State.Buffers[b][FrontIdx].PushToNiagara(NC, VirtualTraversal);
	}
}

void AUniverseActor::CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx)
{
	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	FCachedCellData& Cache = State.CellCache.FindOrAdd(Coord);
	Cache.ParticleCount = LiveCount;
	Cache.CenterOffset = FVector::ZeroVector; // Positions are stored absolute.
	Cache.PerBufferPositions.SetNum(NumBuffers);
	Cache.PerBufferExtents.SetNum(NumBuffers);
	Cache.PerBufferColors.SetNum(NumBuffers);
	Cache.PerBufferRotations.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		const FNiagaraParticleBuffer& Buf = State.Buffers[b][BufferIdx];
		const int32 Start = SlotIndex * Buf.SlotCapacity;
		TArray<FVector>& CPos = Cache.PerBufferPositions[b];
		TArray<float>& CExt = Cache.PerBufferExtents[b];
		TArray<FLinearColor>& CCol = Cache.PerBufferColors[b];
		TArray<FVector>& CRot = Cache.PerBufferRotations[b];
		CPos.SetNumUninitialized(LiveCount);
		CExt.SetNumUninitialized(LiveCount);
		CCol.SetNumUninitialized(LiveCount);
		if (Buf.Rotations.Num() > 0)
		{
			CRot.SetNumUninitialized(LiveCount);
		}
		else
		{
			CRot.Empty();
		}
		for (int32 i = 0; i < LiveCount; ++i)
		{
			const int32 Idx = Start + i;
			CPos[i] = Buf.Positions[Idx];
			CExt[i] = Buf.Extents[Idx];
			CCol[i] = Buf.Colors[Idx];
			if (CRot.Num() > 0) CRot[i] = Buf.Rotations[Idx];
		}
	}
}

void AUniverseActor::CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& NewCenter)
{
	const int32 CullRadius = Config.NeighborhoodRadius + 4;
	TArray<FIntVector> ToEvict;
	for (const auto& Pair : State.CellCache)
	{
		const FIntVector Delta = Pair.Key - NewCenter;
		const int32 ChebyshevDist = FMath::Max3(FMath::Abs(Delta.X), FMath::Abs(Delta.Y), FMath::Abs(Delta.Z));
		if (ChebyshevDist > CullRadius) ToEvict.Add(Pair.Key);
	}
	for (const FIntVector& Coord : ToEvict)
	{
		State.CellCache.Remove(Coord);
	}
	if (ToEvict.Num() > 0) UE_LOG(LogTemp, Verbose, TEXT("AUniverseActor::CullTierCache [%s] — evicted %d entries, %d remaining"), *Config.TierName, ToEvict.Num(), State.CellCache.Num());
}

void AUniverseActor::InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Octree.IsValid()) return;
	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex][BufferIdx];
	const double TreeExtent = Octree->Extent;
	for (auto& Pair : State.ActiveSlots)
	{
		FSlotEntry& Entry = Pair.Value;
		Entry.InsertedNodes.Reset();
		Entry.InsertedNodes.Reserve(InsertBuffer.SlotCapacity);
		const int32 BufferStart = Entry.SlotIndex * InsertBuffer.SlotCapacity;
		for (int32 i = 0; i < InsertBuffer.SlotCapacity; ++i)
		{
			const int32 Idx = BufferStart + i;
			if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
			InsertParticleIntoOctree(Entry, InsertBuffer.Positions[Idx], InsertBuffer.Extents[Idx], Entry.SlotIndex, TreeExtent, Config.TierIndex);
		}
	}
}

void AUniverseActor::InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 SlotIndex, int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Octree.IsValid()) return;
	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex][BufferIdx];
	const double TreeExtent = Octree->Extent;
	FSlotEntry* Entry = nullptr;
	for (auto& Pair : State.ActiveSlots)
	{
		if (Pair.Value.SlotIndex == SlotIndex) { Entry = &Pair.Value; break; }
	}
	if (!Entry) return;
	Entry->InsertedNodes.Reset();
	Entry->InsertedNodes.Reserve(State.SlotCounts[SlotIndex]);
	const int32 BufferStart = SlotIndex * InsertBuffer.SlotCapacity;
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	for (int32 i = 0; i < LiveCount; ++i)
	{
		const int32 Idx = BufferStart + i;
		if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
		InsertParticleIntoOctree(*Entry, InsertBuffer.Positions[Idx], InsertBuffer.Extents[Idx], SlotIndex, TreeExtent, Config.TierIndex);
	}
}

void AUniverseActor::InsertParticleIntoOctree(FSlotEntry& Entry, const FVector& Position, float Extent, int32 SlotIndex, double TreeExtent, int32 TierIndex)
{
	// Extent is already in sector-local units. UnitScale=1.0 avoids double-conversion.
	FPointData PointData = FPointData::MakePointDataFromWorldScale(static_cast<double>(Extent), 1.0, static_cast<int64>(TreeExtent));
	PointData.SetPosition(Position);
	PointData.Data.ObjectId = SlotIndex;
	PointData.Data.TypeId = TierIndex;
	TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
	if (Node.IsValid()) Entry.InsertedNodes.Add(Node);
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
	// Snapshot VirtualTraversal now — it will keep advancing on the
	// game thread while the background task runs, but the new tree
	// center only needs to be approximately correct. Any drift is
	// absorbed by the large PersistentTreeMultiplier margin.
	const FVector RebaseOrigin = VirtualTraversal;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, RebaseOrigin]() {
		AUniverseActor* Self = WeakThis.Get();
		if (!Self) return;
		UE_LOG(LogTemp, Warning, TEXT("AUniverseActor::RebaseOctree -- rebasing to (%.1f, %.1f, %.1f)"), RebaseOrigin.X, RebaseOrigin.Y, RebaseOrigin.Z);
		const double TreeExtent = Self->Params.Extent * PersistentTreeMultiplier;
		Self->Octree = MakeShared<FOctree>(TreeExtent, RebaseOrigin);
		// Re-insert all tiers using their current front buffers.
		// Front buffers are read-only on the game thread and won't be
		// swapped while bRebaseInProgress is true (UpdateTier guards it).
		Self->InsertTierIntoOctree(Self->CoarseTierConfig, Self->CoarseTierState,
			Self->CoarseTierState.FrontIdx.load());
		Self->InsertTierIntoOctree(Self->MidTierConfig, Self->MidTierState,
			Self->MidTierState.FrontIdx.load());
		Self->InsertTierIntoOctree(Self->SmallTierConfig, Self->SmallTierState,
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
	// Push updated relative positions to all tier buffers every tick.
	// Because particle IDs are now stable (no reinit), the GPU-side positions
	// are never reset — so we must re-push LocalPos - VirtualTraversal each
	// frame as VirtualTraversal grows with the player. This replaces the old
	// per-tick ParallaxOffset accumulation which depended on Activate(bReset)
	// clearing the accumulator on each boundary-cross push.
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
}
#pragma endregion

#pragma region Tick
void AUniverseActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ApplyParallaxOffset();  // Resolves player pos once into CurrentFrameOfReferenceLocation

	// Drive all active galaxies with the already-resolved player position.
	// Galaxies have UE tick disabled; this is their only per-frame entry point.
	// Each galaxy cascades down to its own star systems via TickFromParent.
	for (auto& Pair : SpawnedGalaxies)
	{
		if (AGalaxyActor* Galaxy = Pair.Value.Get())
			Galaxy->TickFromParent(DeltaTime, CurrentFrameOfReferenceLocation);
	}

	UpdateTier(CoarseTierConfig, CoarseTierState);
	UpdateTier(MidTierConfig, MidTierState);
	UpdateTier(SmallTierConfig, SmallTierState);
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

	// Derive UnitScale from the particle extent instead of the node extent.
	Galaxy->Params.UnitScale = (static_cast<double>(ParticleExtent) * this->Params.UnitScale) / Galaxy->Params.Extent;
	Galaxy->SpeedScale = SpeedScale;
	Galaxy->Params.Seed = InNode->Data.ObjectId;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	FRandomStream RandStream(InNode->Data.ObjectId);
	Galaxy->Params.Rotation = RandStream.GetUnitVector().Rotation();

	// Place galaxy to match the particle sprite, not the node bounds.
	const FVector SpawnLoc = ComputeChildSpawnLocation(ParticlePos, Galaxy->Params.UnitScale);
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
	//
	// As the player flies toward the galaxy, VT accumulates at rate
	// (SpeedScale / UnitScale).  UnitScale grows smaller as the galaxy actor
	// is reconfigured for closer approach, so VT shrinks toward zero —
	// yielding maximum floating-point precision exactly when the player is
	// inside the galaxy.
	Galaxy->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;

	UE_LOG(LogTemp, Warning, TEXT("=== SpawnGalaxyFromPool ==="));
	UE_LOG(LogTemp, Warning, TEXT("  Node: center=(%.1f, %.1f, %.1f) extent=%.2f objId=%d tier=%s"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Data.ObjectId, *MatchedConfig.TierName);
	UE_LOG(LogTemp, Warning, TEXT("  Particle: pos=(%.1f, %.1f, %.1f) extent=%.2f"),
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent);
	UE_LOG(LogTemp, Warning, TEXT("  Galaxy: spawnLoc=(%.1f, %.1f, %.1f) unitScale=%.2e extent=%.0f sizeRatio=%.2f seed=%d"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		Galaxy->Params.UnitScale, Galaxy->Params.Extent,
		Params.UnitScale / Galaxy->Params.UnitScale,
		Galaxy->Params.Seed);

	Galaxy->Initialize();
	Galaxy->SetActorHiddenInGame(false);
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
			FPlatformProcess::Sleep(0.05f);
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
					TSet<TSharedPtr<FOctreeNode>> NearbySet(NearbyArray);
					for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
					{
						if (!InnerSelf->TrackedSpawnNodes.Contains(Node))
						{
							InnerSelf->LogSpawnNodeEnter(Node);
							InnerSelf->SpawnGalaxyFromPool(Node);
						}
						if (InnerSelf->bDebugDrawSpawnNodes) InnerSelf->DebugDrawSpawnNode(Node);
					}
					TSet<TSharedPtr<FOctreeNode>> Exited = InnerSelf->TrackedSpawnNodes.Difference(NearbySet);
					for (const TSharedPtr<FOctreeNode>& Node : Exited)
					{
						InnerSelf->LogSpawnNodeExit(Node);
						InnerSelf->ReturnGalaxyToPool(Node);
					}
					InnerSelf->TrackedSpawnNodes = MoveTemp(NearbySet);
					InnerSelf->bSpawnScanInProgress.store(false);
				});
		});
}

void AUniverseActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const int32 SlotId = InNode->Data.ObjectId;
	const int32 ExtraCount = InNode->Data.AdditionalObjectIds.Num();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan ENTER — node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d slot=%d extras=%d scale=%.3f"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, SlotId, ExtraCount, InNode->Data.ScaleFactor);
	if (bLogSpawnEnterExitBuffers)
	{
		// Identify tier by checking whether the slot index appears in the coarse tier's active slots.
		bool bIsCoarse = false;
		for (const auto& Pair : CoarseTierState.ActiveSlots)
		{
			if (Pair.Value.SlotIndex == SlotId)
			{
				bIsCoarse = true;
				break;
			}
		}
		if (bIsCoarse)
		{
			const FNiagaraParticleBuffer& Front = CoarseTierState.Buffers[0][CoarseTierState.FrontIdx.load()];
			const int32 Start = SlotId * CoarseTierConfig.SlotCapacity;
			int32 LiveCount = 0;
			for (int32 i = 0; i < CoarseTierConfig.SlotCapacity; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log, TEXT("  coarse slot %d: %d live cluster particles of %d capacity"), SlotId, LiveCount, CoarseTierConfig.SlotCapacity);
		}
		else
		{
			const FNiagaraParticleBuffer& Front = SmallTierState.Buffers[0][SmallTierState.FrontIdx.load()];
			const int32 Start = SlotId * SmallTierConfig.SlotCapacity;
			int32 LiveCount = 0;
			for (int32 i = 0; i < SmallTierConfig.SlotCapacity; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log, TEXT("  proximity slot %d: %d live particles of %d capacity"), SlotId, LiveCount, SmallTierConfig.SlotCapacity);
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
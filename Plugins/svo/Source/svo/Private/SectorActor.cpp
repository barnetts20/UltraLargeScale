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

	SectorLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorLarge.NG_SectorLarge"));
	SectorMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorMid.NG_SectorMid"));
	SectorSmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorSmall.NG_SectorSmall"));
	SectorGasCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorGas.NG_SectorGas"));
	GalaxyActorClass = AGalaxyActor::StaticClass();

	// Initial tree centered at origin, sized large enough for extended
	// traversal without rebasing. Grid cell sizes use the smaller
	// GridExtentMultiplier; only the octree spatial index uses the
	// larger PersistentTreeMultiplier.
	Octree = MakeShared<FOctree>(Params.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
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
	// overridden after construction. Centered at origin with persistent sizing.
	Octree = MakeShared<FOctree>(Params.Extent * PersistentTreeMultiplier, FVector::ZeroVector);
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
}

void ASectorActor::InitializeData()
{
	double TotalStart = FPlatformTime::Seconds();

	UniverseGenerator.Params = Params;
	UniverseGenerator.Initialize();

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeData total: %.3f sec"), FPlatformTime::Seconds() - TotalStart);
}

void ASectorActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();

	BuildTierConfigs();
	InitializeTier(CoarseTierConfig, CoarseTierState);
	InitializeTier(MidTierConfig, MidTierState);
	InitializeTier(ProximityTierConfig, ProximityTierState);

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Unified Particle Tier System

// ---------------------------------------------------------------------------
// Generic grid coord helpers — parameterized by GridDepth
// ---------------------------------------------------------------------------

FIntVector ASectorActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FIntVector(
		FMath::FloorToInt32(InPos.X / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Y / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
}

FVector ASectorActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FVector(
		static_cast<double>(InCoord.X) * CellSize,
		static_cast<double>(InCoord.Y) * CellSize,
		static_cast<double>(InCoord.Z) * CellSize);
}

double ASectorActor::GetGridCellExtent(int32 InGridDepth) const
{
	return (Params.Extent * GridExtentMultiplier) / (1 << (InGridDepth + 1));
}

// ---------------------------------------------------------------------------
// BuildTierConfigs — populate config structs from Params + Niagara assets
// ---------------------------------------------------------------------------

void ASectorActor::BuildTierConfigs()
{
	// Derive MinScale/MaxScale for all tiers from MaxEntityScale + depth spacing.
	// Must be called before any generate callback reads scale ranges.
	Params.DeriveScaleRanges();

	// --- Large tier (was "Coarse") ---
	CoarseTierConfig.TierName = TEXT("Large");
	CoarseTierConfig.GridDepth = Params.LargeTier.GridDepth;
	CoarseTierConfig.NeighborhoodRadius = Params.LargeTier.NeighborhoodRadius;
	CoarseTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	CoarseTierConfig.NiagaraAssets = { SectorLargeCloud, SectorGasCloud };
	CoarseTierConfig.bWantRotations = { true, false };
	CoarseTierConfig.OctreeInsertBufferIndex = 0;

	CoarseTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			const FVector NodeCenter = GridCoordToCenter(Coord, CoarseTierConfig.GridDepth);
			UniverseGenerator.GenerateLargeTierNode(
				Coord, SlotIndex, *Buffers[0], *Buffers[1],
				NodeCenter, CoarseTierState.SlotCounts[SlotIndex]);
		};

	CoarseTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.LargeTier.NeighborhoodRadius + 1) * GetGridCellExtent(Params.LargeTier.GridDepth) * 2.0;
			return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
		};

	// --- Mid tier ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.MaxParticlesPerSlot;
	MidTierConfig.NiagaraAssets = { SectorMidCloud };
	MidTierConfig.bWantRotations = { true };
	MidTierConfig.OctreeInsertBufferIndex = 0;

	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			const FVector NodeCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
			const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);
			UniverseGenerator.GenerateMidTierNode(
				Coord, SlotIndex, *Buffers[0],
				NodeCenter, CellExt, MidTierState.SlotCounts[SlotIndex]);
		};

	MidTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.MidTier.NeighborhoodRadius + 1) * GetGridCellExtent(Params.MidTier.GridDepth) * 2.0;
			return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
		};

	// --- Small tier (was "Proximity") ---
	ProximityTierConfig.TierName = TEXT("Small");
	ProximityTierConfig.GridDepth = Params.SmallTier.GridDepth;
	ProximityTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	ProximityTierConfig.SlotCapacity = Params.SmallTier.MaxParticlesPerSlot;
	ProximityTierConfig.NiagaraAssets = { SectorSmallCloud };
	ProximityTierConfig.bWantRotations = { false };
	ProximityTierConfig.OctreeInsertBufferIndex = 0;

	ProximityTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			const FVector NodeCenter = GridCoordToCenter(Coord, ProximityTierConfig.GridDepth);
			const double CellExt = GetGridCellExtent(ProximityTierConfig.GridDepth);
			UniverseGenerator.GenerateSmallTierNode(
				Coord, SlotIndex, *Buffers[0],
				NodeCenter, CellExt, ProximityTierState.SlotCounts[SlotIndex]);
		};

	ProximityTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.SmallTier.NeighborhoodRadius + 1) * GetGridCellExtent(Params.SmallTier.GridDepth) * 2.0;
			return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
		};
}

// ---------------------------------------------------------------------------
// InitializeTier — generic init pipeline
// ---------------------------------------------------------------------------

void ASectorActor::InitializeTier(FParticleTierConfig& Config, FParticleTierState& State)
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

	ParallelFor(ToGenerate.Num(), [this, &Config, &ToGenerate, &PerSlotBufferPtrs](int32 i)
		{
			Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
		}, EParallelForFlags::BackgroundPriority);

	// Insert this tier's generated particles into the octree.
	InsertTierIntoOctree(Config, State, /*BufferIdx=*/ 0);

	// Cache generated data for each cell so re-entry can skip procgen.
	for (const auto& Pair : ToGenerate)
	{
		CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, /*BufferIdx=*/ 0);
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
	AsyncTask(ENamedThreads::GameThread, [this, &Config, &State, NumBuffers,
		CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			const FBox Bounds = Config.ComputeBounds();

			State.NiagaraComponents.SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraSystem* Template = Config.NiagaraAssets[b];
				if (!Template)
				{
					UE_LOG(LogTemp, Warning, TEXT("ASectorActor::InitializeTier [%s] - NiagaraAssets[%d] not assigned."),
						*Config.TierName, b);
				}

				UNiagaraComponent* NC = UNiagaraFunctionLibrary::SpawnSystemAttached(
					Template,
					GetRootComponent(),
					NAME_None,
					FVector::ZeroVector,
					FRotator::ZeroRotator,
					EAttachLocation::SnapToTarget,
					/*bAutoDestroy=*/ false,
					/*bAutoActivate=*/ false);

				if (NC)
				{
					NC->SetSystemFixedBounds(Bounds);
					NC->TranslucencySortPriority = 0;
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeTier [%s] - Failed to create NiagaraComponent[%d]"),
						*Config.TierName, b);
				}

				State.NiagaraComponents[b] = NC;
				TierNiagaraComponents.Add(NC);
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
					State.Buffers[b][FrontIdx].ActivateOnce(NC, VirtualTraversal);
				}
			}

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	const int32 Side_Log = 2 * Config.NeighborhoodRadius + 1;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeTier [%s] took %.3f sec (%d slots, %d max particles/slot, center %d,%d,%d)"),
		*Config.TierName, FPlatformTime::Seconds() - StartTime,
		Side_Log * Side_Log * Side_Log, Config.SlotCapacity,
		State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z);
}

// ---------------------------------------------------------------------------
// UpdateTier — per-tick streaming update
// ---------------------------------------------------------------------------

void ASectorActor::UpdateTier(FParticleTierConfig& Config, FParticleTierState& State)
{
	if (InitializationState != ELifecycleState::Ready) return;

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

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateTier [%s] - boundary cross: (%d,%d,%d) -> (%d,%d,%d)"),
		*Config.TierName,
		State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z,
		NewCoord.X, NewCoord.Y, NewCoord.Z);

	State.bUpdateInProgress.store(true);
	const FIntVector OldCenter = State.CenterCoord;
	State.CenterCoord = NewCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, &Config, &State, OldCenter, NewCoord]()
		{
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
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
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
			if (Config.OnBoundaryCross)
			{
				Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);
			}

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
					UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateTier [%s] - no free slots; dropping cell (%d,%d,%d)"),
						*Config.TierName, Coord.X, Coord.Y, Coord.Z);
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

			ParallelFor(ToGenerate.Num(), [this, &Config, &ToGenerate, &PerSlotBufferPtrs](int32 i)
				{
					Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
				}, EParallelForFlags::BackgroundPriority);

			// Cache generated data for each cache-miss cell.
			for (const auto& Pair : ToGenerate)
			{
				CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, BackIdx);
			}

			// Incremental octree insert for all entering cells (hits + misses).
			for (int32 Slot : AllEnteringSlots)
			{
				InsertSlotIntoOctree(Config, State, Slot, BackIdx);
			}

			State.FrontIdx.store(BackIdx);
			State.bNeedsPush.store(true);
			State.bUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateTier [%s] - %d entering (%d cached, %d generated), %d exiting in %.3f sec"),
				*Config.TierName, EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

// ---------------------------------------------------------------------------
// PushTierToNiagara — push front buffers to all Niagara components
// ---------------------------------------------------------------------------

void ASectorActor::PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State)
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
		if (NC)
		{
			NC->SetSystemFixedBounds(Bounds);
		}
		State.Buffers[b][FrontIdx].PushToNiagara(NC, VirtualTraversal);
	}
}

// ---------------------------------------------------------------------------
// InsertTierIntoOctree — insert one tier's active front-buffer particles
// ---------------------------------------------------------------------------

void ASectorActor::InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Octree.IsValid()) return;

	const int32 InsertBufIdx = Config.OctreeInsertBufferIndex;
	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[InsertBufIdx][BufferIdx];
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
			const float Extent = InsertBuffer.Extents[Idx];
			if (Extent <= 0.0f) continue;

			// See InitializeTier comment: Extent is already in sector-local
			// units. UnitScale=1.0 avoids double-conversion.
			FPointData PointData = FPointData::MakePointDataFromWorldScale(
				static_cast<double>(Extent),
				/*InUnitScale=*/ 1.0,
				static_cast<int64>(TreeExtent));
			PointData.SetPosition(InsertBuffer.Positions[Idx]);
			PointData.Data.ObjectId = Entry.SlotIndex;
			PointData.Data.TypeId = GalaxyTypeId;

			TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
				PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
			if (Node.IsValid())
			{
				Entry.InsertedNodes.Add(Node);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// CacheCellFromBuffers — snapshot live particles into the persistent cache
// ---------------------------------------------------------------------------

void ASectorActor::CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State,
	const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx)
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
			if (CRot.Num() > 0)
			{
				CRot[i] = Buf.Rotations[Idx];
			}
		}
	}
}

// ---------------------------------------------------------------------------
// InsertSlotIntoOctree — insert a single slot's particles incrementally
// ---------------------------------------------------------------------------

void ASectorActor::InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State,
	int32 SlotIndex, int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Octree.IsValid()) return;

	const int32 InsertBufIdx = Config.OctreeInsertBufferIndex;
	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[InsertBufIdx][BufferIdx];
	const double TreeExtent = Octree->Extent;

	FSlotEntry* Entry = nullptr;
	for (auto& Pair : State.ActiveSlots)
	{
		if (Pair.Value.SlotIndex == SlotIndex)
		{
			Entry = &Pair.Value;
			break;
		}
	}
	if (!Entry) return;

	Entry->InsertedNodes.Reset();
	const int32 BufferStart = SlotIndex * InsertBuffer.SlotCapacity;
	const int32 LiveCount = State.SlotCounts[SlotIndex];

	Entry->InsertedNodes.Reserve(LiveCount);

	for (int32 i = 0; i < LiveCount; ++i)
	{
		const int32 Idx = BufferStart + i;
		const float Extent = InsertBuffer.Extents[Idx];
		if (Extent <= 0.0f) continue;

		FPointData PointData = FPointData::MakePointDataFromWorldScale(
			static_cast<double>(Extent),
			/*InUnitScale=*/ 1.0,
			static_cast<int64>(TreeExtent));
		PointData.SetPosition(InsertBuffer.Positions[Idx]);
		PointData.Data.ObjectId = SlotIndex;
		PointData.Data.TypeId = GalaxyTypeId;

		TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
			PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
		if (Node.IsValid())
		{
			Entry->InsertedNodes.Add(Node);
		}
	}
}

// ---------------------------------------------------------------------------
// RebaseOctree — exceptional full rebuild centered on VirtualTraversal
// ---------------------------------------------------------------------------

void ASectorActor::RebaseOctree()
{
	if (!Octree.IsValid()) return;

	UE_LOG(LogTemp, Warning, TEXT("ASectorActor::RebaseOctree — rebasing octree to VirtualTraversal (%.1f, %.1f, %.1f)"),
		VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z);

	const double TreeExtent = Params.Extent * PersistentTreeMultiplier;
	Octree = MakeShared<FOctree>(TreeExtent, VirtualTraversal);

	// Re-insert all tiers' active slot particles.
	InsertTierIntoOctree(CoarseTierConfig, CoarseTierState, CoarseTierState.FrontIdx.load());
	InsertTierIntoOctree(MidTierConfig, MidTierState, MidTierState.FrontIdx.load());
	InsertTierIntoOctree(ProximityTierConfig, ProximityTierState, ProximityTierState.FrontIdx.load());
}

// ---------------------------------------------------------------------------
// CheckOctreeBounds — trigger rebase if VirtualTraversal nears tree edge
// ---------------------------------------------------------------------------

void ASectorActor::CheckOctreeBounds()
{
	if (!Octree.IsValid()) return;

	// Rebase when VirtualTraversal is within 25% of the tree edge on any axis.
	const FVector TreeCenter = Octree->Root->Center;
	const double TreeExtent = Octree->Extent;
	const double Margin = TreeExtent * 0.75;

	const FVector Delta = VirtualTraversal - TreeCenter;
	if (FMath::Abs(Delta.X) > Margin ||
		FMath::Abs(Delta.Y) > Margin ||
		FMath::Abs(Delta.Z) > Margin)
	{
		// Only rebase if no tier is mid-update.
		const bool bAnyUpdating =
			CoarseTierState.bUpdateInProgress.load() ||
			MidTierState.bUpdateInProgress.load() ||
			ProximityTierState.bUpdateInProgress.load();
		if (!bAnyUpdating)
		{
			RebaseOctree();
		}
	}
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

	// Peg the actor. Components follow via attachment.
	SetActorLocation(CurrentPlayerPos);

	// Push updated relative positions to all tier buffers every tick.
	// Because particle IDs are now stable (no reinit), the GPU-side positions
	// are never reset — so we must re-push LocalPos - VirtualTraversal each
	// frame as VirtualTraversal grows with the player. This replaces the old
	// per-tick ParallaxOffset accumulation which depended on Activate(bReset)
	// clearing the accumulator on each boundary-cross push.
	for (FParticleTierState* Tier : { &CoarseTierState, &MidTierState, &ProximityTierState })
	{
		const int32 FrontIdx = Tier->FrontIdx.load();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;

			const TArray<FVector>& RelPos =
				Tier->Buffers[b][FrontIdx].MakeRelativePositions(VirtualTraversal);
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
				NC, NiagaraBufferParams::Positions, RelPos);
		}
	}
}
#pragma endregion

#pragma region Tick
void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	ApplyParallaxOffset();
	UpdateTier(CoarseTierConfig, CoarseTierState);
	UpdateTier(MidTierConfig, MidTierState);
	UpdateTier(ProximityTierConfig, ProximityTierState);

	// Exceptional rebase check — only triggers when VirtualTraversal
	// approaches the persistent octree's bounds (very rare).
	CheckOctreeBounds();
}
#pragma endregion

#pragma region Shutdown
void ASectorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopSpawnScanTimer();

	for (FParticleTierState* Tier : { &CoarseTierState, &MidTierState, &ProximityTierState })
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
			UE_LOG(LogTemp, Log,
				TEXT("  coarse slot %d: %d live cluster particles of %d capacity"),
				SlotId, LiveCount, CoarseTierConfig.SlotCapacity);
		}
		else
		{
			const FNiagaraParticleBuffer& Front = ProximityTierState.Buffers[0][ProximityTierState.FrontIdx.load()];
			const int32 Start = SlotId * ProximityTierConfig.SlotCapacity;
			int32 LiveCount = 0;
			for (int32 i = 0; i < ProximityTierConfig.SlotCapacity; ++i)
			{
				if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log,
				TEXT("  proximity slot %d: %d live particles of %d capacity"),
				SlotId, LiveCount, ProximityTierConfig.SlotCapacity);
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
		/*Thickness=*/ 3000.0f);
}
#pragma endregion
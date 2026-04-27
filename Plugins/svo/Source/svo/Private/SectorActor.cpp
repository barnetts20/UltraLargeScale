#pragma region Includes/ForwardDec
#include "SectorActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
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

	// Corner-align the tree so depth-2 cells line up with sector coarse cells.
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}
#pragma endregion

#pragma region Lifecycle
void ASectorActor::Initialize()
{
	InitializationState = ELifecycleState::Initializing;

	// Sample initial player position for delta tracking.
	// Both accumulators start at zero — accumulation begins from here.
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
	// overridden after construction.
	const FVector TreeCenter(Params.Extent, Params.Extent, Params.Extent);
	Octree = MakeShared<FOctree>(Params.Extent * TreeExtentMultiplier, TreeCenter);
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
}

void ASectorActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();

	// Sync generator params and build the shared noise instance once.
	Generator.Params = Params;
	DensityNoise = Generator.BuildNoise(69);

	BuildTierConfigs();
	InitializeTier(LargeTierConfig, LargeTierState);
	InitializeTier(MidTierConfig, MidTierState);
	InitializeTier(SmallTierConfig, SmallTierState);

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Unified Particle Tier System

// ---------------------------------------------------------------------------
// Generic grid coord helper — parameterized by GridDepth
// ---------------------------------------------------------------------------

FIntVector ASectorActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * TreeExtentMultiplier) / (1 << InGridDepth);
	return FIntVector(
		FMath::FloorToInt32(InPos.X / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Y / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
}

// ---------------------------------------------------------------------------
// BuildTierConfigs — populate config structs from Params + Niagara assets
// ---------------------------------------------------------------------------

void ASectorActor::BuildTierConfigs()
{
	// Derive MinScale/MaxScale for all tiers from MaxEntityScale + depth spacing.
	Params.DeriveScaleRanges();
	Generator.Params = Params;

	// --- Large tier ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = Params.LargeTier.NeighborhoodRadius;
	LargeTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	LargeTierConfig.NiagaraAssets = { SectorLargeCloud, SectorGasCloud };
	LargeTierConfig.bWantRotations = { true, false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;

	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) -> int32
		{
			const int32 Count = Generator.GenerateLargeNode(Coord, SlotIndex, *Buffers[0], *Buffers[1],
				DensityNoise, LargeTierConfig.GridDepth);
			LargeTierState.SlotCounts[SlotIndex] = Count;
			return Count;
		};

	LargeTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.LargeTier.NeighborhoodRadius + 1) * Params.Extent;
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

	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) -> int32
		{
			const int32 Count = Generator.GenerateMidNode(Coord, SlotIndex, *Buffers[0],
				DensityNoise, MidTierConfig.GridDepth);
			MidTierState.SlotCounts[SlotIndex] = Count;
			return Count;
		};

	MidTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.MidTier.NeighborhoodRadius + 1)
				* UniverseDataGenerator::GetGridCellExtent(Params.MidTier.GridDepth, Params.Extent, TreeExtentMultiplier) * 2.0;
			return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
		};

	// --- Small tier ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.MaxParticlesPerSlot;
	SmallTierConfig.NiagaraAssets = { SectorSmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;

	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) -> int32
		{
			const int32 Count = Generator.GenerateSmallNode(Coord, SlotIndex, *Buffers[0],
				DensityNoise, SmallTierConfig.GridDepth);
			SmallTierState.SlotCounts[SlotIndex] = Count;
			return Count;
		};

	SmallTierConfig.ComputeBounds = [this]() -> FBox
		{
			const double BoundsExtent = (2 * Params.SmallTier.NeighborhoodRadius + 1)
				* UniverseDataGenerator::GetGridCellExtent(Params.SmallTier.GridDepth, Params.Extent, TreeExtentMultiplier) * 2.0;
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

	// Allocate both buffer sets — they will both contain valid data at all times.
	State.Buffers.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b].SetNum(2);
		State.Buffers[b][0].Allocate(TotalSlots, Config.SlotCapacity, Config.bWantRotations[b]);
		State.Buffers[b][1].Allocate(TotalSlots, Config.SlotCapacity, Config.bWantRotations[b]);
	}
	State.ActiveIdx.store(0);

	State.SlotCounts.SetNumZeroed(TotalSlots);
	State.FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		State.FreeSlots.Add(i);
	}
	State.ActiveSlots.Empty();

	// Use GridVirtualOffset as the player's position in virtual space for grid lookups.
	const FVector LocalPlayerPos = GridVirtualOffset;
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

	// Build a single shared buffer pointer array — all slots in a tier write
	// to the same buffer set (buffer 0 at init time).
	TArray<FNiagaraParticleBuffer*> SharedBufferPtrs;
	SharedBufferPtrs.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		SharedBufferPtrs[b] = &State.Buffers[b][0];
	}

	ParallelFor(ToGenerate.Num(), [this, &Config, &ToGenerate, &SharedBufferPtrs](int32 i)
		{
			Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, SharedBufferPtrs);
		}, EParallelForFlags::BackgroundPriority);

	// Serial octree insert.
	if (Config.OctreeInsertBufferIndex >= 0 && Octree.IsValid())
	{
		const int32 InsertBufIdx = Config.OctreeInsertBufferIndex;
		const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[InsertBufIdx][0];
		const double TreeExtent = Octree->Extent;

		for (const TPair<FIntVector, int32>& Pair : ToGenerate)
		{
			FSlotEntry* Entry = State.ActiveSlots.Find(Pair.Key);
			if (!Entry) continue;

			const int32 BufferStart = Pair.Value * InsertBuffer.SlotCapacity;
			Entry->InsertedNodes.Reserve(InsertBuffer.SlotCapacity);

			for (int32 i = 0; i < InsertBuffer.SlotCapacity; ++i)
			{
				const int32 Idx = BufferStart + i;
				const float Extent = InsertBuffer.Extents[Idx];
				if (Extent <= 0.0f) continue;

				FPointData PointData = FPointData::MakePointDataFromWorldScale(
					static_cast<double>(Extent),
					/*InUnitScale=*/ 1.0,
					static_cast<int64>(TreeExtent));
				PointData.SetPosition(InsertBuffer.Positions[Idx]);
				PointData.Data.ObjectId = Pair.Value;
				PointData.Data.TypeId = GalaxyTypeId;

				TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
					PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
				if (Node.IsValid())
				{
					Entry->InsertedNodes.Add(Node);
				}
			}
		}
	}

	// Copy buffer 0 → buffer 1 so both start with identical valid data.
	// After this point, both buffers always contain valid data for all active
	// slots — UpdateTier only writes changed slots without a full copy.
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b][1].CopyFrom(State.Buffers[b][0]);
	}

	// Spawn Niagara components on game thread, push real data, activate.
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
					/*bAutoActivate=*/ true);

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

			PushTierToNiagara(Config, State);
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeTier [%s] took %.3f sec (%d slots, %d max particles/slot, center %d,%d,%d)"),
		*Config.TierName, FPlatformTime::Seconds() - StartTime,
		TotalSlots, Config.SlotCapacity,
		State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z);
}

// ---------------------------------------------------------------------------
// UpdateTier — per-tick streaming update (continuing-slot copy + index-swap)
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

	// If async generation completed and pushed arrays from the background
	// thread, just clear the flag. Living particles will pick up the new
	// array contents on their next Particle Update tick — no need to
	// reinitialize or reactivate the system.
	if (State.bNeedsPush.load())
	{
		State.bNeedsPush.store(false);
	}

	if (State.bUpdateInProgress.load()) return;

	const FVector LocalPlayerPos = GridVirtualOffset;
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
			const int32 ActiveIdx = State.ActiveIdx.load();
			const int32 InactiveIdx = 1 - ActiveIdx;

			// No full copy needed — the inactive buffer already has valid data
			// for all continuing slots from when it was last active. We only
			// need to clear exiting slots and generate entering slots.

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

			// Free exiting slots: dead-stub their data in the inactive buffer,
			// retire from octree, return slot.
			const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FSlotEntry* Entry = State.ActiveSlots.Find(Coord);
				if (Entry)
				{
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						State.Buffers[b][InactiveIdx].ClearSlot(Entry->SlotIndex, DeadPos);
					}
					State.FreeSlots.Add(Entry->SlotIndex);

					for (const TSharedPtr<FOctreeNode>& Node : Entry->InsertedNodes)
					{
						if (Octree.IsValid())
						{
							Octree->RemoveObjectIdFromNode(Node, Entry->SlotIndex);
						}
					}
					State.ActiveSlots.Remove(Coord);
				}
			}

			// Fire optional boundary-cross hook.
			if (Config.OnBoundaryCross)
			{
				Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);
			}

			// Sync continuing slots: copy their data from the active buffer
			// into the inactive buffer. The inactive buffer's data for these
			// slot indices is stale (from when it was last active — slot indices
			// may have been recycled via FreeSlots since then). We must copy
			// only continuing slots; entering slots will be freshly generated
			// and exiting slots were already cleared above.
			{
				TSet<FIntVector> EnteringSet(EnteringNodes);
				for (const auto& Pair : State.ActiveSlots)
				{
					// ActiveSlots has already had exiting coords removed and
					// doesn't yet contain entering coords (added below).
					// Everything remaining is a continuing slot.
					const int32 SlotIndex = Pair.Value.SlotIndex;
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						State.Buffers[b][InactiveIdx].CopySlotFrom(
							State.Buffers[b][ActiveIdx], SlotIndex);
					}
				}
			}

			// Generate entering nodes: serial slot alloc → parallel gen → serial octree insert.
			TArray<TPair<FIntVector, int32>> ToGenerate;
			ToGenerate.Reserve(EnteringNodes.Num());
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
				ToGenerate.Emplace(Coord, SlotIndex);
			}

			// Build a single shared buffer pointer array for the inactive buffer.
			TArray<FNiagaraParticleBuffer*> SharedBufferPtrs;
			SharedBufferPtrs.SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				SharedBufferPtrs[b] = &State.Buffers[b][InactiveIdx];
			}

			ParallelFor(ToGenerate.Num(), [this, &Config, &ToGenerate, &SharedBufferPtrs](int32 i)
				{
					Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, SharedBufferPtrs);
				}, EParallelForFlags::BackgroundPriority);

			// Serial octree insert.
			if (Config.OctreeInsertBufferIndex >= 0 && Octree.IsValid())
			{
				const int32 InsertBufIdx = Config.OctreeInsertBufferIndex;
				const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[InsertBufIdx][InactiveIdx];
				const double TreeExtent = Octree->Extent;

				for (const TPair<FIntVector, int32>& Pair : ToGenerate)
				{
					FSlotEntry* Entry = State.ActiveSlots.Find(Pair.Key);
					if (!Entry) continue;

					const int32 BufferStart = Pair.Value * InsertBuffer.SlotCapacity;
					Entry->InsertedNodes.Reserve(InsertBuffer.SlotCapacity);

					for (int32 i = 0; i < InsertBuffer.SlotCapacity; ++i)
					{
						const int32 Idx = BufferStart + i;
						const float Extent = InsertBuffer.Extents[Idx];
						if (Extent <= 0.0f) continue;

						FPointData PointData = FPointData::MakePointDataFromWorldScale(
							static_cast<double>(Extent),
							/*InUnitScale=*/ 1.0,
							static_cast<int64>(TreeExtent));
						PointData.SetPosition(InsertBuffer.Positions[Idx]);
						PointData.Data.ObjectId = Pair.Value;
						PointData.Data.TypeId = GalaxyTypeId;

						TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(
							PointData.GetPosition(), PointData.InsertDepth, PointData.Data);
						if (Node.IsValid())
						{
							Entry->InsertedNodes.Add(Node);
						}
					}
				}
			}

			// Swap the active index — the inactive buffer now has valid data
			// for all current slots (continuing slots copied from active,
			// entering slots freshly generated, exiting slots dead-stubbed).
			State.ActiveIdx.store(InactiveIdx);

			// Push arrays to Niagara data interfaces from this background thread.
			// Only the Activate call needs the game thread.
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				State.Buffers[b][InactiveIdx].PushArraysToNiagara(State.NiagaraComponents[b]);
			}

			State.bNeedsPush.store(true);
			State.bUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateTier [%s] - %d entering, %d exiting in %.3f sec"),
				*Config.TierName, EnteringNodes.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

// ---------------------------------------------------------------------------
// PushTierToNiagara — push active buffers to all Niagara components
// ---------------------------------------------------------------------------

void ASectorActor::PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State)
{
	const int32 ActiveIdx = State.ActiveIdx.load();
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		State.Buffers[b][ActiveIdx].PushToNiagara(State.NiagaraComponents[b]);
	}
}

#pragma endregion

#pragma region Player-Centered Parallax
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
	const FVector VirtualDelta = PlayerDelta * Ratio;

	// Grid accumulator — runs forever, only needs integer-cell accuracy.
	GridVirtualOffset += VirtualDelta;

	// Drift accumulator — bounded by threshold rebase.
	Drift += VirtualDelta;

	// Check rebase threshold. When any component of Drift exceeds the
	// limit, fold the entire drift into RebaseAccum and reset Drift to
	// zero. No buffer writes, no scratch pad writes — just two vector
	// parameters.
	if (FMath::Abs(Drift.X) > RebaseThreshold ||
		FMath::Abs(Drift.Y) > RebaseThreshold ||
		FMath::Abs(Drift.Z) > RebaseThreshold)
	{
		RebaseAccum += Drift;
		Drift = FVector::ZeroVector;

		UE_LOG(LogTemp, Log, TEXT("ASectorActor::ParallaxRebase — folded drift, RebaseAccum now (%.4f, %.4f, %.4f)"),
			RebaseAccum.X, RebaseAccum.Y, RebaseAccum.Z);
	}

	// Broadcast combined offset to all Niagara components.
	// Niagara composes: Position = PositionArray[UniqueID] + ParallaxOffset
	// The split (RebaseAccum + Drift) is internal to the CPU for precision;
	// Niagara receives the sum as a single parameter.
	const FVector ParallaxOffset = -(RebaseAccum + Drift);
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent* NC : Tier->NiagaraComponents)
		{
			if (NC)
			{
				NC->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
			}
		}
	}

	// Peg the actor to the player.
	SetActorLocation(CurrentPlayerPos);
}
#pragma endregion

#pragma region Tick
void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	ApplyParallaxOffset();
	UpdateTier(LargeTierConfig, LargeTierState);
	UpdateTier(MidTierConfig, MidTierState);
	UpdateTier(SmallTierConfig, SmallTierState);

	// Update debug tier coord display for the editor.
	LargeTierPlayerCoord = LargeTierState.CenterCoord;
	MidTierPlayerCoord = MidTierState.CenterCoord;
	SmallTierPlayerCoord = SmallTierState.CenterCoord;
}
#pragma endregion

#pragma region Shutdown
void ASectorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopSpawnScanTimer();

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

	const FVector LocalPlayerPos = GridVirtualOffset;

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

TPair<const FParticleTierConfig*, const FParticleTierState*> ASectorActor::FindTierForSlot(int32 InSlotIndex) const
{
	// Check all three tiers — Large, Mid, Small.
	const FParticleTierConfig* Configs[] = { &LargeTierConfig, &MidTierConfig, &SmallTierConfig };
	const FParticleTierState* States[] = { &LargeTierState, &MidTierState, &SmallTierState };

	for (int32 t = 0; t < 3; ++t)
	{
		for (const auto& Pair : States[t]->ActiveSlots)
		{
			if (Pair.Value.SlotIndex == InSlotIndex)
			{
				return { Configs[t], States[t] };
			}
		}
	}
	return { nullptr, nullptr };
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
		auto [Config, State] = FindTierForSlot(SlotId);
		if (Config && State)
		{
			const int32 BufIdx = Config->OctreeInsertBufferIndex;
			if (BufIdx >= 0 && BufIdx < State->Buffers.Num())
			{
				const FNiagaraParticleBuffer& Front = State->Buffers[BufIdx][State->ActiveIdx.load()];
				const int32 Start = SlotId * Config->SlotCapacity;
				int32 LiveCount = 0;
				for (int32 i = 0; i < Config->SlotCapacity; ++i)
				{
					if (Front.Extents[Start + i] > 0.0f) ++LiveCount;
				}
				UE_LOG(LogTemp, Log,
					TEXT("  %s slot %d: %d live particles of %d capacity"),
					*Config->TierName, SlotId, LiveCount, Config->SlotCapacity);
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("  slot %d not found in any tier's active slots"), SlotId);
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

	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - GridVirtualOffset;
	const FVector BoxExtent(InNode->Extent);

	DrawDebugBox(
		World,
		NodeCenterWorld,
		BoxExtent,
		FColor::Green,
		/*bPersistent=*/ false,
		/*Lifetime=*/ SpawnScanInterval,
		/*DepthPriority=*/ 0,
		/*Thickness=*/ 10.0f);
}
#pragma endregion
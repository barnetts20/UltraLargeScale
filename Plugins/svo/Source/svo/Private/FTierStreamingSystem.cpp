#include "FTierStreamingSystem.h"

// ============================================================================
//  InitializeTier
// ============================================================================
void FTierStreamingSystem::InitializeTier(
	const FTierStreamingContext& Ctx,
	FParticleTierConfig& Config,
	FParticleTierState& State,
	TArray<UNiagaraComponent*>& OutComponents)
{
	double StartTime = FPlatformTime::Seconds();

	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 Side = 2 * Config.NeighborhoodRadius + 1;
	const int32 TotalSlots = Side * Side * Side;

	// Allocate double-buffered particle data — one pair per Niagara asset.
	State.Buffers.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b].SetNum(2);
		bool bRotations = Config.bWantRotations.IsValidIndex(b) && Config.bWantRotations[b];
		State.Buffers[b][0].Allocate(TotalSlots, Config.SlotCapacity, bRotations);
		State.Buffers[b][1].Allocate(TotalSlots, Config.SlotCapacity, bRotations);
	}

	State.FrontIdx.store(0);
	State.SlotCounts.SetNumZeroed(TotalSlots);
	State.FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
		State.FreeSlots.Add(i);
	State.ActiveSlots.Empty();

	// Streaming tiers (NeighborhoodRadius > 0) defer initial population to the
	// first UpdateTier call by setting CenterCoord = INT32_MIN. Exhaustive tiers
	// (radius 0) generate immediately.
	const bool bIsStreamingTier = (Config.NeighborhoodRadius > 0);

	if (bIsStreamingTier)
	{
		State.CenterCoord = FIntVector(INT32_MIN, INT32_MIN, INT32_MIN);
	}
	else
	{
		// Exhaustive tier: generate the single cell at origin.
		State.CenterCoord = PositionToGridCoord(
			Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);

		TArray<TPair<FIntVector, int32>> ToGenerate;
		for (int32 dz = -Config.NeighborhoodRadius; dz <= Config.NeighborhoodRadius; ++dz)
			for (int32 dy = -Config.NeighborhoodRadius; dy <= Config.NeighborhoodRadius; ++dy)
				for (int32 dx = -Config.NeighborhoodRadius; dx <= Config.NeighborhoodRadius; ++dx)
				{
					const FIntVector Coord = State.CenterCoord + FIntVector(dx, dy, dz);
					const int32 SlotIndex = State.FreeSlots.Pop();
					FSlotEntry& Entry = State.ActiveSlots.Add(Coord);
					Entry.SlotIndex = SlotIndex;
					ToGenerate.Add({ Coord, SlotIndex });
				}

		// Build buffer pointer arrays for the front buffer (index 0 at init).
		TArray<TArray<FNiagaraParticleBuffer*>> PerSlotBufferPtrs;
		PerSlotBufferPtrs.SetNum(ToGenerate.Num());
		for (int32 i = 0; i < ToGenerate.Num(); ++i)
		{
			PerSlotBufferPtrs[i].SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
				PerSlotBufferPtrs[i][b] = &State.Buffers[b][0];
		}

		ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i) {
			Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
			}, EParallelForFlags::BackgroundPriority);

		if (Ctx.InitializationState == ELifecycleState::Pooling) return;

		// Insert into octree.
		InsertTierIntoOctree(Ctx, Config, State, 0);

		// Cache generated data for each cell.
		for (const auto& Pair : ToGenerate)
			CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, 0);

		// Cache MaxExtent before mirroring so CopyFrom propagates it.
		for (int32 b = 0; b < NumBuffers; ++b)
			State.Buffers[b][0].RecomputeMaxExtent();

		// Mirror front → back so either buffer is a valid starting state.
		for (int32 b = 0; b < NumBuffers; ++b)
			State.Buffers[b][1].CopyFrom(State.Buffers[b][0]);
	}

	if (Ctx.InitializationState == ELifecycleState::Pooling) return;

	// GT rendezvous: spawn Niagara components and activate.
	TPromise<void> Promise;
	TFuture<void> Future = Promise.GetFuture();

	// Capture context values needed on game thread (not the full Ctx reference).
	USceneComponent* AttachRoot = Ctx.AttachRoot;
	const bool bAbsolutePos = Ctx.bNiagaraAbsolutePosition;
	const FVector VT = Ctx.VirtualTraversal;

	AsyncTask(ENamedThreads::GameThread, [AttachRoot, bAbsolutePos, VT,
		&Config, &State, &OutComponents, NumBuffers,
		Promise = MoveTemp(Promise)]() mutable
		{
			State.NiagaraComponents.SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraSystem* Template = Config.NiagaraAssets[b];
				if (!Template)
				{
					UE_LOG(LogTemp, Warning, TEXT("FTierStreamingSystem::InitializeTier [%s] — NiagaraAssets[%d] not assigned."),
						*Config.TierName, b);
				}

				UNiagaraComponent* NC = UNiagaraFunctionLibrary::SpawnSystemAttached(
					Template, AttachRoot, NAME_None,
					FVector::ZeroVector, FRotator::ZeroRotator,
					EAttachLocation::SnapToTarget,
					!bAbsolutePos, // bAutoActivate for non-absolute; absolute activates after push
					false);

				if (NC)
				{
					if (bAbsolutePos)
						NC->SetAbsolute(true, false, false);

					const FBox Bounds = Config.ComputeBounds();
					NC->SetSystemFixedBounds(Bounds);
					NC->TranslucencySortPriority = -1000;
					NC->SetCustomDepthStencilValue(-1000);
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("FTierStreamingSystem::InitializeTier [%s] — Failed to create NiagaraComponent[%d]"),
						*Config.TierName, b);
				}

				State.NiagaraComponents[b] = NC;
				OutComponents.Add(NC);
			}

			// Push initial data and activate each component exactly once.
			const int32 FrontIdx = State.FrontIdx.load();
			const FBox Bounds = Config.ComputeBounds();
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraComponent* NC = State.NiagaraComponents[b];
				if (NC)
				{
					NC->SetSystemFixedBounds(Bounds);
					State.Buffers[b][FrontIdx].ActivateOnce(NC, VT);
				}
			}

			Promise.SetValue();
		});
	Future.Wait();

	State.FrontIdx.store(0);
	UE_LOG(LogTemp, Log, TEXT("FTierStreamingSystem::InitializeTier [%s] — %d slots, %d capacity, streaming=%d, %.3f sec"),
		*Config.TierName, TotalSlots, Config.SlotCapacity, bIsStreamingTier ? 1 : 0,
		FPlatformTime::Seconds() - StartTime);
}

// ============================================================================
//  UpdateTier
// ============================================================================
void FTierStreamingSystem::UpdateTier(
	const FTierStreamingContext& Ctx,
	FParticleTierConfig& Config,
	FParticleTierState& State)
{
	if (Ctx.InitializationState != ELifecycleState::Ready) return;
	if (Ctx.bRebaseInProgress) return;

	// Check that at least one Niagara component exists.
	bool bHasComponent = false;
	for (UNiagaraComponent* NC : State.NiagaraComponents)
	{
		if (NC) { bHasComponent = true; break; }
	}
	if (!bHasComponent) return;

	// If async generation completed, push the new front buffer.
	if (State.bNeedsPush.load())
	{
		PushTierToNiagara(Ctx, Config, State);
		State.bNeedsPush.store(false);
	}

	// No streaming for single-cell tiers (e.g. Galaxy Large with radius 0).
	if (Config.NeighborhoodRadius == 0) return;

	if (State.bUpdateInProgress.load()) return;

	const FIntVector NewCoord = PositionToGridCoord(
		Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);
	if (NewCoord == State.CenterCoord) return;

	UE_LOG(LogTemp, Verbose, TEXT("FTierStreamingSystem::UpdateTier [%s] — boundary cross: (%d,%d,%d) → (%d,%d,%d)"),
		*Config.TierName,
		State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z,
		NewCoord.X, NewCoord.Y, NewCoord.Z);

	State.bUpdateInProgress.store(true);
	const FIntVector OldCenter = State.CenterCoord;
	const bool bIsInitialPopulation = (OldCenter.X == INT32_MIN);
	State.CenterCoord = NewCoord;

	// Snapshot context values for the async task. The FTierStreamingContext
	// reference may alias stack or member data that changes between frames,
	// so we capture what we need by value.
	const double CtxExtent = Ctx.Extent;
	const double CtxUnitScale = Ctx.UnitScale;
	const double CtxGridExtentMultiplier = Ctx.GridExtentMultiplier;
	TSharedPtr<FOctree> CtxOctree = Ctx.Octree;
	const FString CtxOwnerName = Ctx.OwnerName;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask,
		[&Config, &State, OldCenter, NewCoord, bIsInitialPopulation,
		CtxExtent, CtxUnitScale, CtxGridExtentMultiplier, CtxOctree, CtxOwnerName]()
		{
			double StartTime = FPlatformTime::Seconds();
			const int32 R = Config.NeighborhoodRadius;
			const int32 NumBuffers = Config.NiagaraAssets.Num();
			const int32 FrontIdx = State.FrontIdx.load();
			const int32 BackIdx = 1 - FrontIdx;

			// Copy front → back; unchanged slots carry over untouched.
			for (int32 b = 0; b < NumBuffers; ++b)
				State.Buffers[b][BackIdx].CopyFrom(State.Buffers[b][FrontIdx]);

			// Diff old and new neighborhoods.
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			for (int32 dz = -R; dz <= R; ++dz)
				for (int32 dy = -R; dy <= R; ++dy)
					for (int32 dx = -R; dx <= R; ++dx)
					{
						const FIntVector Offset(dx, dy, dz);
						if (!bIsInitialPopulation)
							OldSet.Add(OldCenter + Offset);
						NewSet.Add(NewCoord + Offset);
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

			// Free exiting slots.
			const FVector DeadPos(CtxExtent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FSlotEntry* Entry = State.ActiveSlots.Find(Coord);
				if (!Entry) continue;
				for (int32 b = 0; b < NumBuffers; ++b)
					State.Buffers[b][BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
				State.FreeSlots.Add(Entry->SlotIndex);
				State.ActiveSlots.Remove(Coord);
			}

			// Fire optional boundary-cross hook (e.g. streaming volumetric).
			if (Config.OnBoundaryCross)
				Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);

			// Generate entering cells.
			int32 CacheHitCount = 0;
			TArray<TPair<FIntVector, int32>> ToGenerate;
			TArray<TPair<FIntVector, int32>> AllEnteringSlots;
			ToGenerate.Reserve(EnteringNodes.Num());
			AllEnteringSlots.Reserve(EnteringNodes.Num());

			for (const FIntVector& Coord : EnteringNodes)
			{
				if (State.FreeSlots.Num() == 0)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("FTierStreamingSystem::UpdateTier [%s] — no free slots; dropping cell (%d,%d,%d)"),
						*Config.TierName, Coord.X, Coord.Y, Coord.Z);
					continue;
				}

				const int32 SlotIndex = State.FreeSlots.Pop();
				FSlotEntry& Entry = State.ActiveSlots.Add(Coord);
				Entry.SlotIndex = SlotIndex;
				AllEnteringSlots.Emplace(Coord, SlotIndex);

				// Optional volume-culling: skip cells entirely outside the
				// actor's bounded region (e.g. galaxy volume).
				if (Config.ShouldSkipCell && Config.ShouldSkipCell(Coord))
				{
					State.SlotCounts[SlotIndex] = 0;
					for (int32 b = 0; b < NumBuffers; ++b)
						State.Buffers[b][BackIdx].PadSlotDead(SlotIndex, 0, DeadPos);
					continue;
				}

				// Cache-hit path.
				const FCachedCellData* Cached = State.CellCache.Find(Coord);
				if (Cached && Cached->ParticleCount > 0)
				{
					const int32 LiveCount = Cached->ParticleCount;
					State.SlotCounts[SlotIndex] = LiveCount;
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						FNiagaraParticleBuffer& Buf = State.Buffers[b][BackIdx];
						const int32 Start = SlotIndex * Config.SlotCapacity;
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
								Buf.Rotations[Idx] = CRot[i];
						}
						Buf.PadSlotDead(SlotIndex, LiveCount, DeadPos);
					}
					++CacheHitCount;
				}
				else
				{
					// Cache-miss: queue for generation.
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
					PerSlotBufferPtrs[i][b] = &State.Buffers[b][BackIdx];
			}

			ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i) {
				Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
				}, EParallelForFlags::BackgroundPriority);

			// Cache newly generated cells.
			for (const auto& Pair : ToGenerate)
				CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, BackIdx);

			// Build a temporary context for octree insert (async-safe snapshot).
			FTierStreamingContext InsertCtx;
			InsertCtx.Extent = CtxExtent;
			InsertCtx.UnitScale = CtxUnitScale;
			InsertCtx.GridExtentMultiplier = CtxGridExtentMultiplier;
			InsertCtx.Octree = CtxOctree;

			// Incremental octree insert for all entering cells.
			for (const auto& EnteringPair : AllEnteringSlots)
				InsertSlotIntoOctree(InsertCtx, Config, State, EnteringPair.Key, EnteringPair.Value, BackIdx);

			// Cache MaxExtent so PushTierToNiagara doesn't need a full scan.
			for (int32 b = 0; b < NumBuffers; ++b)
				State.Buffers[b][BackIdx].RecomputeMaxExtent();

			// Swap and signal.
			State.FrontIdx.store(BackIdx);
			State.bNeedsPush.store(true);
			CullTierCache(Config, State, NewCoord);
			State.bUpdateInProgress.store(false);

			UE_LOG(LogTemp, Verbose,
				TEXT("FTierStreamingSystem::UpdateTier [%s] — %d entering (%d cached, %d generated), %d exiting in %.3f sec"),
				*Config.TierName, EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(),
				ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

// ============================================================================
//  PushTierToNiagara
// ============================================================================
void FTierStreamingSystem::PushTierToNiagara(
	const FTierStreamingContext& Ctx,
	const FParticleTierConfig& Config,
	FParticleTierState& State)
{
	const int32 FrontIdx = State.FrontIdx.load();
	const FBox BaseBounds = Config.ComputeBounds();
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		UNiagaraComponent* NC = State.NiagaraComponents[b];
		if (!NC) continue;

		const auto& Buf = State.Buffers[b][FrontIdx];
		const FBox BufferBounds = BaseBounds.ExpandBy(static_cast<double>(Buf.MaxExtent));
		NC->SetSystemFixedBounds(BufferBounds);
		Buf.PushToNiagara(NC, Ctx.VirtualTraversal);
	}
}

// ============================================================================
//  PushTierPositions
// ============================================================================
void FTierStreamingSystem::PushTierPositions(
	std::initializer_list<FParticleTierState*> Tiers,
	const FVector& VirtualTraversal)
{
	for (FParticleTierState* Tier : Tiers)
	{
		if (!Tier) continue;
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

// ============================================================================
//  Octree Integration
// ============================================================================
void FTierStreamingSystem::InsertTierIntoOctree(
	const FTierStreamingContext& Ctx,
	const FParticleTierConfig& Config,
	FParticleTierState& State,
	int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Ctx.Octree.IsValid()) return;

	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex][BufferIdx];
	const double TreeExtent = Ctx.Octree->Extent;

	for (auto& Pair : State.ActiveSlots)
	{
		FSlotEntry& Entry = Pair.Value;
		const FIntVector& Coord = Pair.Key;
		Entry.InsertedNodes.Reset();
		const int32 Count = State.SlotCounts[Entry.SlotIndex];
		Entry.InsertedNodes.Reserve(Count);
		const int32 BufferStart = Entry.SlotIndex * Config.SlotCapacity;
		for (int32 i = 0; i < Count; ++i)
		{
			const int32 Idx = BufferStart + i;
			if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
			InsertParticleIntoOctree(Ctx, Entry, InsertBuffer.Positions[Idx],
				InsertBuffer.Extents[Idx], Coord, i, Idx, TreeExtent, Config.TierIndex);
		}
	}
}

void FTierStreamingSystem::InsertSlotIntoOctree(
	const FTierStreamingContext& Ctx,
	const FParticleTierConfig& Config,
	FParticleTierState& State,
	const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Ctx.Octree.IsValid()) return;

	FSlotEntry* Entry = State.ActiveSlots.Find(Coord);
	if (!Entry || Entry->SlotIndex != SlotIndex) return;

	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex][BufferIdx];
	const double TreeExtent = Ctx.Octree->Extent;
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	const int32 BufferStart = SlotIndex * Config.SlotCapacity;

	Entry->InsertedNodes.Reset();
	Entry->InsertedNodes.Reserve(LiveCount);

	for (int32 i = 0; i < LiveCount; ++i)
	{
		const int32 Idx = BufferStart + i;
		if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
		InsertParticleIntoOctree(Ctx, *Entry, InsertBuffer.Positions[Idx],
			InsertBuffer.Extents[Idx], Coord, i, Idx, TreeExtent, Config.TierIndex);
	}
}

void FTierStreamingSystem::InsertParticleIntoOctree(
	const FTierStreamingContext& Ctx,
	FSlotEntry& Entry, const FVector& Position, float Extent,
	const FIntVector& GridCoord, int32 GenerationIndex, int32 AbsoluteBufferIndex,
	double TreeExtent, int32 TierIndex)
{
	if (Extent <= 0.0f) return;

	FPointData PD = FPointData::MakePointDataFromWorldScale(
		static_cast<double>(Extent) * Ctx.UnitScale,
		Ctx.UnitScale,
		static_cast<int64>(TreeExtent));
	PD.SetPosition(Position);
	PD.Data.ObjectId = FVoxelData::ComposeSeed(Ctx.ParentSeed, GridCoord, GenerationIndex);
	PD.Data.TypeId = TierIndex;
	PD.Data.ParticleIndex = AbsoluteBufferIndex;

	TSharedPtr<FOctreeNode> Node = Ctx.Octree->InsertPosition(
		PD.GetPosition(), PD.InsertDepth, PD.Data);
	if (Node.IsValid())
		Entry.InsertedNodes.Add(Node);
}

// ============================================================================
//  Cell Cache
// ============================================================================
void FTierStreamingSystem::CacheCellFromBuffers(
	const FParticleTierConfig& Config,
	FParticleTierState& State,
	const FIntVector& Coord,
	int32 SlotIndex, int32 BufferIdx)
{
	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 LiveCount = State.SlotCounts[SlotIndex];

	FCachedCellData& Cache = State.CellCache.FindOrAdd(Coord);
	Cache.ParticleCount = LiveCount;
	Cache.CenterOffset = FVector::ZeroVector;
	Cache.PerBufferPositions.SetNum(NumBuffers);
	Cache.PerBufferExtents.SetNum(NumBuffers);
	Cache.PerBufferColors.SetNum(NumBuffers);
	Cache.PerBufferRotations.SetNum(NumBuffers);

	for (int32 b = 0; b < NumBuffers; ++b)
	{
		const FNiagaraParticleBuffer& Buf = State.Buffers[b][BufferIdx];
		const int32 Start = SlotIndex * Config.SlotCapacity;

		TArray<FVector>& CPos = Cache.PerBufferPositions[b];
		TArray<float>& CExt = Cache.PerBufferExtents[b];
		TArray<FLinearColor>& CCol = Cache.PerBufferColors[b];
		TArray<FVector>& CRot = Cache.PerBufferRotations[b];

		CPos.SetNumUninitialized(LiveCount);
		CExt.SetNumUninitialized(LiveCount);
		CCol.SetNumUninitialized(LiveCount);

		if (Buf.Rotations.Num() > 0)
			CRot.SetNumUninitialized(LiveCount);
		else
			CRot.Empty();

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

void FTierStreamingSystem::CullTierCache(
	const FParticleTierConfig& Config,
	FParticleTierState& State,
	const FIntVector& NewCenter)
{
	const int32 CullRadius = Config.NeighborhoodRadius + 4;
	TArray<FIntVector> ToEvict;
	for (const auto& Pair : State.CellCache)
	{
		const FIntVector Delta = Pair.Key - NewCenter;
		const int32 ChebyshevDist = FMath::Max3(
			FMath::Abs(Delta.X), FMath::Abs(Delta.Y), FMath::Abs(Delta.Z));
		if (ChebyshevDist > CullRadius)
			ToEvict.Add(Pair.Key);
	}
	for (const FIntVector& Coord : ToEvict)
		State.CellCache.Remove(Coord);

	if (ToEvict.Num() > 0)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("FTierStreamingSystem::CullTierCache [%s] — evicted %d entries, %d remaining"),
			*Config.TierName, ToEvict.Num(), State.CellCache.Num());
	}
}
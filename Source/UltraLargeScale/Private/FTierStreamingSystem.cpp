#include "FTierStreamingSystem.h"
#include "UltraLargeScale.h"
#include "Misc/ScopeLock.h"
#include "Misc/ScopeTryLock.h"

#pragma region InitializeTier
void FTierStreamingSystem::InitializeTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State, TArray<UNiagaraComponent*>& OutComponents)
{
	double StartTime = FPlatformTime::Seconds();

	//Abort checks inspect live state via GetLiveState - TODO: Get rid of branching here... use one unified interface to access lifecycle states
	auto IsPooling = [&Ctx]() { return (Ctx.GetLiveState ? Ctx.GetLiveState() : Ctx.InitializationState) == ELifecycleState::Pooling; };

	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 Side = 2 * Config.NeighborhoodRadius + 1;
	const int32 TotalSlots = Side * Side * Side;

	// Toroidal addressing invariant: Side must be odd on every axis so the window has a well-defined center cell and Side==1 is a clean degenerate subset.
	checkf((Side % 2) == 1 && Config.NeighborhoodRadius >= 0, TEXT("InitializeTier [%s]: Side (=2R+1) must be odd and positive, got %d"), *Config.TierName, Side);

	// Allocate particle data: ONE buffer per Niagara asset (single-buffered; see FParticleTierState::Buffers for why no back buffer is needed).
	State.Buffers.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		bool bRotations = Config.bWantRotations.IsValidIndex(b) && Config.bWantRotations[b];
		State.Buffers[b].Allocate(TotalSlots, Config.SlotCapacity, bRotations);
	}

	State.SlotCounts.SetNumZeroed(TotalSlots);
	State.SlotEntries.Empty(TotalSlots);
	State.SlotEntries.SetNum(TotalSlots);
	State.StampedCenter = FIntVector(INT32_MIN);
	State.StampedNCenter = FVector::ZeroVector;

	// Streaming tiers (NeighborhoodRadius > 0) defer initial population to the first UpdateTier call by setting CenterCoord = INT32_MIN. Exhaustive tiers (radius 0) generate immediately.
	const bool bIsStreamingTier = (Config.NeighborhoodRadius > 0);

	if (bIsStreamingTier)
	{
		State.CenterCoord = FIntVector(INT32_MIN, INT32_MIN, INT32_MIN);
	}
	else
	{
		// Exhaustive tier: generate the full (usually 1x1x1) window immediately.
		State.CenterCoord = PositionToGridCoord(Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);

		// Stamp the center the lattice/uniform derive from. Radius-0 tiers never stream, so this stamp is permanent for the actor's lifetime.
		State.StampedCenter = State.CenterCoord;
		State.StampedNCenter = GridCoordToCenter(State.CenterCoord, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);

		TArray<TPair<FIntVector, int32>> ToGenerate;
		for (int32 dz = -Config.NeighborhoodRadius; dz <= Config.NeighborhoodRadius; ++dz)
			for (int32 dy = -Config.NeighborhoodRadius; dy <= Config.NeighborhoodRadius; ++dy)
				for (int32 dx = -Config.NeighborhoodRadius; dx <= Config.NeighborhoodRadius; ++dx)
				{
					const FIntVector Coord = State.CenterCoord + FIntVector(dx, dy, dz);
					const int32 SlotIndex = SlotOf(Coord, Side);
					ToGenerate.Add({ Coord, SlotIndex });

					// Record the resident coord + derived center for the slot.
					const FVector InitCellCenter = GridCoordToCenter(Coord, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);
					for (int32 bb = 0; bb < NumBuffers; ++bb)
					{
						State.Buffers[bb].SlotCoord[SlotIndex] = Coord;
						State.Buffers[bb].SlotCenters[SlotIndex] = InitCellCenter;
					}
				}

		TArray<TArray<FNiagaraParticleBuffer*>> PerSlotBufferPtrs;
		PerSlotBufferPtrs.SetNum(ToGenerate.Num());
		for (int32 i = 0; i < ToGenerate.Num(); ++i)
		{
			PerSlotBufferPtrs[i].SetNum(NumBuffers);

			for (int32 b = 0; b < NumBuffers; ++b)
				PerSlotBufferPtrs[i][b] = &State.Buffers[b];
		}

		ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i) 
			{
				Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
			}, EParallelForFlags::BackgroundPriority);

		if (IsPooling()) return;

		InsertTierIntoOctree(Ctx, Config, State);

		for (const auto& Pair : ToGenerate)
			CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value);
	}

	if (IsPooling()) return;

	// GT rendezvous: spawn Niagara components and activate.
	TPromise<void> Promise;
	TFuture<void> Future = Promise.GetFuture();

	// Capture context values needed on game thread (not the full Ctx reference).
	USceneComponent* AttachRoot = Ctx.AttachRoot;
	const bool bAbsolutePos = Ctx.bNiagaraAbsolutePosition;
	const FVector VT = Ctx.VirtualTraversal;
	
	// Backdrop membership, sourced from the actor's REAL UnitScale via IsVirtualSpace()
	const bool bVirtualSpace = Ctx.bVirtualSpace;

	// Radius-0 tiers stamped their permanent center above; streaming tiers are still at the sentinel (ZeroVector NCenter)
	const FVector NCenterSeed = State.StampedNCenter;

	AsyncTask(ENamedThreads::GameThread, [AttachRoot, bAbsolutePos, VT, NCenterSeed, bVirtualSpace, &Config, &State, &OutComponents, NumBuffers, Promise = MoveTemp(Promise)]() mutable
		{
			State.NiagaraComponents.SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraSystem* Template = Config.NiagaraAssets[b];
				if (!Template)
					UE_LOG(LogTemp, Warning, TEXT("FTierStreamingSystem::InitializeTier [%s] - NiagaraAssets[%d] not assigned."), *Config.TierName, b);

				UNiagaraComponent* NC = UNiagaraFunctionLibrary::SpawnSystemAttached(Template, AttachRoot, NAME_None, FVector::ZeroVector, FRotator::ZeroRotator, EAttachLocation::SnapToTarget, !bAbsolutePos, false);
				if (NC)
				{
					// Virtual backdrop: hidden in the main renderer, visible only to the backdrop SceneCapture.
					NC->bVisibleInSceneCaptureOnly = bVirtualSpace;

					if (bAbsolutePos)
						NC->SetAbsolute(true, false, false);

					// Fixed bounds are set once here and never updated. The box surrounds the camera, so it can't be frustum-culled.
					const float boundsMulti = 1.1; // Slightly scale up bounds to avoid particle overhang
					const FBox Base = Config.ComputeBounds();
					const FBox Bounds(Base.Min * boundsMulti, Base.Max * boundsMulti);
					NC->SetSystemFixedBounds(Bounds);
					NC->TranslucencySortPriority = -1000;
					NC->SetCustomDepthStencilValue(-1000);
					NC->SetVariableInt(NiagaraBufferParams::SlotCapacity, Config.SlotCapacity);
				}
				else
				{
					UE_LOG(LogTemp, Error, TEXT("FTierStreamingSystem::InitializeTier [%s] - Failed to create NiagaraComponent[%d]"),
						*Config.TierName, b);
				}

				State.NiagaraComponents[b] = NC;
				OutComponents.Add(NC);
			}

			for (int32 b = 0; b < NumBuffers; ++b)
			{
				if (UNiagaraSystem* Template = Config.NiagaraAssets[b])
					if (!Template->IsReadyToRun())
						Template->WaitForCompilationComplete();
			}

			// Push initial data and activate each component exactly once.
			const FBox Base = Config.ComputeBounds();
			const FBox Bounds(Base.Min * 2.0, Base.Max * 2.0);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraComponent* NC = State.NiagaraComponents[b];
				if (NC)
				{
					NC->SetSystemFixedBounds(Bounds);
					State.Buffers[b].ActivateOnce(NC, VT, NCenterSeed);
				}
			}

			Promise.SetValue();
		});

	Future.Wait();

	UE_LOG(LogTemp, Log, TEXT("FTierStreamingSystem::InitializeTier [%s] - %d slots, %d capacity, streaming=%d, %.3f sec"), *Config.TierName, TotalSlots, Config.SlotCapacity, bIsStreamingTier ? 1 : 0, FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region UpdateTier
void FTierStreamingSystem::UpdateTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State)
{
	SVO_GT_SCOPE("Tier::UpdateTier");
	if (Ctx.InitializationState != ELifecycleState::Ready) return;
	if (Ctx.bRebaseInProgress) return;

	// Check that at least one Niagara component exists.
	bool bHasComponent = false;
	for (UNiagaraComponent* NC : State.NiagaraComponents)
	{
		if (NC) {
			bHasComponent = true;
			break;
		}
	}

	if (!bHasComponent) return;

	// The full boundary-cross push happens at the tail of the async generation task (below), on the worker thread, while bUpdateInProgress is still held. It never runs on the game thread.

	// No streaming for single-cell tiers (e.g. Galaxy Large with radius 0).
	if (Config.NeighborhoodRadius == 0) return;

	if (State.bUpdateInProgress.load()) return;

	const FIntVector NewCoord = PositionToGridCoord(Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);
	
	if (NewCoord == State.CenterCoord) return;

	if (Config.ShouldSkipCell && Config.ShouldSkipCell(NewCoord)) return;

	UE_LOG(LogTemp, Verbose, TEXT("FTierStreamingSystem::UpdateTier [%s] - boundary cross: (%d,%d,%d) -> (%d,%d,%d)"), *Config.TierName, State.CenterCoord.X, State.CenterCoord.Y, State.CenterCoord.Z, NewCoord.X, NewCoord.Y, NewCoord.Z);

	State.bUpdateInProgress.store(true);

	const FIntVector OldCenter = State.CenterCoord;
	const bool bIsInitialPopulation = (OldCenter.X == INT32_MIN);
	State.CenterCoord = NewCoord;

	// Snapshot context values for the async task.
	const double CtxExtent = Ctx.Extent;
	const double CtxUnitScale = Ctx.UnitScale;
	const double CtxGridExtentMultiplier = Ctx.GridExtentMultiplier;
	TSharedPtr<FOctree> CtxOctree = Ctx.Octree;

	// The tail push reads the freshest VirtualTraversal at execution time via this accessor (guarded on the actor), not a value captured now: a generation can span several frames, and re-seeding the GPU with a stale VT would jitter the boundary cross.
	TFunction<FVector()> GetLatestVT = Ctx.GetLatestVT;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask,
		[&Config, &State, OldCenter, NewCoord, bIsInitialPopulation,
		CtxExtent, CtxUnitScale, CtxGridExtentMultiplier, CtxOctree,
		GetLatestVT]()
		{
			double StartTime = FPlatformTime::Seconds();
			const int32 R = Config.NeighborhoodRadius;
			const int32 Side = 2 * R + 1;
			const int32 NumBuffers = Config.NiagaraAssets.Num();

			// SINGLE BUFFER, IN-PLACE: entering slots are overwritten directly in the live CPU arrays.

			// Diff old and new neighborhoods.
			const int32 TotalSlots = Side * Side * Side;
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;
			OldSet.Reserve(TotalSlots);
			NewSet.Reserve(TotalSlots);
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
				if (!NewSet.Contains(Coord)) ExitingNodes.Add(Coord);

			for (const FIntVector& Coord : NewSet)
				if (!OldSet.Contains(Coord)) EnteringNodes.Add(Coord);

			// Fire optional boundary-cross hook (e.g. streaming volumetric).
			if (Config.OnBoundaryCross)
				Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);

			// Generate entering cells into their MODULAR slots. slot(coord) is a bijection over the new window, so no free-list is needed and no cell can ever be dropped for lack of a slot.
			int32 CacheHitCount = 0;
			TArray<TPair<FIntVector, int32>> ToGenerate;
			TArray<TPair<FIntVector, int32>> AllEnteringSlots;
			TArray<int32> EnteringSlots;
			ToGenerate.Reserve(EnteringNodes.Num());
			AllEnteringSlots.Reserve(EnteringNodes.Num());
			EnteringSlots.Reserve(EnteringNodes.Num());

			TArray<bool> SlotClaimed;
			SlotClaimed.Init(false, TotalSlots);

			for (const FIntVector& Coord : EnteringNodes)
			{
				const int32 SlotIndex = SlotOf(Coord, Side);
				checkf(!SlotClaimed[SlotIndex], TEXT("UpdateTier [%s]: modular slot collision at slot %d for cell (%d,%d,%d) - bijection violated"), *Config.TierName, SlotIndex, Coord.X, Coord.Y, Coord.Z);
				SlotClaimed[SlotIndex] = true;

				AllEnteringSlots.Emplace(Coord, SlotIndex);
				EnteringSlots.Add(SlotIndex);

				// Record the new resident coord + derived cell center.
				const FVector EnterCellCenter = GridCoordToCenter(
					Coord, Config.GridDepth, CtxExtent, CtxGridExtentMultiplier);
				for (int32 b = 0; b < NumBuffers; ++b)
				{
					State.Buffers[b].SlotCoord[SlotIndex] = Coord;
					State.Buffers[b].SlotCenters[SlotIndex] = EnterCellCenter;
				}

				// Optional volume-culling: skip cells entirely outside the actor's bounded region (e.g. galaxy volume).
				if (Config.ShouldSkipCell && Config.ShouldSkipCell(Coord))
				{
					State.SlotCounts[SlotIndex] = 0;
					for (int32 b = 0; b < NumBuffers; ++b)
						State.Buffers[b].PadSlotDead(SlotIndex, 0);
					continue;
				}

				// Cache-hit path
				const FCachedCellData* Cached = State.CellCache.Find(Coord);
				if (Cached)
				{
					const int32 LiveCount = Cached->ParticleCount;
					State.SlotCounts[SlotIndex] = LiveCount;
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						FNiagaraParticleBuffer& Buf = State.Buffers[b];
						const int32 Start = SlotIndex * Config.SlotCapacity;
						if (LiveCount > 0)
						{
							const TArray<FVector>& CPos = Cached->PerBufferPositions[b];
							const TArray<float>& CExt = Cached->PerBufferExtents[b];
							const TArray<FLinearColor>& CCol = Cached->PerBufferColors[b];
							const TArray<FVector>& CRot = Cached->PerBufferRotations[b];
							FMemory::Memcpy(&Buf.Positions[Start], CPos.GetData(), LiveCount * sizeof(FVector));
							FMemory::Memcpy(&Buf.Extents[Start], CExt.GetData(), LiveCount * sizeof(float));
							FMemory::Memcpy(&Buf.Colors[Start], CCol.GetData(), LiveCount * sizeof(FLinearColor));
							
							if (Buf.Rotations.Num() > 0 && CRot.Num() >= LiveCount)
								FMemory::Memcpy(&Buf.Rotations[Start], CRot.GetData(), LiveCount * sizeof(FVector));
						}
						Buf.PadSlotDead(SlotIndex, LiveCount);
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
					PerSlotBufferPtrs[i][b] = &State.Buffers[b];
			}

			ParallelFor(ToGenerate.Num(), [&Config, &ToGenerate, &PerSlotBufferPtrs](int32 i)
				{
				Config.GenerateCallback(ToGenerate[i].Key, ToGenerate[i].Value, PerSlotBufferPtrs[i]);
				}, EParallelForFlags::BackgroundPriority);

			// Cache newly generated cells.
			for (const auto& Pair : ToGenerate)
				CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value);

			// Build a temporary context for octree insert (async-safe snapshot).
			FTierStreamingContext InsertCtx;
			InsertCtx.Extent = CtxExtent;
			InsertCtx.UnitScale = CtxUnitScale;
			InsertCtx.GridExtentMultiplier = CtxGridExtentMultiplier;
			InsertCtx.Octree = CtxOctree;

			// Incremental octree insert for all entering cells.
			for (const auto& EnteringPair : AllEnteringSlots)
				InsertSlotIntoOctree(InsertCtx, Config, State, EnteringPair.Key, EnteringPair.Value);

			// Commit: stamp the new center and upload arrays + lattice + uniform, all inside the tier PushCS, so a concurrent per-frame push can never pair a stale VT or a lattice/uniform built against different centers. bUpdateInProgress is still held.
			const FVector NewNCenter = GridCoordToCenter(NewCoord, Config.GridDepth, CtxExtent, CtxGridExtentMultiplier);
			PushTierToNiagara(GetLatestVT, NewCoord, NewNCenter, Config, State);
			CullTierCache(Config, State, NewCoord);
			State.bUpdateInProgress.store(false);

			UE_LOG(LogTemp, Verbose, TEXT("FTierStreamingSystem::UpdateTier [%s] - %d entering (%d cached, %d generated), %d exiting in %.3f sec"), *Config.TierName, EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(), ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}
#pragma endregion

#pragma region PushTierToNiagara
// Publishes the transition by uploading the in-place-updated buffer, and makes it atomic with respect to every other Niagara write.
void FTierStreamingSystem::PushTierToNiagara(const TFunction<FVector()>& GetLatestVT, const FIntVector& NewCenter, const FVector& NewNCenter, const FParticleTierConfig& Config, FParticleTierState& State)
{
	FScopeLock Lock(&State.PushCS);
	if (State.bShuttingDown.load()) return;

	State.StampedCenter = NewCenter;
	State.StampedNCenter = NewNCenter;

	const FVector VirtualTraversal = GetLatestVT();
	TArray<UNiagaraComponent*> Comps = State.NiagaraComponents;
	AsyncTask(ENamedThreads::GameThread,
		[&State, Comps, VirtualTraversal, NewNCenter, NumBuffers = Config.NiagaraAssets.Num()]()
		{
			for (int32 b = 0; b < NumBuffers && b < Comps.Num(); ++b)
				if (UNiagaraComponent* NC = Comps[b])
					State.Buffers[b].PushToNiagara(NC, VirtualTraversal, NewNCenter);
		});
}
#pragma endregion

#pragma region PushTierVT
void FTierStreamingSystem::PushTierVT(std::initializer_list<FParticleTierState*> Tiers, const TFunction<FVector()>& GetLatestVT)
{
	for (FParticleTierState* Tier : Tiers)
	{
		if (!Tier) continue;

		FScopeTryLock Lock(&Tier->PushCS);
		if (!Lock.IsLocked()) continue;
		if (Tier->bShuttingDown.load()) continue;

		const FVector VirtualTraversal = GetLatestVT();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;

			Tier->Buffers[b].PushNCenterMinusVT(NC, Tier->StampedNCenter, VirtualTraversal);
		}
	}
}
#pragma endregion

#pragma region BeginShutdownDrain
// Marks the tier as shutting down, then acquires PushCS once to block until any in-flight push has released it.
void FTierStreamingSystem::BeginShutdownDrain(FParticleTierState& State)
{
	State.bShuttingDown.store(true);
	FScopeLock Lock(&State.PushCS);
}
#pragma endregion

#pragma region Octree Integration
void FTierStreamingSystem::InsertTierIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Ctx.Octree.IsValid()) return;

	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex];
	const double TreeExtent = Ctx.Octree->Extent;

	for (int32 SlotIndex = 0; SlotIndex < InsertBuffer.TotalSlots; ++SlotIndex)
	{
		if (!InsertBuffer.IsSlotOccupied(SlotIndex)) continue;
		const FIntVector Coord = InsertBuffer.SlotCoord[SlotIndex];
		FSlotEntry& Entry = State.SlotEntries[SlotIndex];
		Entry.InsertedNodes.Reset();
		const int32 Count = State.SlotCounts[SlotIndex];
		Entry.InsertedNodes.Reserve(Count);
		const int32 BufferStart = SlotIndex * Config.SlotCapacity;
		for (int32 i = 0; i < Count; ++i)
		{
			const int32 Idx = BufferStart + i;
			if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
			InsertParticleIntoOctree(Ctx, Entry, InsertBuffer.Positions[Idx], InsertBuffer.Extents[Idx], InsertBuffer.Colors[Idx], Coord, i, Idx, TreeExtent, Config.TierIndex);
		}
	}
}

void FTierStreamingSystem::InsertSlotIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex)
{
	if (Config.OctreeInsertBufferIndex < 0 || !Ctx.Octree.IsValid()) return;

	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex];

	// Residency check: the buffer's SlotCoord is the per-slot source of truth for which cell owns the slot.
	if (!InsertBuffer.SlotCoord.IsValidIndex(SlotIndex) || InsertBuffer.SlotCoord[SlotIndex] != Coord) return;
	if (!State.SlotEntries.IsValidIndex(SlotIndex)) return;

	const double TreeExtent = Ctx.Octree->Extent;
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	const int32 BufferStart = SlotIndex * Config.SlotCapacity;

	// Resetting here releases the shared node refs held for the slot's previous occupant (the congruent exiting cell); this is the slot's exit-time release of octree node references.
	FSlotEntry& Entry = State.SlotEntries[SlotIndex];
	Entry.InsertedNodes.Reset();
	Entry.InsertedNodes.Reserve(LiveCount);

	for (int32 i = 0; i < LiveCount; ++i)
	{
		const int32 Idx = BufferStart + i;
		if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
		InsertParticleIntoOctree(Ctx, Entry, InsertBuffer.Positions[Idx], InsertBuffer.Extents[Idx], InsertBuffer.Colors[Idx], Coord, i, Idx, TreeExtent, Config.TierIndex);
	}
}

void FTierStreamingSystem::InsertParticleIntoOctree(const FTierStreamingContext& Ctx, FSlotEntry& Entry, const FVector& Position, const float& Extent, const FLinearColor& Color, const FIntVector& GridCoord, int32 GenerationIndex, int32 AbsoluteBufferIndex, double TreeExtent, int32 TierIndex)
{
	if (Extent <= 0.0f) return;

	FPointData PD = FPointData::MakePointData(Position, static_cast<double>(Extent) * Ctx.UnitScale, Ctx.UnitScale, static_cast<int64>(TreeExtent), TierIndex, FVector(Color.R, Color.B, Color.G));

	PD.Data.Seed = FVoxelData::ComposeSeed(Ctx.ParentSeed, GridCoord, GenerationIndex);

	PD.Data.ParticleIndex = AbsoluteBufferIndex;
	PD.Data.ParticlePosition = Position;
	PD.Data.ParticleExtent = Extent;

	TSharedPtr<FOctreeNode> Node = Ctx.Octree->InsertPosition(PD.GetPosition(), PD.InsertDepth, PD.Data);

	if (Node.IsValid()) Entry.InsertedNodes.Add(Node);
}
#pragma endregion

#pragma region Cell Cache
void FTierStreamingSystem::CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex)
{
	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 LiveCount = State.SlotCounts[SlotIndex];

	FCachedCellData& Cache = State.CellCache.FindOrAdd(Coord);
	Cache.ParticleCount = LiveCount;
	Cache.PerBufferPositions.SetNum(NumBuffers);
	Cache.PerBufferExtents.SetNum(NumBuffers);
	Cache.PerBufferColors.SetNum(NumBuffers);
	Cache.PerBufferRotations.SetNum(NumBuffers);

	for (int32 b = 0; b < NumBuffers; ++b)
	{
		const FNiagaraParticleBuffer& Buf = State.Buffers[b];
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

void FTierStreamingSystem::CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& NewCenter)
{
	const int32 CullRadius = Config.NeighborhoodRadius + 4;
	TArray<FIntVector> ToEvict;
	for (const auto& Pair : State.CellCache)
	{
		const FIntVector Delta = Pair.Key - NewCenter;
		const int32 ChebyshevDist = FMath::Max3(FMath::Abs(Delta.X), FMath::Abs(Delta.Y), FMath::Abs(Delta.Z));
		if (ChebyshevDist > CullRadius) 
			ToEvict.Add(Pair.Key);
	}

	for (const FIntVector& Coord : ToEvict)
		State.CellCache.Remove(Coord);

	if (ToEvict.Num() > 0)
		UE_LOG(LogTemp, Verbose, TEXT("FTierStreamingSystem::CullTierCache [%s] - evicted %d entries, %d remaining"), *Config.TierName, ToEvict.Num(), State.CellCache.Num());
}
#pragma endregion
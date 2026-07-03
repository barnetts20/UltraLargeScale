#include "FTierStreamingSystem.h"
#include "svo.h"
#include "Misc/ScopeLock.h"
#include "Misc/ScopeTryLock.h"

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

	// Toroidal addressing invariant: Side must be odd on every axis so the
	// window has a well-defined center cell and Side==1 is a clean degenerate
	// subset. Trivially true with a scalar radius — asserted so a future
	// per-axis radius can't silently break the bijection.
	checkf((Side % 2) == 1 && Config.NeighborhoodRadius >= 0,
		TEXT("InitializeTier [%s]: Side (=2R+1) must be odd and positive, got %d"),
		*Config.TierName, Side);

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
	State.SlotEntries.Empty(TotalSlots);
	State.SlotEntries.SetNum(TotalSlots);
	State.StampedCenter = FIntVector(INT32_MIN);
	State.StampedNCenter = FVector::ZeroVector;

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
		// Exhaustive tier: generate the full (usually 1x1x1) window immediately.
		State.CenterCoord = PositionToGridCoord(
			Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);

		// Stamp the center the lattice/uniform derive from. Radius-0 tiers
		// never stream, so this stamp is permanent for the actor's lifetime.
		State.StampedCenter = State.CenterCoord;
		State.StampedNCenter = GridCoordToCenter(
			State.CenterCoord, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);

		TArray<TPair<FIntVector, int32>> ToGenerate;
		for (int32 dz = -Config.NeighborhoodRadius; dz <= Config.NeighborhoodRadius; ++dz)
			for (int32 dy = -Config.NeighborhoodRadius; dy <= Config.NeighborhoodRadius; ++dy)
				for (int32 dx = -Config.NeighborhoodRadius; dx <= Config.NeighborhoodRadius; ++dx)
				{
					const FIntVector Coord = State.CenterCoord + FIntVector(dx, dy, dz);
					const int32 SlotIndex = SlotOf(Coord, Side);
					ToGenerate.Add({ Coord, SlotIndex });

					// Record the resident coord + derived center for the slot.
					// Written into buffer[0]; the CopyFrom below mirrors both
					// into buffer[1] so either buffer is a valid start state.
					const FVector InitCellCenter = GridCoordToCenter(
						Coord, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);
					for (int32 bb = 0; bb < NumBuffers; ++bb)
					{
						State.Buffers[bb][0].SlotCoord[SlotIndex] = Coord;
						State.Buffers[bb][0].SlotCenters[SlotIndex] = InitCellCenter;
					}
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
	// Radius-0 tiers stamped their permanent center above; streaming tiers are
	// still at the sentinel (ZeroVector NCenter) — all their particles are dead
	// until the first UpdateTier commit, so the seed uniform is inert.
	const FVector NCenterSeed = State.StampedNCenter;

	AsyncTask(ENamedThreads::GameThread, [AttachRoot, bAbsolutePos, VT, NCenterSeed,
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
					NC->SetVariableInt(NiagaraBufferParams::SlotCapacity, Config.SlotCapacity);
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
					State.Buffers[b][FrontIdx].ActivateOnce(NC, VT, NCenterSeed);
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
	SVO_GT_SCOPE("Tier::UpdateTier");
	if (Ctx.InitializationState != ELifecycleState::Ready) return;
	if (Ctx.bRebaseInProgress) return;

	// Check that at least one Niagara component exists.
	bool bHasComponent = false;
	for (UNiagaraComponent* NC : State.NiagaraComponents)
	{
		if (NC) { bHasComponent = true; break; }
	}
	if (!bHasComponent) return;

	// NOTE: the full boundary-cross push now happens at the tail of the async
	// generation task (below), on the worker thread, while bUpdateInProgress is
	// still held. It is never run on the game thread.

	// No streaming for single-cell tiers (e.g. Galaxy Large with radius 0).
	if (Config.NeighborhoodRadius == 0) return;

	if (State.bUpdateInProgress.load()) return;

	const FIntVector NewCoord = PositionToGridCoord(
		Ctx.VirtualTraversal, Config.GridDepth, Ctx.Extent, Ctx.GridExtentMultiplier);
	if (NewCoord == State.CenterCoord) return;

	// STREAMING GATE: if the player's own cell is skippable (outside the
	// actor's volume), admit no transition — the resident window FREEZES at
	// its last in-bounds center. Live edge cells persist as a boundary halo
	// and keep parallaxing via the per-frame uniform (whose StampedNCenter is
	// likewise frozen, so lattice and uniform stay mutually consistent).
	// This also suppresses the rapid crossing churn of traversing the grid at
	// intergalactic speeds outside the volume: the gate runs before any state
	// mutation or task spawn, so an out-of-bounds crossing costs one AABB
	// test. Re-entry resumes normally — a multi-cell delta lands in the
	// teleport path (Entering == whole window). Cell-granular by construction,
	// so it cannot flap without an actual boundary cross. Radius-0 tiers
	// never reach this (early-out above) and stay resident at any distance.
	if (Config.ShouldSkipCell && Config.ShouldSkipCell(NewCoord)) return;

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

	// The tail push reads the freshest VirtualTraversal at execution time via this
	// accessor (guarded on the actor), NOT a value captured now -- a generation can
	// span several frames, and re-seeding the GPU with a stale VT is exactly what
	// caused the boundary-cross jitter.
	TFunction<FVector()> GetLatestVT = Ctx.GetLatestVT;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask,
		[&Config, &State, OldCenter, NewCoord, bIsInitialPopulation,
		CtxExtent, CtxUnitScale, CtxGridExtentMultiplier, CtxOctree, CtxOwnerName,
		GetLatestVT]()
		{
			double StartTime = FPlatformTime::Seconds();
			const int32 R = Config.NeighborhoodRadius;
			const int32 Side = 2 * R + 1;
			const int32 NumBuffers = Config.NiagaraAssets.Num();
			const int32 FrontIdx = State.FrontIdx.load();
			const int32 BackIdx = 1 - FrontIdx;

			// NO front → back CopyFrom. Every prior commit mirrored its
			// Entering plane into both buffers (PushTierToNiagara step 4), so
			// the back buffer's resident slots are already byte-identical to
			// the front's. The back buffer serves as the off-lock scratch the
			// Entering plane is generated into; nothing observes it until the
			// FrontIdx flip inside the commit. Resident cells are never
			// copied or regenerated — that is the point of the torus.

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

			// Exiting cells need no slot bookkeeping: each exiting coord is
			// congruent mod Side to exactly one entering coord (the residue
			// sets of any two Side-wide windows are identical), so its slot is
			// fully overwritten by that entering cell below — generation,
			// cache-restore, and skip paths all write or dead-pad the whole
			// slot. Octree node refs from the previous occupant are released
			// when InsertSlotIntoOctree resets the slot's entry. ExitingNodes
			// is still computed for the OnBoundaryCross hook and cache
			// semantics, mirroring the old pipeline.
			const FVector DeadPos(CtxExtent * 10.0);

			// Fire optional boundary-cross hook (e.g. streaming volumetric).
			if (Config.OnBoundaryCross)
				Config.OnBoundaryCross(EnteringNodes, ExitingNodes, NewCoord);

			// Generate entering cells into their MODULAR slots. slot(coord) is
			// a bijection over the new window, so no free-list is needed and
			// no cell can ever be dropped for lack of a slot.
			int32 CacheHitCount = 0;
			TArray<TPair<FIntVector, int32>> ToGenerate;
			TArray<TPair<FIntVector, int32>> AllEnteringSlots;
			TArray<int32> EnteringSlots;
			ToGenerate.Reserve(EnteringNodes.Num());
			AllEnteringSlots.Reserve(EnteringNodes.Num());
			EnteringSlots.Reserve(EnteringNodes.Num());

#if DO_CHECK
			TArray<bool> SlotClaimed;
			SlotClaimed.Init(false, Side * Side * Side);
#endif

			for (const FIntVector& Coord : EnteringNodes)
			{
				const int32 SlotIndex = SlotOf(Coord, Side);
#if DO_CHECK
				checkf(!SlotClaimed[SlotIndex],
					TEXT("UpdateTier [%s]: modular slot collision at slot %d for cell (%d,%d,%d) — bijection violated"),
					*Config.TierName, SlotIndex, Coord.X, Coord.Y, Coord.Z);
				SlotClaimed[SlotIndex] = true;
#endif
				AllEnteringSlots.Emplace(Coord, SlotIndex);
				EnteringSlots.Add(SlotIndex);

				// Record the new resident coord + derived cell center in the
				// back (scratch) buffers. The commit mirrors these into the
				// other buffer together with the particle data.
				const FVector EnterCellCenter = GridCoordToCenter(
					Coord, Config.GridDepth, CtxExtent, CtxGridExtentMultiplier);
				for (int32 b = 0; b < NumBuffers; ++b)
				{
					State.Buffers[b][BackIdx].SlotCoord[SlotIndex] = Coord;
					State.Buffers[b][BackIdx].SlotCenters[SlotIndex] = EnterCellCenter;
				}

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

			// Commit: publish the buffer, stamp the new center, upload
			// lattice + uniform + arrays, and mirror the Entering plane into
			// the other buffer — all inside the tier PushCS, so a concurrent
			// per-frame push can never observe a half-published buffer, a
			// stale VT, or a lattice/uniform pair from different centers.
			// bUpdateInProgress is still held here.
			const FVector NewNCenter = GridCoordToCenter(
				NewCoord, Config.GridDepth, CtxExtent, CtxGridExtentMultiplier);
			PushTierToNiagara(GetLatestVT, BackIdx, EnteringSlots, NewCoord, NewNCenter,
				Config, State);
			CullTierCache(Config, State, NewCoord);
			State.bUpdateInProgress.store(false);

			UE_LOG(LogTemp, Log,
				TEXT("FTierStreamingSystem::UpdateTier [%s] — %d entering (%d cached, %d generated), %d exiting in %.3f sec"),
				*Config.TierName, EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(),
				ExitingNodes.Num(), FPlatformTime::Seconds() - StartTime);
		});
}

// ============================================================================
//  PushTierToNiagara  (transition commit)
// ============================================================================
// Publishes the back buffer whose Entering plane was generated off-lock, and
// makes the whole transition atomic with respect to every other Niagara write.
// Holds the tier PushCS across flip + stamp + upload + mirror and reads the
// freshest VirtualTraversal via GetLatestVT at execution time, so it can
// neither race the per-frame push nor re-seed the GPU with a stale VT.
//
// Atomicity contract: a slot's particle data (cell-local positions) and its
// lattice entry go live TOGETHER, and the (NCenter - VT) uniform always
// derives from the same stamped center as the live lattice. If the GPU ever
// saw a wrapped slot's new cellLocal with its old lattice (or a uniform from
// a different center), that slot would teleport by ~Side*CellSize for a
// frame. Everything below the lock acquisition exists to make that state
// unrepresentable. Bounds are deferred to the game thread. No SVO_GT_SCOPE
// here -- the profiler accumulator is game-thread only.
void FTierStreamingSystem::PushTierToNiagara(
	const TFunction<FVector()>& GetLatestVT,
	int32 BackIdx,
	const TArray<int32>& EnteringSlots,
	const FIntVector& NewCenter,
	const FVector& NewNCenter,
	const FParticleTierConfig& Config,
	FParticleTierState& State)
{
	// Serialize against the per-frame push and perform the FrontIdx swap under
	// the same lock, so no per-frame push can slip between "buffer published"
	// and "new data uploaded". VT is read here, at execution time, never captured.
	FScopeLock Lock(&State.PushCS);
	if (State.bShuttingDown.load()) return;

	// 1. Publish: the freshly written buffer becomes the front.
	State.FrontIdx.store(BackIdx);

	// 2. C_stamp: record the center the published lattice derives from. The
	//    per-frame push reads these under the same lock, so its uniform can
	//    never disagree with the lattice uploaded below.
	State.StampedCenter = NewCenter;
	State.StampedNCenter = NewNCenter;

	// 3. Upload. Cell-anchored buffers push cell-local positions, the per-slot
	//    lattice (vs NewNCenter), and the seeded (NCenter - VT) uniform; legacy
	//    buffers push camera-relative positions as before.
	const FVector VirtualTraversal = GetLatestVT();
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		UNiagaraComponent* NC = State.NiagaraComponents[b];
		if (!NC) continue;
		State.Buffers[b][BackIdx].PushToNiagara(NC, VirtualTraversal, NewNCenter);
	}

	// 4. Mirror exactly the Entering plane into the other buffer so the two
	//    buffers stay identical incrementally — this is what lets the next
	//    transition skip the full CopyFrom. Resident slots are never written
	//    on either buffer. Writing this one small plane twice per transition
	//    is the entire cost of keeping the double buffer.
	const int32 OtherIdx = 1 - BackIdx;
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		FNiagaraParticleBuffer& Dst = State.Buffers[b][OtherIdx];
		const FNiagaraParticleBuffer& Src = State.Buffers[b][BackIdx];
		for (const int32 SlotIndex : EnteringSlots)
			Dst.MirrorSlotFrom(Src, SlotIndex);
	}

	// SetSystemFixedBounds is a component render-state touch; defer it to the game
	// thread (ApplyPendingBounds). MaxExtent is stable by the time we set the flag.
	State.bBoundsDirty.store(true);
}

// ============================================================================
//  PushTierPositions
// ============================================================================
void FTierStreamingSystem::PushTierPositions(
	std::initializer_list<FParticleTierState*> Tiers,
	const TFunction<FVector()>& GetLatestVT)
{
	for (FParticleTierState* Tier : Tiers)
	{
		if (!Tier) continue;

		// Same lock the transition commit holds — but TRY, don't block. If
		// it's contended, the holder is the commit itself, which seeds a
		// fresher (NCenter - VT) uniform (fresh VT read) as part of its
		// upload; the value we'd have pushed is already superseded. Blocking
		// here is what convoyed the whole per-frame worker behind one tier's
		// wholesale array upload and froze motion on EVERY tier in the list
		// during any transition. Skipping is strictly better: uncontended
		// tiers get their uniform this pass, the contended tier gets a newer
		// one from the commit, and the next SchedulePush pass covers it again.
		FScopeTryLock Lock(&Tier->PushCS);
		if (!Lock.IsLocked()) continue;
		if (Tier->bShuttingDown.load()) continue;

		const int32 FrontIdx = Tier->FrontIdx.load();
		const FVector VirtualTraversal = GetLatestVT();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;

			// ONE FVector uniform — (NCenter - VT) — the entire per-frame
			// cost. The stamp is read under the same PushCS the commit wrote
			// it under (with the same FrontIdx), so this uniform always
			// matches the live lattice, even on a transition frame.
			// StampedNCenter is ZeroVector before the first commit, when
			// every particle is dead — the value is inert.
			Tier->Buffers[b][FrontIdx].PushNCenterMinusVT(
				NC, Tier->StampedNCenter, VirtualTraversal);
		}
	}
}

// ============================================================================
//  ApplyPendingBounds  (game thread)
// ============================================================================
// Consumes the bBoundsDirty flag raised by the transition commit. GROW-ONLY:
// the base box from ComputeBounds is constant per tier, and cell-anchored
// reconstruction bounds every rendered position by the neighborhood half-
// extent plus one particle radius — streaming never moves the box. The only
// variable is the MaxExtent pad, and shrinking it buys nothing (culling
// already can't reject a camera-surrounding box) while re-registering render
// state costs. So SetSystemFixedBounds fires only when the pad exceeds the
// tier's high-water mark; steady-state transitions apply nothing at all.
void FTierStreamingSystem::ApplyPendingBounds(
	FParticleTierConfig& Config, FParticleTierState& State)
{
	if (!State.bBoundsDirty.exchange(false)) return;

	// Pad by the largest particle across ALL buffers so the one shared
	// high-water mark is valid for every component in the tier.
	const int32 FrontIdx = State.FrontIdx.load();
	double CandidatePad = 0.0;
	for (int32 b = 0; b < State.Buffers.Num(); ++b)
		CandidatePad = FMath::Max(CandidatePad,
			static_cast<double>(State.Buffers[b][FrontIdx].MaxExtent));

	if (CandidatePad <= State.AppliedBoundsPad) return;   // box wouldn't grow
	State.AppliedBoundsPad = CandidatePad;

	const FBox Bounds = Config.ComputeBounds().ExpandBy(CandidatePad);
	for (int32 b = 0; b < State.NiagaraComponents.Num(); ++b)
	{
		UNiagaraComponent* NC = State.NiagaraComponents[b];
		if (!NC) continue;
		NC->SetSystemFixedBounds(Bounds);
	}
}

// ============================================================================
//  BeginShutdownDrain  (game thread, teardown)
// ============================================================================
// Marks the tier as shutting down, then acquires PushCS once to block until any
// in-flight push has released it. After this returns, every future push bails at
// the bShuttingDown check before touching a component, so the caller can safely
// destroy the tier's Niagara components.
void FTierStreamingSystem::BeginShutdownDrain(FParticleTierState& State)
{
	State.bShuttingDown.store(true);
	FScopeLock Lock(&State.PushCS);
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

	const FNiagaraParticleBuffer& InsertBuffer = State.Buffers[Config.OctreeInsertBufferIndex][BufferIdx];

	// Residency check replaces the old ActiveSlots map lookup: the buffer's
	// SlotCoord is the per-slot source of truth for which cell owns the slot.
	if (!InsertBuffer.SlotCoord.IsValidIndex(SlotIndex) ||
		InsertBuffer.SlotCoord[SlotIndex] != Coord) return;
	if (!State.SlotEntries.IsValidIndex(SlotIndex)) return;

	const double TreeExtent = Ctx.Octree->Extent;
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	const int32 BufferStart = SlotIndex * Config.SlotCapacity;

	// Resetting here releases the shared node refs held for the slot's
	// PREVIOUS occupant (the congruent exiting cell) — the exit-time release
	// the old pipeline did via ActiveSlots.Remove.
	FSlotEntry& Entry = State.SlotEntries[SlotIndex];
	Entry.InsertedNodes.Reset();
	Entry.InsertedNodes.Reserve(LiveCount);

	for (int32 i = 0; i < LiveCount; ++i)
	{
		const int32 Idx = BufferStart + i;
		if (InsertBuffer.Extents[Idx] <= 0.0f) continue;
		InsertParticleIntoOctree(Ctx, Entry, InsertBuffer.Positions[Idx],
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
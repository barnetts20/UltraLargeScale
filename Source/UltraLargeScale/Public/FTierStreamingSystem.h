/**
 * FTierStreamingSystem.h
 * Owns the tier system's authored config (FTierParams), its runtime structures, the per-call
 * actor context (FTierStreamingContext), and the stateless pipeline over them. Every layer
 * delegates here.
 */
#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "FNiagaraParticleBuffer.h"
#include "FOctree.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "FTierStreamingSystem.generated.h"

class USceneComponent;

#pragma region Data Structures

/** Per-tier streaming parameters exposed in the editor, the authored counterpart to the
 *  runtime FParticleTierConfig. Each layer's params struct holds one per tier, and
 *  BuildTierConfigs() reads them to populate the runtime config. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FTierParams
{
	GENERATED_BODY()

	/** Octree grid depth defining this tier's cell size:
	 *  half-extent = (Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)). Use evenly
	 *  spaced depths; the adjacent-tier scale ratio is 2^spacing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 GridDepth = 6;

	/** Half-width of the 3D neighborhood streamed around the player; 1 gives 3x3x3 = 27. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 NeighborhoodRadius = 1;

	/** Max particles per slot, and for GPU generation THE ONLY PLACEMENT KNOB. It is both the
	 *  buffer size and the target: the dispatch solves each cell's candidate count so the tier
	 *  accepts this many in total, distributed by cell mass. Pooling is across the whole BATCH,
	 *  not per slot -- per-slot targets would give every slot the same count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 SlotCapacity = 500;

	/** Levels of extra subdivision applied to each streamed cell before generation. 0 generates
	 *  at the streaming depth; each level splits every cell into eight.
	 *
	 *  GENERATION GRANULARITY IS NOT SLOT GRANULARITY: a streamed cell has to be big enough
	 *  that a neighbourhood stays resident, and the field's structure has no reason to be that
	 *  size. With structure occupying a few percent of a cell, rejection against a per-cell
	 *  envelope accepts at exactly that ratio, and subdividing cuts both costs.
	 *
	 *  THERE IS A CROSSOVER, per tier, since probe cost grows as 8^N while candidate cost
	 *  falls. Read the C/P ratio in the batch log: below about 1 the tier wants one level
	 *  fewer, above about 9 one more, and a tier that is right sits near 4-6. THE CEILING IS
	 *  THE CALIBRATION GRID, which descends the WHOLE grid at this depth. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming",
		meta = (ClampMin = "0", ClampMax = "3"))
	int32 GenerationSubdivision = 0;

	/** The clamp the call sites apply, so ClampMax and the runtime bound cannot drift apart. */
	static constexpr int32 MaxGenerationSubdivision = 3;

	/** Shapes the tier's particle size distribution between MinScale and MaxScale. Below 1
	 *  biases toward MinScale, above 1 toward MaxScale, 1 is uniform. AN EXPONENT RATHER THAN A
	 *  CURVE, and it has to be: placement runs on the GPU and no shader can evaluate a
	 *  UCurveFloat, so a curve asset would put the size rule out of the dispatch's reach. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution",
		meta = (ClampMin = "0.01"))
	float ExtentExponent = 1.0f;

	/** Shapes how aggressively the density field is read before the rejection gate. Above 1
	 *  concentrates entities into dense regions, below 1 lifts the faint ones, 1 is raw. An
	 *  exponent rather than a curve, for the reason above. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution",
		meta = (ClampMin = "0.01"))
	float SpawnExponent = 1.0f;

	/** Largest entity scale this tier represents. Derived by DeriveTierScaleRanges. */
	double MaxScale = 0.0;

	/** Smallest entity scale this tier represents. Derived by DeriveTierScaleRanges. */
	double MinScale = 0.0;

	/** Derives MinScale and MaxScale for an ordered, shallowest-first tier array from one
	 *  MaxEntityScale and the inter-tier depth sequence -- ratio = 2^(nextDepth - thisDepth),
	 *  the last tier mirroring the previous pair's spacing. */
	static void DeriveTierScaleRanges(double MaxEntityScale, TArrayView<FTierParams*> Tiers)
	{
		const int32 NumTiers = Tiers.Num();
		if (NumTiers == 0) return;

		Tiers[0]->MaxScale = MaxEntityScale;
		for (int32 i = 0; i < NumTiers; ++i)
		{
			int32 DepthDelta;
			if (i + 1 < NumTiers) DepthDelta = Tiers[i + 1]->GridDepth - Tiers[i]->GridDepth;
			else DepthDelta = Tiers[i]->GridDepth - Tiers[i - 1]->GridDepth;
			const double Ratio = static_cast<double>(1 << FMath::Clamp(DepthDelta, 1, 20));
			Tiers[i]->MinScale = Tiers[i]->MaxScale / Ratio;
			if (i + 1 < NumTiers) Tiers[i + 1]->MaxScale = Tiers[i]->MinScale;
		}
	}
};

/** Octree bookkeeping for one flat-buffer slot, stored slot-indexed in
 *  FParticleTierState::SlotEntries -- the slot index IS the array index, since toroidal
 *  addressing makes coord->slot a fixed modular function. Enables targeted node removal. */
struct FSlotEntry
{
	/** Octree nodes inserted from this slot's live particles. Reset when a new cell generates
	 *  into the slot, releasing the previous occupant's shared refs. */
	TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
};

/** Immutable descriptor for one particle streaming tier. Populated once by BuildTierConfigs()
 *  and NEVER MUTATED AT RUNTIME. All per-tier behavioural differences are encoded here. */
struct FParticleTierConfig
{
	/** Human-readable name for log output (e.g. "Large", "Mid", "Small"). */
	FString TierName;

	/** Octree depth defining this tier's streaming cell size:
	 *  half-extent = (Params.Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)). */
	int32 GridDepth = 1;

	/** Half-width of the 3D cell neighborhood streamed around the player. Total active slots =
	 *  (2 * NeighborhoodRadius + 1)^3. */
	int32 NeighborhoodRadius = 1;

	/** Maximum particles written per slot (candidate count before rejection). */
	int32 SlotCapacity = 0;

	/** Tier index written into octree node TypeId on insert, so spawn hooks identify a node's
	 *  tier without searching scale ranges. */
	int32 TierIndex = 0;

	/** One Niagara system template per logical buffer in this tier. InitializeTier spawns one
	 *  UNiagaraComponent per entry. */
	TArray<UNiagaraSystem*> NiagaraAssets;

	/** Parallel to NiagaraAssets. True if the buffer allocates the Rotations array, giving face
	 *  normals for non-billboard rendering. */
	TArray<bool> bWantRotations;

	/** Index into NiagaraAssets and Buffers walked during octree insertion; -1 skips octree
	 *  insertion entirely for this tier. */
	int32 OctreeInsertBufferIndex = 0;

	/** Particle generation callback, invoked once per entering cell during parallel generation.
	 *  Writes directly into each buffer's slot region, receiving the cell's grid Coord, the flat
	 *  SlotIndex, and one buffer pointer per NiagaraAsset. OPTIONAL: a tier may bind this,
	 *  GenerateBatchCallback, or both, and both call sites check before invoking. */
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	/** Optional whole-batch generator, tried before GenerateCallback, because a GPU dispatch
	 *  wants the batch rather than a slot. Returning true means it has filled every queued slot
	 *  and written SlotCounts, and selects between the two paths for a tier that binds both. A
	 *  tier binding only this one has nothing to select, so THE BATCH GENERATOR OWNS THOSE
	 *  SLOTS EITHER WAY: on false it must leave them blanked with counts zeroed, and log. */
	TFunction<bool(const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutSlotCounts)> GenerateBatchCallback;

	/** Returns the fixed AABB set once at InitializeTier as the Niagara bounds
	 *  for this tier's components. */
	TFunction<FBox()> ComputeBounds;

	/** Optional hook fired inside UpdateTier's async task after exiting slots are freed but
	 *  before generation begins, receiving the Entering and Exiting cell coords and the tier's
	 *  NewCenter. For work that must stay in lockstep with boundary crosses. */
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;

	/** Optional per-cell culling predicate, taking the Coord under test and returning true to
	 *  skip it. Used by bounded actors to confine generation to their volume. Two roles:
	 *  1. PER-CELL CULL: evaluated per entering cell in UpdateTier. A skipped cell is
	 *     dead-padded with zero particles, with no generation or cache lookup.
	 *  2. STREAMING GATE: evaluated against the player's own new center cell before a
	 *     transition is admitted. If that cell is skippable the window freezes at its last
	 *     in-bounds center, live edge cells persisting as a boundary halo kept parallaxing by
	 *     the per-frame uniform. Being a function of the center CELL, it only changes at a
	 *     boundary, so no hysteresis is needed. */
	TFunction<bool(const FIntVector& Coord)> ShouldSkipCell;
};

/** Mutable runtime state for one particle streaming tier, fully owned by the tier pipeline.
 *  A plain struct rather than a USTRUCT: the UNiagaraComponent* pointers alias entries in the
 *  owning actor's TierNiagaraComponents for GC safety, so DO NOT STORE THEM ELSEWHERE.
 *
 *  THREADING CONTRACT:
 *  - CenterCoord, SlotEntries, SlotCounts, CellCache and the Buffers' CPU arrays are written
 *    exclusively by the async task spawned from UpdateTier, in place. Anything outside the
 *    pipeline reading Buffers must hold bUpdateInProgress == false; on the game thread that
 *    read is race-free, since the flag is set on the GT before the task spawns.
 *  - The per-frame uniform push reads only the stamps, which is what makes in-place
 *    generation safe without a back buffer. They are written and read only under PushCS, so
 *    the (NCenter - VT) uniform can never disagree with the live lattice.
 *  - NiagaraComponents are touched only on the game thread. */
struct FParticleTierState
{
	/** Particle data, one buffer per Niagara asset. SINGLE-BUFFERED: the transition task
	 *  overwrites entering slots in place, safe because the GPU holds its own copy between
	 *  pushes and other readers gate on bUpdateInProgress. */
	TArray<FNiagaraParticleBuffer> Buffers;

	/** Raw component pointers aliasing TierNiagaraComponents, parallel to
	 *  FParticleTierConfig::NiagaraAssets. GAME THREAD ONLY. */
	TArray<UNiagaraComponent*> NiagaraComponents;

	/** True while an async boundary-cross task owns the tier's buffers and state. The game
	 *  thread must not begin a new update, nor read the CPU buffer arrays, while set. */
	std::atomic<bool> bUpdateInProgress{ false };

	/** Serializes all Niagara writes for this tier: the stamp write and upload happen under it,
	 *  so a per-frame push can never pair a stale stamp with fresh data. THE GAME THREAD MUST
	 *  NEVER ACQUIRE IT -- it would stall behind an upload. */
	FCriticalSection PushCS;

	/** Set on teardown (BeginShutdownDrain) so in-flight pushes bail before
	 *  touching a component about to be destroyed. */
	std::atomic<bool> bShuttingDown{ false };

	/** Grid coordinate at the center of the streaming neighborhood. Written on the game thread
	 *  before the async task starts, then read-only to both sides until it clears
	 *  bUpdateInProgress. INT32_MIN forces a full generate on the first UpdateTier. */
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	/** Per-slot octree bookkeeping, indexed by modular slot index and sized TotalSlots at
	 *  InitializeTier. The resident coord lives in the buffers' SlotCoord arrays. */
	TArray<FSlotEntry> SlotEntries;

	/** Center coord the live lattice and (NCenter - VT) uniform were built against. Written
	 *  under PushCS in the same section as the lattice upload. INT32_MIN until the first
	 *  commit, when all particles are dead and the uniform inert. */
	FIntVector StampedCenter = FIntVector(INT32_MIN);

	/** GridCoordToCenter(StampedCenter), precomputed at commit so the per-frame push needs no
	 *  grid parameters. Same PushCS discipline as StampedCenter. */
	FVector StampedNCenter = FVector::ZeroVector;

	/** Per-slot accepted particle count, written by GenerateCallback, letting the cache and
	 *  octree paths skip dead padding without walking the full SlotCapacity. */
	TArray<int32> SlotCounts;

	/** Persistent procgen cache keyed by grid coord: a first visit generates and stores, and
	 *  re-entry blits the stored arrays back. The slot recycles on exit but the entry survives
	 *  until CullTierCache evicts it. */
	TMap<FIntVector, FCachedCellData> CellCache;

	/** Resets all plain state to the freshly constructed baseline so a pooled owner can reuse
	 *  this tier as if new. Does not touch NiagaraComponents, which the owning actor destroys
	 *  first. CALL ONLY after the push worker has exited and in-flight tier tasks drained. */
	void ResetState()
	{
		Buffers.Empty();
		SlotEntries.Empty();
		SlotCounts.Empty();
		CellCache.Empty();
		CenterCoord = FIntVector(INT32_MIN);
		StampedCenter = FIntVector(INT32_MIN);
		StampedNCenter = FVector::ZeroVector;
		bUpdateInProgress.store(false);
		bShuttingDown.store(false);
	}
};

/** Read-only context supplied by the owning actor to the tier pipeline, passed into every
 *  FTierStreamingSystem call so the pipeline reads spatial parameters without knowing the
 *  concrete actor type. Populated via BuildStreamingContext(). */
struct FTierStreamingContext
{
	/** Sector/galaxy extent; drives grid cell sizing and dead-pos parking. */
	double Extent = 0.0;

	/** UnitScale of the owning actor, used by InsertParticleIntoOctree to compute insert depth.
	 *  Universe passes 1.0, its extents already being local. */
	double UnitScale = 1.0;

	/** Backdrop membership for this actor's tier sprites, derived from the actor's REAL
	 *  UnitScale via IsVirtualSpace(). DELIBERATELY SEPARATE from the UnitScale above, which
	 *  the Universe forces to 1.0 for octree-insert math. InitializeTier stamps
	 *  bVisibleInSceneCaptureOnly from it. */
	bool bVirtualSpace = false;

	/** Multiplier applied to Extent for grid cell sizing.
	 *  CellSize = (Extent * GridExtentMultiplier) / (1 << GridDepth). */
	double GridExtentMultiplier = 4.0;

	/** The actor's current virtual traversal, driving grid coord derivation and
	 *  camera-relative position computation. */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** The actor's persistent spatial index. Octree insert calls go here. */
	TSharedPtr<FOctree> Octree;

	/** Current lifecycle state. The pipeline early-outs if not Ready. */
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	/** True while an octree rebase is in progress (Universe only). UpdateTier
	 *  must not proceed while this is set. */
	bool bRebaseInProgress = false;

	/** Root component to attach spawned Niagara components to. */
	USceneComponent* AttachRoot = nullptr;

	/** When true, spawned Niagara components use absolute world position, decoupled from the
	 *  actor transform. Used by Galaxy, whose actor is pegged to the player while its Niagara
	 *  positions are galaxy-local. */
	bool bNiagaraAbsolutePosition = false;

	/** Owning actor name for log output. */
	FString OwnerName;

	/** The owning actor's Params.Seed, used by the octree insertion pipeline to compose
	 *  deterministic child seeds via FVoxelData::ComposeSeed. Each level passes its own seed,
	 *  itself a ComposeSeed output from its parent. */
	int32 ParentSeed = 0;

	/** The actor's freshest VirtualTraversal, thread-safely. Push tasks call this at EXECUTION
	 *  time, under PushCS, so a push composites against current VT rather than a stale one. */
	TFunction<FVector()> GetLatestVT;

	/** The actor's LIVE lifecycle state. InitializationState above is a by-value snapshot and
	 *  can never observe a mid-init flip to Pooling, so the pipeline's abort checks read this.
	 *  When unset, the snapshot is used and nothing aborts. */
	TFunction<ELifecycleState()> GetLiveState;
};

#pragma endregion

#pragma region Generation Grid

/** One cell of a tier's GENERATION grid, as the entity-gen dispatches consume it.
 *
 *  SHARED BY EVERY LAYER, deliberately: the child COORD is the placement key, and a layer with
 *  its own copy that drifted in how it labels children would regenerate its entities somewhere
 *  else, with nothing to compare against.
 *
 *  THE CENTRE IS SUPPLIED, not derived. Grid-coord-to-centre lives on the actor, which owns
 *  the grid; a generator inferring it from a buffer's slot centres puts every candidate
 *  somewhere else and every batch comes back with nothing accepted.
 *
 *  DOUBLE, AND IT STAYS DOUBLE HERE. Narrowing belongs at the marshal into the layer's own
 *  gen-cell record, where the layer knows whether its caller space is bounded -- see
 *  FGalaxyGenCell::Centre against FUniverseGenCell::CentreCell for the two answers. */
struct FTierBatchCell
{
	FIntVector Coord = FIntVector::ZeroValue;
	int32 SlotIndex = 0;

	/** Index into the array this cell was subdivided FROM, or its own index when nothing was.
	 *  Calibration needs it and generation does not: a tier with one cell per slot calibrates
	 *  against the largest sum over one parent's children, and SlotIndex cannot answer that --
	 *  a batch of neighbouring cells shares no slot, and a whole-grid pass has none at all. */
	int32 ParentIndex = 0;

	FVector Centre = FVector::ZeroVector;
	double HalfExtent = 0.0;
};

/** How a subdivision labels its children's coords. BOTH ARE UNIQUE ACROSS PARENTS, which is
 *  the only property the placement key needs; they differ only in where a parent's run of
 *  children sits relative to its own scaled index. NOT A FREE CHOICE ONCE A LAYER HAS SHIPPED:
 *  the coord seeds every probe, candidate and accept draw, so switching rerolls everything. */
enum class ETierChildCoords : uint8
{
	/** ParentCoord * Side + i, so a parent's children run upward from ParentCoord * Side. */
	Ascending,

	/** ParentCoord * Side + i - Side/2, centring the run on the parent's own scaled index.
	 *  Produces coords symmetric about the origin for a grid centred there. */
	Centred
};

#pragma endregion

#pragma region Stateless Pipeline

/** Stateless utility implementing the entire tier streaming pipeline: grid coord math,
 *  InitializeTier, UpdateTier, PushTierToNiagara, cell caching and octree integration. Every
 *  layer delegates here rather than maintaining a copy; all actor-specific behaviour is
 *  injected via FParticleTierConfig callbacks and FTierStreamingContext. See
 *  FParticleTierState for the threading contract. */
struct FTierStreamingSystem
{
#pragma region Generation Grid

	/** Splits each cell into 8^Levels children that tile it exactly, in place of it.
	 *  InKeepChild receives a child's centre in caller units and its half extent, and returns
	 *  whether to keep it. A TEMPLATE so the predicate inlines.
	 *
	 *  EXACT TILING IS LOAD-BEARING: the sum of the children's masses is what the tier's
	 *  calibrated constant is scaled against, so children that overlapped or left gaps would
	 *  make calibration solve for a different volume from the one generation fills.
	 *
	 *  CHILD COORDS DEPEND ON THE PARENT AND NOTHING ELSE -- not on the batch, not on arrival
	 *  order. The coord is the placement key, so a child whose coord shifted with the batch
	 *  would regenerate differently on the next visit. They do NOT correspond to positions on
	 *  the streaming grid at the deeper level.
	 *
	 *  THE CULL IS THE ONLY THING THAT VARIES BETWEEN LAYERS, hence a parameter. A BOUNDED
	 *  field is zero outside its own volume, so a child past it can hold nothing; an UNBOUNDED
	 *  field has no outside, and a cull there deletes cells that belong. */
	template <typename TKeepChild>
	static void SubdivideCells(
		const TArray<FTierBatchCell>& InCells,
		int32 InLevels,
		ETierChildCoords InCoordMode,
		TArray<FTierBatchCell>& OutCells,
		TKeepChild&& InKeepChild)
	{
		OutCells.Reset();

		if (InLevels <= 0)
		{
			OutCells = InCells;

			// Every cell is its own parent, so a caller that groups by ParentIndex gets the
			// same answer whether or not the tier subdivides.
			for (int32 i = 0; i < OutCells.Num(); ++i)
			{
				OutCells[i].ParentIndex = i;
			}

			return;
		}

		const int32 Side = 1 << InLevels;
		const int32 PerCell = Side * Side * Side;

		// THE UNCULLED COUNT, deliberately. A cull only ever removes, so this is an upper
		// bound rather than an estimate, and over-reserving a transient array beats
		// reallocating inside the loop.
		OutCells.Reserve(InCells.Num() * PerCell);

		// The coord offset that distinguishes the two labellings; see ETierChildCoords.
		const int32 CoordBias = (InCoordMode == ETierChildCoords::Centred) ? (Side / 2) : 0;

		for (int32 ParentIndex = 0; ParentIndex < InCells.Num(); ++ParentIndex)
		{
			const FTierBatchCell& Parent = InCells[ParentIndex];

			const double SubHalf = Parent.HalfExtent / static_cast<double>(Side);
			const double SubFull = SubHalf * 2.0;

			// Centres the run of children on the parent, so they tile it exactly.
			const double Origin = -(static_cast<double>(Side) - 1.0) * 0.5;

			for (int32 iz = 0; iz < Side; ++iz)
			{
				for (int32 iy = 0; iy < Side; ++iy)
				{
					for (int32 ix = 0; ix < Side; ++ix)
					{
						const FVector Centre = Parent.Centre + FVector(
							(Origin + static_cast<double>(ix)) * SubFull,
							(Origin + static_cast<double>(iy)) * SubFull,
							(Origin + static_cast<double>(iz)) * SubFull);

						if (!InKeepChild(Centre, SubHalf))
						{
							continue;
						}

						FTierBatchCell Child;

						// THE SLOT IS THE PARENT'S. Children are a generation detail; the
						// buffer still holds one region per streamed cell.
						Child.SlotIndex = Parent.SlotIndex;
						Child.ParentIndex = ParentIndex;

						Child.Coord = FIntVector(
							Parent.Coord.X * Side + ix - CoordBias,
							Parent.Coord.Y * Side + iy - CoordBias,
							Parent.Coord.Z * Side + iz - CoordBias);

						Child.Centre = Centre;
						Child.HalfExtent = SubHalf;

						OutCells.Add(Child);
					}
				}
			}
		}
	}

	/** The uncalled-cull form, for a field with no outside. */
	static void SubdivideCells(
		const TArray<FTierBatchCell>& InCells,
		int32 InLevels,
		ETierChildCoords InCoordMode,
		TArray<FTierBatchCell>& OutCells)
	{
		SubdivideCells(InCells, InLevels, InCoordMode, OutCells,
			[](const FVector&, double) { return true; });
	}

	/** The cull a SPHERE-BOUNDED field wants: drop a child whose nearest point already lies
	 *  outside the sphere of radius InExtent. A sphere fills pi/6 of its bounding cube, so this
	 *  removes about a fifth of a full grid before any of it reaches the GPU. */
	static auto MakeSphereBoundsCull(double InExtent)
	{
		const double InvExtent = 1.0 / FMath::Max(InExtent, 1e-9);

		return [InvExtent](const FVector& InCentre, double InHalfExtent) -> bool
			{
				const FVector Nearest(
					FMath::Max(FMath::Abs(InCentre.X) - InHalfExtent, 0.0) * InvExtent,
					FMath::Max(FMath::Abs(InCentre.Y) - InHalfExtent, 0.0) * InvExtent,
					FMath::Max(FMath::Abs(InCentre.Z) - InHalfExtent, 0.0) * InvExtent);

				return Nearest.SizeSquared() < 1.0;
			};
	}

#pragma endregion

#pragma region Grid Coord Helpers

	/** Narrows a grid coordinate to int32 with a DEFINED result at any magnitude.
	 *
	 *  FMath::FloorToInt32 ends in a plain cast, and casting an out-of-range double to int32 is
	 *  undefined behaviour. On x86 it yields INT32_MIN rather than wrapping, so EVERY coordinate
	 *  past the limit collapses onto that one and the whole streaming grid folds to a single
	 *  cell, which reads as a spawn or pooling bug. Wrapping instead makes the far behaviour
	 *  "generation repeats" and costs no range: the wrap period IS the int32 span, so every
	 *  coordinate the old cast handled correctly comes back bit-identical. */
	static int32 WrapGridAxis(double InValue)
	{
		constexpr double Span = 4294967296.0;   // 2^32
		constexpr double Min = -2147483648.0;   // INT32_MIN

		// Floored modulo onto [INT32_MIN, INT32_MAX], matching the convention used for the
		// field cell wrap: truncation toward zero would fold the two halves of the range
		// together and map two distinct regions onto one coordinate.
		const double Shifted = InValue - Min;
		const double Wrapped = Shifted - FMath::Floor(Shifted / Span) * Span + Min;

		return static_cast<int32>(FMath::Clamp(Wrapped, Min, 2147483647.0));
	}

	/** Converts a local position to a grid coordinate at the given depth. THE FLOOR HAPPENS IN
	 *  DOUBLE AND THE NARROWING AFTER IT -- see WrapGridAxis. The + 0.5 is the centre-aligned
	 *  lattice. */
	static FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth,
		double Extent, double GridExtentMultiplier)
	{
		const double CellSize = (Extent * GridExtentMultiplier) / (1 << InGridDepth);
		return FIntVector(
			WrapGridAxis(FMath::Floor(InPos.X / CellSize + 0.5)),
			WrapGridAxis(FMath::Floor(InPos.Y / CellSize + 0.5)),
			WrapGridAxis(FMath::Floor(InPos.Z / CellSize + 0.5)));
	}

	/** Converts a grid coordinate back to the cell center position. */
	static FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth, double Extent, double GridExtentMultiplier)
	{
		const double CellSize = (Extent * GridExtentMultiplier) / (1 << InGridDepth);
		return FVector(static_cast<double>(InCoord.X) * CellSize, static_cast<double>(InCoord.Y) * CellSize, static_cast<double>(InCoord.Z) * CellSize);
	}

	/** Returns the half-extent of a grid cell at the given depth. */
	static double GetGridCellExtent(int32 InGridDepth, double Extent, double GridExtentMultiplier)
	{
		return (Extent * GridExtentMultiplier) / (1 << (InGridDepth + 1));
	}

#pragma endregion

#pragma region Toroidal Slot Addressing
	/** slot(coord) is a fixed bijection over any Side-wide window of cells: each per-axis
	 *  residue appears once within the window, so every resident coord keeps its slot and its
	 *  particle data as the window slides, and the coord entering on the leading face is
	 *  congruent mod Side to the one exiting on the trailing face and reuses its slot.
	 *  Side = 1 is the degenerate torus -- one slot, no wrapping, same code path. */

	 /** Non-negative modulo: C++ '%' is negative for negative operands. m must be > 0. */
	static FORCEINLINE int32 PosMod(int32 a, int32 m)
	{
		return ((a % m) + m) % m;
	}

	/** Flattens three per-axis residues, each in [0, Side), to a slot index in [0, Side^3). */
	static FORCEINLINE int32 FlattenResidues(int32 a, int32 b, int32 c, int32 Side)
	{
		return (a * Side + b) * Side + c;
	}

	/** Modular slot index for a cell coordinate. Side = 2*NeighborhoodRadius+1, odd by
	 *  construction and asserted at InitializeTier; never changes for a resident coord. */
	static FORCEINLINE int32 SlotOf(const FIntVector& Coord, int32 Side)
	{
		return FlattenResidues(PosMod(Coord.X, Side), PosMod(Coord.Y, Side), PosMod(Coord.Z, Side), Side);
	}

#pragma endregion

#pragma region Tier Initialization

	/** Allocates buffers and slot bookkeeping for a tier, then spawns and activates its Niagara
	 *  components on the game thread. Exhaustive tiers generate, octree-insert and cache their
	 *  full window immediately; streaming tiers defer to the first UpdateTier. Aborts if the
	 *  actor enters Pooling mid-init. */
	static void InitializeTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State, TArray<UNiagaraComponent*>& OutComponents);

#pragma endregion

#pragma region Tier Streaming Update

	/** Streams the resident window to follow the player. On a boundary cross it diffs the old
	 *  and new neighborhoods, generates or cache-restores entering cells in place into their
	 *  modular slots, octree-inserts them, and commits the push on a worker, holding
	 *  bUpdateInProgress throughout. Radius-0 tiers never stream. */
	static void UpdateTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State);

#pragma endregion

#pragma region Niagara Push

	/** Boundary-cross transition commit, on a worker at the tail of UpdateTier's async task.
	 *  Under the tier PushCS it stamps StampedCenter and StampedNCenter with the center the new
	 *  lattice derives from, then uploads cell-local positions, the per-slot lattice and the
	 *  (NCenter - VT) uniform, reading the freshest VT at execution time. The GPU flips
	 *  atomically at upload, rendering its prior copy until these land, so in-place CPU
	 *  generation is never visible mid-write; and because the per-frame push takes the same
	 *  lock and reads the same stamp, no push can pair a lattice and uniform built against
	 *  different centers. */
	static void PushTierToNiagara(const TFunction<FVector()>& GetLatestVT, const FIntVector& NewCenter, const FVector& NewNCenter, const FParticleTierConfig& Config, FParticleTierState& State);

	/** Per-frame parallax re-push: one FVector uniform per component, (StampedNCenter - VT),
	 *  with no array traffic. SKIPS rather than blocks on a tier whose PushCS is contended,
	 *  since the holder is the commit and seeds a fresher uniform itself. */
	static void PushTierVT(std::initializer_list<FParticleTierState*> Tiers, const TFunction<FVector()>& GetLatestVT);

	/** Game thread (teardown): blocks until in-flight pushes drain, then bars
	 *  new ones. */
	static void BeginShutdownDrain(FParticleTierState& State);

#pragma endregion

#pragma region Octree Integration

	/** Inserts every occupied slot's live particles into the context octree. Full-window pass
	 *  after exhaustive-tier init; no-op with no insert buffer or octree. */
	static void InsertTierIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State);

	/** Inserts one entering slot's live particles into the octree, first resetting the slot
	 *  entry to release the previous occupant's node refs. Guards on residency:
	 *  SlotCoord[SlotIndex] must equal Coord. */
	static void InsertSlotIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	/** Inserts one particle: builds its FPointData, converting world extent to octree-local via
	 *  UnitScale, composes a deterministic seed, inserts, and records the node on Entry. */
	static void InsertParticleIntoOctree(const FTierStreamingContext& Ctx, FSlotEntry& Entry, const FVector& Position, const float& Extent, const FLinearColor& Color, const FIntVector& GridCoord, int32 GenerationIndex, int32 AbsoluteBufferIndex, double TreeExtent, int32 TierIndex);

#pragma endregion

#pragma region Cell Cache

	/** Snapshots a slot's live particle data into the persistent cell cache keyed by Coord, so
	 *  re-entry blits the stored arrays instead of regenerating. A zero-count cell is cached
	 *  as a legitimate empty result. */
	static void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	/** Evicts cell-cache entries beyond NeighborhoodRadius + 4 cells, Chebyshev, from NewCenter,
	 *  bounding cache growth as the window streams. */
	static void CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& NewCenter);

#pragma endregion
};

#pragma endregion
/**
 * FTierStreamingSystem.h
 * Owns the tier system's authored config (FTierParams), runtime structures
 * (FSlotEntry, FParticleTierConfig, FParticleTierState), the per-call actor
 * context (FTierStreamingContext), and the stateless pipeline over them.
 * AUniverseActor, AGalaxyActor, and AStarSystemActor delegate here.
 */
#pragma once

#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
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

/** Per-tier streaming parameters exposed in the editor. Authored counterpart
 *  to the runtime FParticleTierConfig: each layer's params struct
 *  (FUniverseParams, FGalaxyParams, FStarSystemParams) holds one per tier, and
 *  BuildTierConfigs() reads them to populate the runtime config. */
USTRUCT(BlueprintType)
struct ULTRALARGESCALE_API FTierParams
{
	GENERATED_BODY()

	/** Octree grid depth defining this tier's cell size.
	 *  Cell half-extent = (Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)).
	 *  Use evenly-spaced depths (1, 3, 5 -> spacing 2); adjacent-tier scale
	 *  ratio = 2^spacing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 GridDepth = 6;

	/** Half-width of the 3D neighborhood streamed around the player.
	 *  1 -> 3x3x3 = 27 slots. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 NeighborhoodRadius = 1;

	/** Max particles per slot, and for GPU generation THE ONLY PLACEMENT KNOB.
	 *
	 *  It is both the buffer size and the target: the dispatch solves each cell's
	 *  candidate count so the tier accepts this many in total, distributed by cell mass.
	 *  Raising it makes the tier denser and its buffer larger, together, which is the
	 *  relationship the name always implied.
	 *
	 *  Pooling is across the whole batch, not per slot. Per-slot targets would give every
	 *  slot the same count regardless of what is in it, erasing exactly the structure
	 *  between cells that per-cell normalisation exists to preserve. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 SlotCapacity = 500;

	/** Levels of extra subdivision applied to each streamed cell before generation.
	 *  0 generates at the streaming depth; each level splits every cell into eight.
	 *
	 *  GENERATION GRANULARITY IS NOT SLOT GRANULARITY. A streamed cell has to be big
	 *  enough that a neighbourhood of them is affordable to keep resident; the field's
	 *  structure has no reason to be that size. A galaxy disc is a few percent of a cell
	 *  in its thin axis, so a cell's mean density is a small fraction of its peak -- and
	 *  rejection against a per-cell envelope accepts at exactly that ratio.
	 *
	 *  Subdividing cuts both costs at once. Subcells clear of the structure are culled by
	 *  the probe pass for sixty-four evaluations and draw no candidates at all, and the
	 *  ones that survive have a peak much closer to their own mean. Measured on a disc
	 *  occupying seven percent of its cell, two levels ran about three times cheaper.
	 *
	 *  THERE IS A CROSSOVER, and every tier has its own. Probe cost grows as 8^N while
	 *  candidate cost falls, so past it more levels cost more. Read the C/P ratio in the
	 *  batch log -- probes and candidates are both field evaluations, so their ratio says
	 *  which side the tier is on. Below about 1 the probes have overtaken placement and
	 *  the tier wants one level fewer; above about 9 it wants one more. Every tier sits
	 *  near 4-6 when it is right.
	 *
	 *  Which level that lands on depends on the tier's GridDepth and its slot capacity,
	 *  not on the field, so it is per-tier rather than global: a tier that spreads few
	 *  entities over the whole galaxy reaches the crossover early, one holding tens of
	 *  thousands per resident slot reaches it late.
	 *
	 *  THE CEILING IS THE CALIBRATION GRID, not the batch. Calibration descends the
	 *  tier's WHOLE grid at this depth, so a level costs 8x there whether or not any of
	 *  it ever streams in. At depth 5 with three levels that is already over a million
	 *  cells and a 22 MB counter readback; a fourth would be nine million and would not
	 *  fit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming",
		meta = (ClampMin = "0", ClampMax = "3"))
	int32 GenerationSubdivision = 0;

	/** The clamp the call sites apply, so the property's ClampMax and the runtime bound
	 *  cannot drift apart. They did: the property said four while the call sites allowed
	 *  six, and neither was the real ceiling. */
	static constexpr int32 MaxGenerationSubdivision = 3;

	/** Shapes the tier's particle size distribution between MinScale and MaxScale.
	 *  Below 1 biases toward MinScale, above 1 toward MaxScale, 1 is uniform.
	 *
	 *  Replaces the ScaleDistribution curve. A UCurveFloat is an authored asset no
	 *  shader can evaluate, and placement now lives in GalaxyDensityCore.ush so that
	 *  both sides read the same rule. Placeholder until each layer gets a size
	 *  distribution with a physical basis; a shaped uniform is honest until then. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution",
		meta = (ClampMin = "0.01"))
	float ExtentExponent = 1.0f;

	/** Shapes how aggressively the density field is read before the rejection gate.
	 *  Above 1 concentrates entities into the dense regions, below 1 lifts the faint
	 *  ones, 1 is the raw response.
	 *
	 *  Replaces the DensityResponse curve, for the same reason. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution",
		meta = (ClampMin = "0.01"))
	float SpawnExponent = 1.0f;

	// TRANSITIONAL. FTierParams is shared, and UniverseDataGenerator still reads both
	// curves; the galaxy layer no longer does. They go when the universe layer moves
	// its placement into UniverseDensityCore.ush as well. Until then a tier authored
	// for the galaxy configures the exponents and a tier authored for the universe
	// configures the curves, and neither reads the other's.

	/** UNIVERSE LAYER ONLY. Superseded by ExtentExponent for the galaxy. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution|Legacy")
	FRuntimeFloatCurve ScaleDistribution;

	/** UNIVERSE LAYER ONLY. Superseded by SpawnExponent for the galaxy. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution|Legacy")
	FRuntimeFloatCurve DensityResponse;

	/** Largest entity scale this tier represents. Derived by DeriveTierScaleRanges. */
	double MaxScale = 0.0;

	/** Smallest entity scale this tier represents. Derived by DeriveTierScaleRanges. */
	double MinScale = 0.0;

	/** Initializes the legacy curves to identity: f(x) = x. */
	FTierParams()
	{
		ScaleDistribution.GetRichCurve()->AddKey(0.0f, 0.0f);
		ScaleDistribution.GetRichCurve()->AddKey(1.0f, 1.0f);

		DensityResponse.GetRichCurve()->AddKey(0.0f, 0.0f);
		DensityResponse.GetRichCurve()->AddKey(1.0f, 1.0f);
	}

	/** Derives MinScale/MaxScale for an ordered (shallowest-first: Large, Mid,
	 *  Small) tier array from one MaxEntityScale and the inter-tier depth
	 *  sequence: ratio = 2^(nextDepth - thisDepth); the last tier mirrors the
	 *  previous pair's spacing. Depths 1/3/5 with MaxEntityScale 1e22 give
	 *  Large 2.5e21..1e22, Mid 6.25e20..2.5e21, Small 1.5625e20..6.25e20. */
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
 *  FParticleTierState::SlotEntries (the slot index IS the array index: under
 *  toroidal addressing coord->slot is a fixed modular function, so no
 *  coord-keyed map is needed). Enables targeted octree node removal without a
 *  full tree scan. */
struct FSlotEntry
{
	/** Octree nodes inserted from this slot's live particles. Reset and
	 *  repopulated when a new cell generates into the slot; the previous
	 *  occupant's shared refs are released then, so the spatial index survives
	 *  beyond the streaming window. */
	TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
};

/** Immutable descriptor for one particle streaming tier. Populated once by
 *  BuildTierConfigs() and never mutated at runtime. All Large/Mid/Small
 *  behavioural differences are encoded here; the generic pipeline
 *  (InitializeTier, UpdateTier) reads these fields and delegates generation via
 *  the callbacks. */
struct FParticleTierConfig
{
	/** Human-readable name for log output (e.g. "Large", "Mid", "Small"). */
	FString TierName;

	/** Octree depth defining this tier's streaming cell size.
	 *  Cell half-extent = (Params.Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)).
	 *  With GridExtentMultiplier = 4 and depths Large 1 / Mid 3 / Small 5:
	 *  depth 1 -> Extent, depth 3 -> Extent/4, depth 5 -> Extent/16. */
	int32 GridDepth = 1;

	/** Half-width of the 3D cell neighborhood streamed around the player.
	 *  Total active slots = (2 * NeighborhoodRadius + 1)^3; radius 1 -> 27. */
	int32 NeighborhoodRadius = 1;

	/** Maximum particles written per slot (candidate count before rejection). */
	int32 SlotCapacity = 0;

	/** Tier index written into octree node TypeId on insert, so spawn hooks can
	 *  identify a node's tier without searching scale ranges. Large=0, Mid=1,
	 *  Small=2. */
	int32 TierIndex = 0;

	/** One Niagara system template per logical buffer in this tier. Large has
	 *  two (cluster + gas); Mid and Small have one each. InitializeTier spawns
	 *  one UNiagaraComponent per entry. */
	TArray<UNiagaraSystem*> NiagaraAssets;

	/** Parallel to NiagaraAssets. True if the buffer allocates the Rotations
	 *  array (face normals for non-billboard rendering). Large cluster = true;
	 *  gas, Mid, Small = false. */
	TArray<bool> bWantRotations;

	/** Index into NiagaraAssets / Buffers walked during octree insertion; -1
	 *  skips octree insertion entirely for this tier. Large/Mid/Small = 0. */
	int32 OctreeInsertBufferIndex = 0;

	/** Particle generation callback, invoked once per entering cell during
	 *  parallel generation (InitializeTier and UpdateTier). Writes directly into
	 *  each buffer's slot region. Receives the cell's grid Coord, the flat
	 *  SlotIndex within the tier's buffers, and Buffers (one raw pointer per
	 *  NiagaraAsset, this tier's back buffer).
	 *
	 *  OPTIONAL. A tier may bind this, GenerateBatchCallback, or both; the star-system
	 *  and universe layers bind only this one, the galaxy layer only the batch form.
	 *  Both call sites check it is bound before invoking it. */
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	/** Optional whole-batch generator, tried before GenerateCallback.
	 *
	 *  Exists because a GPU dispatch wants the batch, not a slot: per-slot dispatches
	 *  serialise on a GPU round-trip each and the latency compounds across a
	 *  neighbourhood. Returning true means it has filled every queued slot and written
	 *  SlotCounts; returning false means it could not run.
	 *
	 *  The bool selects between the two paths for a tier that binds both. For a tier
	 *  that binds only this one there is nothing to select, so false means those slots
	 *  get nothing -- and the batch generator is then responsible for leaving them in a
	 *  defined state (blanked, counts zeroed) and for logging why. Silently returning
	 *  false from a batch-only tier produces an empty region with no explanation, which
	 *  is the failure this system keeps being debugged for. */
	TFunction<bool(const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutSlotCounts)> GenerateBatchCallback;

	/** Returns the fixed AABB set once at InitializeTier as the Niagara bounds
	 *  for this tier's components. */
	TFunction<FBox()> ComputeBounds;

	/** Optional hook fired inside UpdateTier's async task after exiting slots
	 *  are freed but before generation begins. Used by the streaming volumetric
	 *  to update density sub-regions in lockstep with Large-tier boundary
	 *  crosses. Not set for Mid or Small. Receives the Entering and Exiting cell
	 *  coords (entering/leaving the streaming window) and NewCenter (the tier's
	 *  new center coord after the boundary cross). */
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;

	/** Optional per-cell culling predicate with two roles:
	 *  1. PER-CELL CULL: evaluated per entering cell in UpdateTier; returns true
	 *     to skip the cell (dead-padded with zero particles, no generation or
	 *     cache lookup).
	 *  2. STREAMING GATE: evaluated against the player's own new center cell
	 *     before a transition is admitted. If that cell is skippable the tier
	 *     does not transition: the resident window freezes at its last in-bounds
	 *     center (live edge cells persist as a boundary halo, kept parallaxing by
	 *     the per-frame uniform). Streaming resumes on the first in-bounds
	 *     crossing; multi-cell re-entry deltas fall into the teleport path. As a
	 *     function of the center CELL, the gate only changes at a cell boundary,
	 *     so no hysteresis is needed.
	 *  Used by bounded actors (GalaxyActor, StarSystemActor) to confine
	 *  generation and boundary-cross tracking to their volume. Not set for
	 *  unbounded actors (UniverseActor) or radius-0 exhaustive tiers. Takes the
	 *  Coord of the cell being tested; returns true to skip the cell / suppress
	 *  streaming from it. */
	TFunction<bool(const FIntVector& Coord)> ShouldSkipCell;
};

/** Mutable runtime state for one particle streaming tier. Fully owned by the
 *  tier pipeline; generation callbacks write into Buffers directly but touch no
 *  other fields.
 *
 *  This is a plain struct, not a USTRUCT: the UNiagaraComponent* pointers here
 *  alias entries in the owning actor's TierNiagaraComponents (a UPROPERTY
 *  TArray) for GC safety. Do not store them elsewhere.
 *
 *  Threading contract:
 *  - CenterCoord, SlotEntries, SlotCounts, CellCache, and the Buffers' CPU
 *    arrays are written exclusively by the async task spawned from UpdateTier,
 *    in place (single buffer: entering slots are overwritten directly; nothing
 *    reads them mid-write). Anything outside the pipeline reading Buffers must
 *    hold bUpdateInProgress == false; on the game thread that read is race-free,
 *    since the flag is set on the GT before the task spawns and cleared by the
 *    worker after commit.
 *  - The per-frame uniform push reads no CPU arrays, only the stamps, which is
 *    what makes in-place generation safe without a back buffer.
 *  - bUpdateInProgress is std::atomic and safe for lock-free reads.
 *  - StampedCenter / StampedNCenter are written only under PushCS by the
 *    boundary-cross commit and read only under PushCS by the per-frame push, so
 *    the (NCenter - VT) uniform can never disagree with the live lattice.
 *  - NiagaraComponents are touched only on the game thread. */
struct FParticleTierState
{
	/** Particle data, one buffer per Niagara asset (parallel to
	 *  FParticleTierConfig::NiagaraAssets). Single-buffered: the transition task
	 *  overwrites entering slots in place, safe because the GPU holds its own
	 *  copy between pushes, the per-frame push reads no CPU arrays, and all other
	 *  readers gate on bUpdateInProgress (see threading contract). */
	TArray<FNiagaraParticleBuffer> Buffers;

	/** Raw component pointers aliasing TierNiagaraComponents. Parallel to
	 *  FParticleTierConfig::NiagaraAssets. Game-thread only. */
	TArray<UNiagaraComponent*> NiagaraComponents;

	/** True while an async boundary-cross task owns the tier's buffers and
	 *  state. The game thread must not begin a new update, nor read the CPU
	 *  buffer arrays, while this is set. */
	std::atomic<bool> bUpdateInProgress{ false };

	/** Serializes all Niagara writes for this tier (transition commit +
	 *  per-frame uniform push): the stamp write and upload happen under this
	 *  lock, so a per-frame push can never pair a stale stamp with fresh data or
	 *  vice versa. The game thread must never acquire it (it would stall behind
	 *  an upload); the GT hands off via GetLatestVT and the flags below. */
	FCriticalSection PushCS;

	/** Set on teardown (BeginShutdownDrain) so in-flight pushes bail before
	 *  touching a component about to be destroyed. */
	std::atomic<bool> bShuttingDown{ false };

	/** Grid coordinate of the cell at the center of the streaming neighborhood.
	 *  Written on the game thread before the async task starts, then read-only to
	 *  both sides until the task clears bUpdateInProgress. INT32_MIN forces a full
	 *  generate on the first UpdateTier call. */
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	/** Per-slot octree bookkeeping, indexed by modular slot index. Sized
	 *  TotalSlots at InitializeTier. The coord resident in a slot lives in the
	 *  buffers' SlotCoord arrays; this carries only the octree node refs. Written
	 *  only by the async task while bUpdateInProgress is true. */
	TArray<FSlotEntry> SlotEntries;

	/** Center coord the live lattice and (NCenter - VT) uniform were built
	 *  against. Written under PushCS by the boundary-cross commit in the same
	 *  section as the lattice upload; read under PushCS by the per-frame push.
	 *  INT32_MIN until the first commit (all particles dead, uniform inert). */
	FIntVector StampedCenter = FIntVector(INT32_MIN);

	/** GridCoordToCenter(StampedCenter), precomputed at commit so the per-frame
	 *  push needs no grid parameters. Same PushCS discipline as StampedCenter.
	 *  ZeroVector until the first commit. */
	FVector StampedNCenter = FVector::ZeroVector;

	/** Per-slot accepted particle count, written by GenerateCallback. Lets
	 *  CacheCellFromBuffers and InsertSlotIntoOctree skip dead padding without
	 *  iterating the full SlotCapacity. */
	TArray<int32> SlotCounts;

	/** Persistent procgen cache keyed by grid coord. First visit generates a
	 *  cell and stores its output here (miss); re-entry blits the stored data
	 *  into the back buffer, skipping noise and rejection sampling (hit). The
	 *  slot recycles on exit but the cache entry survives. Entries beyond
	 *  NeighborhoodRadius + 4 cells (Chebyshev) are evicted by CullTierCache
	 *  after each boundary cross. */
	TMap<FIntVector, FCachedCellData> CellCache;

	/** Resets all plain state data to the freshly constructed baseline so a
	 *  pooled owner can reuse this tier as if new. Does not touch
	 *  NiagaraComponents (UObjects the owning actor destroys and empties before
	 *  calling this). Call only after the push worker has exited and any
	 *  in-flight tier task has drained. */
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

/** Read-only context supplied by the owning actor to the tier pipeline. Passed
 *  by reference into every FTierStreamingSystem call so the pipeline reads
 *  spatial parameters without knowing the concrete actor type. The actor
 *  populates it via BuildStreamingContext(); the pipeline never writes through
 *  it. */
struct FTierStreamingContext
{
	/** Sector/galaxy extent; drives grid cell sizing and dead-pos parking. */
	double Extent = 0.0;

	/** UnitScale of the owning actor, used by InsertParticleIntoOctree to
	 *  compute octree insert depth. Universe passes 1.0 (extents already local);
	 *  Galaxy passes its actual UnitScale. */
	double UnitScale = 1.0;

	/** Backdrop membership for this actor's tier sprites. Derived from the actor's
	 *  REAL UnitScale via IsVirtualSpace() (compressed virtual space > 1 -> backdrop;
	 *  real space == 1 -> main pass) — deliberately SEPARATE from the UnitScale above,
	 *  which the Universe forces to 1.0 for octree-insert math. InitializeTier stamps
	 *  bVisibleInSceneCaptureOnly from this. */
	bool bVirtualSpace = false;

	/** Multiplier applied to Extent for grid cell sizing.
	 *  CellSize = (Extent * GridExtentMultiplier) / (1 << GridDepth). */
	double GridExtentMultiplier = 4.0;

	/** The actor's current virtual traversal vector; drives grid coord
	 *  derivation and camera-relative position computation. */
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

	/** When true, spawned Niagara components use absolute world position
	 *  (decoupled from actor transform). Used by Galaxy, where the actor is
	 *  pegged to the player but Niagara positions are galaxy-local. */
	bool bNiagaraAbsolutePosition = false;

	/** Owning actor name for log output. */
	FString OwnerName;

	/** The owning actor's Params.Seed, used by the octree insertion pipeline to
	 *  compose deterministic child seeds via FVoxelData::ComposeSeed. Each level
	 *  passes its own seed (itself a ComposeSeed output from its parent):
	 *  UniverseSeed -> GalaxySeed -> StarSystemSeed -> PlanetSeed. */
	int32 ParentSeed = 0;

	/** Returns the actor's freshest VirtualTraversal thread-safely. Push tasks
	 *  call this at execution time (under PushCS) so a push always composites
	 *  against current VT, not a value snapshotted when scheduled. Populated by
	 *  BuildStreamingContext. */
	TFunction<FVector()> GetLatestVT;

	/** Returns the actor's live lifecycle state. InitializationState above is a
	 *  by-value snapshot from BuildStreamingContext, so it can never observe a
	 *  mid-init flip to Pooling; the pipeline's abort checks read this instead.
	 *  When unset, the snapshot is used (no abort). Populated by
	 *  BuildStreamingContext. */
	TFunction<ELifecycleState()> GetLiveState;
};

#pragma endregion

#pragma region Stateless Pipeline

/** Stateless utility implementing the entire tier streaming pipeline: grid
 *  coord math, InitializeTier, UpdateTier, PushTierToNiagara, cell caching, and
 *  octree integration. AUniverseActor, AGalaxyActor, and AStarSystemActor
 *  delegate here rather than each maintaining a copy; all actor-specific
 *  behavior is injected via FParticleTierConfig callbacks and
 *  FTierStreamingContext. See FParticleTierState for the threading contract. */
struct FTierStreamingSystem
{
#pragma region Grid Coord Helpers

	/** Converts a local position to a grid coordinate at the given depth. */
	static FIntVector PositionToGridCoord(const FVector& InPos, int32 InGridDepth,
		double Extent, double GridExtentMultiplier)
	{
		const double CellSize = (Extent * GridExtentMultiplier) / (1 << InGridDepth);
		return FIntVector(FMath::FloorToInt32(InPos.X / CellSize + 0.5), FMath::FloorToInt32(InPos.Y / CellSize + 0.5), FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
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
	/** slot(coord) is a fixed bijection over any Side-wide window of cells:
	 *  within the window each per-axis residue appears once, so every resident
	 *  coord keeps its slot (and particle data) as the window slides, and the
	 *  coord entering on the leading face is congruent mod Side to the coord
	 *  exiting on the trailing face and reuses its slot. Side = 1 (radius 0) is
	 *  the degenerate torus: PosMod(_,1) == 0, one slot, no wrapping, same code
	 *  path. */

	 /** Non-negative modulo. C++ '%' is negative for negative operands; this
	  *  returns a value in [0, m). m must be > 0. */
	static FORCEINLINE int32 PosMod(int32 a, int32 m)
	{
		return ((a % m) + m) % m;
	}

	/** Flattens three per-axis residues (each in [0, Side)) to a slot index in
	 *  [0, Side^3). */
	static FORCEINLINE int32 FlattenResidues(int32 a, int32 b, int32 c, int32 Side)
	{
		return (a * Side + b) * Side + c;
	}

	/** Modular slot index for a cell coordinate. Side = 2*NeighborhoodRadius+1
	 *  (odd by construction, asserted at InitializeTier). Bijective over any
	 *  Side-wide window; never changes for a resident coord. */
	static FORCEINLINE int32 SlotOf(const FIntVector& Coord, int32 Side)
	{
		return FlattenResidues(PosMod(Coord.X, Side), PosMod(Coord.Y, Side), PosMod(Coord.Z, Side), Side);
	}

#pragma endregion

#pragma region Tier Initialization

	/** Allocates buffers and slot bookkeeping for a tier, then spawns and
	 *  activates its Niagara components on the game thread. Exhaustive tiers
	 *  (radius 0) generate, octree-insert, and cache their full window immediately;
	 *  streaming tiers defer population to the first UpdateTier (CenterCoord =
	 *  INT32_MIN). Aborts if the actor enters Pooling mid-init. */
	static void InitializeTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State, TArray<UNiagaraComponent*>& OutComponents);

#pragma endregion

#pragma region Tier Streaming Update

	/** Streams the resident window to follow the player: on a boundary cross,
	 *  diffs the old and new neighborhoods, generates or cache-restores entering
	 *  cells in place into their modular slots, octree-inserts them, and commits
	 *  the transition push on a worker thread (bUpdateInProgress held throughout).
	 *  No-ops unless Ready and actually crossing; the streaming gate freezes the
	 *  window while the player's cell is skippable. Radius-0 tiers never stream. */
	static void UpdateTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State);

#pragma endregion

#pragma region Niagara Push

	/** Boundary-cross transition commit (worker thread, tail of UpdateTier's
	 *  async task). Under the tier PushCS, atomically:
	 *  1. stamps StampedCenter/StampedNCenter with the center the new lattice
	 *     derives from,
	 *  2. uploads the buffer: cell-local positions, per-slot lattice
	 *     (User.CellOffsets), and the (NCenter - VT) uniform, reading the
	 *     freshest VT via GetLatestVT at execution time.
	 *  The GPU flips atomically at upload: it renders its prior copy of every
	 *  array until these sets land, so in-place CPU generation is never visible
	 *  mid-write. Because the per-frame push takes the same lock and reads the
	 *  same stamp, no push can pair a lattice/uniform built against different
	 *  centers. Fixed bounds are set once at init and never updated. */
	static void PushTierToNiagara(const TFunction<FVector()>& GetLatestVT, const FIntVector& NewCenter, const FVector& NewNCenter, const FParticleTierConfig& Config, FParticleTierState& State);

	/** Per-frame parallax re-push: a single FVector uniform per component,
	 *  (StampedNCenter - VT), with no array traffic. Skips (rather than blocks
	 *  on) a tier whose PushCS is contended, since the holder is the commit,
	 *  which seeds a fresher uniform itself. Distinct from PushTierToNiagara,
	 *  which commits a boundary-cross data swap. The threshold gate and
	 *  LastPushedVirtualTraversal bookkeeping stay with the caller; this only
	 *  performs the writes. */
	static void PushTierVT(std::initializer_list<FParticleTierState*> Tiers, const TFunction<FVector()>& GetLatestVT);

	/** Game thread (teardown): blocks until in-flight pushes drain, then bars
	 *  new ones. */
	static void BeginShutdownDrain(FParticleTierState& State);

#pragma endregion

#pragma region Octree Integration

	/** Inserts every occupied slot's live particles (from OctreeInsertBufferIndex)
	 *  into the context octree. Full-window pass used after exhaustive-tier init;
	 *  no-op when the tier has no insert buffer or octree. */
	static void InsertTierIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State);

	/** Inserts one entering slot's live particles into the octree, first resetting
	 *  the slot entry (releasing the previous occupant's node refs). Guards on
	 *  residency: SlotCoord[SlotIndex] must equal Coord. Incremental per-cross path. */
	static void InsertSlotIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	/** Inserts one particle: builds its FPointData (world extent -> octree-local
	 *  via UnitScale), composes a deterministic seed, captures the exact particle
	 *  position/extent, inserts into the octree, and records the node on Entry.
	 *  Skips non-positive extents. */
	static void InsertParticleIntoOctree(const FTierStreamingContext& Ctx, FSlotEntry& Entry, const FVector& Position, const float& Extent, const FLinearColor& Color, const FIntVector& GridCoord, int32 GenerationIndex, int32 AbsoluteBufferIndex, double TreeExtent, int32 TierIndex);

#pragma endregion

#pragma region Cell Cache

	/** Snapshots a slot's live particle data (per buffer) into the persistent cell
	 *  cache keyed by Coord, so re-entry blits the stored arrays instead of
	 *  regenerating. A zero-count cell is cached as a legitimate empty result. */
	static void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	/** Evicts cell-cache entries beyond NeighborhoodRadius + 4 cells (Chebyshev)
	 *  from NewCenter, bounding cache growth as the window streams. */
	static void CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& NewCenter);

#pragma endregion
};

#pragma endregion
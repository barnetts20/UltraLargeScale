// FTierStreamingSystem.h
// Owns the tier system's authored config (FTierParams), runtime data
// structures (FSlotEntry, FParticleTierConfig, FParticleTierState), the
// per-call actor context (FTierStreamingContext), and the stateless utility
// pipeline that operates on them. Both AUniverseActor and AGalaxyActor (and
// AStarSystemActor) include this header and delegate here.
// 
// TODO: LEAVING THE NEXT BLOCK EVEN THOUGH I WAS ABOUT TO DELETE IT - IT FORMS A GREAT EXAMPLE OF THE TYPE OF COMMENT I WANT TO CULL
// TODO: A COMMENT TALKING ABOUT THE WAY THINGS "USED" TO BE IN A CODE BASE WHERE IT IS NO LONGER LIKE THAT, IS WORSE THAN USELESS - ITS BLOAT
// TODO: COMMENTS SHOULD BE STATELESS/TIME UNAWARE - I WANT COMMENTS THAT TELL ME HOW THINGS WORK, NOT HOW IT USED TO WORK AND WHAT WE CHANGED - WE SHOULD KEEP AN EYE OUT FOR SIMILAR COMMENTS IN THE CODE BASE TO CLEAN THEM UP
// FTierParams and FTierStreamingContext previously lived in
// UniverseDataGenerator.h and FTierStreamingContext.h respectively; they were
// consolidated here so every consumer of the tier pipeline pulls a single
// header and no longer cross-includes a generator or a one-struct file.

// TODO: PROJECT WIDE - CONSISTENT DOCUMENTATION COMMENT FORMATTING. WE ALSO WANT TO BE AS TERSE AS POSSIBLE WHILE MAINTAINING CURRENT ARCHITECHTURE DOCUMENTATION SPECIFICS (LESS VERBOSE, WHILE MAINTAINING FUNCTIONAL DOCUMENTATION OF BEHAVIOR)
#pragma once

//TODO: NITPICKY, AND ONLY LEAVING ONE COMMENT BUT WE COULD USE AN INCLUDE ORDERING AUDIT TO ORDER ALL THE INCLUDES ACROSS THE PROJECT ALPHABETICALLY UNLESS THEY REQUIRE A SPECIFIC ORDERING (.generated)
#include "CoreMinimal.h"
#include "FNiagaraParticleBuffer.h"
#include "FOctree.h"
#include "DataTypes.h"
#include "Curves/CurveFloat.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FTierStreamingSystem.generated.h"

class USceneComponent;

// ============================================================================
//  Tier System — Data Structures
//  These are the shared types consumed by both AUniverseActor and AGalaxyActor.
//  They live here (not in either actor header) so any consumer of the tier
//  pipeline can include a single header without pulling in actor definitions.
// ============================================================================

/// Per-tier streaming parameters exposed in the editor. This is the AUTHORED
/// counterpart to the runtime FParticleTierConfig below: each layer's params
/// struct (FUniverseParams, FGalaxyParams, FStarSystemParams) holds one of
/// these per tier, and BuildTierConfigs() reads them to populate the runtime
/// FParticleTierConfig.
USTRUCT(BlueprintType)
struct SVO_API FTierParams
{
	GENERATED_BODY()

	// Octree grid depth that defines this tier's cell size.
	// Cell half-extent = (Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)).
	// Use evenly-spaced depths (current scheme: 1, 3, 5 → spacing of 2).
	// Scale ratio between adjacent tiers = 2^spacing (spacing 2 → ratio 4).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 GridDepth = 6;

	// Half-width of the 3D neighborhood streamed around the player.
	// 1 -> 3x3x3 = 27 slots.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 NeighborhoodRadius = 1;

	// Max particles per slot (candidate count before rejection).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Streaming")
	int32 SlotCapacity = 500;

	// Maps a uniform [0,1] sample to a [0,1] t-value that lerps between
	// MinScale and MaxScale. Controls the size distribution of particles
	// within this tier. Defaults to identity (linear).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution")
	FRuntimeFloatCurve ScaleDistribution;

	// Maps the raw noise density [0,1] to a modified density [0,1] before
	// the rejection gate. Controls how aggressively the noise field is
	// interpreted — e.g. a steep curve concentrates particles in high-density
	// regions, a flat curve makes them more uniform. Defaults to identity.
	// Values > 1.0 are clamped, allowing the curve to sharpen features.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Distribution")
	FRuntimeFloatCurve DensityResponse;

	//TODO: THESE ARE DERIVED... IF WE DONT MAKE USE OF THEM EXTERNALLY WE COULD POTENTIALLY MAKE PRIVATE
	// Largest entity scale this tier represents.
	double MaxScale = 0.0;

	// Smallest entity scale this tier represents.
	double MinScale = 0.0;

	// Initialize both curves to identity: f(x) = x.
	FTierParams()
	{
		ScaleDistribution.GetRichCurve()->AddKey(0.0f, 0.0f);
		ScaleDistribution.GetRichCurve()->AddKey(1.0f, 1.0f);

		DensityResponse.GetRichCurve()->AddKey(0.0f, 0.0f);
		DensityResponse.GetRichCurve()->AddKey(1.0f, 1.0f);
	}

	/// Derive MinScale/MaxScale for an ordered array of tiers from a single
	/// MaxEntityScale value and the depth sequence between tiers.
	///
	/// Tiers must be ordered shallowest-first (Large → Mid → Small).
	/// The depth spacing between adjacent tiers determines the scale ratio:
	///   ratio = 2 ^ (nextDepth - thisDepth)
	/// The last tier mirrors the spacing of the previous pair.
	///
	/// Example with depths 1, 3, 5 and MaxEntityScale = 1e22 (current
	/// universe defaults):
	///   Large: 2.5e21    → 1e22    (ratio 4, spacing 2)
	///   Mid:   6.25e20   → 2.5e21  (ratio 4, spacing 2)
	///   Small: 1.5625e20 → 6.25e20 (ratio 4, mirrors spacing 2)
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

/**
 * Octree bookkeeping for one flat-buffer slot. Stored slot-indexed in
 * FParticleTierState::SlotEntries (the slot index IS the array index —
 * under toroidal addressing coord -> slot is a fixed modular function, so
 * no coord-keyed map is needed). Needed for targeted octree node removal
 * without a full tree scan.
 */
struct FSlotEntry
{
	/** Octree nodes inserted from this slot's live particles. Reset and
	 *  repopulated when a new cell generates into the slot; the shared
	 *  refs from the previous occupant are released at that point, so the
	 *  spatial index survives beyond the streaming window exactly as it
	 *  did under the old exit-time map removal. */
	TArray<TSharedPtr<FOctreeNode>> InsertedNodes;
};

/**
 * Immutable descriptor for one particle streaming tier. Populated once by
 * BuildTierConfigs() and never mutated at runtime. All behavioural differences
 * between Large / Mid / Small tiers are encoded here; the generic pipeline
 * (InitializeTier, UpdateTier) reads these fields and delegates generation
 * via the callbacks.
 */
struct FParticleTierConfig
{
	/** Human-readable name used in log output (e.g. "Large", "Mid", "Small"). */
	FString TierName;

	/**
	 * Octree depth that defines this tier's streaming cell size.
	 * Cell half-extent = (Params.Extent * GridExtentMultiplier) / (1 << (GridDepth + 1)).
	 * With the standard GridExtentMultiplier = 4 and the current depth
	 * sequence (Large = 1, Mid = 3, Small = 5):
	 *   depth 1  →  cell half-extent = Extent
	 *   depth 3  →  cell half-extent = Extent / 4
	 *   depth 5  →  cell half-extent = Extent / 16
	 */
	int32 GridDepth = 1;

	/**
	 * Half-width of the 3D cell neighborhood streamed around the player.
	 * Total active slots = (2 * NeighborhoodRadius + 1)^3.
	 * e.g. radius 1 → 3×3×3 = 27 slots.
	 */
	int32 NeighborhoodRadius = 1;

	/** Maximum particles written per slot (candidate count before rejection). */
	int32 SlotCapacity = 0;

	/** Tier index written into octree node TypeId on insert.
	 *  Used by spawn hooks to identify which tier a node came from
	 *  without searching scale ranges. Large=0, Mid=1, Small=2. */
	int32 TierIndex = 0;

	/**
	 * One Niagara system template per logical buffer in this tier.
	 * Large tier has two (cluster + gas); Mid and Small have one each.
	 * InitializeTier spawns one UNiagaraComponent per entry.
	 */
	TArray<UNiagaraSystem*> NiagaraAssets;

	/**
	 * Parallel to NiagaraAssets. True if the corresponding buffer should
	 * allocate the Rotations array (face normals for non-billboard rendering).
	 * Large cluster = true; gas, Mid, Small = false.
	 */
	TArray<bool> bWantRotations;

	/**
	 * Index into NiagaraAssets / Buffers that is walked during octree insertion.
	 * Set to -1 to skip octree insertion entirely for this tier.
	 * Large = 0 (cluster buffer); Mid = 0; Small = 0.
	 */
	int32 OctreeInsertBufferIndex = 0;

	/**
	 * Particle generation callback. Invoked once per entering cell during
	 * parallel generation (both InitializeTier and UpdateTier). Writes
	 * directly into the slot region of each buffer pointer provided.
	 *
	 * @param Coord      Grid coordinate of the cell being generated.
	 * @param SlotIndex  Flat slot index within the tier's particle buffers.
	 * @param Buffers    One raw pointer per NiagaraAsset, each pointing to
	 *                   the back buffer for this tier.
	 */
	TFunction<void(const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers)> GenerateCallback;

	/**
	 * Returns the axis-aligned bounding box used as the Niagara fixed bounds
	 * for all components in this tier. Recomputed on each boundary cross push
	 * so Lumen/radiance-cache always sees a tight box around actual particles.
	 */
	TFunction<FBox()> ComputeBounds;

	/**
	 * Optional hook fired inside UpdateTier's async task after exiting slots
	 * are freed but before particle generation begins. Used by the streaming
	 * volumetric to update density sub-regions in lockstep with Large-tier
	 * boundary crosses. Not set for Mid or Small tiers.
	 *
	 * @param Entering   Grid coords of cells entering the streaming window.
	 * @param Exiting    Grid coords of cells leaving the streaming window.
	 * @param NewCenter  The tier's new center coord after the boundary cross.
	 */
	TFunction<void(const TArray<FIntVector>& Entering, const TArray<FIntVector>& Exiting, const FIntVector& NewCenter)> OnBoundaryCross;

	/**
	 * Optional per-cell culling predicate with two roles:
	 *
	 * 1. PER-CELL CULL: evaluated during UpdateTier for each entering cell.
	 *    Returns true if the cell should be skipped (dead-padded with zero
	 *    particles, no generation or cache lookup).
	 *
	 * 2. STREAMING GATE: evaluated against the player's own (new) center
	 *    cell before any transition is admitted. If the player's cell is
	 *    itself skippable, the tier does not transition at all — the
	 *    resident window FREEZES at its last in-bounds center (live edge
	 *    cells persist as a boundary halo; the per-frame uniform keeps them
	 *    parallaxing correctly). Streaming resumes on the first in-bounds
	 *    crossing; multi-cell re-entry deltas fall into the teleport path.
	 *    Because the gate is a function of the center CELL, it can only
	 *    change value at a cell boundary — no hysteresis needed.
	 *
	 * Used by bounded actors (GalaxyActor, StarSystemActor) to confine both
	 * generation and boundary-cross tracking to their volume. Not set for
	 * unbounded actors (UniverseActor) or for radius-0 exhaustive tiers
	 * (which never stream and must stay resident at any distance).
	 *
	 * @param Coord  Grid coordinate of the cell being tested.
	 * @return       True to skip this cell / suppress streaming from it.
	 */
	TFunction<bool(const FIntVector& Coord)> ShouldSkipCell;
};

/**
 * Mutable runtime state for one particle streaming tier. Fully owned by the
 * tier pipeline; generation callbacks write into Buffers directly but do not
 * touch any other fields.
 *
 * @note This is a plain struct, not a USTRUCT. UNiagaraComponent* pointers
 *       stored here alias entries in the owning actor's TierNiagaraComponents
 *       (a UPROPERTY TArray) for GC safety. Do not store them elsewhere.
 *
 * Threading contract:
 *   - CenterCoord, SlotEntries, SlotCounts, CellCache, and the Buffers'
 *     CPU arrays are written exclusively by the async task spawned from
 *     UpdateTier, IN PLACE (single buffer — entering slots are overwritten
 *     directly; nothing reads them mid-write, see below). Anything outside
 *     the pipeline that reads Buffers must hold bUpdateInProgress == false;
 *     on the game thread that read is race-free, because the flag is set on
 *     the game thread before the task spawns and cleared by the worker
 *     after the commit — a false read on the GT means no task is running
 *     and none can start within the current GT scope.
 *   - The per-frame uniform push reads NO CPU arrays — only the stamps —
 *     which is what makes in-place generation safe without a back buffer.
 *   - bUpdateInProgress is std::atomic and safe for lock-free reads.
 *   - StampedCenter / StampedNCenter are written only under PushCS by the
 *     boundary-cross commit and read only under PushCS by the per-frame
 *     push, so the (NCenter - VT) uniform can never disagree with the live
 *     lattice — even on a transition frame.
 *   - NiagaraComponents are only touched on the game thread.
 */
struct FParticleTierState
{
	/**
	 * Particle data, ONE buffer per Niagara asset (index parallel to
	 * FParticleTierConfig::NiagaraAssets). Single-buffered: the transition
	 * task overwrites entering slots in place. This is safe because the GPU
	 * holds its own copy of every array between pushes, the per-frame push
	 * reads no CPU arrays, and all other readers gate on bUpdateInProgress
	 * (see the threading contract above).
	 */
	TArray<FNiagaraParticleBuffer> Buffers;

	/** Raw component pointers aliasing TierNiagaraComponents. Parallel to
	 *  FParticleTierConfig::NiagaraAssets. Game-thread only. */
	TArray<UNiagaraComponent*> NiagaraComponents;

	/** True while an async boundary-cross task owns the tier's buffers and
	 *  state. Game thread must not begin a new update — nor read the CPU
	 *  buffer arrays — while this is set. */
	std::atomic<bool> bUpdateInProgress{ false };

	/** Serializes ALL Niagara writes for this tier (transition commit +
	 *  per-frame uniform push). The stamp write and upload happen under this
	 *  lock, so a per-frame push can never pair a stale stamp with fresh
	 *  data or vice versa. The GAME THREAD must never acquire it -- it would
	 *  stall behind an upload; the GT hands off via PublishLatestVT and the
	 *  dirty flags below instead. */
	FCriticalSection PushCS;

	/** Set on teardown (BeginShutdownDrain) so in-flight pushes bail before touching
	 *  a component that is about to be destroyed. */
	std::atomic<bool> bShuttingDown{ false };

	/** Grid coordinate of the cell currently at the center of the streaming
	 *  neighborhood. Written on the game thread before the async task starts,
	 *  then treated as read-only by both sides until the task clears
	 *  bUpdateInProgress. Initialized to INT32_MIN to force a full generate
	 *  on the first UpdateTier call. */
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	/** Per-slot octree bookkeeping, indexed by (modular) slot index. Sized
	 *  TotalSlots at InitializeTier. The coord currently resident in a slot
	 *  lives in the buffers' SlotCoord arrays; this only carries the octree
	 *  node refs. Written only by the async task while bUpdateInProgress is
	 *  true. */
	TArray<FSlotEntry> SlotEntries;

	/** C_stamp: the center coord the LIVE lattice and (NCenter - VT) uniform
	 *  were built against. Written under PushCS by the boundary-cross commit
	 *  in the same critical section as the lattice
	 *  upload; read under PushCS by the per-frame push. INT32_MIN until the
	 *  first commit (all particles dead — the uniform is inert). */
	FIntVector StampedCenter = FIntVector(INT32_MIN);

	/** GridCoordToCenter(StampedCenter), precomputed at commit so the
	 *  per-frame push needs no grid parameters. Same PushCS discipline as
	 *  StampedCenter. ZeroVector until the first commit. */
	FVector StampedNCenter = FVector::ZeroVector;

	/** Per-slot accepted particle count, written by GenerateCallback.
	 *  Used by CacheCellFromBuffers and InsertSlotIntoOctree to skip dead
	 *  padding without iterating the full SlotCapacity. */
	TArray<int32> SlotCounts;

	/** Persistent procgen cache keyed by grid coord. On first visit a cell is
	 *  generated and its output stored here (cache-miss). On re-entry the
	 *  stored data is blitted directly into the back-buffer, skipping all noise
	 *  and rejection sampling (cache-hit). The slot is recycled on exit but the
	 *  cache entry survives. Entries beyond NeighborhoodRadius + 4 cells
	 *  (Chebyshev) are evicted by CullTierCache after each boundary cross. */
	TMap<FIntVector, FCachedCellData> CellCache;

	/** Resets all PLAIN STATE DATA to the freshly - constructed baseline so a
	 *  pooled owner can reuse this tier as if new. Does NOT touch
	 *  NiagaraComponents — those are UObjects the owning actor destroys
	 *  (resource lifetime); it destroys them and empties that container, then
	 *  calls this. Call only AFTER the push worker has exited and any in-flight
	 *  tier task has drained (both read the fields cleared here). */
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

/**
 * Read-only context supplied by the owning actor to the tier pipeline.
 * Passed by pointer into every FTierStreamingSystem call so the pipeline
 * can read spatial parameters without knowing the concrete actor type.
 *
 * The owning actor populates this once per tick (or once per call) via its
 * BuildStreamingContext() and hands it in. The pipeline never writes through
 * this pointer.
 */
struct FTierStreamingContext
{
	/** Sector/galaxy extent — drives grid cell sizing and dead-pos parking. */
	double Extent = 0.0;

	/** UnitScale of the owning actor. Used by InsertParticleIntoOctree to
	 *  compute octree insert depth. Universe passes 1.0 (extents already
	 *  local); Galaxy passes its actual UnitScale. */
	double UnitScale = 1.0;

	/** Multiplier applied to Extent for grid cell size computation.
	 *  CellSize = (Extent * GridExtentMultiplier) / (1 << GridDepth). */
	double GridExtentMultiplier = 4.0;

	/** The actor's current virtual traversal vector. Drives grid coord
	 *  derivation and camera-relative position computation. */
	FVector VirtualTraversal = FVector::ZeroVector;

	/** The actor's persistent spatial index. Octree insert calls go here. */
	TSharedPtr<FOctree> Octree;

	/** Current lifecycle state. Pipeline early-outs if not Ready. */
	ELifecycleState InitializationState = ELifecycleState::Uninitialized;

	/** True while an octree rebase is in progress (Universe only).
	 *  UpdateTier must not proceed while this is set. */
	bool bRebaseInProgress = false;

	/** Root component to attach spawned Niagara components to. */
	USceneComponent* AttachRoot = nullptr;

	/** When true, spawned Niagara components use absolute world position
	 *  (decoupled from actor transform). Used by Galaxy where the actor
	 *  is pegged to the player but Niagara positions are galaxy-local. */
	bool bNiagaraAbsolutePosition = false;

	/** Owning actor name for log output. */
	FString OwnerName;

	/** The owning actor's Params.Seed. Used by the octree insertion pipeline
	 *  to compose deterministic child seeds via FVoxelData::ComposeSeed.
	 *  Each hierarchy level passes its own seed (which is itself a ComposeSeed
	 *  output from its parent), forming a chain:
	 *  UniverseSeed → GalaxySeed → StarSystemSeed → PlanetSeed. */
	int32 ParentSeed = 0;

	/** Returns the actor's freshest VirtualTraversal in a thread-safe way. Push
	 *  tasks call this at execution time (under the tier PushCS) so a push always
	 *  composites against current VT, not a value snapshotted when it was scheduled.
	 *  Populated by each actor's BuildStreamingContext. */
	TFunction<FVector()> GetLatestVT;

	/** Returns the actor's LIVE lifecycle state. InitializationState above is
	 *  a by-value snapshot taken in BuildStreamingContext, so it can never
	 *  observe a mid-init flip to Pooling — the pipeline's abort checks read
	 *  this instead. Optional: when unset, the snapshot is used (no abort).
	 *  Populated by each actor's BuildStreamingContext. */
	TFunction<ELifecycleState()> GetLiveState;
};

// ============================================================================
//  Tier Streaming System — Stateless Pipeline
// ============================================================================

/**
 * Stateless utility that implements the entire tier streaming pipeline:
 * grid coord math, InitializeTier, UpdateTier, PushTierToNiagara,
 * cell caching, octree integration.
 *
 * Both AUniverseActor and AGalaxyActor delegate here instead of each
 * maintaining their own copy. All actor-specific behavior is injected
 * via FParticleTierConfig callbacks (GenerateCallback, ComputeBounds,
 * OnBoundaryCross, ShouldSkipCell) and the FTierStreamingContext.
 *
 * Threading contract: identical to the original per-actor implementations.
 * See FParticleTierState documentation for details.
 */
struct FTierStreamingSystem
{
	// ========================================================================
	//  Grid Coord Helpers
	// ========================================================================

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

	// ========================================================================
	//  Toroidal (Modular) Slot Addressing
	// ========================================================================
	//  slot(coord) is a fixed bijection over any Side-wide window of cells:
	//  within the window each per-axis residue appears exactly once, so every
	//  resident coord keeps its slot (and its particle data) when the window
	//  slides, and the coord entering on the leading face is congruent mod
	//  Side to the coord exiting on the trailing face — it reuses that slot.
	//  Side = 1 (radius 0) is the degenerate torus: PosMod(_,1) == 0, one
	//  slot, no wrapping — same code path, no special case.

	/** Non-negative modulo. UE/C++ '%' is negative for negative operands;
	 *  this always returns a value in [0, m). m must be > 0. */
	static FORCEINLINE int32 PosMod(int32 a, int32 m)
	{
		return ((a % m) + m) % m;
	}

	/** Flattens three per-axis residues (each in [0, Side)) to a slot index
	 *  in [0, Side^3). */
	static FORCEINLINE int32 FlattenResidues(int32 a, int32 b, int32 c, int32 Side)
	{
		return (a * Side + b) * Side + c;
	}

	/** Modular slot index for a cell coordinate. Side = 2*NeighborhoodRadius+1
	 *  (odd by construction — asserted at InitializeTier). Bijective over any
	 *  Side-wide window; NEVER changes for a resident coord. */
	static FORCEINLINE int32 SlotOf(const FIntVector& Coord, int32 Side)
	{
		return FlattenResidues(PosMod(Coord.X, Side), PosMod(Coord.Y, Side), PosMod(Coord.Z, Side), Side);
	}

	// ========================================================================
	//  Tier Initialization
	// ========================================================================

	static void InitializeTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State, TArray<UNiagaraComponent*>& OutComponents);

	// ========================================================================
	//  Tier Streaming Update
	// ========================================================================

	static void UpdateTier(const FTierStreamingContext& Ctx, FParticleTierConfig& Config, FParticleTierState& State);

	// ========================================================================
	//  Niagara Push
	// ========================================================================

	/** Boundary - cross transition COMMIT(worker thread, tail of UpdateTier's
	 * async task). Under the tier PushCS, atomically:
	 *   1. stamps StampedCenter/StampedNCenter with the center the new
	 *      lattice derives from (§ C_stamp),
	 *   2. uploads the buffer — cell-local positions, per-slot lattice
	 *      (User.CellOffsets), and the (NCenter - VT) uniform, reading
	 *      the freshest VT via GetLatestVT at execution time.
	 * The GPU flips atomically at the upload: it keeps rendering its own
	 * prior copy of every array until these sets land, so in-place CPU
	 * generation is never visible mid-write. Because the per-frame push
	 * takes the same lock and reads the same stamp, no push can pair a
	 * lattice/uniform built against different centers. Fixed bounds are
	 * set once at init and never updated.
	 */
	static void PushTierToNiagara(const TFunction<FVector()>& GetLatestVT, const FIntVector& NewCenter, const FVector& NewNCenter, const FParticleTierConfig& Config, FParticleTierState& State);

	/**
	 * Per-frame parallax re-push: a SINGLE FVector uniform per component —
	 * (StampedNCenter - VT) — no array traffic at all. Skips (rather than
	 * blocks on) a tier whose PushCS is contended, since the holder is the
	 * commit, which seeds a fresher uniform itself. Used by the actor
	 * parallax overrides when
	 * VirtualTraversal moves past the push threshold. Distinct from
	 * PushTierToNiagara, which commits a boundary-cross data swap. The
	 * threshold gate and LastPushedVirtualTraversal bookkeeping stay with
	 * the caller — this only performs the writes.
	 */
	static void PushTierPositions(std::initializer_list<FParticleTierState*> Tiers, const TFunction<FVector()>& GetLatestVT);

	// Game thread (teardown): block until in-flight pushes drain, then bar new ones.
	static void BeginShutdownDrain(FParticleTierState& State);

	// ========================================================================
	//  Octree Integration
	// ========================================================================

	static void InsertTierIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State);

	static void InsertSlotIntoOctree(const FTierStreamingContext& Ctx, const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	static void InsertParticleIntoOctree(const FTierStreamingContext& Ctx, FSlotEntry& Entry, const FVector& Position, const float& Extent, const FLinearColor& Color, const FIntVector& GridCoord, int32 GenerationIndex, int32 AbsoluteBufferIndex, double TreeExtent, int32 TierIndex);

	// ========================================================================
	//  Cell Cache
	// ========================================================================

	static void CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& Coord, int32 SlotIndex);

	static void CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State, const FIntVector& NewCenter);
};
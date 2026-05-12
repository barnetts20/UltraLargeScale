// FTierStreamingSystem.h
// Owns the tier system data structures (FSlotEntry, FParticleTierConfig,
// FParticleTierState) and the stateless utility pipeline that operates on them.
// Both AUniverseActor and AGalaxyActor include this header and delegate here.

#pragma once

#include "CoreMinimal.h"
#include "FTierStreamingContext.h"
#include "FNiagaraParticleBuffer.h"
#include "FOctree.h"
#include "DataTypes.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

// ============================================================================
//  Tier System — Data Structures
//  These are the shared types consumed by both AUniverseActor and AGalaxyActor.
//  They live here (not in either actor header) so any consumer of the tier
//  pipeline can include a single header without pulling in actor definitions.
// ============================================================================

/**
 * Maps one active grid cell to its flat-buffer slot and the octree nodes
 * inserted from that slot. Needed for O(1) slot recycling on cell exit and
 * for targeted octree node removal without a full tree scan.
 */
struct FSlotEntry
{
	/** Index into the tier's flat particle buffer. -1 = unassigned. */
	int32 SlotIndex = -1;

	/** Octree nodes inserted from this slot's live particles. Cleared and
	 *  repopulated on each boundary cross; left persistent on cell exit so
	 *  the spatial index survives beyond the streaming window. */
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
	 *   Large = 1  →  cell half-extent = 2 * Extent
	 *   Mid   = 4  →  cell half-extent = Extent / 8
	 *   Small = 7  →  cell half-extent = Extent / 64
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
	 * Optional per-cell culling predicate evaluated during UpdateTier for each
	 * entering cell. Returns true if the cell should be skipped (dead-padded
	 * with zero particles, no generation or cache lookup). Used by bounded
	 * actors (e.g. GalaxyActor) to skip cells entirely outside their volume.
	 * Not set for unbounded actors (UniverseActor).
	 *
	 * @param Coord  Grid coordinate of the entering cell.
	 * @return       True to skip this cell entirely.
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
 *   - CenterCoord, ActiveSlots, FreeSlots, SlotCounts, CellCache, and the
 *     back-buffer are written exclusively by the async task spawned from
 *     UpdateTier. The game thread must not read them while bUpdateInProgress
 *     is true.
 *   - FrontIdx, bUpdateInProgress, and bNeedsPush are std::atomic and safe
 *     for lock-free cross-thread reads.
 *   - The front-buffer (Buffers[b][FrontIdx]) is read-only on the game thread
 *     (ApplyParallaxOffset, PushTierToNiagara). Async tasks must not write it.
 *   - NiagaraComponents are only touched on the game thread.
 */
struct FParticleTierState
{
	/**
	 * Double-buffered particle data, one pair per Niagara asset.
	 * Outer index = asset/buffer index. Inner index is always 2:
	 * [FrontIdx] = live data read by game thread.
	 * [1-FrontIdx] = back buffer written by async generation.
	 */
	TArray<TArray<FNiagaraParticleBuffer>> Buffers;

	/** Raw component pointers aliasing TierNiagaraComponents. Parallel to
	 *  FParticleTierConfig::NiagaraAssets. Game-thread only. */
	TArray<UNiagaraComponent*> NiagaraComponents;

	/** Index of the currently live buffer (0 or 1). Swapped atomically by
	 *  the async task after generation completes. */
	std::atomic<int32> FrontIdx{ 0 };

	/** True while an async boundary-cross task owns the back-buffer and state.
	 *  Game thread must not begin a new update while this is set. */
	std::atomic<bool> bUpdateInProgress{ false };

	/** Set by the async task when a new back-buffer is ready to push.
	 *  Cleared by the game thread after PushTierToNiagara completes. */
	std::atomic<bool> bNeedsPush{ false };

	/** Grid coordinate of the cell currently at the center of the streaming
	 *  neighborhood. Written on the game thread before the async task starts,
	 *  then treated as read-only by both sides until the task clears
	 *  bUpdateInProgress. Initialized to INT32_MIN to force a full generate
	 *  on the first UpdateTier call. */
	FIntVector CenterCoord = FIntVector(INT32_MIN);

	/** Maps grid coord → slot entry (index + inserted octree nodes).
	 *  Written only by the async task while bUpdateInProgress is true. */
	TMap<FIntVector, FSlotEntry> ActiveSlots;

	/** Stack of available slot indices. Popped on cell enter, pushed on exit.
	 *  Written only by the async task while bUpdateInProgress is true. */
	TArray<int32> FreeSlots;

	/** Per-slot accepted particle count, written by GenerateCallback.
	 *  Used by CacheCellFromBuffers and InsertSlotIntoOctree to skip dead
	 *  padding without iterating the full SlotCapacity. */
	TArray<int32> SlotCounts;

	/**
	 * Persistent procgen cache keyed by grid coord. On first visit a cell is
	 * generated and its output stored here (cache-miss). On re-entry the
	 * stored data is blitted directly into the back-buffer, skipping all noise
	 * and rejection sampling (cache-hit). The slot is recycled on exit but the
	 * cache entry survives. Entries beyond NeighborhoodRadius + 4 cells
	 * (Chebyshev) are evicted by CullTierCache after each boundary cross.
	 */
	TMap<FIntVector, FCachedCellData> CellCache;
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
		return FIntVector(
			FMath::FloorToInt32(InPos.X / CellSize + 0.5),
			FMath::FloorToInt32(InPos.Y / CellSize + 0.5),
			FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
	}

	/** Converts a grid coordinate back to the cell center position. */
	static FVector GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth,
		double Extent, double GridExtentMultiplier)
	{
		const double CellSize = (Extent * GridExtentMultiplier) / (1 << InGridDepth);
		return FVector(
			static_cast<double>(InCoord.X) * CellSize,
			static_cast<double>(InCoord.Y) * CellSize,
			static_cast<double>(InCoord.Z) * CellSize);
	}

	/** Returns the half-extent of a grid cell at the given depth. */
	static double GetGridCellExtent(int32 InGridDepth,
		double Extent, double GridExtentMultiplier)
	{
		return (Extent * GridExtentMultiplier) / (1 << (InGridDepth + 1));
	}

	// ========================================================================
	//  Tier Initialization
	// ========================================================================

	static void InitializeTier(const FTierStreamingContext& Ctx,
		FParticleTierConfig& Config, FParticleTierState& State,
		TArray<UNiagaraComponent*>& OutComponents);

	// ========================================================================
	//  Tier Streaming Update
	// ========================================================================

	static void UpdateTier(const FTierStreamingContext& Ctx,
		FParticleTierConfig& Config, FParticleTierState& State);

	// ========================================================================
	//  Niagara Push
	// ========================================================================

	static void PushTierToNiagara(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State);

	// ========================================================================
	//  Octree Integration
	// ========================================================================

	static void InsertTierIntoOctree(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx);

	static void InsertSlotIntoOctree(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State,
		const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx);

	static void InsertParticleIntoOctree(const FTierStreamingContext& Ctx,
		FSlotEntry& Entry, const FVector& Position, float Extent,
		int32 SlotIndex, double TreeExtent, int32 TierIndex);

	// ========================================================================
	//  Cell Cache
	// ========================================================================

	static void CacheCellFromBuffers(const FParticleTierConfig& Config,
		FParticleTierState& State, const FIntVector& Coord,
		int32 SlotIndex, int32 BufferIdx);

	static void CullTierCache(const FParticleTierConfig& Config,
		FParticleTierState& State, const FIntVector& NewCenter);
};
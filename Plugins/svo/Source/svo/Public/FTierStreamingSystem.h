#pragma once

#include "CoreMinimal.h"
#include "FTierStreamingContext.h"
#include "UniverseActor.h"  // FParticleTierConfig, FParticleTierState, FSlotEntry
#include "FNiagaraParticleBuffer.h"
#include "FOctree.h"
#include "DataTypes.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

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

	/**
	 * Full tier initialization: allocates double-buffered particle data,
	 * optionally builds the initial neighborhood (exhaustive tiers only),
	 * runs parallel generation, inserts into the octree, mirrors front→back,
	 * then rendezvouses with the game thread to spawn Niagara components.
	 *
	 * Streaming tiers (NeighborhoodRadius > 0) defer initial population to
	 * the first UpdateTier call by setting CenterCoord = INT32_MIN.
	 *
	 * @param Ctx     Actor context (extent, octree, attach root, etc.).
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state to initialize.
	 * @param OutComponents  GC-safe array to append spawned components to.
	 */
	static void InitializeTier(const FTierStreamingContext& Ctx,
		FParticleTierConfig& Config, FParticleTierState& State,
		TArray<UNiagaraComponent*>& OutComponents);

	// ========================================================================
	//  Tier Streaming Update
	// ========================================================================

	/**
	 * Per-tick streaming update. Checks bNeedsPush and pushes if ready, then
	 * checks for a neighborhood center change. If the player has crossed a
	 * cell boundary, kicks an async task that diffs neighborhoods, frees
	 * exiting slots, generates entering slots, inserts into octree, culls
	 * the cell cache, swaps buffers, and signals bNeedsPush.
	 *
	 * @param Ctx     Actor context.
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state.
	 */
	static void UpdateTier(const FTierStreamingContext& Ctx,
		FParticleTierConfig& Config, FParticleTierState& State);

	// ========================================================================
	//  Niagara Push
	// ========================================================================

	/**
	 * Pushes the front buffer to Niagara components. Recomputes fixed bounds.
	 *
	 * @param Ctx     Actor context (for VirtualTraversal).
	 * @param Config  Immutable tier descriptor.
	 * @param State   Mutable tier runtime state.
	 */
	static void PushTierToNiagara(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State);

	// ========================================================================
	//  Octree Integration
	// ========================================================================

	/** Inserts all active slots of one tier's buffer into the octree. */
	static void InsertTierIntoOctree(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx);

	/** Inserts a single slot's live particles into the octree. */
	static void InsertSlotIntoOctree(const FTierStreamingContext& Ctx,
		const FParticleTierConfig& Config, FParticleTierState& State,
		int32 SlotIndex, int32 BufferIdx);

	/** Inserts one particle into the octree and appends the node to Entry. */
	static void InsertParticleIntoOctree(const FTierStreamingContext& Ctx,
		FSlotEntry& Entry, const FVector& Position, float Extent,
		int32 SlotIndex, double TreeExtent, int32 TierIndex);

	// ========================================================================
	//  Cell Cache
	// ========================================================================

	/** Snapshots live particles from a buffer slot into CellCache. */
	static void CacheCellFromBuffers(const FParticleTierConfig& Config,
		FParticleTierState& State, const FIntVector& Coord,
		int32 SlotIndex, int32 BufferIdx);

	/** Evicts cache entries beyond NeighborhoodRadius + 4 Chebyshev distance. */
	static void CullTierCache(const FParticleTierConfig& Config,
		FParticleTierState& State, const FIntVector& NewCenter);
};
#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"

class FOctree;
class USceneComponent;

/**
 * Read-only context supplied by the owning actor to the tier pipeline.
 * Passed by pointer into every FTierStreamingSystem call so the pipeline
 * can read spatial parameters without knowing the concrete actor type.
 *
 * The owning actor populates this once per tick (or once per call) and
 * hands it in. The pipeline never writes through this pointer.
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
};
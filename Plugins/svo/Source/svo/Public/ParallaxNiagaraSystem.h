// ParallaxNiagaraSystem.h
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "FOctree.h"
#include "ParallaxNiagaraSystem.generated.h"

class ASectorActor;

/// <summary>
/// Abstract base for parallax-driven Niagara visualization layers owned by a
/// procedural space actor (sector, galaxy, etc.).
///
/// Each subclass represents one Niagara system with self-contained CPU data,
/// a lifecycle (Configure → Initialize → ApplyParallax every frame → Shutdown),
/// and optional per-system streaming logic.
///
/// The owning actor holds a UPROPERTY() TArray<TObjectPtr<UParallaxNiagaraSystem>>
/// so both the wrappers and their UNiagaraComponents stay GC-rooted without
/// any TStrongObjectPtr gymnastics.
///
/// Subclasses implement Initialize to spawn their UNiagaraComponent, push their
/// User.* arrays, and activate. The default ApplyParallax broadcasts
/// User.ParallaxOffset and reseats the component at the player — streaming
/// subclasses override to layer in buffer-swap logic.
/// </summary>
UCLASS(Abstract)
class SVO_API UParallaxNiagaraSystem : public UObject
{
	GENERATED_BODY()

public:
	/// <summary>
	/// Spawns the UNiagaraComponent attached to OwnerRoot, pushes initial
	/// User.* arrays, and activates. Subclasses implement the body. May run
	/// async internally; returns a future that resolves after activation.
	///
	/// Must be called on the game thread.
	/// </summary>
	virtual TFuture<void> Initialize(
		ASectorActor* InOwner,
		USceneComponent* InOwnerRoot,
		UNiagaraSystem* InTemplate,
		const FVector& InInitialPlayerPos)
		PURE_VIRTUAL(UParallaxNiagaraSystem::Initialize, return TFuture<void>(););

	/// <summary>
	/// Called every frame after the owning actor has computed the shared
	/// ParallaxOffset and current player location. Default: push
	/// User.ParallaxOffset and reseat the component at CurrentPlayerPos.
	/// Override for streaming systems to also do buffer-swap work.
	/// </summary>
	virtual void ApplyParallax(const FVector& InParallaxOffset, const FVector& InCurrentPlayerPos);

	/// <summary>
	/// Called on sector shutdown. Default: deactivate + destroy the component.
	/// </summary>
	virtual void Shutdown();

	UNiagaraComponent* GetComponent() const { return NiagaraComponent; }

	bool IsActive() const { return NiagaraComponent != nullptr; }

protected:
	UPROPERTY()
	TObjectPtr<UNiagaraComponent> NiagaraComponent = nullptr;

	// Weak back-reference to the owning sector. Subclasses may need sector
	// state (e.g. Params.Extent, DensityVolume) during streaming updates.
	// Weak to avoid cycles and to gracefully handle sector teardown.
	UPROPERTY()
	TWeakObjectPtr<ASectorActor> OwnerSector;
};


/// <summary>
/// Cluster Niagara system — one sprite per cluster-scale point inserted into
/// the octree by UniverseDataGenerator. Always loaded for the sector's
/// lifetime; no streaming. CPU data (positions, rotations, extents, colors)
/// is captured once at ConfigureFromPointNodes time and pushed to Niagara
/// once during Initialize.
/// </summary>
UCLASS()
class SVO_API UClusterNiagaraSystem : public UParallaxNiagaraSystem
{
	GENERATED_BODY()

public:
	/// <summary>
	/// Build the cluster CPU data arrays from the point nodes produced by
	/// Octree::BulkInsertPositions. Called by the sector during InitializeData
	/// while PointNodes is still in scope; the cluster system takes an owned
	/// copy and the caller's PointNodes can go out of scope afterwards.
	///
	/// Can be called on any thread (does ParallelFor internally).
	/// </summary>
	void ConfigureFromPointNodes(const TArray<TSharedPtr<FOctreeNode>>& InPointNodes, double InSectorExtent);

	virtual TFuture<void> Initialize(
		ASectorActor* InOwner,
		USceneComponent* InOwnerRoot,
		UNiagaraSystem* InTemplate,
		const FVector& InInitialPlayerPos) override;

private:
	// Cluster CPU data, owned by this system. Populated in ConfigureFromPointNodes,
	// consumed during Initialize.
	TArray<FVector> Positions;
	TArray<FVector> Rotations;
	TArray<float> Extents;
	TArray<FLinearColor> Colors;

	double SectorExtent = 0.0;
};
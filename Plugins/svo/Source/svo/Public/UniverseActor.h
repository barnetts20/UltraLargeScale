#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "SectorActor.h"
#include "UniverseActor.generated.h"

/// <summary>
/// AUniverseActor — sector grid manager.
///
/// Owns the player-centered (2*NeighborhoodRadius+1)^3 grid of ASectorActors
/// (default 3x3x3 = 27 sectors). Each sector is positioned at its CellOrigin
/// in world space and samples the same shared noise field at a cell-coord
/// spatial offset, so adjacent sectors form one continuous density field.
///
/// Lifecycle (Phase 1 — no pooling):
///   1. BeginPlay — spawns the initial 3x3x3 around the player's cell
///   2. Tick — checks if player crossed into a new center cell; if yes,
///      spawn the cells entering the active set, destroy cells leaving
///   3. Each spawn ConfigureCell()s the new actor then drives Initialize()
///      asynchronously — pop-in is acceptable for now
///
/// Inherits from AProceduralSpaceActor purely for the tick + lifecycle
/// scaffolding. The base-class data generation hooks (InitializeData,
/// InitializeVolumetric, InitializeNiagara) are overridden as no-ops since
/// the universe has no data of its own — sectors do all the heavy work.
/// </summary>
UCLASS()
class SVO_API AUniverseActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	AUniverseActor();

#pragma region Editor Exposed Parameters
	// Sector-level params used to configure spawned sectors. Each sector
	// gets a copy of this; CellCoord differs per sector.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;

	// Sector actor class to spawn for each cell. Allows BP subclasses of
	// ASectorActor to be used.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	TSubclassOf<ASectorActor> SectorActorClass;

	// Half-width of the active grid in cells. 1 = 3x3x3 (27 sectors),
	// 2 = 5x5x5 (125 sectors). Keep small for Phase 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties", meta = (ClampMin = "0", ClampMax = "3"))
	int32 NeighborhoodRadius = 1;
#pragma endregion

protected:
#pragma region Params Accessors (implement pure virtuals from base)
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override { return SpeedScale; }
#pragma endregion

#pragma region Initialization (universe has no data of its own - sectors do all work)
	virtual void InitializeData() override {}
	virtual void InitializeVolumetric() override {}
	virtual void InitializeNiagara() override {}
	virtual void InitializeChildPool() override {}
#pragma endregion

#pragma region Sector Grid State
	// Active sectors keyed by cell coordinate.
	UPROPERTY()
	TMap<FIntVector, TObjectPtr<ASectorActor>> ActiveSectors;

	// Most recent center cell (the cell the player is in). Used to detect
	// crossings.
	FIntVector CurrentCenterCell = FIntVector(INT32_MIN);
#pragma endregion

#pragma region Overrides
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;
	// Universe doesn't render anything, so no parallax of its own —
	// individual sectors handle parallax.
	virtual void ApplyParallaxOffset() override {}
#pragma endregion

#pragma region Sector Grid Operations
	// Convert a world-space position to its cell coordinate.
	FIntVector WorldPositionToCell(const FVector& InWorldPos) const;

	// Spawn a sector for the given cell coordinate, configure it, and
	// kick off its async Initialize(). Adds it to ActiveSectors.
	void SpawnSectorForCell(const FIntVector& InCellCoord);

	// Destroy the sector at the given cell coordinate (if present) and
	// remove it from ActiveSectors. Phase 1: full destroy; Phase 2 will
	// recycle through a pool.
	void DespawnSectorAtCell(const FIntVector& InCellCoord);

	// Recompute which cells should be active based on a new center, then
	// spawn missing cells and despawn cells that left the active set.
	void RebuildActiveSet(const FIntVector& InNewCenter);
#pragma endregion
};
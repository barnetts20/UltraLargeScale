#include "UniverseActor.h"
#include "SectorActor.h"
#include <Kismet/GameplayStatics.h>

AUniverseActor::AUniverseActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SectorActorClass = ASectorActor::StaticClass();
	// Universe is a coordinator, not a renderer. No PointCloudNiagara, no
	// octree of its own — sectors own all visualization data.
}

void AUniverseActor::BeginPlay()
{
	Super::BeginPlay();

	// Determine which cell the player starts in, and spawn the initial
	// active grid around it.
	FVector PlayerPos = FVector::ZeroVector;
	if (auto* Controller = UGameplayStatics::GetPlayerController(GetWorld(), 0))
	{
		if (APawn* Pawn = Controller->GetPawn())
		{
			PlayerPos = Pawn->GetActorLocation();
		}
	}

	const FIntVector StartCell = WorldPositionToCell(PlayerPos);
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::BeginPlay — initial center cell (%d,%d,%d)"),
		StartCell.X, StartCell.Y, StartCell.Z);

	// Mark CurrentCenterCell as INT32_MIN so RebuildActiveSet treats this as
	// a fresh start (every cell counts as "entering").
	CurrentCenterCell = FIntVector(INT32_MIN);
	RebuildActiveSet(StartCell);

	// Call base Initialize so InitializationState transitions to Ready via
	// the async pipeline. Our InitializeData/Volumetric/Niagara/ChildPool
	// overrides are no-ops, so this resolves quickly. Without this, Tick's
	// state check would prevent player-crossing logic from running.
	Initialize();
}

void AUniverseActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Explicitly destroy spawned sectors before the engine's stale-reference
	// scan runs at PIE end. Same rationale as the EndPlay teardown in
	// ASectorActor for its Niagara components.
	for (auto& Pair : ActiveSectors)
	{
		if (ASectorActor* Sector = Pair.Value.Get())
		{
			Sector->Destroy();
		}
	}
	ActiveSectors.Empty();

	Super::EndPlay(EndPlayReason);
}

void AUniverseActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (InitializationState != ELifecycleState::Ready)
	{
		// AProceduralSpaceActor::Initialize() flips state to Ready when its
		// async pipeline finishes. Universe overrides all the init steps to
		// no-ops, so this should resolve quickly.
		return;
	}

	// Track which cell the player is in. If they cross a boundary, rebuild
	// the active set.
	FVector PlayerPos = FVector::ZeroVector;
	if (auto* Controller = UGameplayStatics::GetPlayerController(GetWorld(), 0))
	{
		if (APawn* Pawn = Controller->GetPawn())
		{
			PlayerPos = Pawn->GetActorLocation();
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}

	const FIntVector NewCenter = WorldPositionToCell(PlayerPos);
	if (NewCenter != CurrentCenterCell)
	{
		UE_LOG(LogTemp, Log, TEXT("AUniverseActor: player crossed into cell (%d,%d,%d)"),
			NewCenter.X, NewCenter.Y, NewCenter.Z);
		RebuildActiveSet(NewCenter);
	}
}

FIntVector AUniverseActor::WorldPositionToCell(const FVector& InWorldPos) const
{
	// Cell origin = CellCoord * (2 * SectorExtent). Cell occupies world
	// region [origin - Extent, origin + Extent]. So:
	//   world ∈ [(N-0.5) * 2 * Extent, (N+0.5) * 2 * Extent]  →  cell N
	// Equivalently: floor((world / (2*Extent)) + 0.5)
	const double CellSize = 2.0 * Params.Extent;
	return FIntVector(
		FMath::FloorToInt32(InWorldPos.X / CellSize + 0.5),
		FMath::FloorToInt32(InWorldPos.Y / CellSize + 0.5),
		FMath::FloorToInt32(InWorldPos.Z / CellSize + 0.5));
}

void AUniverseActor::SpawnSectorForCell(const FIntVector& InCellCoord)
{
	if (ActiveSectors.Contains(InCellCoord))
	{
		return; // Already spawned
	}

	UClass* ClassToSpawn = SectorActorClass ? SectorActorClass.Get() : ASectorActor::StaticClass();

	// Spawn deferred so we can configure before BeginPlay runs (which would
	// otherwise auto-Initialize the sector before ConfigureCell sets the
	// cell origin).
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	SpawnParams.Owner = this;

	ASectorActor* Sector = GetWorld()->SpawnActorDeferred<ASectorActor>(
		ClassToSpawn,
		FTransform::Identity,
		this,
		nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

	if (!Sector)
	{
		UE_LOG(LogTemp, Error, TEXT("AUniverseActor::SpawnSectorForCell — SpawnActorDeferred failed for cell (%d,%d,%d)"),
			InCellCoord.X, InCellCoord.Y, InCellCoord.Z);
		return;
	}

	// Configure before finishing spawn so BeginPlay sees the correct values.
	// bAutoInitializeOnBeginPlay stays true — the sector will start its
	// async Initialize() automatically once FinishSpawning fires BeginPlay.
	Sector->Params = Params;
	Sector->ConfigureCell(InCellCoord);

	UGameplayStatics::FinishSpawningActor(Sector, FTransform(Sector->CellOrigin));

	ActiveSectors.Add(InCellCoord, Sector);

	UE_LOG(LogTemp, Log, TEXT("AUniverseActor: spawned sector for cell (%d,%d,%d) at world %s"),
		InCellCoord.X, InCellCoord.Y, InCellCoord.Z, *Sector->CellOrigin.ToString());
}

void AUniverseActor::DespawnSectorAtCell(const FIntVector& InCellCoord)
{
	TObjectPtr<ASectorActor>* Found = ActiveSectors.Find(InCellCoord);
	if (!Found || !Found->Get())
	{
		return;
	}

	ASectorActor* Sector = Found->Get();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor: despawning sector at cell (%d,%d,%d)"),
		InCellCoord.X, InCellCoord.Y, InCellCoord.Z);

	Sector->Destroy();
	ActiveSectors.Remove(InCellCoord);
}

void AUniverseActor::RebuildActiveSet(const FIntVector& InNewCenter)
{
	// Collect the new desired set: all cells within NeighborhoodRadius of
	// InNewCenter on every axis.
	TSet<FIntVector> Desired;
	for (int32 dx = -NeighborhoodRadius; dx <= NeighborhoodRadius; ++dx)
	{
		for (int32 dy = -NeighborhoodRadius; dy <= NeighborhoodRadius; ++dy)
		{
			for (int32 dz = -NeighborhoodRadius; dz <= NeighborhoodRadius; ++dz)
			{
				Desired.Add(InNewCenter + FIntVector(dx, dy, dz));
			}
		}
	}

	// Despawn any active cell that's no longer desired.
	TArray<FIntVector> ToDespawn;
	ToDespawn.Reserve(ActiveSectors.Num());
	for (const auto& Pair : ActiveSectors)
	{
		if (!Desired.Contains(Pair.Key))
		{
			ToDespawn.Add(Pair.Key);
		}
	}
	for (const FIntVector& Cell : ToDespawn)
	{
		DespawnSectorAtCell(Cell);
	}

	// Spawn any desired cell that isn't yet active.
	for (const FIntVector& Cell : Desired)
	{
		if (!ActiveSectors.Contains(Cell))
		{
			SpawnSectorForCell(Cell);
		}
	}

	CurrentCenterCell = InNewCenter;
}
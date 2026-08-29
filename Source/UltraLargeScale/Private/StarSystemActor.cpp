// StarSystemActor.cpp
// Tier streaming system for star systems.
// First-pass: Large tier contains planets placed along +X axis at orbit distances.
// Mid/Small tiers are present but zero-particle placeholders.
// Neighbor scanning is active so planet spawn hooks fire correctly.

#include "StarSystemActor.h"
#include "UltraLargeScale.h"
#include "GalaxyActor.h"
#include "ParallaxProxyActor.h"
#include "StarActor.h"
#include "PooledActor.h"
#include "ParallaxStaticMeshActor.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "ActorPoolManager.h"
#include <DrawDebugHelpers.h>
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>

#pragma region Constructor/Destructor
AStarSystemActor::AStarSystemActor()
{
	// Driven exclusively by the parent galaxy via TickFromParent.
	// UE tick enabled only for level-placed standalone instances.
	PrimaryActorTick.bCanEverTick = true;
	SetActorTickEnabled(false);

	// Load Niagara assets. Paths mirror the galaxy naming convention.
	// Clone NG_GalaxyLarge/Mid/Small into the StarSystem folder and rename.
	// Niagara cloud systems load lazily in BuildTierConfigs() (runtime), NOT here:
	// loading assets during CDO construction runs before Niagara is ready and crashes.

	StarBodyClass = AStarActor::StaticClass();

	Octree = MakeShared<FOctree>(Params.Extent);
}

AStarSystemActor::~AStarSystemActor()
{
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region BeginPlay
void AStarSystemActor::BeginPlay()
{
	Super::BeginPlay();
	if (bAutoInitializeOnBeginPlay)
	{
		SetActorTickEnabled(true);
		InitializationState = ELifecycleState::Initializing;
		TWeakObjectPtr<AStarSystemActor> WeakThis(this);
		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
			{
				if (AStarSystemActor* Self = WeakThis.Get())
					Self->Initialize();
			});
	}
}
#pragma endregion

#pragma region EndPlay
void AStarSystemActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedPlanetNodes.Empty();
	bSpawnScanInProgress.store(false);

	// Destroy any live planet actors.
	for (auto& Pair : SpawnedPlanets)
	{
		if (AActor* Planet = Pair.Value.Get())
		{
			if (IsValid(Planet)) Planet->Destroy();
		}
	}
	SpawnedPlanets.Empty();

	// Destroy the central star (real teardown).
	if (AActor* Star = CentralStar.Get(); Star && IsValid(Star)) Star->Destroy();
	CentralStar = nullptr;
	bStarSpawned = false;

	// Drain any in-flight pushes before destroying the components they may touch.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		FTierStreamingSystem::BeginShutdownDrain(*Tier);

	// Mirror ResetForPool: also wait out an in-flight boundary-cross task -
	// it writes Buffers/SlotEntries/CellCache through teardown otherwise.
	// Safe on the GT (the transition task has no game-thread rendezvous).
	// The async INIT chain is NOT waited here - it rendezvouses with the GT
	// (would deadlock); it aborts on the Pooling state set above.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		while (Tier->bUpdateInProgress.load())
			FPlatformProcess::Sleep(0.0005f);
	}

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
	}
	TierNiagaraComponents.Empty();

	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Pool Lifecycle
void AStarSystemActor::ResetForPool()
{
	// Cascade-release live planets back to the central pool (parks each body dormant)
	// before this star system goes inert. Key snapshot first -- ReturnPlanetToPool
	// mutates the map. Mirrors the galaxy -> star-system cascade.
	{
		TArray<TSharedPtr<FOctreeNode>> LiveNodes;
		SpawnedPlanets.GetKeys(LiveNodes);
		for (const TSharedPtr<FOctreeNode>& Node : LiveNodes)
			ReturnPlanetToPool(Node);
	}

	DestroyCentralStar();

	// DRAIN BEFORE FREE - mirrors AGalaxyActor::ResetForPool. An async
	// boundary-cross task may still be writing this tier's buffers; bar new
	// pushes, wait out any in-flight push, then wait for the task to clear
	// bUpdateInProgress before freeing the arrays it writes.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		FTierStreamingSystem::BeginShutdownDrain(*Tier);
		while (Tier->bUpdateInProgress.load())
		{
			FPlatformProcess::Sleep(0.0005f);
		}
	}

	// The push worker is single-flight and actor-level. BeginShutdownDrain
	// makes its pushes bail (bShuttingDown) but does NOT stop the worker's
	// loop - it can still be alive here. Wait for it to exit (it clears
	// bPushWorkerLive on exit) BEFORE we clear bShuttingDown and free Buffers
	// below: a live worker would otherwise resume on freed memory, and a stale
	// 'live' flag would block the next occupant's push dispatch.
	while (bPushWorkerLive.load(std::memory_order_acquire))
	{
		FPlatformProcess::Sleep(0.0005f);
	}

	// Tear down tier Niagara components.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
		Tier->ResetState();   // plain state data + sentinels + atomics
	}
	TierNiagaraComponents.Empty();
	DiagTickCount = 0;

	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedPlanetNodes.Empty();
	bSpawnScanInProgress.store(false);

	PlanetPositions.Empty();
	PlanetExtents.Empty();
	PlanetColors.Empty();
	SystemGenerator.GeneratedData.Empty();
	SystemGenerator.GeneratedOrbits.Empty();

	Super::ResetForPool();
}

void AStarSystemActor::ResetForSpawn()
{
	Super::ResetForSpawn();   // resets the VT cluster (incl. LatestVT)
	DiagTickCount = 0;
}
#pragma endregion

#pragma region Params Accessors
double AStarSystemActor::GetEffectiveSpeedScale() const
{
	return Galaxy ? Galaxy->GetEffectiveSpeedScale() : 1.0;
}
#pragma endregion

#pragma region InitializeData
void AStarSystemActor::InitializeData()
{
	// Rebuild the octree against the current Params.Extent.
	Octree = MakeShared<FOctree>(Params.Extent);

	UE_LOG(LogTemp, Warning, TEXT("StarSystem::InitializeData - UnitScale=%.6e, Extent=%.0f, Seed=%d"),
		Params.UnitScale, Params.Extent, Params.Seed);
	// Analytically build planet positions along +X axis
	// Planets are evenly spaced between InnerOrbit and OuterOrbit along +X.
	// Sizes use absolute world-cm scales converted to octree-local extents
	// via UnitScale, so planets have consistent physical sizes regardless
	// of which galaxy spawned the parent star.
	const int32 NumPlanets = FMath::Max(1, Params.MaxPlanets);
	const double InnerRadius = Params.Extent * Params.InnerOrbitFraction;
	const double OuterRadius = Params.Extent * Params.OuterOrbitFraction;

	PlanetPositions.SetNumUninitialized(NumPlanets);
	PlanetExtents.SetNumUninitialized(NumPlanets);
	PlanetColors.SetNumUninitialized(NumPlanets);
	SystemGenerator.GeneratedOrbits.SetNum(NumPlanets);

	FRandomStream Stream(Params.Seed);

	// Decide frost line - inner planets are rocky, outer are gas giants.
	const int32 FrostLineIndex = FMath::Clamp(
		FMath::RoundToInt32(NumPlanets * Stream.FRandRange(0.35f, 0.55f)),
		1, NumPlanets - 1);

	for (int32 i = 0; i < NumPlanets; i++)
	{
		if (InitializationState == ELifecycleState::Pooling) return;

		// Evenly space planets from inner to outer orbit (this is the semi-major axis).
		const double t = (NumPlanets > 1)
			? static_cast<double>(i) / static_cast<double>(NumPlanets - 1)
			: 0.5;
		const double OrbitRadius = FMath::Lerp(InnerRadius, OuterRadius, t);

		// Static placement on a randomized ellipse, stored in GeneratedOrbits for the
		// debug draw (and later the debris tiers). GetOrbitPosition is a pure function
		// of the orbit, so the generator needs no other setup here.
		StarSystemDataGenerator::FOrbit& Orbit = SystemGenerator.GeneratedOrbits[i];
		Orbit.Center = FVector::ZeroVector;
		Orbit.SemiMajorAxis = OrbitRadius;
		Orbit.Eccentricity = FMath::Pow(Stream.FRand(), 2.0) * Params.MaxEccentricity;
		const double InclRad = FMath::DegreesToRadians(
			Stream.FRandRange(-Params.MaxInclinationDegrees, Params.MaxInclinationDegrees));
		FVector TiltAxis(Stream.FRandRange(-1.f, 1.f), Stream.FRandRange(-1.f, 1.f), 0.f);
		TiltAxis = TiltAxis.GetSafeNormal();
		if (TiltAxis.IsNearlyZero()) TiltAxis = FVector::RightVector;
		Orbit.Normal = FQuat(TiltAxis, InclRad).RotateVector(FVector::UpVector).GetSafeNormal();
		Orbit.Phase = Stream.FRandRange(0.0, 2.0 * PI);
		Orbit.Type = (i < FrostLineIndex)
			? StarSystemDataGenerator::EObjectType::TerrestrialPlanet
			: StarSystemDataGenerator::EObjectType::GasPlanet;

		PlanetPositions[i] = SystemGenerator.GetOrbitPosition(Orbit);

		// Absolute planet scale (world cm) -> octree-local extent
		// Inner planets (below frost line) draw from terrestrial range.
		// Outer planets draw from gas giant range.
		// Both are converted to octree-local units by dividing by UnitScale.
		double PlanetWorldScale;
		if (i < FrostLineIndex)
		{
			// Terrestrial: inner planets tend larger (Earth/Venus zone),
			// outer rocky worlds tend smaller (Mars-class).
			const double InnerT = (FrostLineIndex > 1)
				? static_cast<double>(i) / static_cast<double>(FrostLineIndex - 1)
				: 0.5;
			const double MaxForSlot = FMath::Lerp(Params.TerrestrialMaxScale, Params.TerrestrialMinScale, InnerT);
			PlanetWorldScale = Stream.FRandRange(Params.TerrestrialMinScale, MaxForSlot);
		}
		else
		{
			// Gas giant: closer to frost line = larger (Jupiter zone),
			// far outer = smaller (Neptune/ice giants).
			const double OuterT = (NumPlanets - FrostLineIndex > 1)
				? static_cast<double>(i - FrostLineIndex) / static_cast<double>(NumPlanets - FrostLineIndex - 1)
				: 0.5;
			const double MaxForSlot = FMath::Lerp(Params.GasGiantMaxScale, Params.GasGiantMinScale, OuterT);
			PlanetWorldScale = Stream.FRandRange(Params.GasGiantMinScale, MaxForSlot);
		}

		// Convert world-cm to octree-local extent.
		const double PlanetLocalSize = PlanetWorldScale / Params.UnitScale;
		PlanetExtents[i] = static_cast<float>(FMath::Max(PlanetLocalSize, 1.0));

		// Vary color by orbit: inner = warm rocky, outer = cool icy blue.
		PlanetColors[i] = FLinearColor(
			FMath::Lerp(0.9f, 0.1f, static_cast<float>(t)),  // R
			FMath::Lerp(0.4f, 0.6f, static_cast<float>(t)),  // G
			FMath::Lerp(0.1f, 1.0f, static_cast<float>(t)),  // B
			1.0f
		);
	}

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::InitializeData - %d planets on orbits [%.0f ... %.0f octree units], "
			"frost line at index %d, UnitScale=%.4e, terrestrial=[%.2e..%.2e] gas=[%.2e..%.2e] cm"),
		NumPlanets, InnerRadius, OuterRadius, FrostLineIndex, Params.UnitScale,
		Params.TerrestrialMinScale, Params.TerrestrialMaxScale,
		Params.GasGiantMinScale, Params.GasGiantMaxScale);
}
#pragma endregion

#pragma region InitializeVolumetric
void AStarSystemActor::InitializeVolumetric()
{
	// No volumetric component for first-pass star systems.
	// The star sprite is managed by the parent galaxy and intentionally NOT
	// faded out when the star system spawns - it acts as the star itself.
}
#pragma endregion

#pragma region InitializeNiagara
void AStarSystemActor::InitializeNiagara()
{
	if (InitializationState == ELifecycleState::Pooling) return;

	BuildTierConfigs();

	// Insert planet data into the octree so the spawn scan has nodes to
	// discover. This is THE insert for planets: it carries Composition
	// (planet color) and the semantic EObjectType TypeId, which the tier
	// pipeline's InsertParticleIntoOctree does not write. The Large tier's
	// pipeline insert is disabled (OctreeInsertBufferIndex = -1 in
	// BuildTierConfigs) - the radius-0 exhaustive branch would otherwise
	// insert every planet a second time.
	for (int32 i = 0; i < PlanetPositions.Num(); i++)
	{
		if (InitializationState == ELifecycleState::Pooling) return;

		// Choose insertion depth so the node extent roughly matches the planet.
		const double PlanetExtentLocal = static_cast<double>(PlanetExtents[i]);
		int32 BestDepth = 1;
		for (int32 d = 1; d <= static_cast<int32>(FMath::Log2(static_cast<double>(Params.Extent))); d++)
		{
			const int64 ExtentAtDepth = static_cast<int64>(Params.Extent) >> d;
			if (PlanetExtentLocal > static_cast<double>(ExtentAtDepth)) break;
			BestDepth = d;
		}

		FVoxelData VD;
		VD.ScaleFactor = 0.5f;
		VD.Density = 1.0f;
		VD.Composition = FVector(PlanetColors[i].R, PlanetColors[i].G, PlanetColors[i].B);
		VD.Seed = FVoxelData::ComposeSeed(Params.Seed, FIntVector::ZeroValue, i);
		VD.TypeId = static_cast<int32>(StarSystemDataGenerator::EObjectType::TerrestrialPlanet);
		VD.ParticleIndex = i;
		// Exact particle capture: the spawn scan tests against this, giving
		// planets the same 1:1 angular-size behavior as the sprite material.
		VD.ParticlePosition = PlanetPositions[i];
		VD.ParticleExtent = PlanetExtents[i];

		// FOctree exposes InsertPosition (not Insert).
		Octree->InsertPosition(PlanetPositions[i], BestDepth, VD);
	}

	// Initialize all three tiers. Large = planet sprites (full slot, always loaded).
	// Mid/Small = zero-particle placeholders; their Niagara components are
	// created but never filled.
	const FTierStreamingContext Ctx = BuildStreamingContext();

	FTierStreamingSystem::InitializeTier(Ctx, LargeTierConfig, LargeTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;

	FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;

	FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;

	// NOTE: do NOT set Ready here - AProceduralSpaceActor::Initialize() sets
	// it after this returns; setting it early let the actor tick mid-init.

	// No timer start needed - Universe::DetermineAndDispatchScan drives scans.

	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::InitializeNiagara - Ready"));
}
#pragma endregion

#pragma region BuildTierConfigs
void AStarSystemActor::LoadRuntimeAssets()
{
	// Game thread (Initialize prologue, before async dispatch): LoadObject is not
	// thread-safe, so the Niagara systems BuildTierConfigs reads must load here.
	// Mid/Small are placeholders sharing the Large asset so InitializeTier never sees
	// a null NiagaraSystem; swap for NG_StarSystemMid / NG_StarSystemSmall later.
	if (!StarSystemLargeCloud) StarSystemLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/StarSystem/NG_StarSystemLarge.NG_StarSystemLarge"));
	if (!StarSystemMidCloud)   StarSystemMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/StarSystem/NG_StarSystemMid.NG_StarSystemMid"));
	if (!StarSystemSmallCloud) StarSystemSmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/StarSystem/NG_StarSystemSmall.NG_StarSystemSmall"));
}

void AStarSystemActor::BuildTierConfigs()
{
	// Large tier: planets, single cell exhaustive, always loaded
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = Params.LargeTier.NeighborhoodRadius;
	LargeTierConfig.SlotCapacity = FMath::Max(Params.LargeTier.SlotCapacity, Params.MaxPlanets);
	LargeTierConfig.NiagaraAssets = { StarSystemLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	// Planets are inserted into the octree DIRECTLY in InitializeNiagara -
	// that insert carries Composition (planet color) and the semantic
	// EObjectType TypeId, neither of which the tier-pipeline insert writes.
	// -1 disables InsertTierIntoOctree here: InitializeTier's radius-0
	// exhaustive branch DOES run the pipeline insert when this is >= 0,
	// which double-inserted every planet. That was only benign because both
	// depth heuristics happened to agree (the collision path kept the direct
	// insert's data); if they ever diverged, each planet would get two nodes
	// and spawn two mesh actors.
	LargeTierConfig.OctreeInsertBufferIndex = -1;

	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex,
		TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			// Analytically fill the slot with our pre-computed planet positions.
			// Density function = 0 - we bypass any noise and write directly.
			FNiagaraParticleBuffer& Buf = *Buffers[0];
			const int32 NumPlanets = PlanetPositions.Num();
			int32 Written = 0;

			for (int32 i = 0; i < NumPlanets && Written < LargeTierConfig.SlotCapacity; i++)
			{
				const int32 Base = SlotIndex * LargeTierConfig.SlotCapacity + Written;
				if (Base >= Buf.Positions.Num()) break;

				Buf.Positions[Base] = PlanetPositions[i];
				Buf.Extents[Base] = PlanetExtents[i];
				Buf.Colors[Base] = PlanetColors[i];
				Written++;
			}

			// Record actual planet count so InsertSlotIntoOctree skips dead padding.
			if (SlotIndex < LargeTierState.SlotCounts.Num())
				LargeTierState.SlotCounts[SlotIndex] = Written;
		};

	// Large tier bounds: cover all planetary orbits.
	// OuterOrbit = Extent * OuterOrbitFraction, padded by BoundsScaleMultiplier.
	LargeTierConfig.ComputeBounds = [this]() {
		const double HalfBound = Params.Extent * Params.OuterOrbitFraction;
		return FBox(FVector(-HalfBound), FVector(HalfBound));
		};

	// No ShouldSkipCell or OnBoundaryCross needed for the exhaustive large tier.

	// Mid tier: asteroid belt -- density-driven particles, no bodies.
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.TierIndex = 1;
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.SlotCapacity;
	MidTierConfig.NiagaraAssets = { StarSystemMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = -1; // Skip octree insertion

	// Volume predicate: cull + streaming gate. Zero-capacity today, so the
	// payoff is purely the gate - no transition churn while crossing this
	// tier's grid at speed outside the system volume. Bound: the outer-orbit
	// region, matching the Large tier's planetary coverage; widen when these
	// tiers gain real content if it should extend past the orbits.
	MidTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) {
		return !CellOverlapsVolume(Coord, MidTierConfig.GridDepth);
		};

	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex,
		TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			FNiagaraParticleBuffer& Buf = *Buffers[0];
			const int32 Cap = MidTierConfig.SlotCapacity;
			const FVector CellCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
			const double  CellHalf = GetGridCellExtent(MidTierConfig.GridDepth);

			// Stable per-cell stream so belt asteroids don't flicker as cells re-stream.
			FRandomStream Stream(int32(HashCombine(GetTypeHash(Coord), uint32(Params.Seed))));

			int32 Written = 0;
			for (int32 i = 0; i < Cap; i++)
			{
				const int32 Base = SlotIndex * Cap + Written;
				if (Base >= Buf.Positions.Num()) break;

				const FVector P = CellCenter + FVector(
					Stream.FRandRange(-CellHalf, CellHalf),
					Stream.FRandRange(-CellHalf, CellHalf),
					Stream.FRandRange(-CellHalf, CellHalf));

				if (Stream.FRand() >= MidBeltDensity(P)) continue;   // density rejection

				const float B = Stream.FRandRange(0.7f, 1.1f);
				Buf.Positions[Base] = P;
				Buf.Extents[Base] = Stream.FRandRange(0.15f, 0.6f);   // octree-local; sprite min-angular clamp covers far viz
				Buf.Colors[Base] = FLinearColor(0.42f * B, 0.38f * B, 0.34f * B, 1.f);
				Written++;
			}
			if (SlotIndex < MidTierState.SlotCounts.Num())
				MidTierState.SlotCounts[SlotIndex] = Written;
		};

	MidTierConfig.ComputeBounds = [this]() {
		const double HalfCell = GetGridCellExtent(MidTierConfig.GridDepth)
			* (2 * MidTierConfig.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfCell), FVector(HalfCell));
		};

	// Small tier: low-concentration dust -- density-driven particles, no bodies.
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.SlotCapacity;
	SmallTierConfig.NiagaraAssets = { StarSystemSmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = -1; // Skip octree insertion

	// Volume predicate: cull + streaming gate (see Mid tier note).
	SmallTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) {
		return !CellOverlapsVolume(Coord, SmallTierConfig.GridDepth);
		};

	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex,
		TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			FNiagaraParticleBuffer& Buf = *Buffers[0];
			const int32 Cap = SmallTierConfig.SlotCapacity;
			const FVector CellCenter = GridCoordToCenter(Coord, SmallTierConfig.GridDepth);
			const double  CellHalf = GetGridCellExtent(SmallTierConfig.GridDepth);

			// Stable per-cell stream (salted so dust doesn't correlate with the belt).
			FRandomStream Stream(int32(HashCombine(GetTypeHash(Coord), uint32(Params.Seed)) ^ 0x5bd1e995));

			int32 Written = 0;
			for (int32 i = 0; i < Cap; i++)
			{
				const int32 Base = SlotIndex * Cap + Written;
				if (Base >= Buf.Positions.Num()) break;

				const FVector P = CellCenter + FVector(
					Stream.FRandRange(-CellHalf, CellHalf),
					Stream.FRandRange(-CellHalf, CellHalf),
					Stream.FRandRange(-CellHalf, CellHalf));

				if (Stream.FRand() >= SmallDustDensity(P)) continue;   // density rejection

				const float B = Stream.FRandRange(0.6f, 1.0f);
				Buf.Positions[Base] = P;
				Buf.Extents[Base] = Stream.FRandRange(0.03f, 0.12f);   // octree-local; fine dust/rocks
				Buf.Colors[Base] = FLinearColor(0.30f * B, 0.29f * B, 0.27f * B, 1.f);
				Written++;
			}
			if (SlotIndex < SmallTierState.SlotCounts.Num())
				SmallTierState.SlotCounts[SlotIndex] = Written;
		};

	SmallTierConfig.ComputeBounds = [this]() {
		const double HalfCell = GetGridCellExtent(SmallTierConfig.GridDepth)
			* (2 * SmallTierConfig.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfCell), FVector(HalfCell));
		};
}
#pragma endregion

#pragma region BuildStreamingContext
FTierStreamingContext AStarSystemActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = Params.Extent;
	Ctx.UnitScale = Params.UnitScale;
	Ctx.bVirtualSpace = IsVirtualSpace();
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = false;
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = GetName();
	Ctx.ParentSeed = Params.Seed;
	Ctx.GetLatestVT = [this] { return ReadLatestVT(); };
	Ctx.GetLiveState = [this] { return InitializationState; };
	return Ctx;
}
#pragma endregion

#pragma region Grid Coord Helpers
double AStarSystemActor::MidBeltDensity(const FVector& P) const
{
	const double R = FMath::Sqrt(P.X * P.X + P.Y * P.Y);
	const double BeltR = Params.Extent * Params.BeltRadiusFraction;
	const double BeltW = FMath::Max(1.0, Params.Extent * Params.BeltWidthFraction);
	const double DiscZ = FMath::Max(1.0, Params.Extent * Params.BeltThicknessFraction);
	const double Radial = FMath::Exp(-FMath::Square((R - BeltR) / BeltW));
	const double Vert = FMath::Exp(-FMath::Square(P.Z / DiscZ));
	return Radial * Vert;
}

double AStarSystemActor::SmallDustDensity(const FVector& P) const
{
	const double R = FMath::Sqrt(P.X * P.X + P.Y * P.Y);
	if (R > Params.Extent * Params.OuterOrbitFraction) return 0.0;
	const double DiscZ = FMath::Max(1.0, Params.Extent * Params.DustThicknessFraction);
	const double Vert = FMath::Exp(-FMath::Square(P.Z / DiscZ));
	return Params.DustDensity * Vert;
}

bool AStarSystemActor::CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const
{
	const FVector Center = GridCoordToCenter(Coord, GridDepth);
	const double HalfCell = GetGridCellExtent(GridDepth);
	const double Ext = Params.Extent * Params.OuterOrbitFraction;
	return (Center.X + HalfCell > -Ext && Center.X - HalfCell < Ext) &&
		(Center.Y + HalfCell > -Ext && Center.Y - HalfCell < Ext) &&
		(Center.Z + HalfCell > -Ext && Center.Z - HalfCell < Ext);
}
#pragma endregion

#pragma region Tick
void AStarSystemActor::Tick(float DeltaTime)
{
	// Only runs for level-placed standalone star systems (bAutoInitializeOnBeginPlay).
	AActor::Tick(DeltaTime);
	if (InitializationState != ELifecycleState::Ready) return;

	FVector CurrentPlayerPos;
	if (!GetPlayerLocation(GetWorld(), CurrentPlayerPos)) return;

	TickFromParent(DeltaTime, CurrentPlayerPos);
}

void AStarSystemActor::ApplyParallaxOffset(const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("StarSystem::ApplyParallaxOffset");
	// VirtualTraversal accumulation (mirrors GalaxyActor)
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double ActiveSpeedScale = GetEffectiveSpeedScale();
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	// Peg actor to the player so UE's scene graph stays numerically clean.
	SetActorLocation(InPlayerPos);

	// Publish EVERY frame so a late-completing full push still uses current VT.
	PublishLatestVT(VirtualTraversal);

	// Niagara position push (gated by push threshold)
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		LastPushedVirtualTraversal = VirtualTraversal;
		FTierStreamingSystem::PushTierVT({ &LargeTierState, &MidTierState, &SmallTierState }, [this] { return ReadLatestVT(); });
	}
}

// Coalesced, single-flight per-frame push (see AUniverseActor::SchedulePush).
void AStarSystemActor::SchedulePush()
{
	bPushDirty.store(true, std::memory_order_release);
	if (bPushWorkerLive.exchange(true)) return;
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			for (;;)
			{
				bPushDirty.store(false, std::memory_order_relaxed);
				FTierStreamingSystem::PushTierVT(
					{ &LargeTierState, &MidTierState, &SmallTierState },
					[this] { return ReadLatestVT(); });
				if (!bPushDirty.load(std::memory_order_acquire))
				{
					bPushWorkerLive.store(false, std::memory_order_release);
					if (!bPushDirty.load(std::memory_order_acquire)) return;
					if (bPushWorkerLive.exchange(true)) return;
				}
			}
		});
}

void AStarSystemActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("StarSystem::TickFromParent");
	if (InitializationState != ELifecycleState::Ready) return;

	ApplyParallaxOffset(InPlayerPos);

	// Tier streaming
	// Large tier has NeighborhoodRadius=0 so UpdateTier is effectively a no-op
	// after the initial load - it just checks the center coord each tick.
	// Mid/Small stream but produce zero particles.
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, LargeTierConfig, LargeTierState);
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	// Drive live planets
	// Each planet's world position is recomputed from the current VT every
	// frame so it stays locked to its parallax-correct location.
	const double ActiveSpeedScale = GetEffectiveSpeedScale();

	// Central star: spawn once (GT) after the system is placed + ready, then drive its
	// parallax wrapper FIRST, so we can hand its resolved world position to the planets.
	if (!bStarSpawned && InitializationState == ELifecycleState::Ready && !bPendingPlacement)
		SpawnCentralStar();

	FVector StarWorldPos = FVector::ZeroVector;
	bool bHaveStarPos = false;
	if (AParallaxProxyActor* Star = Cast<AParallaxProxyActor>(CentralStar.Get()))
	{
		Star->TickParallax(DeltaTime, InPlayerPos, ActiveSpeedScale);
		if (AActor* StarBody = Star->Wrapped)
		{
			StarWorldPos = StarBody->GetActorLocation();
			bHaveStarPos = true;
		}
	}

	// Drive each planet's parallax, then hand it the star's world position so its
	// atmosphere can orient its light/raymarch toward the star.
	for (auto& Pair : SpawnedPlanets)
		if (auto* Proxy = Cast<AParallaxProxyActor>(Pair.Value.Get()))
		{
			Proxy->TickParallax(DeltaTime, InPlayerPos, ActiveSpeedScale);
			if (bHaveStarPos)
				if (IStarLit* Lit = Cast<IStarLit>(Proxy->Wrapped))
					Lit->SetStarWorldPosition(StarWorldPos);
		}

	// Planet spawn scan
	// VirtualTraversal is resolved for this frame; process any pending
	// octree query results to fire SpawnPlanetFromPool / ReturnPlanetToPool.
	ProcessPendingScanResults();

	if (IsDebug) DrawDebugBounds();
	if (bShowOrbits) DrawDebugOrbits();

	if (IsDebug && ++DiagTickCount % 60 == 0)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("StarSystem [%s] VT=(%.1f,%.1f,%.1f) Planets=%d SpawnedPlanets=%d"),
			*GetName(),
			VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z,
			PlanetPositions.Num(), SpawnedPlanets.Num());
	}
}
#pragma endregion

#pragma region Central Star
void AStarSystemActor::SpawnCentralStar()
{
	SVO_GT_SCOPE("StarSystem::SpawnCentralStar");
	UWorld* World = GetWorld();
	if (!World || !StarBodyClass) return;

	// The star rides the SAME parallax wrapper the planets use, so its motion is
	// identical - the only differences are that this proxy is spawned directly (not
	// pooled) and never despawns. Spawn location + VT seed are the planet particle
	// projection applied to the system center (octree-local origin).
	const FVector SpawnLoc = ComputeChildSpawnLocation(FVector::ZeroVector, 1.0);
	const FVector InitialVT = CurrentFrameOfReferenceLocation - SpawnLoc;
	const double WorldRadius = double(Params.Extent) * Params.StarRadiusFraction * Params.UnitScale;

	FActorSpawnParameters SP;
	SP.Owner = this;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AParallaxProxyActor* Proxy = World->SpawnActor<AParallaxProxyActor>(
		AParallaxProxyActor::StaticClass(), SpawnLoc, FRotator::ZeroRotator, SP);
	if (!Proxy) return;

	// Wraps StarBodyClass at unit scale 1 and seeds VT. The proxy scales the body via
	// SetActorScale3D(WorldRadius) - correct for a unit-radius star mesh.
	Proxy->ReInit(StarBodyClass, WorldRadius, SpawnLoc, InitialVT);
	CentralStar = Proxy;
	bStarSpawned = true;

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnCentralStar - worldRadius=%.1f at (%.1f,%.1f,%.1f) seed=%d"),
		WorldRadius, SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z, Params.Seed);
}

void AStarSystemActor::DestroyCentralStar()
{
	if (AActor* Star = CentralStar.Get(); Star && IsValid(Star))
		Star->Destroy();
	CentralStar = nullptr;
	bStarSpawned = false;
}
#pragma endregion

#pragma region Debug
void AStarSystemActor::DrawDebugOrbits()
{
	UWorld* World = GetWorld();
	if (!World || InitializationState != ELifecycleState::Ready) return;

	// Orbits are in octree-local space. The virtual layer renders a local point at
	// (ActorLocation + local - VirtualTraversal) - the same mapping the planet sprites
	// use - so sampling the ellipse through it traces where the planets appear.
	const FVector Base = GetActorLocation() - VirtualTraversal;
	constexpr int32 Segments = 64;

	for (const StarSystemDataGenerator::FOrbit& Orbit : SystemGenerator.GeneratedOrbits)
	{
		const float Thickness = static_cast<float>(Orbit.SemiMajorAxis * 0.004);
		FColor Color;
		switch (Orbit.Type)
		{
		case StarSystemDataGenerator::EObjectType::GasPlanet:         Color = FColor::Orange; break;
		case StarSystemDataGenerator::EObjectType::TerrestrialPlanet: Color = FColor::Green;  break;
		case StarSystemDataGenerator::EObjectType::Debris:            Color = FColor(120, 120, 120); break;
		default:                                                      Color = FColor::Silver; break;
		}

		StarSystemDataGenerator::FOrbit Sample = Orbit;
		FVector Prev = FVector::ZeroVector;
		for (int32 seg = 0; seg <= Segments; seg++)
		{
			Sample.Phase = (2.0 * PI) * static_cast<double>(seg) / static_cast<double>(Segments);
			const FVector WorldPt = Base + SystemGenerator.GetOrbitPosition(Sample);
			if (seg > 0)
				DrawDebugLine(World, Prev, WorldPt, Color, false, -1.0f, 0, Thickness);
			Prev = WorldPt;
		}
	}
}
#pragma endregion

#pragma region Spawn Range Scanning
void AStarSystemActor::RequestScan()
{
	SVO_GT_SCOPE("StarSystem::RequestScan");
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;

	// GT snapshot of the tree ref. The Octree member is now only reassigned
	// on the GT (FinishStarSystemPoolReturn), but the worker must still read
	// a snapshot - the member can be swapped between dispatch and execution.
	TSharedPtr<FOctree> TreeSnapshot = Octree;
	TWeakObjectPtr<AStarSystemActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos, TreeSnapshot]()
		{
			AStarSystemActor* Self = WeakThis.Get();
			if (!Self || !TreeSnapshot.IsValid()) return;

			const TArray<TSharedPtr<FOctreeNode>> NearbyArray =
				TreeSnapshot->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray, TreeSnapshot]()
				{
					AStarSystemActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->bSpawnScanInProgress.store(false);
					// Same guard as AUniverseActor::RequestScan: results from
					// a scan whose tree was swapped mid-flight (pool return
					// installs a fresh tree) are the previous identity's
					// nodes - processing them would spawn ghost planets with
					// old seeds. Drop them; the next interval rescans.
					if (InnerSelf->Octree != TreeSnapshot) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
				});
		});
}

bool AStarSystemActor::IsPlayerInsideBounds() const
{
	if (!Octree.IsValid()) return false;
	const double E = Octree->Extent;
	return FMath::Abs(VirtualTraversal.X) <= E
		&& FMath::Abs(VirtualTraversal.Y) <= E
		&& FMath::Abs(VirtualTraversal.Z) <= E;
}

void AStarSystemActor::ProcessPendingScanResults()
{
	SVO_GT_SCOPE("StarSystem::ProcessPendingScanResults");
	if (!bHasPendingScanResults) return;
	bHasPendingScanResults = false;

	TSet<TSharedPtr<FOctreeNode>> NearbySet(PendingScanResults);
	PendingScanResults.Empty();

	for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
	{
		if (!TrackedPlanetNodes.Contains(Node))
		{
			LogSpawnNodeEnter(Node);
			SpawnPlanetFromPool(Node);
		}
		if (bDebugDrawSpawnNodes) DebugDrawSpawnNode(Node);
	}

	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedPlanetNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
		ReturnPlanetToPool(Node);
	}

	TrackedPlanetNodes = MoveTemp(NearbySet);
}

void AStarSystemActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnScan ENTER - center=(%.1f,%.1f,%.1f) extent=%.2f seed=%d planetIdx=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Data.Seed, InNode->Data.ParticleIndex);
}

void AStarSystemActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnScan EXIT  - center=(%.1f,%.1f,%.1f) extent=%.2f seed=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Data.Seed);
}

void AStarSystemActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	// Rendered world pos = PlayerPos + NodeCenter - VirtualTraversal
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	DrawDebugBox(World, NodeCenterWorld, FVector(InNode->Extent * Params.UnitScale),
		FColor::Green, false, SpawnScanInterval, 0, 5.0f);
}
#pragma endregion

#pragma region Planet Spawn Hooks
void AStarSystemActor::SpawnPlanetFromPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("StarSystem::SpawnPlanetFromPool");
	if (!InNode.IsValid() || SpawnedPlanets.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	UActorPoolManager* PM = GetPoolManager();
	if (!PM) return;

	// Resolve the real particle position/extent (octree node center is quantized).
	// ParticleIndex stores the planet index directly (set in InitializeNiagara).
	FVector  ParticlePos = InNode->Center;
	float    ParticleExtent = static_cast<float>(InNode->Extent);
	const int32 PlanetIndex = InNode->Data.ParticleIndex;
	if (PlanetIndex >= 0 && PlanetIndex < PlanetPositions.Num())
	{
		ParticlePos = PlanetPositions[PlanetIndex];
		ParticleExtent = PlanetExtents[PlanetIndex];
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("AStarSystemActor::SpawnPlanetFromPool - invalid ParticleIndex=%d "
				"(seed=%d, %d planets), falling back to octree node center"),
			PlanetIndex, Params.Seed, PlanetPositions.Num());
	}

	// The body lives in real UE space (UnitScale = 1); expand the camera-to-node vector
	// by UnitScale to preserve angular position.
	const FVector SpawnLoc = ComputeChildSpawnLocation(ParticlePos, 1.0);
	const FVector InitialVT = CurrentFrameOfReferenceLocation - SpawnLoc;

	// Body class: default sprite placeholder, or the star system's injected voxel body.
	UClass* BodyClass = Params.WrappedBodyClass.IsNull()
		? AParallaxStaticMeshActor::StaticClass()
		: Params.WrappedBodyClass.LoadSynchronous();
	if (!BodyClass) BodyClass = AParallaxStaticMeshActor::StaticClass();
	const double WorldRadius = double(ParticleExtent) * Params.UnitScale;

	// Acquire a pooled proxy and (re)init it. ReInit wakes a persistent wrapped body
	// (same class) via IPooledActor, or swaps to a new body class.
	AParallaxProxyActor* Proxy = PM->Acquire<AParallaxProxyActor>();
	if (!Proxy) return;   // pool grow failed; manager already warned
	Proxy->ReInit(BodyClass, WorldRadius, SpawnLoc, InitialVT);
	SpawnedPlanets.Add(InNode, TWeakObjectPtr<AActor>(Proxy));

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnPlanetFromPool - inert=%d planet[%d] at (%.1f,%.1f,%.1f) worldRadius=%.1f seed=%d"),
		PM->NumInert(AParallaxProxyActor::StaticClass()),
		PlanetIndex, SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z, WorldRadius, Params.Seed);
}

void AStarSystemActor::FinalizePlanetPlacement(AActor* Planet, TSharedPtr<FOctreeNode> InNode)
{
	// Static mesh planets have no async init - placement is immediate in
	// SpawnPlanetFromPool. This function is a hook for when real planet actors
	// (with async initialization) replace the placeholder.
	// For now it's unused but declared so the header compiles cleanly.
}

void AStarSystemActor::ReturnPlanetToPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("StarSystem::ReturnPlanetToPool");
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AActor> WeakPlanet;
	if (!SpawnedPlanets.RemoveAndCopyValue(InNode, WeakPlanet)) return;
	AActor* Proxy = WeakPlanet.Get();
	if (!Proxy || !IsValid(Proxy)) return;

	// Release through the manager: OnReturnToPool parks the wrapped body dormant
	// (no tick/LOD/raymarch/ocean while pooled), then the proxy goes inert + re-pools.
	// We're on the game thread here (ProcessPendingScanResults / ResetForPool).
	if (UActorPoolManager* PM = GetPoolManager())
		PM->Release(Proxy);
	else
		Proxy->Destroy();
}
#pragma endregion

#pragma region Pool Accessors + Re-Init
UActorPoolManager* AStarSystemActor::GetPoolManager() const { return Galaxy ? Galaxy->GetPoolManager() : nullptr; }
AUniverseActor* AStarSystemActor::GetUniverse()    const { return Galaxy ? Galaxy->GetUniverse() : nullptr; }

void AStarSystemActor::ReInit(const FStarSystemParams& InParams, const FTransform& InXform)
{
	bAutoInitializeOnBeginPlay = false;
	Params = InParams;
	PendingNodeCenter = InXform.GetLocation();
	bPendingPlacement = true;
	SetActorHiddenInGame(true);
	Initialize();
}
#pragma endregion
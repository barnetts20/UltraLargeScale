// StarSystemActor.cpp
// Tier streaming system for star systems.
// First-pass: Large tier contains planets placed along +X axis at orbit distances.
// Mid/Small tiers are present but zero-particle placeholders.
// Neighbor scanning is active so planet spawn hooks fire correctly.

#pragma region Includes
#include "StarSystemActor.h"
#include "GalaxyActor.h"
#include "ParallaxStaticMeshActor.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <DrawDebugHelpers.h>
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>

#pragma endregion

// ---------------------------------------------------------------------------
//  Constructor / Destructor
// ---------------------------------------------------------------------------
#pragma region Constructor/Destructor
AStarSystemActor::AStarSystemActor()
{
	// Driven exclusively by the parent galaxy via TickFromParent.
	// UE tick enabled only for level-placed standalone instances.
	PrimaryActorTick.bCanEverTick = true;
	SetActorTickEnabled(false);

	// Load Niagara assets. Paths mirror the galaxy naming convention.
	// Clone NG_GalaxyLarge/Mid/Small into the StarSystem folder and rename.
	// Mid/Small are zero-particle placeholders for now — they share the Large
	// asset so InitializeTier doesn't fail on a null NiagaraSystem pointer.
	// Swap StarSystemMidCloud / StarSystemSmallCloud for dedicated assets
	// once you've created NG_StarSystemMid and NG_StarSystemSmall.
	StarSystemLargeCloud = LoadObject<UNiagaraSystem>(nullptr,
		TEXT("/svo/StarSystem/NG_StarSystemLarge.NG_StarSystemLarge"));
	StarSystemMidCloud = StarSystemLargeCloud;  // placeholder — replace with NG_StarSystemMid
	StarSystemSmallCloud = StarSystemLargeCloud;  // placeholder — replace with NG_StarSystemSmall

	Octree = MakeShared<FOctree>(Params.Extent);
}

AStarSystemActor::~AStarSystemActor()
{
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

// ---------------------------------------------------------------------------
//  BeginPlay / EndPlay
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
//  Parallax stub (VirtualTraversal model handles everything in TickFromParent)
// ---------------------------------------------------------------------------
#pragma region Parallax Stub
void AStarSystemActor::ApplyParallaxOffset()
{
	// VirtualTraversal accumulation and Niagara pushes are handled inline in
	// TickFromParent — this stub satisfies the pure virtual.
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Pool Lifecycle
// ---------------------------------------------------------------------------
#pragma region Pool Lifecycle
void AStarSystemActor::ResetForPool()
{
	// Tear down any live planet actors on the game thread.
	for (auto& Pair : SpawnedPlanets)
	{
		if (AActor* Planet = Pair.Value.Get())
		{
			if (IsValid(Planet)) Planet->Destroy();
		}
	}
	SpawnedPlanets.Empty();

	// Tear down tier Niagara components.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
		Tier->NiagaraComponents.Empty();
		Tier->Buffers.Empty();
		Tier->ActiveSlots.Empty();
		Tier->FreeSlots.Empty();
		Tier->SlotCounts.Empty();
		Tier->CellCache.Empty();
		Tier->CenterCoord = FIntVector(INT32_MIN);
		Tier->FrontIdx.store(0);
		Tier->bUpdateInProgress.store(false);
		Tier->bNeedsPush.store(false);
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
	Super::ResetForSpawn();
	VirtualTraversal = FVector::ZeroVector;
	LastPushedVirtualTraversal = FVector::ZeroVector;
	LastFrameOfReferenceLocation = FVector::ZeroVector;
	CurrentFrameOfReferenceLocation = FVector::ZeroVector;
	DiagTickCount = 0;
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Params Accessors
// ---------------------------------------------------------------------------
#pragma region Params Accessors
double AStarSystemActor::GetParentSpeedScale() const
{
	if (Galaxy && Galaxy->Universe)
		return Galaxy->Universe->SpeedScale;
	if (Galaxy)
		return Galaxy->SpeedScale;
	return SpeedScale;
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Initialization — Data
// ---------------------------------------------------------------------------
#pragma region InitializeData
void AStarSystemActor::InitializeData()
{
	// Rebuild the octree against the current Params.Extent.
	Octree = MakeShared<FOctree>(Params.Extent);

	// --- Analytically build planet positions along +X axis ---
	// We skip the noise generator entirely (density function = 0).
	// Planets are evenly spaced between InnerOrbit and OuterOrbit along +X.
	const int32 NumPlanets = FMath::Max(1, Params.MaxPlanets);
	const double InnerRadius = Params.Extent * Params.InnerOrbitFraction;
	const double OuterRadius = Params.Extent * Params.OuterOrbitFraction;

	PlanetPositions.SetNumUninitialized(NumPlanets);
	PlanetExtents.SetNumUninitialized(NumPlanets);
	PlanetColors.SetNumUninitialized(NumPlanets);

	FRandomStream Stream(Params.Seed);

	for (int32 i = 0; i < NumPlanets; i++)
	{
		if (InitializationState == ELifecycleState::Pooling) return;

		// Evenly space planets along +X from inner to outer orbit.
		const double t = (NumPlanets > 1)
			? static_cast<double>(i) / static_cast<double>(NumPlanets - 1)
			: 0.5;
		const double OrbitRadius = FMath::Lerp(InnerRadius, OuterRadius, t);

		// Place along +X axis — easy to traverse in a straight line.
		PlanetPositions[i] = FVector(OrbitRadius, 0.0, 0.0);

		// Planet sprite size: 1% of the spacing between adjacent orbits.
		// Keeps planets clearly distinct from each other and from the star.
		// Tune PlanetExtentFraction on FStarSystemParams if needed.
		const double Spacing = (NumPlanets > 1) ? (OuterRadius - InnerRadius) / static_cast<double>(NumPlanets - 1) : OuterRadius;
		const double PlanetExtent = Spacing * Params.PlanetExtentFraction;
		PlanetExtents[i] = static_cast<float>(FMath::Max(PlanetExtent, 1.0));

		// Vary color by orbit: inner = warm rocky, outer = cool icy blue.
		PlanetColors[i] = FLinearColor(
			FMath::Lerp(0.9f, 0.1f, static_cast<float>(t)),  // R
			FMath::Lerp(0.4f, 0.6f, static_cast<float>(t)),  // G
			FMath::Lerp(0.1f, 1.0f, static_cast<float>(t)),  // B
			1.0f
		);
	}

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::InitializeData — %d planets along +X [%.0f … %.0f octree units], extents ~%.0f (fraction=%.3f)"),
		NumPlanets, InnerRadius, OuterRadius,
		NumPlanets > 1 ? (OuterRadius - InnerRadius) / static_cast<double>(NumPlanets - 1) * Params.PlanetExtentFraction : 1.0,
		Params.PlanetExtentFraction);
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Initialization — Volumetric (no volumetric for star system first pass)
// ---------------------------------------------------------------------------
#pragma region InitializeVolumetric
void AStarSystemActor::InitializeVolumetric()
{
	// No volumetric component for first-pass star systems.
	// The star sprite is managed by the parent galaxy and intentionally NOT
	// faded out when the star system spawns — it acts as the star itself.
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Initialization — Niagara (tier setup mirrors GalaxyActor)
// ---------------------------------------------------------------------------
#pragma region InitializeNiagara
void AStarSystemActor::InitializeNiagara()
{
	if (InitializationState == ELifecycleState::Pooling) return;

	BuildTierConfigs();

	// Insert planet data into the octree once before InitializeTier so the
	// large tier's neighbor scanning has nodes to discover.
	// We do a direct octree insert (no tier pipeline insert) here because the
	// large tier has NeighborhoodRadius=0 — it is always fully loaded.
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
		VD.ObjectId = Params.Seed * 1000 + i;
		VD.TypeId = static_cast<int32>(StarSystemDataGenerator::EObjectType::TerrestrialPlanet);

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

	InitializationState = ELifecycleState::Ready;

	// No timer start needed — Universe::DetermineAndDispatchScan drives scans.

	UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::InitializeNiagara — Ready"));
}
#pragma endregion

// ---------------------------------------------------------------------------
//  BuildTierConfigs
// ---------------------------------------------------------------------------
#pragma region BuildTierConfigs
void AStarSystemActor::BuildTierConfigs()
{
	// --- Large tier: planets, single cell exhaustive, always loaded ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = Params.LargeTier.NeighborhoodRadius;
	LargeTierConfig.SlotCapacity = FMath::Max(Params.LargeTier.MaxParticlesPerSlot, Params.MaxPlanets);
	LargeTierConfig.NiagaraAssets = { StarSystemLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0; // Insert planet nodes for spawn scanning

	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex,
		TArray<FNiagaraParticleBuffer*>& Buffers)
		{
			// Analytically fill the slot with our pre-computed planet positions.
			// Density function = 0 — we bypass any noise and write directly.
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
		const double HalfBound = Params.Extent * Params.OuterOrbitFraction * Params.BoundsScaleMultiplier;
		return FBox(FVector(-HalfBound), FVector(HalfBound));
		};

	// No ShouldSkipCell or OnBoundaryCross needed for the exhaustive large tier.

	// --- Mid tier: zero-particle placeholder ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.TierIndex = 1;
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = 0; // Zero particles for now
	MidTierConfig.NiagaraAssets = { StarSystemMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = -1; // Skip octree insertion

	MidTierConfig.GenerateCallback = [this](const FIntVector&, int32, TArray<FNiagaraParticleBuffer*>&) {
		// No-op: zero-particle tier.
		};

	MidTierConfig.ComputeBounds = [this]() {
		const double HalfCell = GetGridCellExtent(MidTierConfig.GridDepth)
			* (2 * MidTierConfig.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfCell), FVector(HalfCell));
		};

	// --- Small tier: zero-particle placeholder ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = 0; // Zero particles for now
	SmallTierConfig.NiagaraAssets = { StarSystemSmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = -1; // Skip octree insertion

	SmallTierConfig.GenerateCallback = [this](const FIntVector&, int32, TArray<FNiagaraParticleBuffer*>&) {
		// No-op: zero-particle tier.
		};

	SmallTierConfig.ComputeBounds = [this]() {
		const double HalfCell = GetGridCellExtent(SmallTierConfig.GridDepth)
			* (2 * SmallTierConfig.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfCell), FVector(HalfCell));
		};
}
#pragma endregion

// ---------------------------------------------------------------------------
//  BuildStreamingContext
// ---------------------------------------------------------------------------
#pragma region BuildStreamingContext
FTierStreamingContext AStarSystemActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = Params.Extent;
	Ctx.UnitScale = Params.UnitScale;
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = false;
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = GetName();
	return Ctx;
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Grid Coord Helpers
// ---------------------------------------------------------------------------
#pragma region Grid Coord Helpers
FIntVector AStarSystemActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	return FTierStreamingSystem::PositionToGridCoord(InPos, InGridDepth, Params.Extent, GridExtentMultiplier);
}

FVector AStarSystemActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	return FTierStreamingSystem::GridCoordToCenter(InCoord, InGridDepth, Params.Extent, GridExtentMultiplier);
}

double AStarSystemActor::GetGridCellExtent(int32 InGridDepth) const
{
	return FTierStreamingSystem::GetGridCellExtent(InGridDepth, Params.Extent, GridExtentMultiplier);
}
#pragma endregion

// ---------------------------------------------------------------------------
//  ComputeChildSpawnLocation
// ---------------------------------------------------------------------------
#pragma region Child Spawn Location
FVector AStarSystemActor::ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const
{
	// Mirrors GalaxyActor::ComputeChildSpawnLocation exactly.
	// Rendered position = ActorLocation + NodeCenter - VirtualTraversal.
	// Scale the camera-to-node vector proportionally to place the (physically
	// larger) child actor at the same angular size.
	const double SizeRatio = Params.UnitScale / ChildUnitScale;
	const FVector RenderedPos = GetActorLocation() + NodeCenter - VirtualTraversal;
	const FVector CameraToNode = RenderedPos - CurrentFrameOfReferenceLocation;
	return CurrentFrameOfReferenceLocation + CameraToNode * SizeRatio;
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Tick
// ---------------------------------------------------------------------------
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

void AStarSystemActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	if (InitializationState != ELifecycleState::Ready) return;

	// --- VirtualTraversal accumulation (mirrors GalaxyActor) ---
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double ActiveSpeedScale = GetParentSpeedScale();
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	// Peg actor to the player so UE's scene graph stays numerically clean.
	SetActorLocation(InPlayerPos);

	// --- Niagara position push (gated by push threshold) ---
	// StarSystem Niagara components are attached (not absolute-positioned),
	// so they follow the actor via SetActorLocation above. Only enter
	// the per-component loop when positions actually need re-pushing.
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	const bool bNeedsPush = (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold);

	if (bNeedsPush)
	{
		for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		{
			const int32 FrontIdx = Tier->FrontIdx.load();
			for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
			{
				UNiagaraComponent* NC = Tier->NiagaraComponents[b];
				if (!NC || b >= Tier->Buffers.Num()) continue;
				const TArray<FVector>& RelPos =
					Tier->Buffers[b][FrontIdx].MakeRelativePositions(VirtualTraversal);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
					NC, NiagaraBufferParams::Positions, RelPos);
			}
		}
		LastPushedVirtualTraversal = VirtualTraversal;
	}

	// --- Tier streaming ---
	// Large tier has NeighborhoodRadius=0 so UpdateTier is effectively a no-op
	// after the initial load — it just checks the center coord each tick.
	// Mid/Small stream but produce zero particles.
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, LargeTierConfig, LargeTierState);
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	// --- Drive live planets ---
	// Each planet's world position is recomputed from the current VT every
	// frame so it stays locked to its parallax-correct location.
	for (auto& Pair : SpawnedPlanets)
	{
		if (AParallaxStaticMeshActor* Planet = Cast<AParallaxStaticMeshActor>(Pair.Value.Get()))
			Planet->TickFromStarSystem(InPlayerPos);
	}

	// --- Planet spawn scan ---
	// VirtualTraversal is resolved for this frame; process any pending
	// octree query results to fire SpawnPlanetFromPool / ReturnPlanetToPool.
	ProcessPendingScanResults();

	if (IsDebug) DrawDebugBounds();

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

// ---------------------------------------------------------------------------
//  Spawn Range Scanning
// ---------------------------------------------------------------------------
#pragma region Spawn Range Scanning
void AStarSystemActor::RequestScan()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;

	TWeakObjectPtr<AStarSystemActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos]()
		{
			AStarSystemActor* Self = WeakThis.Get();
			if (!Self) return;

			const TArray<TSharedPtr<FOctreeNode>> NearbyArray =
				Self->Octree->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray]()
				{
					AStarSystemActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
					InnerSelf->bSpawnScanInProgress.store(false);
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
		TEXT("AStarSystemActor::SpawnScan ENTER — center=(%.1f,%.1f,%.1f) extent=%.2f objId=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Data.ObjectId);
}

void AStarSystemActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnScan EXIT  — center=(%.1f,%.1f,%.1f) extent=%.2f objId=%d"),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		InNode->Extent, InNode->Data.ObjectId);
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

// ---------------------------------------------------------------------------
//  Planet Spawn / Despawn Hooks
// ---------------------------------------------------------------------------
#pragma region Planet Spawn Hooks
void AStarSystemActor::SpawnPlanetFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || SpawnedPlanets.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	UWorld* World = GetWorld();
	if (!World) return;

	// --- Resolve the actual particle position and extent from the buffer ---
	// The octree node center is a quantized cell center, and InNode->Extent
	// is the cell half-size at that depth — neither matches the real particle.
	// ObjectId encodes: Seed * 1000 + PlanetIndex  (set in InitializeNiagara).
	// Extract the planet index and read the authoritative values directly.
	FVector  ParticlePos = InNode->Center;                           // fallback
	float    ParticleExtent = static_cast<float>(InNode->Extent);       // fallback

	const int32 PlanetIndex = InNode->Data.ObjectId - (Params.Seed * 1000);
	if (PlanetIndex >= 0 && PlanetIndex < PlanetPositions.Num())
	{
		ParticlePos = PlanetPositions[PlanetIndex];
		ParticleExtent = PlanetExtents[PlanetIndex];
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("AStarSystemActor::SpawnPlanetFromPool — could not resolve planet index "
				"from ObjectId=%d Seed=%d, falling back to octree node center"),
			InNode->Data.ObjectId, Params.Seed);
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// Initial world position: PlayerPos + (ParticlePos - VirtualTraversal).
	// TickFromStarSystem will recompute this every frame, so the spawn
	// position just needs to be roughly correct to avoid a one-frame pop.
	const FVector SpawnLoc = CurrentFrameOfReferenceLocation + (ParticlePos - VirtualTraversal);

	AParallaxStaticMeshActor* Planet = World->SpawnActor<AParallaxStaticMeshActor>(
		AParallaxStaticMeshActor::StaticClass(),
		SpawnLoc,
		FRotator::ZeroRotator,
		SpawnParams);

	if (!Planet)
	{
		UE_LOG(LogTemp, Warning, TEXT("AStarSystemActor::SpawnPlanetFromPool — SpawnActor failed"));
		return;
	}

	// Wire back-reference and store the ACTUAL particle position (not the
	// quantized node center) so TickFromStarSystem recomputes the correct
	// world position every frame.
	Planet->System = this;
	Planet->NodeCenter = ParticlePos;

	// Visual size: Niagara positions and extents are pushed in octree-local
	// units, and the Niagara component sits at the player position.
	// 1 octree unit = 1 cm of offset from the camera. The mesh must match:
	// its world-space radius IS the raw particle extent (the same value
	// Niagara uses for sprite sizing).
	//
	// UnitSphere has a 50 cm radius → diameter = 100 cm.
	// To make the sphere match a radius of 'R' cm: scale = (R * 2) / 100.
	const double WorldRadiusCm = static_cast<double>(ParticleExtent); //TODO: We already have particle extent, this is the same, we dont need world radius cm
	const double MeshScale = WorldRadiusCm;

	UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitSphere.UnitSphere"));
	if (SphereMesh && Planet->MeshComponent)
	{
		Planet->MeshComponent->SetStaticMesh(SphereMesh);
		Planet->MeshComponent->SetWorldScale3D(FVector(MeshScale));

		UMaterialInterface* BaseMat = Planet->MeshComponent->GetMaterial(0);
		if (BaseMat)
		{
			UMaterialInstanceDynamic* DynMat =
				UMaterialInstanceDynamic::Create(BaseMat, Planet->MeshComponent);
			DynMat->SetVectorParameterValue(FName("BaseColor"),
				FLinearColor(InNode->Data.Composition));
			Planet->MeshComponent->SetMaterial(0, DynMat);
		}
	}

	SpawnedPlanets.Add(InNode, TWeakObjectPtr<AActor>(Planet));

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnPlanetFromPool — planet[%d] at (%.1f,%.1f,%.1f) "
			"radius=%.1fcm scale=%.4f particlePos=(%.1f,%.1f,%.1f) "
			"nodeCenter=(%.1f,%.1f,%.1f) VT=(%.1f,%.1f,%.1f)"),
		PlanetIndex,
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		WorldRadiusCm, MeshScale,
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z,
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z,
		VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z);
}

void AStarSystemActor::FinalizePlanetPlacement(AActor* Planet, TSharedPtr<FOctreeNode> InNode)
{
	// Static mesh planets have no async init — placement is immediate in
	// SpawnPlanetFromPool. This function is a hook for when real planet actors
	// (with async initialization) replace the placeholder.
	// For now it's unused but declared so the header compiles cleanly.
}

void AStarSystemActor::ReturnPlanetToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AActor> WeakPlanet;
	if (SpawnedPlanets.RemoveAndCopyValue(InNode, WeakPlanet))
	{
		TWeakObjectPtr<AActor> WeakP(WeakPlanet);
		AsyncTask(ENamedThreads::GameThread, [WeakP]()
			{
				if (AActor* P = WeakP.Get(); P && IsValid(P))
				{
					P->Destroy();
					UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::ReturnPlanetToPool — destroyed"));
				}
			});
	}
}
#pragma endregion

// ---------------------------------------------------------------------------
//  Diagnostics
// ---------------------------------------------------------------------------
#pragma region Diagnostics
void AStarSystemActor::DrawPlanetDebugPositions() const
{
	if (!GetWorld()) return;

	const FVector ActorLoc = GetActorLocation();
	for (int32 i = 0; i < PlanetPositions.Num(); i++)
	{
		const FVector WorldPos = ActorLoc + PlanetPositions[i] - VirtualTraversal;
		const float Radius = static_cast<float>(PlanetExtents[i] * Params.UnitScale);
		DrawDebugSphere(GetWorld(), WorldPos, FMath::Max(Radius, 1.0f),
			8, FColor::Cyan, false, 0.15f);
	}
}
#pragma endregion
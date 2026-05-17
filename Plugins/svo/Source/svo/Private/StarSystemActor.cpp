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

	// Load Niagara assets. Mid/Small use the same asset as Large for now
	// since they have zero particles — swap for proper assets when adding content.
	StarSystemLargeCloud = LoadObject<UNiagaraSystem>(nullptr,
		TEXT("/svo/StarSystem/NG_StarSystemLarge.NG_StarSystemLarge"));
	StarSystemMidCloud = LoadObject<UNiagaraSystem>(nullptr,
		TEXT("/svo/StarSystem/NG_StarSystemMid.NG_StarSystemMid"));
	StarSystemSmallCloud = LoadObject<UNiagaraSystem>(nullptr,
		TEXT("/svo/StarSystem/NG_StarSystemSmall.NG_StarSystemSmall"));

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

		// Place along +X axis.
		PlanetPositions[i] = FVector(OrbitRadius, 0.0, 0.0);

		// Assign a simple scale that grows with orbit distance (inner = smaller).
		// Use FPointData::MakePointDataFromWorldScale to get a depth-consistent
		// particle extent. For display we just store the octree-space radius.
		// Planet world diameter: lerp from ~Mercury to ~Neptune range.
		const double WorldDiameterCm = FMath::Lerp(
			5.0e8,    // ~Mercury diameter in cm
			5.0e10,   // ~Neptune diameter in cm
			t);
		const double OctreeRadius = (WorldDiameterCm * 0.5) / Params.UnitScale;
		PlanetExtents[i] = static_cast<float>(FMath::Max(OctreeRadius, 1.0));

		// Vary color by orbit: inner = warm rocky, outer = cool icy blue.
		PlanetColors[i] = FLinearColor(
			FMath::Lerp(0.8f, 0.2f, static_cast<float>(t)),  // R: red → blue
			FMath::Lerp(0.5f, 0.7f, static_cast<float>(t)),  // G: mid
			FMath::Lerp(0.2f, 1.0f, static_cast<float>(t)),  // B: orange → blue
			1.0f
		);
	}

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::InitializeData — %d planets generated along +X [%.0f … %.0f octree units]"),
		NumPlanets, InnerRadius, OuterRadius);
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
	Ctx.bNiagaraAbsolutePosition = true;
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
	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	const bool bNeedsPush = (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold);

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		const int32 FrontIdx = Tier->FrontIdx.load();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;
			NC->SetWorldLocation(InPlayerPos);
			if (bNeedsPush)
			{
				const TArray<FVector>& RelPos =
					Tier->Buffers[b][FrontIdx].MakeRelativePositions(VirtualTraversal);
				UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
					NC, NiagaraBufferParams::Positions, RelPos);
			}
		}
	}
	if (bNeedsPush) LastPushedVirtualTraversal = VirtualTraversal;

	// --- Tier streaming ---
	// Large tier has NeighborhoodRadius=0 so UpdateTier is effectively a no-op
	// after the initial load — it just checks the center coord each tick.
	// Mid/Small stream but produce zero particles.
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, LargeTierConfig, LargeTierState);
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	// --- Spawn scan: process nodes from the large tier's octree ---
	// The large tier's InitializeTier ran InsertSlotIntoOctree for all planet
	// nodes, so the octree now has FOctreeNodes we can scan against the player's
	// virtual position for spawn/despawn thresholds.
	// This mirrors how GalaxyActor drives star system spawning via
	// ProceduralSpaceActor::TickFromParent's spawn scan.
	// Base class TickFromParent handles the scan using SpawnScreenSpaceThreshold
	// and calls SpawnEntityFromPool / ReturnEntityToPool on the results.
	// We defer to base here:
	AProceduralSpaceActor::TickFromParent(DeltaTime, InPlayerPos);

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
//  Planet Spawn / Despawn Hooks  (mirrors GalaxyActor star system hooks)
// ---------------------------------------------------------------------------
#pragma region Planet Spawn Hooks
void AStarSystemActor::SpawnPlanetFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || SpawnedPlanets.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	UWorld* World = GetWorld();
	if (!World) return;

	// Compute spawn location using the same parallax-aware formula as
	// GalaxyActor::SpawnStarSystemFromPool.
	// For planet placeholders, UnitScale = (InNode->Extent * Params.UnitScale) / PlanetExtent.
	// We don't have a dedicated pool yet, so we spawn a fresh actor each time.
	// When a proper planet actor class exists, swap this for a pool pop.
	const double NodeWorldExtentCm = static_cast<double>(InNode->Extent) * Params.UnitScale;
	// Placeholder UnitScale = the star system's UnitScale shrunk by the node's depth ratio.
	const double PlanetUnitScale = NodeWorldExtentCm / static_cast<double>(Params.Extent);

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride =
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// -- Placeholder: spawn a ParallaxStaticMeshActor as a stand-in --
	AParallaxStaticMeshActor* Planet = World->SpawnActor<AParallaxStaticMeshActor>(
		AParallaxStaticMeshActor::StaticClass(),
		ComputeChildSpawnLocation(InNode->Center, PlanetUnitScale),
		FRotator::ZeroRotator,
		SpawnParams);

	if (!Planet)
	{
		UE_LOG(LogTemp, Warning, TEXT("AStarSystemActor::SpawnPlanetFromPool — SpawnActor failed"));
		return;
	}

	// Configure the placeholder mesh.
	Planet->UnitScale = PlanetUnitScale;
	Planet->SpeedScale = GetParentSpeedScale();
	// Planet->System = this;  // Uncomment when AParallaxStaticMeshActor gains a StarSystem ref.

	UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitSphere.UnitSphere"));
	if (SphereMesh && Planet->MeshComponent)
	{
		Planet->MeshComponent->SetStaticMesh(SphereMesh);

		// Scale the mesh so it visually matches the node's world extent.
		const double MeshScaleCm = NodeWorldExtentCm * 2.0; // diameter in cm
		// UnitSphere radius = 50 cm, so divide by 50 to get scale factor.
		const double MeshScale = MeshScaleCm / 50.0;
		Planet->MeshComponent->SetWorldScale3D(FVector(MeshScale));

		// Apply planet color from the node's composition.
		UMaterialInterface* BaseMat = Planet->MeshComponent->GetMaterial(0);
		if (BaseMat)
		{
			UMaterialInstanceDynamic* DynMat =
				UMaterialInstanceDynamic::Create(BaseMat, Planet->MeshComponent);
			DynMat->SetVectorParameterValue(
				FName("BaseColor"),
				FLinearColor(InNode->Data.Composition));
			Planet->MeshComponent->SetMaterial(0, DynMat);
		}
	}

	SpawnedPlanets.Add(InNode, TWeakObjectPtr<AActor>(Planet));

	UE_LOG(LogTemp, Log,
		TEXT("AStarSystemActor::SpawnPlanetFromPool — spawned planet at %s (UnitScale=%.4f)"),
		*ComputeChildSpawnLocation(InNode->Center, PlanetUnitScale).ToString(),
		PlanetUnitScale);
}

void AStarSystemActor::ReturnPlanetToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AActor> WeakPlanet;
	if (SpawnedPlanets.RemoveAndCopyValue(InNode, WeakPlanet))
	{
		if (AActor* Planet = WeakPlanet.Get())
		{
			// Destroy on game thread. When we have a real planet pool, this
			// becomes a ResetForPool + pool re-insert instead.
			TWeakObjectPtr<AActor> WeakP(Planet);
			AsyncTask(ENamedThreads::GameThread, [WeakP]()
				{
					if (AActor* P = WeakP.Get(); P && IsValid(P))
					{
						P->Destroy();
						UE_LOG(LogTemp, Log, TEXT("AStarSystemActor::ReturnPlanetToPool — destroyed planet"));
					}
				});
		}
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
		// Render position = ActorLoc + PlanetPos - VirtualTraversal
		const FVector WorldPos = ActorLoc + PlanetPositions[i] - VirtualTraversal;
		const float Radius = static_cast<float>(PlanetExtents[i] * Params.UnitScale);
		DrawDebugSphere(GetWorld(), WorldPos, FMath::Max(Radius, 1.0f),
			8, FColor::Cyan, false, 0.15f);
	}
}
#pragma endregion
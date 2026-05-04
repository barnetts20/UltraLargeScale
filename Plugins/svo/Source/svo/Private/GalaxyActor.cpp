// GalaxyActor.cpp
// Full tier streaming system mirroring UniverseActor pattern.

#pragma region Includes
#include "GalaxyActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
#include "FVolumeTextureUtils.h"
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor/Destructor
AGalaxyActor::AGalaxyActor()
{
	PrimaryActorTick.bCanEverTick = true;

	GalaxyLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge"));
	GalaxyMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge"));
	GalaxySmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge"));

	StarSystemActorClass = AStarSystemActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent);
}

AGalaxyActor::~AGalaxyActor()
{
	if (Octree.IsValid()) Octree.Reset();
}
#pragma endregion

#pragma region BeginPlay
void AGalaxyActor::BeginPlay()
{
	Super::BeginPlay();
	if (bAutoInitializeOnBeginPlay)
	{
		InitializationState = ELifecycleState::Initializing;
		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
			{
				Initialize();
			});
	}
}
#pragma endregion

#pragma region Pool Lifecycle
void AGalaxyActor::ResetForPool()
{
	// Tear down tier Niagara components before base class teardown
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC)
			{
				NC->Deactivate();
				NC->DestroyComponent();
				NC = nullptr;
			}
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

	// Base class handles VolumetricComponent and legacy NiagaraComponent
	Super::ResetForPool();
}

void AGalaxyActor::ResetForSpawn()
{
	Super::ResetForSpawn();
	VirtualTraversal = FVector::ZeroVector;
}
#pragma endregion

#pragma region Initialization
void AGalaxyActor::InitializeChildPool()
{
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			for (int i = 0; i < StarSystemPoolSize; i++) {
				AStarSystemActor* System = GetWorld()->SpawnActor<AStarSystemActor>(StarSystemActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
				System->Galaxy = this;
				StarSystemPool.Add(System);
			}
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
}

void AGalaxyActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	GalaxyGenerator.Params = Params;
	GalaxyGenerator.Initialize();

	if (InitializationState == ELifecycleState::Pooling) return;

	TArray<uint8> VolumeData = GalaxyGenerator.SampleNoiseVolume(Params.DensityVolumeResolution);

	if (InitializationState == ELifecycleState::Pooling) return;

	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(
		FVolumeTextureUtils::PackToPseudoVolumeLayout(
			FVolumeTextureUtils::UpscaleVolumeData(VolumeData, Params.DensityVolumeResolution)));

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeData took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			VolumetricComponent = NewObject<UStaticMeshComponent>(this);
			VolumetricComponent->SetVisibility(false);
			VolumetricComponent->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/svo/UnitBoxInvertedNormals.UnitBoxInvertedNormals")));
			VolumetricComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			VolumetricComponent->TranslucencySortPriority = 1;
			VolumetricComponent->DepthPriorityGroup = ESceneDepthPriorityGroup::SDPG_MAX;
			VolumetricComponent->bRenderInDepthPass = false;
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

			VolumeMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath), this);

			VolumeMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumeMaterial->SetTextureParameterValue(FName("NoiseTexture"), LoadObject<UVolumeTexture>(nullptr, *Params.VolumeNoise));
			VolumeMaterial->SetVectorParameterValue(FName("AmbientColor"), Params.VolumeAmbientColor);
			VolumeMaterial->SetVectorParameterValue(FName("CoolShift"), Params.VolumeCoolShift);
			VolumeMaterial->SetVectorParameterValue(FName("HotShift"), Params.VolumeHotShift);
			VolumeMaterial->SetScalarParameterValue(FName("HueVariance"), Params.VolumeHueVariance);
			VolumeMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Params.VolumeHueVarianceScale);
			VolumeMaterial->SetScalarParameterValue(FName("SaturationVariance"), Params.VolumeSaturationVariance);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Params.VolumeTemperatureInfluence);
			VolumeMaterial->SetScalarParameterValue(FName("TemperatureScale"), Params.VolumeTemperatureScale);
			VolumeMaterial->SetScalarParameterValue(FName("ScaleFactor"), Params.VolumeDensity);
			VolumeMaterial->SetScalarParameterValue(FName("WarpAmount"), Params.VolumeWarpAmount);
			VolumeMaterial->SetScalarParameterValue(FName("WarpScale"), Params.VolumeWarpScale);

			VolumetricComponent->SetMaterial(0, VolumeMaterial);
			VolumetricComponent->SetVisibility(true);
			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeVolumetric took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();
	InitializeTier(LargeTierConfig, LargeTierState);
	if (InitializationState == ELifecycleState::Pooling) return;
	InitializeTier(MidTierConfig, MidTierState);
	if (InitializationState == ELifecycleState::Pooling) return;
	InitializeTier(SmallTierConfig, SmallTierState);
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeNiagara total: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Grid Coord Helpers
FIntVector AGalaxyActor::PositionToGridCoord(const FVector& InPos, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FIntVector(
		FMath::FloorToInt32(InPos.X / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Y / CellSize + 0.5),
		FMath::FloorToInt32(InPos.Z / CellSize + 0.5));
}

FVector AGalaxyActor::GridCoordToCenter(const FIntVector& InCoord, int32 InGridDepth) const
{
	const double CellSize = (Params.Extent * GridExtentMultiplier) / (1 << InGridDepth);
	return FVector(
		static_cast<double>(InCoord.X) * CellSize,
		static_cast<double>(InCoord.Y) * CellSize,
		static_cast<double>(InCoord.Z) * CellSize);
}

double AGalaxyActor::GetGridCellExtent(int32 InGridDepth) const
{
	return (Params.Extent * GridExtentMultiplier) / (1 << (InGridDepth + 1));
}

bool AGalaxyActor::CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const
{
	const FVector Center = GridCoordToCenter(Coord, GridDepth);
	const double HalfCell = GetGridCellExtent(GridDepth);
	const double Ext = Params.Extent;
	return (Center.X + HalfCell > -Ext && Center.X - HalfCell < Ext) &&
		(Center.Y + HalfCell > -Ext && Center.Y - HalfCell < Ext) &&
		(Center.Z + HalfCell > -Ext && Center.Z - HalfCell < Ext);
}

FVector AGalaxyActor::GetPlayerLocalPosition() const
{
	// The galaxy actor's attached components (volumetric, Niagara) are in
	// galaxy-local coordinates where 1 unit = 1 UE unit. No actor scaling
	// is applied, so the world-space offset from actor to player maps
	// directly to galaxy-local coordinates.
	return CurrentFrameOfReferenceLocation - GetActorLocation();
}
#pragma endregion

#pragma region Tier System - BuildTierConfigs
void AGalaxyActor::BuildTierConfigs()
{
	Params.DeriveScaleRanges();

	// --- Large tier: exhaustive single cell, no streaming ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = 0;  // Single cell, always loaded
	LargeTierConfig.SlotCapacity = Params.LargeTier.MaxParticlesPerSlot;
	LargeTierConfig.NiagaraAssets = { GalaxyLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, LargeTierConfig.GridDepth);
		GalaxyGenerator.GenerateLargeTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, LargeTierState.SlotCounts[SlotIndex]);
		};

	// --- Mid tier: neighborhood streaming ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.MaxParticlesPerSlot;
	MidTierConfig.NiagaraAssets = { GalaxyMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, MidTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);
		GalaxyGenerator.GenerateMidTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, MidTierState.SlotCounts[SlotIndex]);
		};

	// --- Small tier: neighborhood streaming ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.MaxParticlesPerSlot;
	SmallTierConfig.NiagaraAssets = { GalaxySmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.GenerateCallback = [this](const FIntVector& Coord, int32 SlotIndex, TArray<FNiagaraParticleBuffer*>& Buffers) {
		const FVector NodeCenter = GridCoordToCenter(Coord, SmallTierConfig.GridDepth);
		const double CellExt = GetGridCellExtent(SmallTierConfig.GridDepth);
		GalaxyGenerator.GenerateSmallTierNode(Coord, SlotIndex, *Buffers[0], NodeCenter, CellExt, SmallTierState.SlotCounts[SlotIndex]);
		};

	// Shared ComputeBounds for all tiers.
	// Since galaxy particles are in actor space (no VirtualTraversal offset),
	// bounds are centered on the actor origin.
	auto MakeBounds = [this](const FParticleTierConfig& Config) {
		const double HalfExt = GetGridCellExtent(Config.GridDepth) * (2 * Config.NeighborhoodRadius + 1);
		return FBox(FVector(-HalfExt), FVector(HalfExt));
		};

	LargeTierConfig.ComputeBounds = [this, MakeBounds]() {
		return FBox(FVector(-Params.Extent), FVector(Params.Extent));
		};
	MidTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(MidTierConfig); };
	SmallTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(SmallTierConfig); };
}
#pragma endregion

#pragma region Tier System - InitializeTier
void AGalaxyActor::InitializeTier(FParticleTierConfig& Config, FParticleTierState& State)
{
	double StartTime = FPlatformTime::Seconds();

	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 Side = 2 * Config.NeighborhoodRadius + 1;
	const int32 TotalSlots = Side * Side * Side;

	// Allocate double-buffered particle data
	State.Buffers.SetNum(NumBuffers);
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		State.Buffers[b].SetNum(2);
		bool bRotations = Config.bWantRotations.IsValidIndex(b) && Config.bWantRotations[b];
		State.Buffers[b][0].Allocate(TotalSlots, Config.SlotCapacity, bRotations);
		State.Buffers[b][1].Allocate(TotalSlots, Config.SlotCapacity, bRotations);
	}

	// Initialize slot tracking
	State.SlotCounts.SetNumZeroed(TotalSlots);
	State.FreeSlots.Empty();
	for (int32 i = TotalSlots - 1; i >= 0; --i)
		State.FreeSlots.Add(i);
	State.ActiveSlots.Empty();

	// For streaming tiers (mid/small), set CenterCoord to INT32_MIN so the
	// first UpdateTier call in Tick detects a "boundary cross" and generates
	// the neighborhood around the player's actual position at that time.
	// For the large tier (radius 0, exhaustive), generate immediately.
	const bool bIsStreamingTier = (Config.NeighborhoodRadius > 0);

	if (bIsStreamingTier)
	{
		State.CenterCoord = FIntVector(INT32_MIN, INT32_MIN, INT32_MIN);
	}
	else
	{
		State.CenterCoord = FIntVector(0, 0, 0);

		// Build neighborhood and generate for exhaustive tier
		TArray<FIntVector> InitialCoords;
		for (int32 z = -Config.NeighborhoodRadius; z <= Config.NeighborhoodRadius; ++z)
			for (int32 y = -Config.NeighborhoodRadius; y <= Config.NeighborhoodRadius; ++y)
				for (int32 x = -Config.NeighborhoodRadius; x <= Config.NeighborhoodRadius; ++x)
					InitialCoords.Add(FIntVector(x, y, z));

		TArray<TPair<FIntVector, int32>> ToGenerate;
		for (const FIntVector& Coord : InitialCoords)
		{
			int32 SlotIndex = State.FreeSlots.Pop();
			FSlotEntry& Entry = State.ActiveSlots.Add(Coord);
			Entry.SlotIndex = SlotIndex;
			ToGenerate.Add({ Coord, SlotIndex });
		}

		if (ToGenerate.Num() > 0)
		{
			ParallelFor(ToGenerate.Num(), [&](int32 i)
				{
					const FIntVector& Coord = ToGenerate[i].Key;
					int32 SlotIndex = ToGenerate[i].Value;
					TArray<FNiagaraParticleBuffer*> BackBuffers;
					for (int32 b = 0; b < NumBuffers; ++b)
						BackBuffers.Add(&State.Buffers[b][0]);
					Config.GenerateCallback(Coord, SlotIndex, BackBuffers);
				}, EParallelForFlags::BackgroundPriority);
		}

		if (InitializationState == ELifecycleState::Pooling) return;

		// Mirror front to back
		for (int32 b = 0; b < NumBuffers; ++b)
			State.Buffers[b][1].CopyFrom(State.Buffers[b][0]);

		// Insert into octree
		if (Config.OctreeInsertBufferIndex >= 0)
			InsertTierIntoOctree(Config, State, 0);
	}

	if (InitializationState == ELifecycleState::Pooling) return;

	// GT rendezvous: spawn Niagara components and activate
	TPromise<void> Promise;
	TFuture<void> Future = Promise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, &Config, &State, NumBuffers, Promise = MoveTemp(Promise)]() mutable
		{
			State.NiagaraComponents.SetNum(NumBuffers);
			for (int32 b = 0; b < NumBuffers; ++b)
			{
				UNiagaraComponent* NC = UNiagaraFunctionLibrary::SpawnSystemAttached(
					Config.NiagaraAssets[b],
					GetRootComponent(),
					NAME_None,
					FVector::ZeroVector,
					FRotator::ZeroRotator,
					EAttachLocation::SnapToTarget,
					true, false);

				const FBox Bounds = Config.ComputeBounds();
				NC->SetSystemFixedBounds(Bounds);
				NC->SetVariableFloat(FName("MaxExtent"), Params.Extent);

				State.NiagaraComponents[b] = NC;
				TierNiagaraComponents.Add(NC);

				State.Buffers[b][0].ActivateOnce(NC, FVector::ZeroVector);
			}
			Promise.SetValue();
		});
	Future.Wait();

	State.FrontIdx.store(0);
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeTier [%s] - %d slots, %d capacity, streaming=%d, %.3f sec"),
		*Config.TierName, TotalSlots, Config.SlotCapacity, bIsStreamingTier ? 1 : 0, FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Tier System - UpdateTier
void AGalaxyActor::UpdateTier(FParticleTierConfig& Config, FParticleTierState& State)
{
	if (InitializationState != ELifecycleState::Ready) return;

	// Push if async generation completed
	if (State.bNeedsPush.load())
	{
		PushTierToNiagara(Config, State);
		State.bNeedsPush.store(false);
	}

	// No streaming for single-cell tiers (Large)
	if (Config.NeighborhoodRadius == 0) return;

	// Check for boundary cross
	const FIntVector NewCoord = PositionToGridCoord(VirtualTraversal, Config.GridDepth);
	if (NewCoord == State.CenterCoord) return;
	if (State.bUpdateInProgress.load()) return;

	State.bUpdateInProgress.store(true);
	const FIntVector OldCoord = State.CenterCoord;
	const bool bIsInitialPopulation = (OldCoord.X == INT32_MIN);
	State.CenterCoord = NewCoord;

	TWeakObjectPtr<AGalaxyActor> WeakThis(this);

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, &Config, &State, OldCoord, NewCoord, bIsInitialPopulation]()
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self || Self->InitializationState == ELifecycleState::Pooling)
			{
				State.bUpdateInProgress.store(false);
				return;
			}

			double StartTime = FPlatformTime::Seconds();
			const int32 R = Config.NeighborhoodRadius;
			const int32 NumBuffers = Config.NiagaraAssets.Num();
			const int32 BackIdx = 1 - State.FrontIdx.load();

			// Copy front to back as baseline
			for (int32 b = 0; b < NumBuffers; ++b)
				State.Buffers[b][BackIdx].CopyFrom(State.Buffers[b][State.FrontIdx.load()]);

			// Diff neighborhoods
			TArray<FIntVector> EnteringNodes, ExitingNodes;

			if (bIsInitialPopulation)
			{
				// First population: everything around NewCoord enters, nothing exits
				for (int32 z = -R; z <= R; ++z)
					for (int32 y = -R; y <= R; ++y)
						for (int32 x = -R; x <= R; ++x)
							EnteringNodes.Add(NewCoord + FIntVector(x, y, z));
			}
			else
			{
				for (int32 z = -R; z <= R; ++z)
					for (int32 y = -R; y <= R; ++y)
						for (int32 x = -R; x <= R; ++x)
						{
							FIntVector NewCell = NewCoord + FIntVector(x, y, z);
							FIntVector OldCell = OldCoord + FIntVector(x, y, z);

							int32 OldDist = FMath::Max3(
								FMath::Abs(NewCell.X - OldCoord.X),
								FMath::Abs(NewCell.Y - OldCoord.Y),
								FMath::Abs(NewCell.Z - OldCoord.Z));
							if (OldDist > R)
								EnteringNodes.Add(NewCell);

							int32 NewDist = FMath::Max3(
								FMath::Abs(OldCell.X - NewCoord.X),
								FMath::Abs(OldCell.Y - NewCoord.Y),
								FMath::Abs(OldCell.Z - NewCoord.Z));
							if (NewDist > R)
								ExitingNodes.Add(OldCell);
						}
			}

			// Free exiting slots
			const FVector DeadPos(Self->Params.Extent * 10.0);
			for (const FIntVector& Coord : ExitingNodes)
			{
				FSlotEntry* Entry = State.ActiveSlots.Find(Coord);
				if (!Entry) continue;
				for (int32 b = 0; b < NumBuffers; ++b)
					State.Buffers[b][BackIdx].ClearSlot(Entry->SlotIndex, DeadPos);
				State.FreeSlots.Add(Entry->SlotIndex);
				State.ActiveSlots.Remove(Coord);
			}

			// Generate entering cells
			int32 CacheHitCount = 0;
			TArray<TPair<FIntVector, int32>> ToGenerate;
			TArray<int32> AllEnteringSlots;

			for (const FIntVector& Coord : EnteringNodes)
			{
				if (State.FreeSlots.Num() == 0) break;
				int32 SlotIndex = State.FreeSlots.Pop();
				FSlotEntry& Entry = State.ActiveSlots.Add(Coord);
				Entry.SlotIndex = SlotIndex;
				AllEnteringSlots.Add(SlotIndex);

				// Skip cells entirely outside the galaxy volume
				if (!Self->CellOverlapsVolume(Coord, Config.GridDepth))
				{
					State.SlotCounts[SlotIndex] = 0;
					for (int32 b = 0; b < NumBuffers; ++b)
						State.Buffers[b][BackIdx].PadSlotDead(SlotIndex, 0, DeadPos);
					continue;
				}

				// Check cache
				FCachedCellData* Cached = State.CellCache.Find(Coord);
				if (Cached && Cached->ParticleCount > 0)
				{
					// Cache hit: blit stored data into back buffer
					State.SlotCounts[SlotIndex] = Cached->ParticleCount;
					const int32 Start = SlotIndex * Config.SlotCapacity;
					for (int32 b = 0; b < NumBuffers; ++b)
					{
						FNiagaraParticleBuffer& Buf = State.Buffers[b][BackIdx];
						for (int32 p = 0; p < Cached->ParticleCount; ++p)
						{
							Buf.Positions[Start + p] = Cached->PerBufferPositions[b][p];
							Buf.Extents[Start + p] = Cached->PerBufferExtents[b][p];
							Buf.Colors[Start + p] = Cached->PerBufferColors[b][p];
						}
						Buf.PadSlotDead(SlotIndex, Cached->ParticleCount, DeadPos);
					}
					CacheHitCount++;
				}
				else
				{
					ToGenerate.Add({ Coord, SlotIndex });
				}
			}

			// Generate cache misses in parallel
			if (ToGenerate.Num() > 0)
			{
				ParallelFor(ToGenerate.Num(), [&](int32 i)
					{
						const FIntVector& Coord = ToGenerate[i].Key;
						int32 SlotIndex = ToGenerate[i].Value;
						TArray<FNiagaraParticleBuffer*> BackBuffers;
						for (int32 b = 0; b < NumBuffers; ++b)
							BackBuffers.Add(&State.Buffers[b][BackIdx]);
						Config.GenerateCallback(Coord, SlotIndex, BackBuffers);
					}, EParallelForFlags::BackgroundPriority);
			}

			// Cache newly generated cells
			for (const auto& Pair : ToGenerate)
				Self->CacheCellFromBuffers(Config, State, Pair.Key, Pair.Value, BackIdx);

			// Incremental octree insert
			if (Config.OctreeInsertBufferIndex >= 0)
			{
				for (int32 Slot : AllEnteringSlots)
					Self->InsertSlotIntoOctree(Config, State, Slot, BackIdx);
			}

			// Swap and signal
			State.FrontIdx.store(BackIdx);
			State.bNeedsPush.store(true);
			Self->CullTierCache(Config, State, NewCoord);
			State.bUpdateInProgress.store(false);

			int32 TotalParticles = 0;
			for (int32 s = 0; s < State.SlotCounts.Num(); ++s)
				TotalParticles += State.SlotCounts[s];

			UE_LOG(LogTemp, Warning, TEXT("AGalaxyActor::UpdateTier [%s] - center=(%d,%d,%d) %d entering (%d cached, %d gen), %d exiting, %d total particles, %.3f sec"),
				*Config.TierName, NewCoord.X, NewCoord.Y, NewCoord.Z,
				EnteringNodes.Num(), CacheHitCount, ToGenerate.Num(), ExitingNodes.Num(),
				TotalParticles, FPlatformTime::Seconds() - StartTime);
		});
}
#pragma endregion

#pragma region Tier System - PushTierToNiagara
void AGalaxyActor::PushTierToNiagara(const FParticleTierConfig& Config, FParticleTierState& State)
{
	const int32 FrontIdx = State.FrontIdx.load();
	const FBox Bounds = Config.ComputeBounds();
	for (int32 b = 0; b < Config.NiagaraAssets.Num(); ++b)
	{
		UNiagaraComponent* NC = State.NiagaraComponents[b];
		if (NC) NC->SetSystemFixedBounds(Bounds);
		State.Buffers[b][FrontIdx].PushToNiagara(NC, VirtualTraversal);
	}
}
#pragma endregion

#pragma region Tier System - Octree Integration
void AGalaxyActor::InsertTierIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 BufferIdx)
{
	const int32 BufIndex = Config.OctreeInsertBufferIndex;
	if (BufIndex < 0 || BufIndex >= State.Buffers.Num()) return;

	const FNiagaraParticleBuffer& Buffer = State.Buffers[BufIndex][BufferIdx];
	const double TreeExtent = Octree->Extent;

	for (auto& Pair : State.ActiveSlots)
	{
		FSlotEntry& Entry = Pair.Value;
		Entry.InsertedNodes.Empty();
		const int32 Start = Entry.SlotIndex * Config.SlotCapacity;
		const int32 Count = State.SlotCounts[Entry.SlotIndex];
		for (int32 i = 0; i < Count; ++i)
		{
			InsertParticleIntoOctree(Entry, Buffer.Positions[Start + i], Buffer.Extents[Start + i], Entry.SlotIndex, TreeExtent);
		}
	}
}

void AGalaxyActor::InsertSlotIntoOctree(const FParticleTierConfig& Config, FParticleTierState& State, int32 SlotIndex, int32 BufferIdx)
{
	const int32 BufIndex = Config.OctreeInsertBufferIndex;
	if (BufIndex < 0 || BufIndex >= State.Buffers.Num()) return;

	// Find the slot entry that owns this slot index
	FSlotEntry* Entry = nullptr;
	for (auto& Pair : State.ActiveSlots)
	{
		if (Pair.Value.SlotIndex == SlotIndex)
		{
			Entry = &Pair.Value;
			break;
		}
	}
	if (!Entry) return;

	Entry->InsertedNodes.Empty();
	const FNiagaraParticleBuffer& Buffer = State.Buffers[BufIndex][BufferIdx];
	const int32 Start = SlotIndex * Config.SlotCapacity;
	const int32 Count = State.SlotCounts[SlotIndex];
	const double TreeExtent = Octree->Extent;

	for (int32 i = 0; i < Count; ++i)
	{
		InsertParticleIntoOctree(*Entry, Buffer.Positions[Start + i], Buffer.Extents[Start + i], SlotIndex, TreeExtent);
	}
}

void AGalaxyActor::InsertParticleIntoOctree(FSlotEntry& Entry, const FVector& Position, float Extent, int32 SlotIndex, double TreeExtent)
{
	if (Extent <= 0.0f) return;

	FPointData PD = FPointData::MakePointDataFromWorldScale(
		static_cast<double>(Extent) * Params.UnitScale,
		Params.UnitScale,
		static_cast<int64>(TreeExtent));
	PD.Data.ObjectId = SlotIndex;
	PD.Data.TypeId = 1;

	TSharedPtr<FOctreeNode> Node = Octree->InsertPosition(Position, PD.InsertDepth, PD.Data);
	if (Node.IsValid())
		Entry.InsertedNodes.Add(Node);
}
#pragma endregion

#pragma region Tier System - Cell Cache
void AGalaxyActor::CacheCellFromBuffers(const FParticleTierConfig& Config, FParticleTierState& State,
	const FIntVector& Coord, int32 SlotIndex, int32 BufferIdx)
{
	const int32 NumBuffers = Config.NiagaraAssets.Num();
	const int32 LiveCount = State.SlotCounts[SlotIndex];
	FCachedCellData& Cache = State.CellCache.FindOrAdd(Coord);
	Cache.ParticleCount = LiveCount;
	Cache.CenterOffset = FVector::ZeroVector;
	Cache.PerBufferPositions.SetNum(NumBuffers);
	Cache.PerBufferExtents.SetNum(NumBuffers);
	Cache.PerBufferColors.SetNum(NumBuffers);
	Cache.PerBufferRotations.SetNum(NumBuffers);

	const int32 Start = SlotIndex * Config.SlotCapacity;
	for (int32 b = 0; b < NumBuffers; ++b)
	{
		const FNiagaraParticleBuffer& Buf = State.Buffers[b][BufferIdx];
		Cache.PerBufferPositions[b].SetNumUninitialized(LiveCount);
		Cache.PerBufferExtents[b].SetNumUninitialized(LiveCount);
		Cache.PerBufferColors[b].SetNumUninitialized(LiveCount);
		for (int32 i = 0; i < LiveCount; ++i)
		{
			Cache.PerBufferPositions[b][i] = Buf.Positions[Start + i];
			Cache.PerBufferExtents[b][i] = Buf.Extents[Start + i];
			Cache.PerBufferColors[b][i] = Buf.Colors[Start + i];
		}
		if (Buf.Rotations.Num() > 0)
		{
			Cache.PerBufferRotations[b].SetNumUninitialized(LiveCount);
			for (int32 i = 0; i < LiveCount; ++i)
				Cache.PerBufferRotations[b][i] = Buf.Rotations[Start + i];
		}
	}
}

void AGalaxyActor::CullTierCache(const FParticleTierConfig& Config, FParticleTierState& State,
	const FIntVector& NewCenter)
{
	const int32 MaxDist = Config.NeighborhoodRadius + 4;
	TArray<FIntVector> ToRemove;
	for (const auto& Pair : State.CellCache)
	{
		const FIntVector& Coord = Pair.Key;
		int32 Dist = FMath::Max3(
			FMath::Abs(Coord.X - NewCenter.X),
			FMath::Abs(Coord.Y - NewCenter.Y),
			FMath::Abs(Coord.Z - NewCenter.Z));
		if (Dist > MaxDist)
			ToRemove.Add(Coord);
	}
	for (const FIntVector& Coord : ToRemove)
		State.CellCache.Remove(Coord);
}
#pragma endregion

#pragma region Tick
void AGalaxyActor::Tick(float DeltaTime)
{
	// Base class handles parallax offset (actor drift) and debug bounds.
	Super::Tick(DeltaTime);

	if (InitializationState != ELifecycleState::Ready) return;

	// The galaxy uses actor-level drift, not VirtualTraversal-based particle
	// offset. Particles stay at their raw galaxy-local positions relative to
	// the actor. We derive the player's galaxy-local position from the
	// world-space offset for tier streaming purposes.
	VirtualTraversal = GetPlayerLocalPosition();

	// Push positions to Niagara � no offset needed, particles are in actor space
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		const int32 FrontIdx = Tier->FrontIdx.load();
		for (int32 b = 0; b < Tier->NiagaraComponents.Num(); ++b)
		{
			UNiagaraComponent* NC = Tier->NiagaraComponents[b];
			if (!NC || b >= Tier->Buffers.Num()) continue;
			const TArray<FVector>& RelPos = Tier->Buffers[b][FrontIdx].MakeRelativePositions(GetActorLocation());
			UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(NC, NiagaraBufferParams::Positions, RelPos);
		}
	}

	// Stream mid/small tiers
	{
		const FIntVector MidCoord = PositionToGridCoord(VirtualTraversal, MidTierConfig.GridDepth);
		const FIntVector SmallCoord = PositionToGridCoord(VirtualTraversal, SmallTierConfig.GridDepth);

		static int32 TickCount = 0;
		if (++TickCount % 60 == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Galaxy Tick: localPos=(%.0f, %.0f, %.0f) midGrid=(%d,%d,%d) midCenter=(%d,%d,%d) smallGrid=(%d,%d,%d) smallCenter=(%d,%d,%d) midUpdate=%d smallUpdate=%d"),
				VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z,
				MidCoord.X, MidCoord.Y, MidCoord.Z,
				MidTierState.CenterCoord.X, MidTierState.CenterCoord.Y, MidTierState.CenterCoord.Z,
				SmallCoord.X, SmallCoord.Y, SmallCoord.Z,
				SmallTierState.CenterCoord.X, SmallTierState.CenterCoord.Y, SmallTierState.CenterCoord.Z,
				MidTierState.bUpdateInProgress.load() ? 1 : 0,
				SmallTierState.bUpdateInProgress.load() ? 1 : 0);
		}
	}
	UpdateTier(MidTierConfig, MidTierState);
	UpdateTier(SmallTierConfig, SmallTierState);
}
#pragma endregion

#pragma region Star System Pooled Spawn Hooks
void AGalaxyActor::SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !StarSystemActorClass || SpawnedStarSystems.Contains(InNode) ||
		InitializationState != ELifecycleState::Ready)
		return;

	if (StarSystemPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Star System pool exhausted"));
		return;
	}

	AStarSystemActor* System = StarSystemPool.Pop();
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));
	System->ResetForSpawn();
	System->Params.UnitScale = (InNode->Extent * Params.UnitScale) / System->Params.Extent;
	System->SpeedScale = Universe->SpeedScale;
	System->Params.Seed = InNode->Data.ObjectId;
	System->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	System->Params.Rotation = FRandomStream(InNode->Data.ObjectId).GetUnitVector().Rotation();
	System->SetActorLocation(ComputeChildSpawnLocation(InNode->Center, System->Params.UnitScale));
	System->Initialize();
	System->SetActorHiddenInGame(false);
}

void AGalaxyActor::ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AStarSystemActor> SystemToDestroy;
	if (SpawnedStarSystems.RemoveAndCopyValue(InNode, SystemToDestroy))
	{
		AStarSystemActor* PoolSystem = SystemToDestroy.Get();
		if (PoolSystem)
		{
			PoolSystem->ResetForPool();
			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PoolSystem]()
				{
					PoolSystem->Octree->bIsResetting.store(true);
					FPlatformProcess::Sleep(0.05f);
					PoolSystem->Octree = MakeShared<FOctree>(PoolSystem->Params.Extent);
					PoolSystem->Octree->bIsResetting.store(false);
					AsyncTask(ENamedThreads::GameThread, [this, PoolSystem]()
						{
							StarSystemPool.Insert(PoolSystem, 0);
						});
				});
		}
	}
}
#pragma endregion
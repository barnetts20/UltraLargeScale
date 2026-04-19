#pragma region Includes/ForwardDec
#include "SectorActor.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "FVolumeTextureUtils.h"
#include <PointCloudGenerator.h>
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <NiagaraFunctionLibrary.h>
#pragma endregion

#pragma region Constructor
ASectorActor::ASectorActor()
{
	PointCloudNiagara = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/NG_SectorParallaxCloud.NG_SectorParallaxCloud"));
	SectorClusterCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/svo/Sector/NG_SectorClusterCloud.NG_SectorClusterCloud"));
	GalaxyActorClass = AGalaxyActor::StaticClass();
	Octree = MakeShared<FOctree>(Params.Extent);
}
#pragma endregion

#pragma region Initialization
void ASectorActor::BeginPlay()
{
	Super::BeginPlay();

	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				LastFrameOfReferenceLocation = Pawn->GetActorLocation();
				CurrentFrameOfReferenceLocation = LastFrameOfReferenceLocation;
			}
		}
	}

	Initialize();
}

void ASectorActor::InitializeChildPool()
{
	// TODO: Re-enable when galaxy spawning is wired up to SectorActor
}

FastNoise::SmartNode<> ASectorActor::BuildNoise(int InSeed) {
	//Param staging TODO: Wrap in noise param struct
	float masterScale = 1;
	float clusterFalloff = 32;
	float clusterScale = 3;
	float clusterMulti = 50;
	float clusterRemapMax = 1.001;
	float clusterRemapMin = 0;

	float webFalloff = 4;
	float webRemapMin = -.001;
	float webRemapMax = 1.001

	float warpAmp = .25;
	float warpFreq = 1;
	//end param

	auto Voronoi = FastNoise::New<FastNoise::CellularDistance>();
	Voronoi->SetDistanceFunction(FastNoise::DistanceFunction::EuclideanSquared);
	Voronoi->SetReturnType(FastNoise::CellularDistance::ReturnType::Index0);

	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Voronoi);
	SeedOffset->SetOffset(InSeed);

	auto DomainScale = FastNoise::New<FastNoise::DomainScale>();
	DomainScale->SetSource(SeedOffset);
	DomainScale->SetScale(masterScale);

	auto Fbm0 = FastNoise::New<FastNoise::FractalFBm>();
	Fbm0->SetSource(DomainScale);
	Fbm0->SetOctaveCount(3); //Octave count will probs stay hardcoded

	auto Remap0 = FastNoise::New<FastNoise::Remap>();
	Remap0->SetSource(Fbm0);
	Remap0->SetRemap(0, 1, clusterRemapMax, clusterRemapMin);

	auto Pow0 = FastNoise::New<FastNoise::PowInt>();
	Pow0->SetValue(Remap0);
	Pow0->SetPow(clusterFalloff);

	auto Scale0 = FastNoise::New<FastNoise::DomainScale>();
	Scale0->SetSource(Pow0);
	Scale0->SetScale(clusterScale);

	auto Pow1 = FastNoise::New<FastNoise::PowInt>();
	Pow1->SetValue(Fbm0);
	Pow1->SetPow(webFalloff);

	auto Mul0 = FastNoise::New<FastNoise::Multiply>();
	Mul0->SetLHS(Scale0);
	Mul0->SetRHS(Pow1);

	auto Mul1 = FastNoise::New<FastNoise::Multiply>();
	Mul1->SetLHS(Mul0);
	Mul1->SetRHS(clusterMulti);

	auto Remap1 = FastNoise::New<FastNoise::Remap>();
	Remap1->SetSource(Pow1);
	Remap1->SetRemap(0, 1, webRemapMin, webRemapMax);

	auto Add0 = FastNoise::New<FastNoise::Add>();
	Add0->SetLHS(Remap1);
	Add0->SetRHS(Mul1);

	auto Warp0 = FastNoise::New<FastNoise::DomainWarpGradient>();
	Warp0->SetSource(Add0);
	Warp0->SetWarpAmplitude(warpAmp);
	Warp0->SetWarpFrequency(warpFreq);

	return Warp0;
}

void ASectorActor::InitializeData()
{
	double TotalStart = FPlatformTime::Seconds();
	double StepStart;

	// --- Phase 1: Gas density from noise (low-res, alpha channel only) ---
	StepStart = FPlatformTime::Seconds();
	int noiseResolution = 256;

	auto DensityNoise = BuildNoise(69);

	TArray<uint8> LowResData = FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		noiseResolution,
		Params.Extent,
		nullptr,  // Octree density writes disabled — CPU-side DensityVolume is now authoritative
		-1,       // OctreeDepth unused when no octree passed
		1.0f,
		3      // Alpha only (gas density)
	);
	UE_LOG(LogTemp, Log, TEXT("  [InitData] Noise sampling (%d^3): %.3f sec"), noiseResolution, FPlatformTime::Seconds() - StepStart);
	if (InitializationState == ELifecycleState::Pooling) return;

	// Populate the persistent DensityBuffer (final 256^3 volume used both as
	// the GPU pseudo-volume source and as the CPU-side authoritative density
	// field for rejection sampling).
	if (noiseResolution == 256) {
		DensityBuffer = MoveTemp(LowResData);
	}
	else {
		// --- Upscale noise to 256^3 ---
		StepStart = FPlatformTime::Seconds();
		DensityBuffer = FVolumeTextureUtils::UpscaleVolumeData(LowResData, noiseResolution);
		LowResData.Empty();
		UE_LOG(LogTemp, Log, TEXT("  [InitData] Upscale (%d^3 -> 256^3): %.3f sec"), noiseResolution, FPlatformTime::Seconds() - StepStart);
	}

	// Wrap DensityBuffer in the sampler view. Sector noise spans
	// [-Params.Extent, +Params.Extent] centered at sector local origin, so the
	// view's source-space matches sector local space directly.
	DensityVolume = FDensityVolume(
		DensityBuffer,
		FVector::ZeroVector,
		FVector(Params.Extent, Params.Extent, Params.Extent),
		256);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Phase 2: Object generation (VBOs) ---
	StepStart = FPlatformTime::Seconds();
	UniverseGenerator.Params = Params;
	UniverseGenerator.GenerateData(DensityVolume);
	UE_LOG(LogTemp, Log, TEXT("  [InitData] VBO generation (%d clusters): %.3f sec"), UniverseGenerator.GeneratedData.Num(), FPlatformTime::Seconds() - StepStart);

	StepStart = FPlatformTime::Seconds();
	TArray<TSharedPtr<FOctreeNode>> VolumeChunks;
	TArray<TSharedPtr<FOctreeNode>> PointNodes;
	Octree->BulkInsertPositions(UniverseGenerator.GeneratedData, PointNodes, VolumeChunks);
	UE_LOG(LogTemp, Log, TEXT("  [InitData] BulkInsert (%d nodes): %.3f sec"), PointNodes.Num(), FPlatformTime::Seconds() - StepStart);

	if (InitializationState == ELifecycleState::Pooling) return;

	// --- Build Niagara systems from point nodes ---
	// Cluster visualization data is captured here while PointNodes is in
	// scope. UClusterNiagaraSystem takes an owned copy of the data it needs;
	// PointNodes itself can drop at end of scope.
	StepStart = FPlatformTime::Seconds();
	{
		UClusterNiagaraSystem* ClusterSystem = NewObject<UClusterNiagaraSystem>(this);
		ClusterSystem->ConfigureFromPointNodes(PointNodes, Params.Extent);
		NiagaraSystems.Add(ClusterSystem);
	}
	UE_LOG(LogTemp, Log, TEXT("  [InitData] Niagara system config (%d particles): %.3f sec"), PointNodes.Num(), FPlatformTime::Seconds() - StepStart);

	if (InitializationState == ELifecycleState::Pooling) return;

	StepStart = FPlatformTime::Seconds();
	PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(FVolumeTextureUtils::PackToPseudoVolumeLayout(DensityBuffer));// , "/svo/Generated/BakedTest");
	UE_LOG(LogTemp, Log, TEXT("  [InitData] CreatePseudoVolumeTexture: %.3f sec"), FPlatformTime::Seconds() - StepStart);
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeData total: %.3f sec"), FPlatformTime::Seconds() - TotalStart);
}

void ASectorActor::InitializeVolumetric()
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
			VolumetricComponent->RegisterComponent();
			VolumetricComponent->SetWorldScale3D(FVector(2 * Params.Extent));

			UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
				LoadObject<UMaterialInterface>(nullptr, *VolumetricMaterialPath),
				this
			);

			DynamicMaterial->SetTextureParameterValue(FName("VolumeTexture"), PseudoVolumeTexture);
			VolumetricComponent->SetMaterial(0, DynamicMaterial);
			VolumetricComponent->SetVisibility(true);

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	double VolumetricDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::Volumetric initialization took: %.3f seconds"), VolumetricDuration);
}

void ASectorActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();

	// Resolve the player's current position once — used as the initial
	// origin for relative-space particle data across all systems. Running on
	// whatever thread we're already on; GetPlayerController is thread-safe
	// enough for this read-only usage (and systems re-sync on first frame
	// anyway via ApplyParallax).
	FVector PlayerPos = FVector::ZeroVector;
	if (auto* Controller = UGameplayStatics::GetPlayerController(GetWorld(), 0))
	{
		if (APawn* Pawn = Controller->GetPawn())
		{
			PlayerPos = Pawn->GetActorLocation();
		}
	}

	// Initialize every registered system. Each one runs its own async init
	// pipeline (spawn → push arrays → activate) and returns a future; we wait
	// on all of them before returning.
	//
	// Asset selection: right now the cluster system is the only registered
	// entry, so it gets SectorClusterCloud. When more systems are added,
	// they'll either carry their own asset refs internally or get them from
	// additional sector-side UPROPERTYs here.
	UNiagaraSystem* Template = SectorClusterCloud ? SectorClusterCloud : PointCloudNiagara;

	TArray<TFuture<void>> InitFutures;
	InitFutures.Reserve(NiagaraSystems.Num());
	for (const TObjectPtr<UParallaxNiagaraSystem>& System : NiagaraSystems)
	{
		if (!System) continue;
		InitFutures.Add(System->Initialize(this, GetRootComponent(), Template, PlayerPos));
	}
	for (TFuture<void>& Fut : InitFutures)
	{
		Fut.Wait();
	}

	// Proximity system is not yet migrated to UParallaxNiagaraSystem — its
	// streaming lifecycle is more involved and will be ported in a follow-up.
	InitializeProximitySystem();

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeNiagara total duration: %.3f seconds"), TotalDuration);
}
#pragma endregion

#pragma region Player-Centered Parallax
void ASectorActor::ApplyParallaxOffset()
{
	if (InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	bool bHasReference = false;
	FVector CurrentPlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				CurrentPlayerPos = Pawn->GetActorLocation();
				bHasReference = true;
			}
		}
	}

	if (!bHasReference)
	{
		return;
	}

	FVector PlayerDelta = CurrentPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = CurrentPlayerPos;
	CurrentFrameOfReferenceLocation = CurrentPlayerPos;

	ParallaxRatio = GetParentSpeedScale() / GetUnitScale();
	FVector ParallaxOffset = -PlayerDelta * ParallaxRatio;

	// Broadcast parallax to every managed Niagara system. Each system applies
	// the standard User.ParallaxOffset + reseat-at-player itself; streaming
	// systems may additionally do buffer-swap work in their override.
	for (const TObjectPtr<UParallaxNiagaraSystem>& System : NiagaraSystems)
	{
		if (System)
		{
			System->ApplyParallax(ParallaxOffset, CurrentPlayerPos);
		}
	}

	// Proximity system is not yet migrated — still driven inline. Will move
	// into the NiagaraSystems array when ported.
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), ParallaxOffset);
		ProximityNiagaraComponent->SetWorldLocation(CurrentPlayerPos);
	}

	SetActorLocation(GetActorLocation() + ParallaxOffset);
}
#pragma endregion

#pragma region Shutdown
void ASectorActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Explicitly tear down managed Niagara systems before the engine's GC
	// pass runs. Without this, the stale-reference detector at PIE end can
	// observe partially-destroyed components still referenced through our
	// UPROPERTY chain (sector → NiagaraSystems[i] → NiagaraComponent), and
	// the warning printer itself can crash walking that chain.
	for (const TObjectPtr<UParallaxNiagaraSystem>& System : NiagaraSystems)
	{
		if (System)
		{
			System->Shutdown();
		}
	}
	NiagaraSystems.Empty();

	// Proximity component isn't yet part of the managed systems; tear it
	// down the same way.
	if (ProximityNiagaraComponent)
	{
		ProximityNiagaraComponent->Deactivate();
		ProximityNiagaraComponent->DestroyComponent();
		ProximityNiagaraComponent = nullptr;
	}

	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void ASectorActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid() || !GalaxyActorClass || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready)
	{
		return;
	}

	if (GalaxyPool.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Galaxy pool exhausted, consider increasing GalaxyPoolSize"));
		return;
	}

	AGalaxyActor* Galaxy = GalaxyPool.Pop();
	SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(Galaxy));
	Galaxy->ResetForSpawn();

	Galaxy->Params.UnitScale = (InNode->Extent * this->Params.UnitScale) / Galaxy->Params.Extent;
	Galaxy->SpeedScale = SpeedScale;
	Galaxy->Params.Seed = InNode->Data.ObjectId;
	Galaxy->Params.ParentColor = FLinearColor(InNode->Data.Composition);
	FRandomStream RandStream(InNode->Data.ObjectId);
	Galaxy->Params.Rotation = RandStream.GetUnitVector().Rotation();
	Galaxy->SetActorLocation(ComputeChildSpawnLocation(InNode->Center, Galaxy->Params.UnitScale));
	Galaxy->Initialize();
	Galaxy->SetActorHiddenInGame(false);
}

void ASectorActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	if (!InNode.IsValid())
	{
		return;
	}

	TWeakObjectPtr<AGalaxyActor> GalaxyToDestroy;
	if (SpawnedGalaxies.RemoveAndCopyValue(InNode, GalaxyToDestroy))
	{
		AGalaxyActor* PoolGalaxy = GalaxyToDestroy.Get();
		if (PoolGalaxy)
		{
			UE_LOG(LogTemp, Log, TEXT("Resetting galaxy for node with ObjectId: %d"), InNode->Data.ObjectId);
			PoolGalaxy->ResetForPool();

			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, PoolGalaxy]()
				{
					double StartTime = FPlatformTime::Seconds();

					PoolGalaxy->Octree->bIsResetting.store(true);
					FPlatformProcess::Sleep(0.05f);
					PoolGalaxy->Octree = MakeShared<FOctree>(PoolGalaxy->Params.Extent);
					PoolGalaxy->Octree->bIsResetting.store(false);

					double ODuration = FPlatformTime::Seconds() - StartTime;
					UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), ODuration);

					AsyncTask(ENamedThreads::GameThread, [this, PoolGalaxy]()
						{
							GalaxyPool.Insert(PoolGalaxy, 0);
						});
				});
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Galaxy actor was already invalid for node with ObjectId: %d"), InNode->Data.ObjectId);
		}
	}
}
#pragma endregion

#pragma region Proximity Galaxy Streaming

void ASectorActor::InitializeProximitySystem()
{
	double StartTime = FPlatformTime::Seconds();

	const int32 TotalSlots = 27;
	const int32 TotalParticles = TotalSlots * MaxParticlesPerNode;

	// Allocate both buffers
	ProximityBuffers[0].Allocate(TotalParticles);
	ProximityBuffers[1].Allocate(TotalParticles);
	FrontBufferIndex.store(0);

	SlotParticleCounts.SetNumZeroed(TotalSlots);

	// Initialize free slot pool
	FreeSlots.Empty(TotalSlots);
	for (int32 i = TotalSlots - 1; i >= 0; --i)
	{
		FreeSlots.Add(i);
	}
	ActiveNodeSlots.Empty();
	CurrentScanCoord = FIntVector(INT32_MIN);

	// Get player position
	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	// Determine initial scan coord and populate all 27 nodes into front buffer
	FVector LocalPos = PlayerPos - GetActorLocation();
	CurrentScanCoord = PositionToScanCoord(LocalPos);

	int32 NodesPerSide = 1 << ScanDepth;
	FProximityBuffer& InitBuffer = ProximityBuffers[0];

	for (int32 dz = -1; dz <= 1; ++dz)
	{
		for (int32 dy = -1; dy <= 1; ++dy)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				FIntVector NeighborCoord = CurrentScanCoord + FIntVector(dx, dy, dz);

				if (NeighborCoord.X < 0 || NeighborCoord.X >= NodesPerSide ||
					NeighborCoord.Y < 0 || NeighborCoord.Y >= NodesPerSide ||
					NeighborCoord.Z < 0 || NeighborCoord.Z >= NodesPerSide)
				{
					continue;
				}

				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(NeighborCoord, SlotIndex);
				GenerateNodeGalaxies(NeighborCoord, SlotIndex, InitBuffer);
			}
		}
	}

	// Copy front to back so both start identical
	ProximityBuffers[1].Positions = ProximityBuffers[0].Positions;
	ProximityBuffers[1].Extents = ProximityBuffers[0].Extents;
	ProximityBuffers[1].Colors = ProximityBuffers[0].Colors;

	// Create proximity Niagara and push data on game thread
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	AsyncTask(ENamedThreads::GameThread, [this, PlayerPos, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			ProximityNiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
				PointCloudNiagara,
				GetRootComponent(),
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				false,
				true
			);

			if (ProximityNiagaraComponent)
			{
				ProximityNiagaraComponent->SetSystemFixedBounds(
					FBox(FVector(-Params.Extent), FVector(Params.Extent)));
				ProximityNiagaraComponent->SetWorldLocation(PlayerPos);

				PushProximityToNiagara();
				//ProximityNiagaraComponent->ReinitializeSystem(); 
				ProximityNiagaraComponent->Activate(true);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("ASectorActor::InitializeProximitySystem - Failed to create proximity Niagara component"));
			}

			CompletionPromise.SetValue();
		});
	CompletionFuture.Wait();

	//double Duration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("ASectorActor::InitializeProximitySystem took %.3f sec (%d nodes, %d total particles)"),
	//	Duration, ActiveNodeSlots.Num(), TotalParticles);
}

void ASectorActor::UpdateProximityNodes()
{
	if (InitializationState != ELifecycleState::Ready) return;
	if (!ProximityNiagaraComponent) return;

	// If async generation completed a swap, push the new front buffer to Niagara
	if (bProximityNeedsPush.load())
	{
		PushProximityToNiagara();
		ProximityNiagaraComponent->ReinitializeSystem();
		bProximityNeedsPush.store(false);
	}

	// Don't start a new update if one is already in flight
	if (bProximityUpdateInProgress.load()) return;

	// Get player position in octree-local space
	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	FVector LocalPos = PlayerPos - GetActorLocation();
	FIntVector NewScanCoord = PositionToScanCoord(LocalPos);

	if (NewScanCoord == CurrentScanCoord) return;

	// Boundary crossed — kick off async generation into back buffer
	bProximityUpdateInProgress.store(true);
	FIntVector OldScanCoord = CurrentScanCoord;
	CurrentScanCoord = NewScanCoord;

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, OldScanCoord, NewScanCoord]()
		{
			double StartTime = FPlatformTime::Seconds();

			// Copy front buffer to back buffer as starting point.
			// Unchanged slots carry over; only entering slots get regenerated.
			int32 FrontIdx = FrontBufferIndex.load();
			int32 BackIdx = 1 - FrontIdx;
			FProximityBuffer& BackBuffer = ProximityBuffers[BackIdx];

			BackBuffer.Positions = ProximityBuffers[FrontIdx].Positions;
			BackBuffer.Extents = ProximityBuffers[FrontIdx].Extents;
			BackBuffer.Colors = ProximityBuffers[FrontIdx].Colors;

			// Compute entering/exiting sets
			int32 NodesPerSide = 1 << ScanDepth;
			TSet<FIntVector> OldSet;
			TSet<FIntVector> NewSet;

			for (int32 dz = -1; dz <= 1; ++dz)
			{
				for (int32 dy = -1; dy <= 1; ++dy)
				{
					for (int32 dx = -1; dx <= 1; ++dx)
					{
						FIntVector OldNeighbor = OldScanCoord + FIntVector(dx, dy, dz);
						FIntVector NewNeighbor = NewScanCoord + FIntVector(dx, dy, dz);

						if (OldNeighbor.X >= 0 && OldNeighbor.X < NodesPerSide &&
							OldNeighbor.Y >= 0 && OldNeighbor.Y < NodesPerSide &&
							OldNeighbor.Z >= 0 && OldNeighbor.Z < NodesPerSide)
						{
							OldSet.Add(OldNeighbor);
						}

						if (NewNeighbor.X >= 0 && NewNeighbor.X < NodesPerSide &&
							NewNeighbor.Y >= 0 && NewNeighbor.Y < NodesPerSide &&
							NewNeighbor.Z >= 0 && NewNeighbor.Z < NodesPerSide)
						{
							NewSet.Add(NewNeighbor);
						}
					}
				}
			}

			TArray<FIntVector> ExitingNodes;
			TArray<FIntVector> EnteringNodes;

			for (const FIntVector& Coord : OldSet)
			{
				if (!NewSet.Contains(Coord))
				{
					ExitingNodes.Add(Coord);
				}
			}

			for (const FIntVector& Coord : NewSet)
			{
				if (!OldSet.Contains(Coord))
				{
					EnteringNodes.Add(Coord);
				}
			}
			// Free exiting slots and clear their data in the back buffer
			for (const FIntVector& Coord : ExitingNodes)
			{
				int32* SlotPtr = ActiveNodeSlots.Find(Coord);
				if (SlotPtr)
				{
					//UE_LOG(LogTemp, Log, TEXT("Clearing exiting slot %d in back buffer %d"), *SlotPtr, 1 - FrontBufferIndex.load());
					int32 SlotStart = *SlotPtr * MaxParticlesPerNode;
					for (int32 i = 0; i < MaxParticlesPerNode; ++i)
					{
						BackBuffer.Positions[SlotStart + i] = FVector(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
						BackBuffer.Extents[SlotStart + i] = 0.0f;
						BackBuffer.Colors[SlotStart + i] = FLinearColor::Black;
					}

					FreeSlots.Add(*SlotPtr);
					ActiveNodeSlots.Remove(Coord);
				}
			}

			// Generate entering nodes into back buffer
			for (const FIntVector& Coord : EnteringNodes)
			{
				if (FreeSlots.Num() == 0)
				{
					//UE_LOG(LogTemp, Warning, TEXT("ASectorActor::UpdateProximityNodes - No free slots available!"));
					break;
				}

				int32 SlotIndex = FreeSlots.Pop();
				ActiveNodeSlots.Add(Coord, SlotIndex);
				GenerateNodeGalaxies(Coord, SlotIndex, BackBuffer);
			}

			// Atomic swap: back becomes front
			FrontBufferIndex.store(BackIdx);

			// Signal game thread to push the new front buffer on next Tick
			bProximityNeedsPush.store(true);

			//double Duration = FPlatformTime::Seconds() - StartTime;
			//UE_LOG(LogTemp, Log, TEXT("ASectorActor::UpdateProximityNodes (%d exiting, %d entering) took %.3f sec"),
			//	ExitingNodes.Num(), EnteringNodes.Num(), Duration);

			bProximityUpdateInProgress.store(false);
		});
}

FIntVector ASectorActor::PositionToScanCoord(const FVector& InLocalPos) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	int32 X = FMath::Clamp(FMath::FloorToInt((InLocalPos.X + Params.Extent) / NodeSize), 0, NodesPerSide - 1);
	int32 Y = FMath::Clamp(FMath::FloorToInt((InLocalPos.Y + Params.Extent) / NodeSize), 0, NodesPerSide - 1);
	int32 Z = FMath::Clamp(FMath::FloorToInt((InLocalPos.Z + Params.Extent) / NodeSize), 0, NodesPerSide - 1);

	return FIntVector(X, Y, Z);
}

FVector ASectorActor::ScanCoordToCenter(const FIntVector& InCoord) const
{
	int32 NodesPerSide = 1 << ScanDepth;
	double NodeSize = (2.0 * Params.Extent) / NodesPerSide;

	return FVector(
		-Params.Extent + (InCoord.X + 0.5) * NodeSize,
		-Params.Extent + (InCoord.Y + 0.5) * NodeSize,
		-Params.Extent + (InCoord.Z + 0.5) * NodeSize
	);
}

double ASectorActor::GetScanNodeExtent() const
{
	return Params.Extent / (double)(1 << ScanDepth);
}

void ASectorActor::GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FProximityBuffer& InBuffer)
{
	const int32 BufferStart = InSlotIndex * MaxParticlesPerNode;
	const FVector NodeCenter = ScanCoordToCenter(InNodeCoord);
	const double NodeExt = GetScanNodeExtent();

	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InNodeCoord.X), GetTypeHash(InNodeCoord.Y)),
		GetTypeHash(InNodeCoord.Z)
	);
	int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const double GalaxyDepthDivisor = 1000000.0; // TODO: Resolve scale pipeline discrepancy

	int32 ActualCount = 0;
	int32 MaxAttempts = MaxParticlesPerNode * RejectionOversampleFactor;

	for (int32 i = 0; i < MaxAttempts; ++i)
	{
		if (ActualCount >= MaxParticlesPerNode) break;

		FVector Candidate(
			Stream.FRandRange(-NodeExt, NodeExt),
			Stream.FRandRange(-NodeExt, NodeExt),
			Stream.FRandRange(-NodeExt, NodeExt)
		);
		Candidate += NodeCenter;

		if (FMath::Abs(Candidate.X) > Params.Extent ||
			FMath::Abs(Candidate.Y) > Params.Extent ||
			FMath::Abs(Candidate.Z) > Params.Extent)
		{
			continue;
		}

		// Density field is sampled directly from the CPU-side uint8 BGRA8
		// volume buffer (the same bytes that back the GPU pseudo-volume).
		// Candidate is already in sector-local space, matching DensityVolume's
		// source frame. Alpha channel (3) holds gas density per the
		// SampleNoiseToVolume call in InitializeData.
		float Density = DensityVolume.SampleDensityAtLocalPos(Candidate, 3);
		if (Stream.FRand() > Density) continue;

		FVector CompVec = Stream.GetUnitVector();

		float ScaleSample = Stream.FRand();
		double Scale = FPointData::SampleScaleFromDistribution(
			Params.MinGalaxyScale,
			Params.MaxGalaxyScale,
			ScaleSample, Params.ScaleDistributionCurve);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		float FinalExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = Candidate;
		InBuffer.Extents[Idx] = FinalExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	// Dead particles — offscreen
	for (int32 i = ActualCount; i < MaxParticlesPerNode; ++i)
	{
		int32 Idx = BufferStart + i;
		InBuffer.Positions[Idx] = FVector(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
		InBuffer.Extents[Idx] = 0.0f;
		InBuffer.Colors[Idx] = FLinearColor::Black;
	}

	SlotParticleCounts[InSlotIndex] = ActualCount;
	//UE_LOG(LogTemp, Log, TEXT("GenerateNodeGalaxies wrote %d particles to slot %d, buffer extents[%d]=%.1f"),
	//	ActualCount, InSlotIndex, BufferStart, InBuffer.Extents[BufferStart]);
}

void ASectorActor::PushProximityToNiagara()
{
	if (!ProximityNiagaraComponent) return;

	int32 ActiveCount = 0;
	const FProximityBuffer& Front1 = ProximityBuffers[FrontBufferIndex.load()];
	for (int32 i = 0; i < Front1.Extents.Num(); ++i) { if (Front1.Extents[i] > 0.0f) ActiveCount++; }
	//UE_LOG(LogTemp, Log, TEXT("PushProximityToNiagara: FrontIdx=%d, bufferSize=%d, active=%d"),
	//	FrontBufferIndex.load(), Front1.Positions.Num(), ActiveCount);

	FVector PlayerPos = FVector::ZeroVector;
	if (const auto* World = GetWorld())
	{
		if (auto* Controller = UGameplayStatics::GetPlayerController(World, 0))
		{
			if (APawn* Pawn = Controller->GetPawn())
			{
				PlayerPos = Pawn->GetActorLocation();
			}
		}
	}

	FVector ActorPos = GetActorLocation();
	const FProximityBuffer& Front = ProximityBuffers[FrontBufferIndex.load()];

	TArray<FVector> RelativePositions;
	RelativePositions.SetNumUninitialized(Front.Positions.Num());

	for (int32 i = 0; i < Front.Positions.Num(); ++i)
	{
		if (Front.Extents[i] > 0.0f)
		{
			RelativePositions[i] = (Front.Positions[i] + ActorPos) - PlayerPos;
		}
		else
		{
			RelativePositions[i] = FVector::ZeroVector;
		}
	}

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
		ProximityNiagaraComponent, FName("User.Positions"), RelativePositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
		ProximityNiagaraComponent, FName("User.Extents"), Front.Extents);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
		ProximityNiagaraComponent, FName("User.Colors"), Front.Colors);

	ProximityNiagaraComponent->SetWorldLocation(PlayerPos);
}

void ASectorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateProximityNodes();
}

#pragma endregion
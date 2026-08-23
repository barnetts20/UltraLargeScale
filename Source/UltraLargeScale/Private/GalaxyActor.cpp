#pragma region Includes
#include "GalaxyActor.h"
#include "UltraLargeScale.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
#include "FVolumeTextureUtils.h"
#include <DrawDebugHelpers.h>
#include <Kismet/GameplayStatics.h>
#include <NiagaraFunctionLibrary.h>

#include "UObject/ConstructorHelpers.h"

#pragma endregion

#pragma region Constructor/Destructor
AGalaxyActor::AGalaxyActor()
{
	// Galaxies are driven by their parent universe via TickFromParent. UE tick is only enabled for level-placed standalone galaxies (bAutoInitializeOnBeginPlay = true) in BeginPlay.
	PrimaryActorTick.bCanEverTick = true;
	SetActorTickEnabled(false);
	Octree = MakeShared<FOctree>(Params.Extent);

	// Default the placement noise texture to the same asset the material samples.
	//
	// NOT a convenience. Placement is GPU-only and the dispatch samples this texture,
	// so an unset NoiseTexture means the galaxy generates nothing at all. Defaulting
	// here is what keeps that a deliberate act rather than an easy accident; clearing
	// it on an instance still fails, loudly, at the first batch.
	//
	// Resolved here rather than as a UPROPERTY default because FGalaxyParams is a
	// USTRUCT: ConstructorHelpers only runs during UObject construction, so a plain
	// struct cannot reference an asset by path at all. Assigned only when unset, so
	// anything authored on the instance wins.
	//
	// Set NEVER STREAM on this asset. GalaxyDensity.ush reads mip 0 on both paths, but
	// the material handles streaming residency and a compute dispatch does not: if
	// mip 0 is not resident when the dispatch runs it reads whatever is, and placement
	// silently stops matching the render.
	if (Params.NoiseTexture == nullptr)
	{
		static ConstructorHelpers::FObjectFinder<UVolumeTexture> DefaultNoise(
			TEXT("/UltraLargeScale/VolumeTextures/VT_PerlinWorley_Balanced.VT_PerlinWorley_Balanced"));

		if (DefaultNoise.Succeeded())
		{
			Params.NoiseTexture = DefaultNoise.Object;
		}
	}
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
		SetActorTickEnabled(true);
		InitializationState = ELifecycleState::Initializing;
		TWeakObjectPtr<AGalaxyActor> WeakThis(this);
		AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis]()
			{
				if (AGalaxyActor* Self = WeakThis.Get())
					Self->Initialize();
			});
	}
}
#pragma endregion

#pragma region EndPlay
void AGalaxyActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	// Signal any in-flight star system initializations to abort, then clear tracking.
	for (auto& Pair : SpawnedStarSystems)
	{
		if (AStarSystemActor* System = Pair.Value.Get())
			System->InitializationState = ELifecycleState::Pooling;
	}
	SpawnedStarSystems.Empty();

	// Drain any in-flight pushes before destroying the components they may touch.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		FTierStreamingSystem::BeginShutdownDrain(*Tier);

	// Mirror ResetForPool: also wait out an in-flight boundary-cross task - it writes Buffers/SlotEntries/CellCache through teardown otherwise.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		while (Tier->bUpdateInProgress.load())
			FPlatformProcess::Sleep(0.0005f);
	}

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) {
				NC->Deactivate();
				NC->DestroyComponent();
				NC = nullptr;
			}
		}
	}
	TierNiagaraComponents.Empty();
	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Pool Lifecycle
void AGalaxyActor::ResetForPool()
{
	// Return any live star systems BEFORE clearing scan state
	{
		TArray<TSharedPtr<FOctreeNode>> LiveNodes;
		SpawnedStarSystems.GetKeys(LiveNodes);

		for (const TSharedPtr<FOctreeNode>& Node : LiveNodes)
			ReturnStarSystemToPool(Node);
	}

	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	// DRAIN BEFORE FREE: an async boundary-cross task may still own this tier's buffers (it writes Buffers/SlotCounts/SlotEntries and its tail commit touches NiagaraComponents under PushCS). Bar new pushes and wait out any in-flight push (BeginShutdownDrain), then wait for the task itself to clear bUpdateInProgress — its commit bails on bShuttingDown, so the wait is bounded by the remaining generation work.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		FTierStreamingSystem::BeginShutdownDrain(*Tier);
		while (Tier->bUpdateInProgress.load())
		{
			FPlatformProcess::Sleep(0.0005f);
		}
	}

	// The push worker is single-flight and actor-level. BeginShutdownDrain makes its pushes bail (bShuttingDown) but does NOT stop the worker's loop — it can still be alive here. Wait for it to exit (it clears bPushWorkerLive on exit) BEFORE we clear bShuttingDown and free Buffers below: a live worker would otherwise resume on freed memory, and a stale 'live' flag would block the next occupant's push dispatch.
	while (bPushWorkerLive.load(std::memory_order_acquire))
	{
		FPlatformProcess::Sleep(0.0005f);
	}

	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		for (UNiagaraComponent*& NC : Tier->NiagaraComponents)
		{
			if (NC) { NC->Deactivate(); NC->DestroyComponent(); NC = nullptr; }
		}
		Tier->ResetState();
	}

	TierNiagaraComponents.Empty();
	DiagTickCount = 0;

	Super::ResetForPool();
}

void AGalaxyActor::ResetForSpawn()
{
	Super::ResetForSpawn();
	DiagTickCount = 0;
}
#pragma endregion

#pragma region Initialization
void AGalaxyActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();

	// MaxEntityScale is a fixed absolute world-cm value on FGalaxyParams. DeriveScaleRanges cascades it through the tier depth sequence to set MinScale/MaxScale per tier. No per-instance derivation needed.
	GalaxyGenerator.Params = Params;
	GalaxyGenerator.Params.DeriveScaleRanges();
	GalaxyGenerator.Initialize();

	if (InitializationState == ELifecycleState::Pooling) return;

	// The analytic material evaluates the field directly, so nothing reads this
	// texture. Baking it costs a full 256^3 evaluation plus a 4096^2 upscale and
	// upload on every spawn, and roughly 64 MB resident per galaxy -- which is most
	// of what keeps galaxies from being loaded further out.
	//
	// Kept behind a flag rather than deleted: it is an INDEPENDENT cross-check of the
	// CPU implementation. The CPU evaluates and bakes, the old material renders the
	// texture, and that should agree with the new material evaluating the same
	// parameters live.
	if (Params.MaterialParams.bBakeDensityVolume)
	{
		TArray<uint8> VolumeData = GalaxyGenerator.SampleNoiseVolume(Params.MaterialParams.DensityVolumeResolution);

		if (InitializationState == ELifecycleState::Pooling) return;

		PseudoVolumeTexture = FVolumeTextureUtils::CreatePseudoVolumeTexture(FVolumeTextureUtils::PackToPseudoVolumeLayout(FVolumeTextureUtils::UpscaleVolumeData(VolumeData, Params.MaterialParams.DensityVolumeResolution)));
	}

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeData took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();
	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self || Self->InitializationState == ELifecycleState::Pooling)
			{
				CompletionPromise.SetValue();
				return;
			}

			// Resolve every asset up front and bail loudly.
			UStaticMesh* BoxMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/UltraLargeScale/UnitBoxInvertedNormals.UnitBoxInvertedNormals"));
			UMaterialInterface* ParentMat = LoadObject<UMaterialInterface>(nullptr, *Self->VolumetricMaterialPath);
			UVolumeTexture* NoiseTex = LoadObject<UVolumeTexture>(nullptr, *Self->Params.MaterialParams.VolumeNoise);

			// PseudoVolumeTexture is NOT required: the analytic material evaluates the
			// field rather than sampling a bake, so gating on it here would keep the
			// bake on the critical path for a texture nothing reads.
			if (!BoxMesh || !ParentMat)
			{
				UE_LOG(LogTemp, Error, TEXT("AGalaxyActor::InitializeVolumetric - ABORT, required assets unresolved. ") TEXT("mesh=%s material=%s"), BoxMesh ? TEXT("ok") : TEXT("NULL /UltraLargeScale/UnitBoxInvertedNormals"), ParentMat ? TEXT("ok") : *FString::Printf(TEXT("NULL %s"), *Self->VolumetricMaterialPath));
				CompletionPromise.SetValue();
				return;
			}
			if (!NoiseTex)
			{
				UE_LOG(LogTemp, Warning, TEXT("AGalaxyActor::InitializeVolumetric - noise texture '%s' unresolved; ") TEXT("the field will render without modulation or positional warp."), *Self->Params.MaterialParams.VolumeNoise);
			}

			Self->VolumeMaterial = UMaterialInstanceDynamic::Create(ParentMat, Self);
			if (NoiseTex) Self->VolumeMaterial->SetTextureParameterValue(FName("NoiseTex"), NoiseTex);

			// One call, one source of truth. See PushDensityParams.
			Self->PushDensityParams(Self->VolumeMaterial);

			// The colour parameters the pseudovolume material took -- AmbientColor,
			// CoolShift, HotShift, HueVariance, SaturationVariance, Temperature*,
			// ScaleFactor, Warp* -- do not exist on the analytic material, which
			// returns flat white emission. Pushing them would be silent no-ops.
			// Colour belongs to the shading pass, not the density field.

			Self->VolumetricComponent = NewObject<UStaticMeshComponent>(Self);
			Self->VolumetricComponent->SetVisibility(false);
			Self->VolumetricComponent->SetStaticMesh(BoxMesh);
			Self->VolumetricComponent->AttachToComponent(Self->RootComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
			Self->VolumetricComponent->SetAbsolute(true, false, false);
			Self->VolumetricComponent->TranslucencySortPriority = 1;
			Self->VolumetricComponent->SetDepthPriorityGroup(SDPG_World);
			Self->VolumetricComponent->bRenderInDepthPass = false;
			Self->VolumetricComponent->bVisibleInSceneCaptureOnly = Self->IsVirtualSpace(); // Virtual backdrop: "Virtual Space" components get rendered to the backdrop.
			Self->VolumetricComponent->RegisterComponent();
			Self->VolumetricComponent->SetWorldScale3D(FVector(2 * Self->Params.Extent));
			Self->VolumetricComponent->SetMaterial(0, Self->VolumeMaterial);
			Self->VolumetricComponent->SetVisibility(true);

			CompletionPromise.SetValue();
		});

	CompletionFuture.Wait();
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeVolumetric took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::PushDensityParams(UMaterialInstanceDynamic* InMID) const
{
	if (!InMID) return;

	const FGalaxyDensityParams& D = Params.DensityParams;

	// --- LATERAL SCALES ---
	InMID->SetScalarParameterValue(TEXT("ArmRadius"), D.ArmRadius);
	InMID->SetScalarParameterValue(TEXT("DiscRadius"), D.DiscRadius);
	InMID->SetScalarParameterValue(TEXT("BulgeRadius"), D.BulgeRadius);
	InMID->SetScalarParameterValue(TEXT("BackgroundRadius"), D.BackgroundRadius);

	// --- VERTICAL RATIOS ---
	InMID->SetScalarParameterValue(TEXT("ArmVerticalRatio"), D.ArmVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("DiscVerticalRatio"), D.DiscVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("BulgeVerticalRatio"), D.BulgeVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("BackgroundVerticalRatio"), D.BackgroundVerticalRatio);

	// --- LAYER OPTICAL DEPTHS ---
	InMID->SetScalarParameterValue(TEXT("ArmDensity"), D.ArmDensity);
	InMID->SetScalarParameterValue(TEXT("DiscDensity"), D.DiscDensity);
	InMID->SetScalarParameterValue(TEXT("BulgeDensity"), D.BulgeDensity);
	InMID->SetScalarParameterValue(TEXT("BackgroundDensity"), D.BackgroundDensity);

	// --- NOISE RESPONSE (render only: the CPU evaluates the analytic field) ---
	InMID->SetScalarParameterValue(TEXT("ArmNoiseAmount"), D.ArmNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("DiscNoiseAmount"), D.DiscNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("BulgeNoiseAmount"), D.BulgeNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("BackgroundNoiseAmount"), D.BackgroundNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("WarpAmountArms"), D.WarpAmountArms);
	InMID->SetScalarParameterValue(TEXT("WarpAmountDisc"), D.WarpAmountDisc);
	InMID->SetScalarParameterValue(TEXT("WarpAmountBulge"), D.WarpAmountBulge);
	InMID->SetScalarParameterValue(TEXT("WarpAmountBackground"), D.WarpAmountBackground);

	// --- ARM ASYMMETRY ---
	InMID->SetScalarParameterValue(TEXT("ArmAsymPitch"), D.ArmAsymPitch);
	InMID->SetScalarParameterValue(TEXT("ArmAsymPhase"), D.ArmAsymPhase);
	InMID->SetScalarParameterValue(TEXT("ArmAsymDensity"), D.ArmAsymDensity);
	InMID->SetScalarParameterValue(TEXT("ArmAsymLength"), D.ArmAsymLength);
	InMID->SetScalarParameterValue(TEXT("ArmAsymSeed"), D.ArmAsymSeed);

	// --- SPIRAL ---
	InMID->SetScalarParameterValue(TEXT("ArmPitchAngle"), D.ArmPitchAngle);
	InMID->SetScalarParameterValue(TEXT("ArmPitchTightening"), D.ArmPitchTightening);
	InMID->SetScalarParameterValue(TEXT("ArmPhaseOffset"), D.ArmPhaseOffset);
	InMID->SetScalarParameterValue(TEXT("HaloTwistInherit"), D.HaloTwistInherit);
	InMID->SetScalarParameterValue(TEXT("ArmCount"), D.ArmCount);
	InMID->SetScalarParameterValue(TEXT("ArmProfileExponent"), D.ArmProfileExponent);
	InMID->SetScalarParameterValue(TEXT("ArmRadialGrowth"), D.ArmRadialGrowth);
	InMID->SetScalarParameterValue(TEXT("ArmHostFalloff"), D.ArmHostFalloff);

	// --- DISC SHAPE AND ASYMMETRY ---
	InMID->SetScalarParameterValue(TEXT("DiscScaleRatio"), D.DiscScaleRatio);
	InMID->SetScalarParameterValue(TEXT("DiscVerticalFalloff"), D.DiscVerticalFalloff);
	InMID->SetScalarParameterValue(TEXT("DiscFlare"), D.DiscFlare);
	InMID->SetScalarParameterValue(TEXT("DiscWarpAmplitude"), D.DiscWarpAmplitude);
	InMID->SetScalarParameterValue(TEXT("DiscWarpPhase"), D.DiscWarpPhase);
	InMID->SetScalarParameterValue(TEXT("DiscWarpTwist"), D.DiscWarpTwist);
	InMID->SetScalarParameterValue(TEXT("DiscLopsidedAmount"), D.DiscLopsidedAmount);
	InMID->SetScalarParameterValue(TEXT("DiscLopsidedPhase"), D.DiscLopsidedPhase);

	// --- PROFILE EXPONENTS AND BOUNDS ---
	InMID->SetScalarParameterValue(TEXT("BulgeConcentration"), D.BulgeConcentration);
	InMID->SetScalarParameterValue(TEXT("BackgroundConcentration"), D.BackgroundConcentration);
	InMID->SetScalarParameterValue(TEXT("BoundsFadeStart"), D.BoundsFadeStart);

	// --- CENTRAL VOID ---
	InMID->SetScalarParameterValue(TEXT("CentralVoidRadius"), D.CentralVoidRadius);
	InMID->SetScalarParameterValue(TEXT("CentralVoidAmount"), D.CentralVoidAmount);
	InMID->SetScalarParameterValue(TEXT("CentralVoidExponent"), D.CentralVoidExponent);

	// --- NOISE FIELD SHAPE ---
	InMID->SetScalarParameterValue(TEXT("NoiseDiscLateralScale"), D.NoiseDiscLateralScale);
	InMID->SetScalarParameterValue(TEXT("NoiseDiscVerticalScale"), D.NoiseDiscVerticalScale);
	InMID->SetScalarParameterValue(TEXT("NoiseHaloLateralScale"), D.NoiseHaloLateralScale);
	InMID->SetScalarParameterValue(TEXT("NoiseHaloVerticalScale"), D.NoiseHaloVerticalScale);
	InMID->SetScalarParameterValue(TEXT("WarpDiscLateralScale"), D.WarpDiscLateralScale);
	InMID->SetScalarParameterValue(TEXT("WarpDiscVerticalScale"), D.WarpDiscVerticalScale);
	InMID->SetScalarParameterValue(TEXT("WarpHaloLateralScale"), D.WarpHaloLateralScale);
	InMID->SetScalarParameterValue(TEXT("WarpHaloVerticalScale"), D.WarpHaloVerticalScale);
	InMID->SetVectorParameterValue(TEXT("NoiseChannelWeights"), D.NoiseChannelWeights);
	InMID->SetVectorParameterValue(TEXT("NoiseOffset"), FLinearColor(D.NoiseOffset.X, D.NoiseOffset.Y, D.NoiseOffset.Z, 0.0f));
	InMID->SetScalarParameterValue(TEXT("NoiseOctaves"), D.NoiseOctaves);
	InMID->SetScalarParameterValue(TEXT("NoiseRidged"), D.NoiseRidged);

	// --- RENDER ---
	InMID->SetScalarParameterValue(TEXT("MasterDensityScale"), D.MasterDensityScale);
	InMID->SetScalarParameterValue(TEXT("MasterDensityPower"), D.MasterDensityPower);
	InMID->SetScalarParameterValue(TEXT("MaxSteps"), Params.MaterialParams.VolumeMaxSteps);

	// NOT PUSHED: EnableNoise. It is a StaticSwitchParameter, resolved at material
	// compile time -- a MID cannot change one, and setting it silently does nothing.
	// It stays a material-asset setting. Making it per-galaxy would mean converting it
	// back to a scalar, which costs the dead-stripping of the texture reads.
}

void AGalaxyActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();

	const FTierStreamingContext Ctx = BuildStreamingContext();

	FTierStreamingSystem::InitializeTier(Ctx, LargeTierConfig, LargeTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;

	FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	if (InitializationState == ELifecycleState::Pooling) return;

	FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeNiagara total: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Grid Coord Helpers
bool AGalaxyActor::CellOverlapsVolume(const FIntVector& Coord, int32 GridDepth) const
{
	const FVector Center = GridCoordToCenter(Coord, GridDepth);
	const double HalfCell = GetGridCellExtent(GridDepth);
	const double Ext = Params.Extent;
	return (Center.X + HalfCell > -Ext && Center.X - HalfCell < Ext) && (Center.Y + HalfCell > -Ext && Center.Y - HalfCell < Ext) && (Center.Z + HalfCell > -Ext && Center.Z - HalfCell < Ext);
}
#pragma endregion

#pragma region Tier System - BuildTierConfigs
void AGalaxyActor::LoadRuntimeAssets()
{
	// Game thread (Initialize prologue, before async dispatch): LoadObject is not thread-safe, so the Niagara systems BuildTierConfigs reads must load here. Guarded so pooled reuse does not reload.
	if (!GalaxyLargeCloud) GalaxyLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Galaxy/NG_GalaxyLarge.NG_GalaxyLarge"));
	if (!GalaxyMidCloud)   GalaxyMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Galaxy/NG_GalaxyMid.NG_GalaxyMid"));
	if (!GalaxySmallCloud) GalaxySmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Galaxy/NG_GalaxySmall.NG_GalaxySmall"));
}

void AGalaxyActor::BuildTierConfigs()
{
	Params.DeriveScaleRanges();

	// --- Large tier: exhaustive single cell, no streaming ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.GridDepth = Params.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = 0;
	LargeTierConfig.SlotCapacity = Params.LargeTier.SlotCapacity;
	LargeTierConfig.NiagaraAssets = { GalaxyLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.TierIndex = 0;
	// The large tier goes through the SAME GPU path as the others. It differs only in
	// where its cells come from -- an active set from the cull prepass rather than a
	// grid neighbourhood -- and in rejecting against each cell's own peak density
	// instead of the global reference. Both are data, not a second code path.
	//
	// NO GenerateCallback. This layer is GPU-only: the batch callback is the whole
	// generation path, and a false return means these slots are deliberately blanked
	// rather than filled by something that cannot read the volume texture.
	LargeTierConfig.GenerateBatchCallback =
		[this](const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutCounts) -> bool
		{
			TArray<GalaxyDataGenerator::FTierBatchCell> Cells;

			if (!GalaxyGenerator.BuildLargeTierCells(Slots, Cells))
			{
				return false;
			}

			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, LargeTierState.Buffers[0], Params.LargeTier, 0, OutCounts);
		};

	// --- Mid tier: neighborhood streaming ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.GridDepth = Params.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.MidTier.SlotCapacity;
	MidTierConfig.NiagaraAssets = { GalaxyMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.TierIndex = 1;
	MidTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) { return !CellOverlapsVolume(Coord, MidTierConfig.GridDepth); };
	// The only generation path. Returns false when it cannot run -- no texture, readback
	// timed out -- and blanks the affected slots on the way out rather than leaving the
	// previous occupant's entities in them.
	//
	// All three tiers run this same path. They differ only in where their cells come
	// from -- a streamed neighbourhood here, a culled active set for the large tier --
	// which is data, not a second code path.
	MidTierConfig.GenerateBatchCallback =
		[this](const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutCounts) -> bool
		{
			// Centres come from the SAME helper the per-slot callback below uses.
			// Deriving them anywhere else is how every candidate ended up in the wrong
			// place and every batch came back with nothing accepted -- the grid belongs
			// to the actor, so the actor states where each cell is.
			const double CellExt = GetGridCellExtent(MidTierConfig.GridDepth);

			TArray<GalaxyDataGenerator::FTierBatchCell> Cells;
			Cells.Reserve(Slots.Num());

			for (const TPair<FIntVector, int32>& Slot : Slots)
			{
				GalaxyDataGenerator::FTierBatchCell Cell;
				Cell.Coord = Slot.Key;
				Cell.SlotIndex = Slot.Value;
				Cell.Centre = GridCoordToCenter(Slot.Key, MidTierConfig.GridDepth);
				Cell.HalfExtent = CellExt;
				Cells.Add(Cell);
			}

			// Per-cell rejection envelope, and a candidate budget weighted by it.
			//
			// These two tiers used to pass DensityReference = 0, meaning the global
			// reference. Against a field that runs four decades that made acceptance
			// bimodal rather than low: dense cells sat at probability 1 everywhere,
			// accepted nearly every candidate, overflowed their run and rendered as
			// structureless mush, while sparse cells starved. The large tier has
			// rejected against its own cell peaks since its cull prepass existed; this
			// is the same treatment, from the same helper.
			GalaxyGenerator.ApplyCellEnvelopes(Cells, Params.MidTier);

			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, MidTierState.Buffers[0], Params.MidTier, 7, OutCounts);
		};

	// --- Small tier: neighborhood streaming ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.GridDepth = Params.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.SmallTier.SlotCapacity;
	SmallTierConfig.NiagaraAssets = { GalaxySmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) { return !CellOverlapsVolume(Coord, SmallTierConfig.GridDepth); };
	// The only generation path. Returns false when it cannot run -- no texture, readback
	// timed out -- and blanks the affected slots on the way out rather than leaving the
	// previous occupant's entities in them.
	//
	// All three tiers run this same path. They differ only in where their cells come
	// from -- a streamed neighbourhood here, a culled active set for the large tier --
	// which is data, not a second code path.
	SmallTierConfig.GenerateBatchCallback =
		[this](const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutCounts) -> bool
		{
			// Centres come from the SAME helper the per-slot callback below uses.
			// Deriving them anywhere else is how every candidate ended up in the wrong
			// place and every batch came back with nothing accepted -- the grid belongs
			// to the actor, so the actor states where each cell is.
			const double CellExt = GetGridCellExtent(SmallTierConfig.GridDepth);

			TArray<GalaxyDataGenerator::FTierBatchCell> Cells;
			Cells.Reserve(Slots.Num());

			for (const TPair<FIntVector, int32>& Slot : Slots)
			{
				GalaxyDataGenerator::FTierBatchCell Cell;
				Cell.Coord = Slot.Key;
				Cell.SlotIndex = Slot.Value;
				Cell.Centre = GridCoordToCenter(Slot.Key, SmallTierConfig.GridDepth);
				Cell.HalfExtent = CellExt;
				Cells.Add(Cell);
			}

			// Per-cell rejection envelope, and a candidate budget weighted by it.
			//
			// These two tiers used to pass DensityReference = 0, meaning the global
			// reference. Against a field that runs four decades that made acceptance
			// bimodal rather than low: dense cells sat at probability 1 everywhere,
			// accepted nearly every candidate, overflowed their run and rendered as
			// structureless mush, while sparse cells starved. The large tier has
			// rejected against its own cell peaks since its cull prepass existed; this
			// is the same treatment, from the same helper.
			GalaxyGenerator.ApplyCellEnvelopes(Cells, Params.SmallTier);

			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, SmallTierState.Buffers[0], Params.SmallTier, 13, OutCounts);
		};

	// Shared bounds convention — see the derivation in AUniverseActor::BuildTierConfigs. Tight half-bound is (2R+2) * CellHalfExtent; we provision 2 * (2R+1) to match the universe tiers. The previous (2R+1) * CellHalfExtent under-bounded by up to one half-cell when VT sat near a cell boundary, risking edge-cell culling pops.
	auto MakeBounds = [this](const FParticleTierConfig& Config) {
		const double HalfExt = GetGridCellExtent(Config.GridDepth) * (2 * Config.NeighborhoodRadius + 1) * 2.0;
		return FBox(FVector(-HalfExt), FVector(HalfExt));
		};

	LargeTierConfig.ComputeBounds = [this]() { return FBox(FVector(-Params.Extent), FVector(Params.Extent)); };
	MidTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(MidTierConfig); };
	SmallTierConfig.ComputeBounds = [this, MakeBounds]() { return MakeBounds(SmallTierConfig); };
}
#pragma endregion

#pragma region Tier System - BuildStreamingContext
FTierStreamingContext AGalaxyActor::BuildStreamingContext() const
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

#pragma region Tick
void AGalaxyActor::Tick(float DeltaTime)
{
	AActor::Tick(DeltaTime);
	if (InitializationState != ELifecycleState::Ready) return;

	FVector CurrentPlayerPos;
	if (!GetPlayerLocation(GetWorld(), CurrentPlayerPos)) return;

	TickFromParent(DeltaTime, CurrentPlayerPos);
}

void AGalaxyActor::ApplyParallaxOffset(const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Galaxy::ApplyParallaxOffset");
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double ActiveSpeedScale = GetEffectiveSpeedScale();
	const double Ratio = (Params.UnitScale > 0.0) ? (ActiveSpeedScale / Params.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(InPlayerPos);

	if (VolumetricComponent)
		VolumetricComponent->SetWorldLocation(InPlayerPos - VirtualTraversal);

	// Publish EVERY frame so a late-completing full push still uses current VT.
	PublishLatestVT(VirtualTraversal);

	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		LastPushedVirtualTraversal = VirtualTraversal;

		FTierStreamingSystem::PushTierVT({ &LargeTierState, &MidTierState, &SmallTierState }, [this] { return ReadLatestVT(); });
	}
}

// Coalesced, single-flight per-frame push (see AUniverseActor::SchedulePush).
void AGalaxyActor::SchedulePush()
{
	bPushDirty.store(true, std::memory_order_release);
	if (bPushWorkerLive.exchange(true)) return;
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this]()
		{
			while (true)
			{
				bPushDirty.store(false, std::memory_order_relaxed);
				FTierStreamingSystem::PushTierVT({ &LargeTierState, &MidTierState, &SmallTierState }, [this] { return ReadLatestVT(); });
				if (!bPushDirty.load(std::memory_order_acquire))
				{
					bPushWorkerLive.store(false, std::memory_order_release);
					if (!bPushDirty.load(std::memory_order_acquire)) return;
					if (bPushWorkerLive.exchange(true)) return;
				}
			}
		});
}

void AGalaxyActor::TickFromParent(float DeltaTime, const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Galaxy::TickFromParent");
	if (InitializationState != ELifecycleState::Ready) return;

	// --- VirtualTraversal accumulation ---
	ApplyParallaxOffset(InPlayerPos);

	// --- Process pending spawn-scan results ---
	// VirtualTraversal is resolved for this frame, so SpawnStarSystemFromPool sees the correct parallax state. Mirrors UniverseActor::Tick ordering.
	ProcessPendingScanResults();

	// --- Drive active star systems and handle deferred placement ---
	for (auto& Pair : SpawnedStarSystems)
	{
		AStarSystemActor* System = Pair.Value.Get();
		if (!System) continue;

		if (System->InitializationState == ELifecycleState::Ready)
		{
			if (System->bPendingPlacement)
				FinalizeStarSystemPlacement(System);

			System->TickFromParent(DeltaTime, InPlayerPos);
		}
	}

	// --- Tier streaming ---
	const FTierStreamingContext Ctx = BuildStreamingContext();
	FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);

	if (IsDebug) DrawDebugBounds();

	if (IsDebug && ++DiagTickCount % 60 == 0)
	{
		const FIntVector MidCoord = PositionToGridCoord(VirtualTraversal, MidTierConfig.GridDepth);
		const FIntVector SmallCoord = PositionToGridCoord(VirtualTraversal, SmallTierConfig.GridDepth);
		UE_LOG(LogTemp, Verbose, TEXT("Galaxy [%s] VT=(%.0f,%.0f,%.0f) midGrid=(%d,%d,%d)->(%d,%d,%d) smallGrid=(%d,%d,%d)->(%d,%d,%d) updates=%d/%d"), *GetName(), VirtualTraversal.X, VirtualTraversal.Y, VirtualTraversal.Z, MidCoord.X, MidCoord.Y, MidCoord.Z, MidTierState.CenterCoord.X, MidTierState.CenterCoord.Y, MidTierState.CenterCoord.Z, SmallCoord.X, SmallCoord.Y, SmallCoord.Z, SmallTierState.CenterCoord.X, SmallTierState.CenterCoord.Y, SmallTierState.CenterCoord.Z, MidTierState.bUpdateInProgress.load() ? 1 : 0, SmallTierState.bUpdateInProgress.load() ? 1 : 0);
	}
}
#pragma endregion

#pragma region Spawn Range Scanning
void AGalaxyActor::RequestScan()
{
	SVO_GT_SCOPE("Galaxy::RequestScan");
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;

	// GT snapshot of the tree ref (one atomic ref-count bump)
	TSharedPtr<FOctree> TreeSnapshot = Octree;
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos, TreeSnapshot]()
		{
			AGalaxyActor* Self = WeakThis.Get();
			if (!Self || !TreeSnapshot.IsValid()) return;

			const TArray<TSharedPtr<FOctreeNode>> NearbyArray = TreeSnapshot->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray, TreeSnapshot]()
				{
					AGalaxyActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->bSpawnScanInProgress.store(false);
					// Same guard as AUniverseActor::RequestScan: if the tree was swapped while this scan was in flight (pool return installs a fresh tree), these nodes belong to a retired tree — for a pooled-and-respawned galaxy they are the PREVIOUS identity's nodes, and processing them would spawn ghost star systems with the old seeds. Drop them; the next interval rescans the live tree.
					if (InnerSelf->Octree != TreeSnapshot) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
				});
		});
}

bool AGalaxyActor::IsPlayerInsideBounds() const
{
	if (!Octree.IsValid()) return false;
	const double E = Octree->Extent;
	return FMath::Abs(VirtualTraversal.X) <= E && FMath::Abs(VirtualTraversal.Y) <= E && FMath::Abs(VirtualTraversal.Z) <= E;
}

void AGalaxyActor::ProcessPendingScanResults()
{
	SVO_GT_SCOPE("Galaxy::ProcessPendingScanResults");
	if (!bHasPendingScanResults) return;
	bHasPendingScanResults = false;

	TSet<TSharedPtr<FOctreeNode>> NearbySet(PendingScanResults);
	PendingScanResults.Empty();

	for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
	{
		if (!TrackedSpawnNodes.Contains(Node))
		{
			LogSpawnNodeEnter(Node);
			SpawnStarSystemFromPool(Node);
		}
		if (bDebugDrawSpawnNodes) DebugDrawSpawnNode(Node);
	}

	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedSpawnNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
		ReturnStarSystemToPool(Node);
	}

	TrackedSpawnNodes = MoveTemp(NearbySet);
}

void AGalaxyActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::SpawnScan ENTER — center=(%.1f,%.1f,%.1f) extent=%.2f depth=%d seed=%d tier=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, InNode->Data.Seed, InNode->Data.TypeId);
}

void AGalaxyActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::SpawnScan EXIT  — center=(%.1f,%.1f,%.1f) extent=%.2f depth=%d seed=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, InNode->Data.Seed);
}

void AGalaxyActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	// Rendered world position = PlayerPos + NodeCenter - VirtualTraversal
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	DrawDebugBox(World, NodeCenterWorld, FVector(InNode->Extent), FColor::Cyan, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion

#pragma region Star System Pooled Spawn Hooks
void AGalaxyActor::SpawnStarSystemFromPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Galaxy::SpawnStarSystemFromPool");
	if (!InNode.IsValid() || SpawnedStarSystems.Contains(InNode) || InitializationState != ELifecycleState::Ready) return;

	UActorPoolManager* PM = GetPoolManager();
	if (!PM) return;

	AStarSystemActor* System = PM->Acquire<AStarSystemActor>();
	if (!System) return;

	System->Galaxy = this;
	SpawnedStarSystems.Add(InNode, TWeakObjectPtr<AStarSystemActor>(System));

	// Resolve the real particle position/extent from the tier buffer (octree node center is a quantized approximation). SINGLE-BUFFER READ GUARD as before.
	const int32 TierIndex = FMath::Clamp(InNode->Data.TypeId, 0, 2);
	FParticleTierState* TierStates[] = { &LargeTierState,  &MidTierState,  &SmallTierState };
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	FVector ParticlePos = InNode->Center;
	float ParticleExtent = static_cast<float>(InNode->Extent);
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	if (AbsIdx >= 0 && MatchedState.Buffers.Num() > 0 && !MatchedState.bUpdateInProgress.load())
	{
		const FNiagaraParticleBuffer& Buf = MatchedState.Buffers[0];
		ParticlePos = Buf.Positions[AbsIdx];
		ParticleExtent = Buf.Extents[AbsIdx];
	}

	// Config: bounds (owned by the Universe) -> Generate -> context overlay (Seed, ParentColor). Universe is guaranteed non-null here (GetPoolManager resolved it).
	FStarSystemParams P = FStarSystemParamBounds::Generate(Universe->StarSystemParamBounds, InNode->Data.Seed).ApplyContext(*InNode);

	// Derived Extent (cross-layer): galaxy UnitScale * BoundsScaleMultiplier / system UnitScale. Unchanged from before.
	{
		const double DerivedExtent = (static_cast<double>(ParticleExtent) * Params.UnitScale * P.BoundsScaleMultiplier) / P.UnitScale;
		P.Extent = FMath::Clamp(DerivedExtent, P.MinDerivedExtent, P.MaxDerivedExtent);
		if (P.Extent != DerivedExtent)
		{
			UE_LOG(LogTemp, Warning, TEXT("AGalaxyActor::SpawnStarSystemFromPool - derived extent %.3e clamped to %.3e; ") TEXT("retune FGalaxyParams::StarSystemUnitScale or the clamp bounds."), DerivedExtent, P.Extent);
		}
	}

	// Rotation is seed-derived and parent-owned for now (folds into Generate in step E).
	P.Rotation = FRandomStream(InNode->Data.Seed).GetUnitVector().Rotation();

	// Typed re-init: sets Params, arms deferred placement (PendingNodeCenter = ParticlePos), hides, and runs the async init chain. FinalizeStarSystemPlacement positions/unhides once Ready, exactly as before.
	System->ReInit(P, FTransform(ParticlePos));

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::SpawnStarSystemFromPool - inert=%d particle=(%.1f,%.1f,%.1f) extent=%.2f unitScale(const)=%.4e derivedExtent=%.4e seed=%d (deferred)"), PM->NumInert(AStarSystemActor::StaticClass()), ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent, P.UnitScale, P.Extent, P.Seed);
}

void AGalaxyActor::FinalizeStarSystemPlacement(AStarSystemActor* System)
{
	SVO_GT_SCOPE("Galaxy::FinalizeStarSystemPlacement");
	// Mirrors AUniverseActor::FinalizeGalaxyPlacement exactly. Called on the first tick after async init completes, so VirtualTraversal and CurrentFrameOfReferenceLocation are resolved for this frame.

	const FVector SpawnLoc = ComputeChildSpawnLocation(System->PendingNodeCenter, System->Params.UnitScale);
	System->SetActorLocation(SpawnLoc);
	System->LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	System->CurrentFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;

	// VT_initial = PlayerPos - SpawnLoc, so that: Rendered pos = PlayerPos + (LocalPos - VT) = SpawnLoc + LocalPos matches the galaxy's particle sprite position.
	System->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;

	// Seed the push threshold baseline too, mirroring FinalizeGalaxyPlacement, so the first tick doesn't fire a spurious full-delta push.
	System->LastPushedVirtualTraversal = System->VirtualTraversal;

	System->SetActorHiddenInGame(false);
	System->bPendingPlacement = false;

	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::FinalizeStarSystemPlacement — spawnLoc=(%.1f,%.1f,%.1f) VT=(%.1f,%.1f,%.1f)"), SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z, System->VirtualTraversal.X, System->VirtualTraversal.Y, System->VirtualTraversal.Z);
}

void AGalaxyActor::ReturnStarSystemToPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Galaxy::ReturnStarSystemToPool");
	if (!InNode.IsValid()) return;

	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	TWeakObjectPtr<AStarSystemActor> WeakSystem;
	if (!SpawnedStarSystems.RemoveAndCopyValue(InNode, WeakSystem)) return;
	AStarSystemActor* PoolSystem = WeakSystem.Get();
	if (!PoolSystem) return;

	// Abort signal for an in-flight async init chain — checked between phases and live (via GetLiveState) inside InitializeTier.
	PoolSystem->InitializationState = ELifecycleState::Pooling;

	if (!PoolSystem->bInitInProgress.load())
	{
		// FAST PATH: no init chain in flight. Race-free on the GT — the flag is raised on the GT in Initialize() before dispatch.
		PoolSystem->ResetForPool();
		FinishStarSystemPoolReturn(WeakSystem);
		return;
	}

	// DEFERRED RETURN: the init chain still owns the tier buffers — its generation writes are not covered by bUpdateInProgress, so ResetForPool would free live arrays under the workers. We also cannot wait on the GAME THREAD: the chain rendezvouses with the GT (component spawns), so a GT spin deadlocks. Wait it out on a worker, then finish teardown through the normal GT path. The system cannot be re-spawned meanwhile — it re-enters the pool only at the end of FinishStarSystemPoolReturn.
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakSystem]()
		{
			while (true)
			{
				AStarSystemActor* S = WeakSystem.Get();
				if (!S) return;
				if (!S->bInitInProgress.load()) break;
				FPlatformProcess::Sleep(0.001f);
			}
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakSystem]()
				{
					AGalaxyActor* Self = WeakThis.Get();
					AStarSystemActor* S = WeakSystem.Get();
					if (!Self || !S) return;
					S->ResetForPool();
					Self->FinishStarSystemPoolReturn(WeakSystem);
				});
		});
}

void AGalaxyActor::FinishStarSystemPoolReturn(TWeakObjectPtr<AStarSystemActor> WeakSystem)
{
	TWeakObjectPtr<AGalaxyActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakSystem]()
		{
			AStarSystemActor* AsyncSystem = WeakSystem.Get();
			if (!AsyncSystem) return;
			AsyncSystem->Octree->bIsResetting.store(true);
			FPlatformProcess::Yield();
			// Build the fresh tree in a LOCAL; the MEMBER swap happens on the game thread below — a background assign races GT readers of the TSharedPtr (BuildStreamingContext, spawn-scan snapshot). The old tree keeps bIsResetting raised and dies with its last shared ref.
			TSharedPtr<FOctree> FreshTree = MakeShared<FOctree>(AsyncSystem->Params.Extent);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakSystem, FreshTree]()
				{
					AGalaxyActor* Self = WeakThis.Get();
					AStarSystemActor* InnerSystem = WeakSystem.Get();
					if (!InnerSystem) return;
					InnerSystem->Octree = FreshTree;
					if (Self) { if (UActorPoolManager* PM = Self->GetPoolManager()) PM->ReturnPrepared(InnerSystem); }
				});
		});
}
#pragma endregion

#pragma region Pooled Re-Init
void AGalaxyActor::ReInit(const FGalaxyParams& InParams, const FTransform& InXform)
{
	bAutoInitializeOnBeginPlay = false;
	Params = InParams;
	// Deferred placement: the real world transform is sampled at finalize time from this frame's resolved VirtualTraversal, so stash the node center now and let FinalizeGalaxyPlacement position/unhide once async init reaches Ready.
	PendingNodeCenter = InXform.GetLocation();
	bPendingPlacement = true;
	SetActorHiddenInGame(true);
	Initialize();
}
#pragma endregion
#pragma region Includes
#include "GalaxyActor.h"
#include "UltraLargeScale.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "StarSystemActor.h"
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

	// Acquire the fallback placement noise texture -- the same asset the material
	// samples. APPLIED IN InitializeData, NOT HERE.
	//
	// NOT a convenience. Placement is GPU-only and the dispatch samples this texture,
	// so an unset NoiseTexture means the galaxy generates nothing at all, silently.
	//
	// Writing it into Params here would accomplish nothing for a POOLED galaxy:
	// ReInit assigns Params wholesale from the universe's resolved params, so the
	// constructor's value is gone before the first batch. It survives only for a
	// level-placed actor, which is the path that matters least. So the reference is
	// parked on the actor and applied after Params is assigned, where every path
	// passes through.
	//
	// Acquired here regardless because ConstructorHelpers only runs during UObject
	// construction -- and keeping it means the asset is cooked rather than left to a
	// runtime LoadObject that may not find it.
	//
	// Set NEVER STREAM on this asset. GalaxyDensityCore.ush reads mip 0 on both paths, but
	// the material handles streaming residency and a compute dispatch does not: if
	// mip 0 is not resident when the dispatch runs it reads whatever is, and placement
	// silently stops matching the render.
	{
		static ConstructorHelpers::FObjectFinder<UVolumeTexture> DefaultNoise(
			TEXT("/UltraLargeScale/VolumeTextures/VT_PerlinWorley_Balanced.VT_PerlinWorley_Balanced"));

		if (DefaultNoise.Succeeded())
		{
			DefaultNoiseTexture = DefaultNoise.Object;
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

	// Bake the resolved texture back into Params so the generator, which only sees
	// Params, samples exactly what the material samples. HERE rather than in the
	// constructor because ReInit assigns Params wholesale.
	if (Params.Procedural.NoiseTexture == nullptr)
	{
		Params.Procedural.NoiseTexture = ResolveNoiseTexture();
		if (!Params.Procedural.NoiseTexture)
		{
			// Nothing behind this. The tier batches will fail closed and log, but they
			// will do it three times per galaxy with no indication of the cause.
			UE_LOG(LogTemp, Error,
				TEXT("AGalaxyActor::InitializeData - NoiseTexture is unset and the ")
				TEXT("fallback asset did not resolve. Placement is GPU-only, so this ")
				TEXT("galaxy will generate NO entities at any tier."));
		}
	}

	// MaxEntityScale is a fixed absolute world-cm value on FGalaxyConfigParams, the
	// same for every galaxy. DeriveScaleRanges cascades it through the tier depth
	// sequence to set MinScale/MaxScale per tier. No per-instance derivation needed.
	GalaxyGenerator.Params = Params;
	GalaxyGenerator.Params.Config.DeriveScaleRanges();
	GalaxyGenerator.Initialize();

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
			// THE SAME OBJECT the compute path samples, not a second lookup by path.
			// See AGalaxyActor::ResolveNoiseTexture.
			UVolumeTexture* NoiseTex = Self->ResolveNoiseTexture();

			if (!BoxMesh || !ParentMat)
			{
				UE_LOG(LogTemp, Error, TEXT("AGalaxyActor::InitializeVolumetric - ABORT, required assets unresolved. ") TEXT("mesh=%s material=%s"), BoxMesh ? TEXT("ok") : TEXT("NULL /UltraLargeScale/UnitBoxInvertedNormals"), ParentMat ? TEXT("ok") : *FString::Printf(TEXT("NULL %s"), *Self->VolumetricMaterialPath));
				CompletionPromise.SetValue();
				return;
			}
			if (!NoiseTex)
			{
				UE_LOG(LogTemp, Warning, TEXT("AGalaxyActor::InitializeVolumetric - no noise texture resolved; ") TEXT("the field will render without modulation or positional warp, and will not match placement."));
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

			// WHAT THE MARCH ACTUALLY COSTS, which is now a property of the parameters
			// alone rather than of where the camera is -- so it can be logged once at init
			// and stay true. The budget and the count it resolves to differ by
			// ln(1+g)/g and the gap is wide enough to surprise: at growth 2 a budget of
			// 192 buys about 105 steps.
			//
			// The base step is here beside them because it is what MinFeatureStep is
			// compared against: when half the narrowest layer thickness exceeds this
			// number, the field's own floor binds and further budget buys nothing. The
			// floor itself is NOT logged here -- it is derived inside
			// MakeGalaxyDensityParams, which this translation unit does not compile, and
			// recomputing it locally would be exactly the second home for one value that
			// every alignment bug so far has turned out to be.
			const FGalaxyMaterialParams& MP = Self->Params.Config.MaterialParams;
			UE_LOG(LogTemp, Log,
				TEXT("AGalaxyActor::InitializeVolumetric - march budget %.0f at growth ")
				TEXT("%.2f resolves to ~%.0f actual steps; base step through the centre ")
				TEXT("%.5f."),
				MP.VolumeStepBudget, MP.VolumeStepGrowth, MP.GetEffectiveStepCount(),
				2.0f / FMath::Max(MP.VolumeStepBudget, 1.0f));

			CompletionPromise.SetValue();
		});

	CompletionFuture.Wait();
	UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::InitializeVolumetric took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

void AGalaxyActor::PushDensityParams(UMaterialInstanceDynamic* InMID) const
{
	if (!InMID) return;

	const FGalaxyProceduralParams& D = Params.Procedural;

	// --- LATERAL SCALES ---
	InMID->SetScalarParameterValue(TEXT("ArmRadius"), D.Arms.ArmRadius);
	InMID->SetScalarParameterValue(TEXT("DiscRadius"), D.Disc.DiscRadius);
	InMID->SetScalarParameterValue(TEXT("BulgeRadius"), D.Bulge.BulgeRadius);
	// Pushed rather than left to the material's own default, so the render cannot
	// disagree with placement about how far the background layer reaches.
	InMID->SetScalarParameterValue(TEXT("BackgroundRadius"),
		FGalaxyBackgroundParams::BackgroundRadius);

	// --- VERTICAL RATIOS ---
	InMID->SetScalarParameterValue(TEXT("ArmVerticalRatio"), D.Arms.ArmVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("DiscVerticalRatio"), D.Disc.DiscVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("BulgeVerticalRatio"), D.Bulge.BulgeVerticalRatio);
	InMID->SetScalarParameterValue(TEXT("BackgroundVerticalRatio"), D.Background.BackgroundVerticalRatio);

	// --- LAYER OPTICAL DEPTHS ---
	InMID->SetScalarParameterValue(TEXT("ArmDensity"), D.Arms.ArmDensity);
	InMID->SetScalarParameterValue(TEXT("DiscDensity"), D.Disc.DiscDensity);
	InMID->SetScalarParameterValue(TEXT("BulgeDensity"), D.Bulge.BulgeDensity);
	InMID->SetScalarParameterValue(TEXT("BackgroundDensity"), D.Background.BackgroundDensity);

	// --- NOISE RESPONSE (render only: the CPU evaluates the analytic field) ---
	InMID->SetScalarParameterValue(TEXT("ArmNoiseAmount"), D.Arms.ArmNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("DiscNoiseAmount"), D.Disc.DiscNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("BulgeNoiseAmount"), D.Bulge.BulgeNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("BackgroundNoiseAmount"), D.Background.BackgroundNoiseAmount);
	InMID->SetScalarParameterValue(TEXT("WarpAmountArms"), D.Arms.WarpAmountArms);
	InMID->SetScalarParameterValue(TEXT("WarpAmountDisc"), D.Disc.WarpAmountDisc);
	InMID->SetScalarParameterValue(TEXT("WarpAmountBulge"), D.Bulge.WarpAmountBulge);
	InMID->SetScalarParameterValue(TEXT("WarpAmountBackground"), D.Background.WarpAmountBackground);

	// --- ARM ASYMMETRY ---
	InMID->SetScalarParameterValue(TEXT("ArmAsymPitch"), D.Arms.ArmAsymPitch);
	InMID->SetScalarParameterValue(TEXT("ArmAsymPhase"), D.Arms.ArmAsymPhase);
	InMID->SetScalarParameterValue(TEXT("ArmAsymDensity"), D.Arms.ArmAsymDensity);
	InMID->SetScalarParameterValue(TEXT("ArmAsymLength"), D.Arms.ArmAsymLength);
	InMID->SetScalarParameterValue(TEXT("ArmAsymSeed"), D.Arms.ArmAsymSeed);

	// --- SPIRAL ---
	InMID->SetScalarParameterValue(TEXT("ArmPitchAngle"), D.Arms.ArmPitchAngle);
	InMID->SetScalarParameterValue(TEXT("ArmPitchTightening"), D.Arms.ArmPitchTightening);
	InMID->SetScalarParameterValue(TEXT("ArmPhaseOffset"), D.Arms.ArmPhaseOffset);
	InMID->SetScalarParameterValue(TEXT("HaloTwistInherit"), D.Noise.HaloTwistInherit);
	InMID->SetScalarParameterValue(TEXT("ArmCount"), D.Arms.ArmCount);
	InMID->SetScalarParameterValue(TEXT("ArmProfileExponent"), D.Arms.ArmProfileExponent);
	InMID->SetScalarParameterValue(TEXT("ArmRadialGrowth"), D.Arms.ArmRadialGrowth);
	InMID->SetScalarParameterValue(TEXT("ArmHostFalloff"), D.Arms.ArmHostFalloff);

	// --- DISC SHAPE AND ASYMMETRY ---
	InMID->SetScalarParameterValue(TEXT("DiscScaleRatio"), D.Disc.DiscScaleRatio);
	InMID->SetScalarParameterValue(TEXT("DiscVerticalFalloff"), D.Disc.DiscVerticalFalloff);
	InMID->SetScalarParameterValue(TEXT("DiscFlare"), D.Disc.DiscFlare);
	InMID->SetScalarParameterValue(TEXT("DiscWarpAmplitude"), D.Disc.DiscWarpAmplitude);
	InMID->SetScalarParameterValue(TEXT("DiscWarpPhase"), D.Disc.DiscWarpPhase);
	InMID->SetScalarParameterValue(TEXT("DiscWarpTwist"), D.Disc.DiscWarpTwist);
	InMID->SetScalarParameterValue(TEXT("DiscLopsidedAmount"), D.Disc.DiscLopsidedAmount);
	InMID->SetScalarParameterValue(TEXT("DiscLopsidedPhase"), D.Disc.DiscLopsidedPhase);

	// --- PROFILE EXPONENTS AND BOUNDS ---
	InMID->SetScalarParameterValue(TEXT("BulgeConcentration"), D.Bulge.BulgeConcentration);
	InMID->SetScalarParameterValue(TEXT("BackgroundConcentration"), D.Background.BackgroundConcentration);
	InMID->SetScalarParameterValue(TEXT("BoundsFadeStart"), D.Background.BoundsFadeStart);

	// --- CENTRAL VOID ---
	InMID->SetScalarParameterValue(TEXT("CentralVoidRadius"), D.Void.CentralVoidRadius);
	InMID->SetScalarParameterValue(TEXT("CentralVoidAmount"), D.Void.CentralVoidAmount);
	InMID->SetScalarParameterValue(TEXT("CentralVoidExponent"), D.Void.CentralVoidExponent);

	// --- NOISE FIELD SHAPE ---
	InMID->SetScalarParameterValue(TEXT("NoiseDiscLateralScale"), D.Noise.NoiseDiscLateralScale);
	InMID->SetScalarParameterValue(TEXT("NoiseDiscVerticalScale"), D.Noise.NoiseDiscVerticalScale);
	InMID->SetScalarParameterValue(TEXT("NoiseHaloLateralScale"), D.Noise.NoiseHaloLateralScale);
	InMID->SetScalarParameterValue(TEXT("NoiseHaloVerticalScale"), D.Noise.NoiseHaloVerticalScale);
	InMID->SetScalarParameterValue(TEXT("WarpDiscLateralScale"), D.Noise.WarpDiscLateralScale);
	InMID->SetScalarParameterValue(TEXT("WarpDiscVerticalScale"), D.Noise.WarpDiscVerticalScale);
	InMID->SetScalarParameterValue(TEXT("WarpHaloLateralScale"), D.Noise.WarpHaloLateralScale);
	InMID->SetScalarParameterValue(TEXT("WarpHaloVerticalScale"), D.Noise.WarpHaloVerticalScale);
	InMID->SetVectorParameterValue(TEXT("NoiseChannelWeights"), D.Noise.NoiseChannelWeights);
	InMID->SetVectorParameterValue(TEXT("NoiseOffset"), FLinearColor(D.Noise.NoiseOffset.X, D.Noise.NoiseOffset.Y, D.Noise.NoiseOffset.Z, 0.0f));
	InMID->SetScalarParameterValue(TEXT("NoiseRidged"), D.Noise.NoiseRidged);

	// ONE VECTOR CARRYING BOTH: xyz the disc normal, w the spin in degrees. The
	// derivation already takes a single float4, so a separate scalar would have been an
	// extra name to keep in agreement across the three marshalling sites for no gain --
	// and every name that can be misspelled is a silent no-op waiting to happen.
	InMID->SetVectorParameterValue(TEXT("FieldNormal"),
		FLinearColor(D.Orientation.FieldNormal.X, D.Orientation.FieldNormal.Y,
			D.Orientation.FieldNormal.Z, D.Orientation.FieldSpin));

	// --- RENDER ---
	InMID->SetScalarParameterValue(TEXT("MasterDensityScale"), D.Master.MasterDensityScale);
	InMID->SetScalarParameterValue(TEXT("MasterDensityPower"), D.Master.MasterDensityPower);
	// TWO, AND ONLY TWO, where there were four. The iteration bound is derived in the
	// Custom node from StepBudget and the step floor comes from the field's own
	// MinFeatureStep, so neither is a value this side has any business supplying; StepRatio
	// and MinSamples describe a stepping rule the marcher no longer uses. See the notes on
	// FGalaxyMaterialParams.
	//
	// A MID SILENTLY IGNORES A NAME THE MATERIAL DOES NOT HAVE, so the two names below are
	// load-bearing: until the material's pins are renamed to match, these push nothing and
	// the marcher runs on whatever the instance carries, with no warning anywhere.
	const FGalaxyMaterialParams& M = Params.Config.MaterialParams;

	InMID->SetScalarParameterValue(TEXT("StepBudget"), M.VolumeStepBudget);
	InMID->SetScalarParameterValue(TEXT("StepGrowth"), M.VolumeStepGrowth);

	// NOT PUSHED: EnableNoise. It is a StaticSwitchParameter, resolved at material
	// compile time -- a MID cannot change one, and setting it silently does nothing.
	//
	// It is a VISUALISATION AID on the material asset, not a field parameter. Placement
	// always samples the texture, so turning it off shows the analytic layers alone
	// while the stars stay where the textured field put them. There is deliberately no
	// per-galaxy property behind it.
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
	Params.Config.DeriveScaleRanges();

	// --- Large tier: exhaustive single cell, no streaming ---
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.GridDepth = Params.Config.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = 0;
	LargeTierConfig.SlotCapacity = Params.Config.LargeTier.SlotCapacity;
	LargeTierConfig.NiagaraAssets = { GalaxyLargeCloud };
	LargeTierConfig.bWantRotations = { false };
	LargeTierConfig.OctreeInsertBufferIndex = 0;
	LargeTierConfig.TierIndex = 0;
	// IDENTICAL IN SHAPE TO THE OTHER TWO. Build one cell per queued slot, hand it over.
	//
	// The large tier differs only in data: one slot covering the whole galaxy, subdivided
	// deeply enough to resolve structure, with every child writing to that one slot. It
	// used to have a bespoke cull grid and a bespoke cell builder; those described the
	// same tiling GenerationSubdivision already produces.
	//
	// bCellsShareSlot is the one flag that distinguishes it, and it is about CALIBRATION,
	// not generation: many cells feeding one slot means the slot holds their sum, so the
	// tier's constant divides capacity by total mass rather than by the largest cell.
	LargeTierConfig.GenerateBatchCallback =
		[this](const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutCounts) -> bool
		{
			const double CellExt = GetGridCellExtent(LargeTierConfig.GridDepth);

			TArray<GalaxyDataGenerator::FTierBatchCell> Cells;
			Cells.Reserve(Slots.Num());

			for (const TPair<FIntVector, int32>& Slot : Slots)
			{
				GalaxyDataGenerator::FTierBatchCell Cell;
				Cell.Coord = Slot.Key;
				Cell.SlotIndex = Slot.Value;
				Cell.Centre = GridCoordToCenter(Slot.Key, LargeTierConfig.GridDepth);
				Cell.HalfExtent = CellExt;
				Cells.Add(Cell);
			}

			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, LargeTierState.Buffers[0], Params.Config.LargeTier, 0, true, OutCounts);
		};

	// --- Mid tier: neighborhood streaming ---
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.GridDepth = Params.Config.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = Params.Config.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = Params.Config.MidTier.SlotCapacity;
	MidTierConfig.NiagaraAssets = { GalaxyMidCloud };
	MidTierConfig.bWantRotations = { false };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.TierIndex = 1;
	MidTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) { return !CellOverlapsVolume(Coord, MidTierConfig.GridDepth); };
	// The only generation path. Returns false when it cannot run -- no texture, readback
	// timed out -- and blanks the affected slots on the way out rather than leaving the
	// previous occupant's entities in them.
	//
	// Cells carry GEOMETRY ONLY. Whether a cell holds anything, what it rejects against
	// and how many candidates it draws are all decided by that cell's thread group in
	// the dispatch, from probes that sample the same textured field acceptance does.
	//
	// All three tiers run this same path, differing only in where their cells come from
	// -- a streamed neighbourhood here, the whole bounding grid for the large tier.
	// That is data, not a second code path.
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

			// ONE CELL PER SLOT, so the tier's constant is calibrated against the
			// largest single cell. Subdivision does not change that: the children of a
			// streamed cell all write to its slot.
			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, MidTierState.Buffers[0], Params.Config.MidTier, 7, false, OutCounts);
		};

	// --- Small tier: neighborhood streaming ---
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.GridDepth = Params.Config.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = Params.Config.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = Params.Config.SmallTier.SlotCapacity;
	SmallTierConfig.NiagaraAssets = { GalaxySmallCloud };
	SmallTierConfig.bWantRotations = { false };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.ShouldSkipCell = [this](const FIntVector& Coord) { return !CellOverlapsVolume(Coord, SmallTierConfig.GridDepth); };
	// The only generation path. Returns false when it cannot run -- no texture, readback
	// timed out -- and blanks the affected slots on the way out rather than leaving the
	// previous occupant's entities in them.
	//
	// Cells carry GEOMETRY ONLY. Whether a cell holds anything, what it rejects against
	// and how many candidates it draws are all decided by that cell's thread group in
	// the dispatch, from probes that sample the same textured field acceptance does.
	//
	// All three tiers run this same path, differing only in where their cells come from
	// -- a streamed neighbourhood here, the whole bounding grid for the large tier.
	// That is data, not a second code path.
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

			// ONE CELL PER SLOT, so the tier's constant is calibrated against the
// largest single cell. Subdivision does not change that: the children of a
// streamed cell all write to its slot.
			return GalaxyGenerator.GenerateTierBatchGPU(
				Cells, SmallTierState.Buffers[0], Params.Config.SmallTier, 13, false, OutCounts);
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
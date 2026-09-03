#include "UniverseActor.h"
#include "ActorPoolManager.h"
#include "ParallaxProxyActor.h"
#include "UltraLargeScale.h"
#include "FTierStreamingSystem.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include <Kismet/GameplayStatics.h>
#include <GalaxyActor.h>
#include <StarSystemActor.h>
#include <NiagaraFunctionLibrary.h>
#include <DrawDebugHelpers.h>
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Camera/PlayerCameraManager.h"
#include "GameFramework/PlayerController.h"
#include "Engine/GameViewportClient.h"
#include "Engine/PostProcessVolume.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/VolumeTexture.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/LocalPlayer.h"
#include "SceneView.h"

#pragma region Constructor
AUniverseActor::AUniverseActor()
{
	bAutoInitializeOnBeginPlay = true;
	// NO PARAMS ASSIGNMENT HERE ANY MORE: UniverseParams is a UPROPERTY with its own defaults,
	// so an edit on a placed actor takes effect without a level reload. Niagara cloud systems
	// load lazily in BuildTierConfigs, NOT here -- asset loads during CDO construction crash.
	GalaxyActorClass = AGalaxyActor::StaticClass();
	StarSystemActorClass = AStarSystemActor::StaticClass();
	ProxyActorClass = AParallaxProxyActor::StaticClass();
	Octree = MakeShared<FOctree>(GetPersistentTreeExtent(), FVector::ZeroVector);

	// Backdrop capture: renders the virtual stack into an HDR target composited behind the main
	// scene. Config happens in InitializeBackdropCapture; here we create and attach.
	BackdropCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("BackdropCapture"));
	BackdropCapture->SetupAttachment(RootComponent);
}
#pragma endregion

#pragma region Lifecycle
void AUniverseActor::Initialize()
{
	Super::Initialize();
}
#pragma endregion

#pragma region Initialization
// ---------------------------------------------------------------------------
// Backdrop capture
// ---------------------------------------------------------------------------

void AUniverseActor::InitializeBackdropCapture()
{
	if (!BackdropCapture) return;

	// Linear HDR scene colour with post disabled *in the capture*: the backdrop rides the MAIN
	// scene's tonemapper and exposure at composite time. SCS_SceneColorHDR is pre-tonemap.
	BackdropCapture->CaptureSource = ESceneCaptureSource::SCS_SceneColorHDR;

	// Driven manually from Tick, after the camera POV is synced, so the ordering is
	// unambiguous.
	BackdropCapture->bCaptureEveryFrame = false;
	BackdropCapture->bCaptureOnMovement = false;

	// Persist rendering state across manual captures (avoids per-capture re-init).
	BackdropCapture->bAlwaysPersistRenderingState = true;

	// Default render mode captures every primitive NOT marked bHiddenInSceneCapture, which
	// includes the virtual stack; real terrain and ocean are hidden, so no ShowOnly list.
	BackdropCapture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;

	// Keep the real planet's environment out of the backdrop. Set on the FEngineShowFlags
	// member directly: the details-panel ShowFlagSettings array is NOT applied to a dynamic
	// component. (Verify these setter names against 5.7's ShowFlagsValues.inl.)
	BackdropCapture->ShowFlags.SetFog(false);
	BackdropCapture->ShowFlags.SetAtmosphere(false);
	BackdropCapture->ShowFlags.SetVolumetricFog(false);
	BackdropCapture->ShowFlags.SetCloud(false);
	BackdropCapture->ShowFlags.SetMotionBlur(false);

	// Keep the RT a CLEAN consolidation of the virtual stack: no post baked into the capture,
	// so bloom, tonemap and exposure apply exactly ONCE on the final composite. Most of these
	// are a guard, since SCS_SceneColorHDR already captures pre-post.
	BackdropCapture->ShowFlags.SetBloom(false);
	BackdropCapture->ShowFlags.SetLensFlares(false);
	BackdropCapture->ShowFlags.SetTonemapper(false);
	BackdropCapture->ShowFlags.SetEyeAdaptation(false);   // no auto-exposure metering
	BackdropCapture->ShowFlags.SetDepthOfField(false);
	BackdropCapture->ShowFlags.SetVignette(false);
	BackdropCapture->ShowFlags.SetSceneColorFringe(false); // chromatic aberration
	BackdropCapture->ShowFlags.SetColorGrading(false);

	// These two ARE in SceneColorHDR, applied during lighting, but are meaningless on emissive
	// backdrop content -- disabling saves capture cost and avoids stray artifacts.
	BackdropCapture->ShowFlags.SetScreenSpaceReflections(false);
	BackdropCapture->ShowFlags.SetAmbientOcclusion(false);

	// Pin exposure to 1.0 so the backdrop never drifts with scene metering; the final composite
	// rides the MAIN view's exposure.
	BackdropCapture->PostProcessSettings.bOverride_AutoExposureMethod = true;
	BackdropCapture->PostProcessSettings.AutoExposureMethod = AEM_Manual;
	BackdropCapture->PostProcessSettings.bOverride_AutoExposureBias = true;
	BackdropCapture->PostProcessSettings.AutoExposureBias = 0.0f;

	EnsureBackdropRenderTarget();
}

void AUniverseActor::EnsureBackdropRenderTarget()
{
	if (!BackdropCapture) return;

	// Debug override: a hand-assigned RT asset instead of the runtime target. Skips sizing by
	// design, but still binds to both capture and composite so the whole path is exercised.
	if (DebugRTOverride)
	{
		BackdropRT = DebugRTOverride;
		BackdropCapture->TextureTarget = BackdropRT;
		if (CompositeMID) CompositeMID->SetTextureParameterValue(BackdropRTParamName, BackdropRT);
		return;
	}

	// Size to the viewport * scale, falling back to 1080p until the viewport is up.
	FIntPoint ViewSize(1920, 1080);
	if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
	{
		const FIntPoint VP = GEngine->GameViewport->Viewport->GetSizeXY();
		if (VP.X > 0 && VP.Y > 0) ViewSize = VP;
	}

	const float S = FMath::Clamp(BackdropResolutionScale, 0.25f, 1.0f);
	const FIntPoint Target(
		FMath::Max(1, FMath::RoundToInt(ViewSize.X * S)),
		FMath::Max(1, FMath::RoundToInt(ViewSize.Y * S)));

	if (BackdropRT && BackdropRTSize == Target) return;   // already correct size

	BackdropRT = UKismetRenderingLibrary::CreateRenderTarget2D(
		this, Target.X, Target.Y, RTF_RGBA16f, FLinearColor::Black, false);

	if (BackdropRT)
	{
		BackdropRT->TargetGamma = 1.0f;   // SceneColorHDR is already linear
		BackdropRTSize = Target;
		BackdropCapture->TextureTarget = BackdropRT;

		// Rebind onto the composite MID: the RT is a NEW pointer after a resize, so the
		// material parameter has to be re-set or it keeps sampling the old target.
		if (CompositeMID) CompositeMID->SetTextureParameterValue(BackdropRTParamName, BackdropRT);
	}
}

void AUniverseActor::UpdateBackdropCapture()
{
	if (!bEnableBackdropCapture || !BackdropCapture) return;

	EnsureBackdropRenderTarget();
	if (!BackdropRT) return;

	// Match the MAIN view, not the pawn peg the parallax uses: the backdrop projection has to
	// line up with what the player actually sees.
	APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	if (!PC) return;

	FVector ViewLoc; FRotator ViewRot;
	PC->GetPlayerViewPoint(ViewLoc, ViewRot);
	BackdropCapture->SetWorldLocationAndRotation(ViewLoc, ViewRot);

	// Match the MAIN view's projection EXACTLY, including its Maintain-Axis aspect handling.
	// SceneCapture treats FOVAngle as HORIZONTAL FOV and derives the vertical from the RT
	// aspect, while the game camera constrains the OTHER axis, so a viewport stretches.
	bool bMatchedProjection = false;
	if (ULocalPlayer* LP = PC->GetLocalPlayer())
	{
		if (LP->ViewportClient && LP->ViewportClient->Viewport)
		{
			FSceneViewProjectionData ProjData;
			if (LP->GetProjectionData(LP->ViewportClient->Viewport, ProjData))
			{
				BackdropCapture->bUseCustomProjectionMatrix = true;
				BackdropCapture->CustomProjectionMatrix = ProjData.ProjectionMatrix;
				bMatchedProjection = true;
			}
		}
	}

	if (!bMatchedProjection)
	{
		// Fallback until the view is available: derive from FOV. Stretches on non-square
		// viewports, but only transiently.
		BackdropCapture->bUseCustomProjectionMatrix = false;
		if (PC->PlayerCameraManager)
			BackdropCapture->FOVAngle = PC->PlayerCameraManager->GetFOVAngle();
	}

	// Manual capture with this frame's resolved virtual positions.
	BackdropCapture->CaptureScene();
}

void AUniverseActor::InitializeBackdropComposite()
{
	// Load here (runtime), not in the ctor: CDO-time asset loads are unsafe.
	if (!CompositeMaterial)
		CompositeMaterial = LoadObject<UMaterialInterface>(nullptr,
			TEXT("/UltraLargeScale/Sector/MT_BackdropPostProcess.MT_BackdropPostProcess"));

	if (!CompositeMaterial)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[Universe] Backdrop composite material missing; backdrop will not render."));
		return;
	}

	CompositeMID = UMaterialInstanceDynamic::Create(CompositeMaterial, this);
	if (!CompositeMID) return;

	CompositeMID->SetScalarParameterValue(BackdropDepthThresholdParamName, BackdropDepthThreshold);
	CompositeMID->SetScalarParameterValue(BackdropIntensityParamName, BackdropIntensity);

	// Unbound PP volume owned by this actor, applying everywhere so the backdrop composites in
	// deep space as well as near a planet, where the atmosphere's own volume adds its pass
	// later and composites over us. The material's Blendable Location fixes the pass ordering.
	if (UWorld* W = GetWorld())
	{
		FActorSpawnParameters SP;
		SP.Owner = this;
		BackdropPPVolume = W->SpawnActor<APostProcessVolume>(SP);
		if (BackdropPPVolume)
		{
			BackdropPPVolume->bUnbound = true;
			BackdropPPVolume->BlendWeight = 1.0f;
			BackdropPPVolume->Priority = 0.0f;
			BackdropPPVolume->Settings.AddBlendable(CompositeMID, 1.0f);
		}
	}
}

void AUniverseActor::BeginPlay()
{
	Super::BeginPlay();
	// Composite first: it creates the MID and volume, so the RT below binds straight onto it.
	InitializeBackdropComposite();
	InitializeBackdropCapture();
	// Central actor pool: create, register types, prewarm, before any child activates. The sole
	// galaxy pool -- SpawnGalaxyFromPool acquires from it and ReturnGalaxyToPool returns.
	PoolManager = NewObject<UActorPoolManager>(this);
	PoolManager->RegisterType(GalaxyActorClass, GalaxyPoolSize);
	PoolManager->RegisterType(StarSystemActorClass, StarSystemPoolSize);
	PoolManager->RegisterType(ProxyActorClass, PlanetPoolSize);
	PoolManager->PrewarmAll(GetWorld());

	if (bAutoInitializeOnBeginPlay) Initialize();
}

void AUniverseActor::InitializeData()
{
	double StartTime = FPlatformTime::Seconds();
	UniverseGenerator.Params = UniverseParams;

	// THE TWO THINGS THE GENERATOR CANNOT DERIVE, both supplied rather than inferred.
	// FieldExtent is the RAY MARCH PROXY'S half extent -- the Large tier's neighbourhood span,
	// not UniverseParams.Extent -- and it converts caller units into the field's frame, so a
	// second derivation inside the generator is how placement and render end up disagreeing.
	UniverseGenerator.FieldExtent = GetVolumetricProxyExtent();
	UniverseGenerator.FieldTextures = GetFieldTextures();

	// NO GENERATOR INITIALIZE: it built the legacy FastNoise graph and nothing else. The
	// generator's remaining state is FieldExtent and NoiseTexture, both set immediately above.
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeData took: %.3f seconds"),
		FPlatformTime::Seconds() - StartTime);
}

#pragma region Volumetric

int32 AUniverseActor::GetFieldCellPeriod() const
{
	return UniverseCellWrap::FieldCellPeriod(UniverseParams.DensityParams.Lattice);
}

double AUniverseActor::GetFieldCellSize() const
{
	// The division guard is on the cell size alone: the proxy extent has its own floor in
	// GetVolumetricProxyExtent, so the product cannot reach zero from either side.
	return GetVolumetricProxyExtent()
		* FMath::Max(static_cast<double>(UniverseParams.DensityParams.Lattice.CellSizeSmall), 1e-6);
}

double AUniverseActor::GetVolumetricProxyExtent() const
{
	// (2R + 1) cells across, half-extent per cell, so this is the resident neighbourhood's
	// half-span. See the header for why it is not UniverseParams.Extent.
	const int32 Radius = FMath::Max(UniverseParams.LargeTier.NeighborhoodRadius, 0);
	const double CellHalf = GetGridCellExtent(UniverseParams.LargeTier.GridDepth);

	return FMath::Max(static_cast<double>(2 * Radius + 1) * CellHalf, UE_DOUBLE_SMALL_NUMBER);
}

FUniverseFieldOffset AUniverseActor::ComputeFieldOffset() const
{
	// THE SCALED RELATIVE VIRTUAL POSITION, not the world position. VirtualTraversal is where
	// the player has travelled through the field, already divided by UnitScale, while the
	// actor's world location is a rendering convenience reset to the player every frame.
	//
	// DIVIDED BY THE FIELD CELL, the proxy half-extent times the small cell size and nothing
	// else. Both factors must be the ones the shader uses: it marches [-1,1] across a box of
	// that half-extent and decomposes with its own CellSizeRange.x, which PushDensityParams
	// writes from this same struct. A factor from elsewhere makes the field SCROLL at the wrong
	// rate, which reads as anything but a scale disagreement.
	//
	// DOUBLE THROUGHOUT, which is why this happens here rather than in the shader:
	// VirtualTraversal reaches the magnitudes the cell/frac formulation exists for.
	const FVector CellPos = VirtualTraversal / GetFieldCellSize();

	// REDUCED BEFORE IT NARROWS, and that ordering is the whole point. The split above runs in
	// double and is exact at any traversal; the pin it is headed for is a float3 and stops
	// having a unit ulp at 2^24, so wrapping here keeps the crossing integer inside the exact
	// range and the material receives the number this function computed.
	//
	// FRAC IS UNTOUCHED: only the integer part is reduced, and it is reduced IN DOUBLE before
	// it narrows. Splitting first and wrapping the int32 afterwards held only to 2.1e9 cells,
	// past which the cast is undefined and the offset collapses.
	//
	// THE CORE REDUCES AGAIN with a period it derives itself, deliberately: this reduction is
	// about float exactness, the core's defines the field's periodicity. If a stale cell size
	// puts the two apart, the field still wraps cleanly and only the offset loses precision.
	return FUniverseFieldOffset::FromCellPositionWrapped(CellPos, GetFieldCellPeriod());
}

void AUniverseActor::PushFieldOffset(UMaterialInstanceDynamic* InMID) const
{
	if (!InMID) return;

	const FUniverseFieldOffset Offset = ComputeFieldOffset();

	// THE CELL COUNT NARROWS HERE, from int32 to a float pin, and it is EXACT because the
	// offset arrives reduced below the field period, which sits under 2^24. What is left is
	// double precision in VirtualTraversal, which decays gracefully rather than wrapping.
	InMID->SetVectorParameterValue(TEXT("OffsetCell"), FLinearColor(
		static_cast<float>(Offset.Cell.X),
		static_cast<float>(Offset.Cell.Y),
		static_cast<float>(Offset.Cell.Z), 0.0f));

	InMID->SetVectorParameterValue(TEXT("OffsetFrac"), FLinearColor(
		static_cast<float>(Offset.Frac.X),
		static_cast<float>(Offset.Frac.Y),
		static_cast<float>(Offset.Frac.Z), 0.0f));
}

void AUniverseActor::PushDensityParams(UMaterialInstanceDynamic* InMID) const
{
	if (!InMID) return;

	// ONE PACK, and every pin below reads it. The three .w passengers -- both warp scales and
	// the large octave's lattice-follow -- are already loaded by Pack, so there is no second
	// place for the packing to drift. VoidSpreadRange's .w IS UNCLAIMED and Pack sends zero.
	//
	// THE OFFSET IS PACKED AND DISCARDED HERE: Pack takes one because the compute path's
	// argument list needs it, and this function deliberately does not push it. A zero
	// placeholder would read as a field centred on the origin, which is a claim.
	const FUniverseDensityArgs A = UniverseParams.DensityParams.Pack(
		UniverseParams.Seed, ComputeFieldOffset());

	auto SetVec = [InMID](const TCHAR* InName, const FVector4f& InValue)
		{
			InMID->SetVectorParameterValue(InName,
				FLinearColor(InValue.X, InValue.Y, InValue.Z, InValue.W));
		};

	// --- WEB GEOMETRY ---
	SetVec(TEXT("CellSizeRange"), A.CellSizeRange);
	InMID->SetScalarParameterValue(TEXT("Seed"), A.Seed);

	// --- FIELD OFFSET --- NOT PUSHED HERE. It has its own path and runs unconditionally every
	// frame; folding it in would make the one parameter that must always be pushed depend on
	// the flag gating the ones that must not.

	// --- WEB DENSITY, WIDTH AND FALLOFF ---
	SetVec(TEXT("WallDensityRange"), A.WallDensityRange);
	SetVec(TEXT("WallFalloffRange"), A.WallFalloffRange);
	SetVec(TEXT("FilamentDensityRange"), A.FilamentDensityRange);
	SetVec(TEXT("FeatureWidthRange"), A.FeatureWidthRange);
	SetVec(TEXT("VoidFloorRange"), A.VoidFloorRange);

	// --- ORGANICS ---
	SetVec(TEXT("VoidSpreadRange"), A.VoidSizeSpreadRange);
	SetVec(TEXT("WarpLargeRange"), A.WarpAmountLargeRange);
	SetVec(TEXT("WarpSmallRange"), A.WarpAmountSmallRange);
	SetVec(TEXT("WarpLargeWeights"), A.WarpLargeWeights);
	SetVec(TEXT("WarpSmallWeights"), A.WarpSmallWeights);
	// THE ONE PARAMETER WHOSE NAME DID NOT MAKE THE RENAME. The node's input is RegionScales,
	// the VectorParameter behind it is still RegionScale, and a MID addresses the parameter --
	// so pushing the plural was a silent no-op. Rename the parameter and this line together.
	SetVec(TEXT("RegionScale"), A.RegionScales);

	// --- BOUNDS --- the march applies the fade rather than the field, so this is the one value
	// in the set that describes how the field is being VIEWED. Leaving it to the asset would
	// let the render disagree with the code about where the horizon is.
	InMID->SetScalarParameterValue(TEXT("BoundsFadeStart"), A.BoundsFadeStart);

	// --- MARCH --- NOT PUSHED HERE. See PushMarchParams; it runs unconditionally.
}

void AUniverseActor::PushMarchParams(UMaterialInstanceDynamic* InMID) const
{
	if (!InMID) return;

	const FUniverseMaterialParams& M = UniverseParams.MaterialParams;

	// FOUR, AND ONLY FOUR. The iteration bound is derived in the Custom node from StepBudget,
	// the step floor comes from the field's own MinFeatureStep, and the dither is computed per
	// pixel in the node, so none of those three is this side's business to supply.
	InMID->SetScalarParameterValue(TEXT("StepBudget"), M.VolumeStepBudget);
	InMID->SetScalarParameterValue(TEXT("StepGrowth"), M.VolumeStepGrowth);
	InMID->SetScalarParameterValue(TEXT("DensityScale"), M.VolumeDensityScale);
	InMID->SetScalarParameterValue(TEXT("NoisePower"), M.VolumeNoisePower);
}

// MUST NOT BE CALLED ON THE GAME THREAD. The body dispatches to the game thread and then blocks
// on the future, so a game-thread caller waits on work that cannot start until it returns -- a
// hard deadlock. AProceduralSpaceActor::Initialize reaches this from its async chain, which is
// why it is safe today; the constraint is on the caller and invisible from the signature.
void AUniverseActor::InitializeVolumetric()
{
	double StartTime = FPlatformTime::Seconds();

	// The constraints the details panel cannot show: the fold ceiling, the coprimality of the
	// four texture scales, and the region fetches a flattened group would silently turn off.
	// Reported before anything is built, so a tearing web has a log line next to it.
	UniverseParams.DensityParams.Validate(TEXT("AUniverseActor::InitializeVolumetric"));

	// The proxy span and the cell count it resolves to, logged because neither is visible
	// elsewhere and both are the first things to check when the field scrolls wrongly.
	UE_LOG(LogTemp, Log,
		TEXT("AUniverseActor::InitializeVolumetric - proxy half-extent %.4g (Large tier ")
		TEXT("depth %d, radius %d)."),
		GetVolumetricProxyExtent(),
		UniverseParams.LargeTier.GridDepth,
		UniverseParams.LargeTier.NeighborhoodRadius);

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::GameThread, [WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self || Self->InitializationState == ELifecycleState::Pooling)
			{
				CompletionPromise.SetValue();
				return;
			}

			// Resolve every asset up front and bail loudly.
			UStaticMesh* BoxMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/UltraLargeScale/UnitBoxInvertedNormals.UnitBoxInvertedNormals"));
			UMaterialInterface* ParentMat = LoadObject<UMaterialInterface>(nullptr, *Self->VolumetricMaterialPath);
			// THE FIELD VOLUMES ARE ALREADY RESOLVED and are deliberately NOT loaded here:
			// Initialize calls LoadRuntimeAssets in its game-thread prologue. A SECOND
			// LoadObject AGAINST THE SAME PATHS WOULD BE WRONG, not merely redundant -- it
			// returns the same objects today and stops the moment anything edits the paths, at
			// which point the material samples one set of assets and the dispatch another. That
			// is silent: both fields look like plausible cosmic webs.

			if (!BoxMesh || !ParentMat)
			{
				UE_LOG(LogTemp, Error, TEXT("AUniverseActor::InitializeVolumetric - ABORT, required assets unresolved. ") TEXT("mesh=%s material=%s"), BoxMesh ? TEXT("ok") : TEXT("NULL /UltraLargeScale/UnitBoxInvertedNormals"), ParentMat ? TEXT("ok") : *FString::Printf(TEXT("NULL %s"), *Self->VolumetricMaterialPath));
				CompletionPromise.SetValue();
				return;
			}
			const FUniverseFieldTextures FieldTextures = Self->GetFieldTextures();

			if (!FieldTextures.IsComplete())
			{
				// LOUDER THAN THE GALAXY'S EQUIVALENT, because it means more here: a missing
				// variance volume takes every regional axis to its midpoint and a missing warp
				// volume straightens the bisectors, either a change in GEOMETRY, not shading.
				UE_LOG(LogTemp, Warning,
					TEXT("AUniverseActor::InitializeVolumetric - field volumes unresolved (%s); ")
					TEXT("the field loses the corresponding structure -- regional variance goes ")
					TEXT("to its midpoints, the domain warp goes to zero, bisectors straighten. ")
					TEXT("Check that the UniverseNoisePack plugin is enabled and the paths ")
					TEXT("are correct."),
					*FieldTextures.DescribeMissing());
			}

			Self->VolumeMaterial = UMaterialInstanceDynamic::Create(ParentMat, Self);

			// PIN NAMES MATCH THE .usf DECLARATIONS EXACTLY. A MID silently ignores a name the
			// material does not have, so a renamed pin is a texture that keeps whatever the
			// parent had -- most likely a default grey, reading as "the warp stopped working".
			// SET INDIVIDUALLY, so a partial set still binds what it has: right for the RENDER,
			// which degrades to something visibly wrong, and emphatically not right for
			// PLACEMENT, which refuses to run rather than place against an undrawn field.
			if (FieldTextures.VarianceA)
			{
				Self->VolumeMaterial->SetTextureParameterValue(
					FName("VarianceTexA"), Self->FieldVarianceTexA);
			}
			if (FieldTextures.VarianceB)
			{
				Self->VolumeMaterial->SetTextureParameterValue(
					FName("VarianceTexB"), Self->FieldVarianceTexB);
			}
			if (FieldTextures.WarpLarge)
			{
				Self->VolumeMaterial->SetTextureParameterValue(
					FName("WarpTexLarge"), Self->FieldWarpTexLarge);
			}
			if (FieldTextures.WarpSmall)
			{
				Self->VolumeMaterial->SetTextureParameterValue(
					FName("WarpTexSmall"), Self->FieldWarpTexSmall);
			}

			// ALL THREE, UNCONDITIONALLY. The instance is a starting point, never a source of
			// truth: the compute path places against FUniverseDensityParams, so any pin it were
			// allowed to keep would put render and placement on different universes.
			Self->PushFieldOffset(Self->VolumeMaterial);
			Self->PushMarchParams(Self->VolumeMaterial);
			Self->PushDensityParams(Self->VolumeMaterial);

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
			// SAME NUMBER THE OFFSET IS NORMALIZED BY: the unit box spans 0..1, so the scale is
			// the full width, twice the half-extent.
			Self->VolumetricComponent->SetWorldScale3D(FVector(2.0 * Self->GetVolumetricProxyExtent()));
			Self->VolumetricComponent->SetMaterial(0, Self->VolumeMaterial);
			Self->VolumetricComponent->SetVisibility(true);

			// HOW MUCH WEB IS IN A CHORD, the first number to reach for when the field
			// saturates and not visible anywhere else. The camera sits inside this volume, so
			// every ray marches a full diameter: four to eight cells is a web, forty is foam.
			const float CellSmall = FMath::Max(
				Self->UniverseParams.DensityParams.Lattice.CellSizeSmall, 1e-6f);
			const FUniverseMaterialParams& MP = Self->UniverseParams.MaterialParams;
			UE_LOG(LogTemp, Log,
				TEXT("AUniverseActor::InitializeVolumetric - proxy spans %.1f small cells ")
				TEXT("(CellSizeSmall %.4f); march budget %.0f at growth %.2f resolves to ")
				TEXT("~%.0f actual steps."),
				2.0f / CellSmall, CellSmall,
				MP.VolumeStepBudget, MP.VolumeStepGrowth,
				MP.GetEffectiveStepCount());

			CompletionPromise.SetValue();
		});

	CompletionFuture.Wait();
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeVolumetric took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}

#pragma endregion

void AUniverseActor::InitializeNiagara()
{
	double StartTime = FPlatformTime::Seconds();
	BuildTierConfigs();
	const FTierStreamingContext Ctx = BuildStreamingContext();
	// GATED FOR ISOLATION: a tier that is off is never initialized, so UpdateTier must skip it
	// too -- it has no buffers and no components to update.
	if (UniverseParams.bEnableLargeTier)
	{
		FTierStreamingSystem::InitializeTier(Ctx, LargeTierConfig, LargeTierState, TierNiagaraComponents);
	}
	if (UniverseParams.bEnableMidTier)
	{
		FTierStreamingSystem::InitializeTier(Ctx, MidTierConfig, MidTierState, TierNiagaraComponents);
	}
	if (UniverseParams.bEnableSmallTier)
	{
		FTierStreamingSystem::InitializeTier(Ctx, SmallTierConfig, SmallTierState, TierNiagaraComponents);
	}
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::InitializeNiagara total duration: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
}
#pragma endregion

#pragma region Tier System - BuildTierConfigs
void AUniverseActor::LoadRuntimeAssets()
{
	// Game thread, before async dispatch: LoadObject is not thread-safe, so the Niagara systems
	// BuildTierConfigs reads must load here. Guarded so pooled reuse does not reload.
	if (!SectorLargeCloud) SectorLargeCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Sector/NG_SectorLarge.NG_SectorLarge"));
	if (!SectorMidCloud)   SectorMidCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Sector/NG_SectorMid.NG_SectorMid"));
	if (!SectorSmallCloud) SectorSmallCloud = LoadObject<UNiagaraSystem>(nullptr, TEXT("/UltraLargeScale/Sector/NG_SectorSmall.NG_SectorSmall"));

	// THE FIELD'S FOUR VOLUMES, resolved once from the authored paths so the material and the
	// dispatch cannot end up on different assets. Game thread, because LoadObject is not thread
	// safe. EACH GUARDED AND REPORTED SEPARATELY, so a single bad path names itself.
	const FUniverseMaterialParams& MatParams = UniverseParams.MaterialParams;

	struct FFieldVolumeSlot
	{
		UVolumeTexture** Target;
		const FString* Path;
		const TCHAR* Name;
	};

	const FFieldVolumeSlot Slots[] = {
		{ &FieldVarianceTexA, &MatParams.VarianceVolumeA, TEXT("VarianceVolumeA") },
		{ &FieldVarianceTexB, &MatParams.VarianceVolumeB, TEXT("VarianceVolumeB") },
		{ &FieldWarpTexLarge, &MatParams.WarpVolumeLarge, TEXT("WarpVolumeLarge") },
		{ &FieldWarpTexSmall, &MatParams.WarpVolumeSmall, TEXT("WarpVolumeSmall") },
	};

	for (const FFieldVolumeSlot& Slot : Slots)
	{
		if (*Slot.Target)
		{
			continue;
		}

		*Slot.Target = LoadObject<UVolumeTexture>(nullptr, **Slot.Path);

		if (!*Slot.Target)
		{
			// Not a warning that can be deferred: without this the material renders a DIFFERENT
			// FIELD, and entity generation refuses to run rather than place against one. A NULL
			// IS AS LIKELY TO BE THE MOUNT POINT AS THE PATH -- UniverseNoisePack may be off.
			UE_LOG(LogTemp, Error,
				TEXT("AUniverseActor::LoadRuntimeAssets - field volume %s ('%s') unresolved. ")
				TEXT("The render will lose part of its structure and entity generation will ")
				TEXT("place nothing until every one of the four resolves. If all four failed, ")
				TEXT("check that the UniverseNoisePack plugin is enabled."),
				Slot.Name, **Slot.Path);
		}
	}

	// THE TWO VARIANCE VOLUMES MUST NOT BE THE SAME ASSET, cheap to check and expensive to
	// notice otherwise. Field A decides the web's structure and field B its painting, and the
	// entire reason they are two fetches is that a province's shape and appearance should
	// change in different places. Pointed at one asset they share all four generators and their
	// boundaries re-align, reading as a field with less variety rather than a misconfiguration.
	if (FieldVarianceTexA && FieldVarianceTexA == FieldVarianceTexB)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("AUniverseActor::LoadRuntimeAssets - VarianceVolumeA and VarianceVolumeB ")
			TEXT("resolve to the same asset ('%s'). The structure and appearance region ")
			TEXT("fields will share every generator and their boundaries will correlate. ")
			TEXT("Point them at two different multinoise variants."),
			*MatParams.VarianceVolumeA);
	}
}

/** The batch callback all three tiers share.
 *
 *  ONE FACTORY RATHER THAN THREE LAMBDAS, because the three tiers now differ only in which
 *  config, state and params they name. The galaxy layer writes its two out separately and
 *  they have already drifted in whitespace; a factory makes a fourth tier a call rather
 *  than a copy.
 *
 *  THE SEED OFFSET IS THE TIER INDEX and it must be stable for the life of the sector: it
 *  keys both the placement hash and the calibrated constant's cache, so renumbering the
 *  tiers reseeds every entity in the universe.
 *
 *  Captures `this` and holds references into the actor. Safe because the tier configs do
 *  not outlive it -- FTierStreamingSystem calls these from generation, which is torn down
 *  in EndPlay before the actor goes. */
TFunction<bool(const TArray<TPair<FIntVector, int32>>&, TArray<int32>&)>
AUniverseActor::MakeTierBatchCallback(
	FParticleTierConfig& InConfig,
	FParticleTierState& InState,
	const FTierParams& InTierParams,
	int32 InSeedOffset)
{
	return [this, &InConfig, &InState, &InTierParams, InSeedOffset](
		const TArray<TPair<FIntVector, int32>>& Slots, TArray<int32>& OutCounts) -> bool
		{
			// GEOMETRY ONLY. The centre comes from the grid the ACTOR owns; a generator
			// inferring it from a buffer's contents would put every candidate somewhere else.
			const double CellExt = GetGridCellExtent(InConfig.GridDepth);

			TArray<FTierBatchCell> Cells;
			Cells.Reserve(Slots.Num());

			for (const TPair<FIntVector, int32>& Slot : Slots)
			{
				FTierBatchCell Cell;
				Cell.Coord = Slot.Key;
				Cell.SlotIndex = Slot.Value;
				Cell.Centre = GridCoordToCenter(Slot.Key, InConfig.GridDepth);
				Cell.HalfExtent = CellExt;
				Cells.Add(Cell);
			}

			// Buffer 0: every tier is single-buffer now that the gas layer is gone.
			return UniverseGenerator.GenerateTierBatchGPU(
				Cells, InState.Buffers[0], InTierParams, InSeedOffset,
				InConfig.GridDepth, OutCounts);
		};
}

void AUniverseActor::BuildTierConfigs()
{
	// Every archetype checked ONCE, here, rather than per spawn. Min above Max and stale
	// parameter names produce SILENT garbage -- FRandRange with inverted bounds returns a value
	// outside the interval with no clamp catching it, and an unresolvable name rolls nothing.
	// Both look identical to "the archetype is authored wrong", indistinguishable from every
	// other galaxy during the phase where they should look unfamiliar.
	FGalaxySpawnConfig::Validate(GalaxySpawnConfig);

	// Derive MinScale/MaxScale for all tiers from MaxEntityScale plus depth spacing, before any
	// generate callback reads scale ranges.
	UniverseParams.DeriveScaleRanges();

	// Large tier
	LargeTierConfig.TierName = TEXT("Large");
	LargeTierConfig.TierIndex = 0;
	LargeTierConfig.GridDepth = UniverseParams.LargeTier.GridDepth;
	LargeTierConfig.NeighborhoodRadius = UniverseParams.LargeTier.NeighborhoodRadius;
	LargeTierConfig.SlotCapacity = UniverseParams.LargeTier.SlotCapacity;
	// ONE ASSET, as Mid and Small have. The second entry was NG_SectorGas, a nebula sprite
	// layer sharing the cluster positions at a larger extent; the universe raymarch supersedes
	// it. FParticleTierConfig's multi-buffer support stays -- generic, just unused here.
	LargeTierConfig.NiagaraAssets = { SectorLargeCloud };
	LargeTierConfig.bWantRotations = { true };
	LargeTierConfig.OctreeInsertBufferIndex = 0;

	// ALL THREE TIERS ARE NOW THE SAME SHAPE, and identical to the galaxy layer's lower two:
	// one cell per queued slot, one dispatch per batch, and the placement constant calibrated
	// against the largest streamed cell. There is no equivalent of the galaxy's Large tier.
	LargeTierConfig.GenerateBatchCallback = MakeTierBatchCallback(
		LargeTierConfig, LargeTierState, UniverseParams.LargeTier, 0);

	// Mid tier
	MidTierConfig.TierName = TEXT("Mid");
	MidTierConfig.TierIndex = 1;
	MidTierConfig.GridDepth = UniverseParams.MidTier.GridDepth;
	MidTierConfig.NeighborhoodRadius = UniverseParams.MidTier.NeighborhoodRadius;
	MidTierConfig.SlotCapacity = UniverseParams.MidTier.SlotCapacity;
	MidTierConfig.NiagaraAssets = { SectorMidCloud };
	MidTierConfig.bWantRotations = { true };
	MidTierConfig.OctreeInsertBufferIndex = 0;
	MidTierConfig.GenerateBatchCallback = MakeTierBatchCallback(
		MidTierConfig, MidTierState, UniverseParams.MidTier, 1);

	// Small tier
	SmallTierConfig.TierName = TEXT("Small");
	SmallTierConfig.TierIndex = 2;
	SmallTierConfig.GridDepth = UniverseParams.SmallTier.GridDepth;
	SmallTierConfig.NeighborhoodRadius = UniverseParams.SmallTier.NeighborhoodRadius;
	SmallTierConfig.SlotCapacity = UniverseParams.SmallTier.SlotCapacity;
	SmallTierConfig.NiagaraAssets = { SectorSmallCloud };
	SmallTierConfig.bWantRotations = { true };
	SmallTierConfig.OctreeInsertBufferIndex = 0;
	SmallTierConfig.GenerateBatchCallback = MakeTierBatchCallback(
		SmallTierConfig, SmallTierState, UniverseParams.SmallTier, 2);

	// Applied to each tier after its GridDepth/NeighborhoodRadius are set. Captures Config by
	// ref, safe since Config outlives all lambda calls.
	//
	// Bounds derivation (actor-relative; the actor is pegged to the player). Rendered offset is
	// cellLocal + lattice + (NCenter - VT), where
	//   |cellLocal| + |lattice| <= (2R+1) * CellHalfExtent   (neighborhood half-span)
	//   |NCenter - VT|          <= CellHalfExtent            (VT resides in the center cell)
	// so the tight half-bound is (2R+2) * CellHalfExtent plus one particle radius. We provision
	// 2 * (2R+1). THE SHARED CONVENTION -- GalaxyActor's MakeBounds must match; a bare (2R+1)
	// clips edge cells by up to one half-cell when VT sits near a cell boundary.
	auto MakeBoundsLambda = [this](const FParticleTierConfig& Cfg) -> TFunction<FBox()>
		{
			return [this, &Cfg]() -> FBox
				{
					const double BoundsExtent = (2 * Cfg.NeighborhoodRadius + 1) * GetGridCellExtent(Cfg.GridDepth) * 2.0;
					return FBox(FVector(-BoundsExtent), FVector(BoundsExtent));
				};
		};
	LargeTierConfig.ComputeBounds = MakeBoundsLambda(LargeTierConfig);
	MidTierConfig.ComputeBounds = MakeBoundsLambda(MidTierConfig);
	SmallTierConfig.ComputeBounds = MakeBoundsLambda(SmallTierConfig);
}
#pragma endregion

#pragma region Tier System - BuildStreamingContext
FTierStreamingContext AUniverseActor::BuildStreamingContext() const
{
	FTierStreamingContext Ctx;
	Ctx.Extent = UniverseParams.Extent;
	Ctx.UnitScale = 1.0;  // Universe extents are already in octree-local units.
	Ctx.bVirtualSpace = IsVirtualSpace();  // real UnitScale (1.6e17) -> backdrop
	Ctx.GridExtentMultiplier = GridExtentMultiplier;
	Ctx.VirtualTraversal = VirtualTraversal;
	Ctx.Octree = Octree;
	Ctx.InitializationState = InitializationState;
	Ctx.bRebaseInProgress = bRebaseInProgress.load();
	Ctx.AttachRoot = GetRootComponent();
	Ctx.bNiagaraAbsolutePosition = false;
	Ctx.OwnerName = TEXT("Universe");
	Ctx.ParentSeed = UniverseParams.Seed;
	Ctx.GetLatestVT = [this] { return ReadLatestVT(); };
	Ctx.GetLiveState = [this] { return InitializationState; };
	return Ctx;
}
#pragma endregion

#pragma region Parallax
void AUniverseActor::ApplyParallaxOffset(const FVector& InPlayerPos)
{
	SVO_GT_SCOPE("Universe::ApplyParallaxOffset");
	const FVector PlayerDelta = InPlayerPos - LastFrameOfReferenceLocation;
	LastFrameOfReferenceLocation = InPlayerPos;
	CurrentFrameOfReferenceLocation = InPlayerPos;

	const double Ratio = (UniverseParams.UnitScale > 0.0) ? (SpeedScale / UniverseParams.UnitScale) : 0.0;
	VirtualTraversal += PlayerDelta * Ratio;

	SetActorLocation(InPlayerPos);

	// THE PROXY IS PEGGED TO THE CAMERA, not to the field. The galaxy layer does the opposite
	// -- its box stays where the galaxy is -- and the difference is that this layer has no
	// outside: a window centred on the viewer, with the field scrolling under it.
	if (VolumetricComponent)
	{
		VolumetricComponent->SetWorldLocation(InPlayerPos);
		PushFieldOffset(VolumeMaterial);
	}

	// Publish EVERY frame, even below the push threshold, so an in-flight full push completing
	// this frame still composites against current VT.
	PublishLatestVT(VirtualTraversal);

	const double DeltaSq = FVector::DistSquared(VirtualTraversal, LastPushedVirtualTraversal);
	if (DeltaSq > ParallaxPushThreshold * ParallaxPushThreshold)
	{
		LastPushedVirtualTraversal = VirtualTraversal;
		// GAME THREAD, deliberately: one FVector uniform per component is trivially cheap, and
		// SetVariable* writes the Niagara parameter store with no internal lock, so racing the
		// GT's own Niagara tick trips FMTAccessDetector.
		FTierStreamingSystem::PushTierVT({ &LargeTierState, &MidTierState, &SmallTierState }, [this] { return ReadLatestVT(); });
	}
}

// Coalesced, single-flight per-frame push. Producers raise bPushDirty; at most one worker
// drains, re-reading the freshest VT under each tier's PushCS every pass, so out-of-order
// scheduling is harmless -- the last write always wins with current VT.
//
// NOTHING CALLS THIS, in any of the three layers that define it. ApplyParallaxOffset pushes
// directly on the game thread instead, because one FVector uniform per component is trivially
// cheap and SetVariable* has no internal lock, so racing the game thread's own Niagara tick
// trips FMTAccessDetector. IT CAPTURES RAW `this`, unlike every other AsyncTask in this file:
// the loop can outlive the actor and the tier states it dereferences are actor members, so
// anything reviving this has to convert the capture first -- the pattern is in RequestScan.
void AUniverseActor::SchedulePush()
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
#pragma endregion

#pragma region Tick
void AUniverseActor::Tick(float DeltaTime)
{
	SVO_GT_SCOPE("Universe::Tick");
	{
		SVO_GT_SCOPE("Universe::SuperTick");
		Super::Tick(DeltaTime);
	}
	FVector CurrentPlayerPos;
	if (InitializationState == ELifecycleState::Ready && GetPlayerLocation(GetWorld(), CurrentPlayerPos))
	{
		ApplyParallaxOffset(CurrentPlayerPos);
	}

	// Process pending spawn-scan results, now that VT and the frame of reference are resolved.
	ProcessPendingScanResults();

	// Drive all active galaxies with the resolved player position. Galaxies have UE tick
	// disabled, so this is their only per-frame entry point, cascading to their star systems.
	{
		SVO_GT_SCOPE("Universe::GalaxyCascade");
		for (auto& Pair : SpawnedGalaxies)
		{
			AGalaxyActor* Galaxy = Pair.Value.Get();
			if (!Galaxy) continue;

			if (Galaxy->InitializationState == ELifecycleState::Ready)
			{
				if (Galaxy->bPendingPlacement)
				{
					FinalizeGalaxyPlacement(Galaxy);
				}
				Galaxy->TickFromParent(DeltaTime, CurrentFrameOfReferenceLocation);
			}
		}
	}

	const FTierStreamingContext Ctx = BuildStreamingContext();
	// Must mirror InitializeNiagara's gates exactly: an uninitialized tier has no buffers.
	if (UniverseParams.bEnableLargeTier)
	{
		FTierStreamingSystem::UpdateTier(Ctx, LargeTierConfig, LargeTierState);
	}
	if (UniverseParams.bEnableMidTier)
	{
		FTierStreamingSystem::UpdateTier(Ctx, MidTierConfig, MidTierState);
	}
	if (UniverseParams.bEnableSmallTier)
	{
		FTierStreamingSystem::UpdateTier(Ctx, SmallTierConfig, SmallTierState);
	}

	CheckOctreeBounds();

	// Single hierarchical scan, after the full tick cascade so all VTs are resolved.
	DetermineAndDispatchScan();

	// Backdrop capture runs last: every virtual position for this frame is resolved, so the
	// captured image matches what the main view will composite.
	UpdateBackdropCapture();

	// Emit the once-per-second game-thread profile summary. Root tick only.
	SVO_GT_FLUSH();
}
#pragma endregion

#pragma region Shutdown
void AUniverseActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	InitializationState = ELifecycleState::Pooling;

	// Tear down the owned backdrop volume, a separate spawned actor; the capture component and
	// MID are UPROPERTYs on this actor and GC with it.
	if (BackdropPPVolume)
	{
		BackdropPPVolume->Destroy();
		BackdropPPVolume = nullptr;
	}

	// Signal any in-flight galaxy initializations to abort, then clear tracking.
	for (auto& Pair : SpawnedGalaxies)
	{
		if (AGalaxyActor* Galaxy = Pair.Value.Get())
			Galaxy->InitializationState = ELifecycleState::Pooling;
	}
	SpawnedGalaxies.Empty();
	if (PoolManager) { PoolManager->Shutdown(); PoolManager = nullptr; }

	// Drain any in-flight pushes before destroying the components they may touch.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
		FTierStreamingSystem::BeginShutdownDrain(*Tier);

	// Mirror ResetForPool: also wait out an in-flight boundary-cross task, which writes actor
	// members and would keep doing so through teardown. Safe on the GT, having no rendezvous.
	// The async INIT chain is NOT waited here -- it would deadlock a GT wait.
	for (FParticleTierState* Tier : { &LargeTierState, &MidTierState, &SmallTierState })
	{
		while (Tier->bUpdateInProgress.load())
			FPlatformProcess::Sleep(0.0005f);
	}

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
	}
	TierNiagaraComponents.Empty();

	// Clear scan state (no timer to stop)
	bHasPendingScanResults = false;
	PendingScanResults.Empty();
	TrackedSpawnNodes.Empty();
	bSpawnScanInProgress.store(false);

	Super::EndPlay(EndPlayReason);
}
#pragma endregion

#pragma region Galaxy Pooled Spawn Hooks
void AUniverseActor::SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Universe::SpawnGalaxyFromPool");
	if (!InNode.IsValid() || SpawnedGalaxies.Contains(InNode) || InitializationState != ELifecycleState::Ready) return;
	if (InNode->Data.ParticleIndex < 0) return;

	UActorPoolManager* PM = GetPoolManager();
	if (!PM) return;
	AGalaxyActor* Galaxy = PM->Acquire<AGalaxyActor>();   // OnAcquired() runs ResetForSpawn
	if (!Galaxy) return;                                  // pool grow failed; manager already warned
	Galaxy->Universe = this;
	SpawnedGalaxies.Add(InNode, TWeakObjectPtr<AGalaxyActor>(Galaxy));

	// TypeId carries the tier index (0=Large, 1=Mid, 2=Small), from InsertParticleIntoOctree.
	const int32 TierIndex = FMath::Clamp(InNode->Data.TypeId, 0, 2);
	FParticleTierState* TierStates[] = { &LargeTierState, &MidTierState, &SmallTierState };
	FParticleTierState& MatchedState = *TierStates[TierIndex];

	// Resolve the real particle position/extent from the tier buffer (the octree node centre is
	// a quantized approximation). SINGLE-BUFFER READ GUARD: only read the CPU arrays while no
	// transition is in flight; otherwise the octree fallback is the correct answer.
	FVector ParticlePos = InNode->Center;
	float ParticleExtent = static_cast<float>(InNode->Extent);
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	if (AbsIdx >= 0 && MatchedState.Buffers.Num() > 0 &&
		!MatchedState.bUpdateInProgress.load())
	{
		// INDEX-CHECKED, as the orientation read below already was. The transition guard says
		// no worker is mid-write; it says nothing about the array being long enough for an
		// index the NODE carries, and a node outlives the buffer contents it was inserted from.
		const FNiagaraParticleBuffer& Buf = MatchedState.Buffers[0];
		if (Buf.Positions.IsValidIndex(AbsIdx) && Buf.Extents.IsValidIndex(AbsIdx))
		{
			ParticlePos = Buf.Positions[AbsIdx];
			ParticleExtent = Buf.Extents[AbsIdx];
		}
	}

	// Config -> Generate -> resolved params. Generate sets Seed and ParentColor from the node
	// itself, so what remains below are the cross-layer spawn-time fields the PARENT owns --
	// derived Extent and seed rotation -- which Generate has no business knowing about.
	FGalaxyParams P = FGalaxySpawnConfig::Generate(GalaxySpawnConfig, *InNode);

	// INVERTED DERIVATION: UnitScale is the per-layer constant and the galaxy's LOCAL Extent is
	// derived from the particle's real size, so real size drives star COUNT while content sizes
	// stay perceptually identical across galaxies.
	{
		const double DerivedExtent =
			(static_cast<double>(ParticleExtent) * this->UniverseParams.UnitScale)
			/ P.UnitScale;
		P.Extent = FMath::Clamp(DerivedExtent, P.MinDerivedExtent, P.MaxDerivedExtent);
		if (P.Extent != DerivedExtent)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("AUniverseActor::SpawnGalaxy - derived extent %.3e clamped to %.3e; ")
				TEXT("retune GalaxyParams.UnitScale (layer constant) or the clamp bounds."),
				DerivedExtent, P.Extent);
		}
	}

	// ORIENTATION INHERITED FROM THE PARENT PARTICLE. The buffer rotation is already a uniform
	// unit vector -- see UniverseDataGenerator, which draws it with GetUnitVector and hands it
	// to Niagara for sprite orientation. Reusing it costs one array read, arrives correctly
	// distributed on the sphere, and keeps the galaxy facing the way the sprite that stood in
	// for it was facing, so approaching one does not snap the disc.
	//
	// Same single-buffer read guard as the position above, so this can only specialise the
	// archetype's value. NOTHING WRITES Params.Rotation ANY MORE -- it was rolled here and read
	// by nobody, which is why every galaxy came out disc-up.
	if (GalaxySpawnConfig.bInheritParticleOrientation && AbsIdx >= 0 &&
		MatchedState.Buffers.Num() > 0 && !MatchedState.bUpdateInProgress.load())
	{
		const FNiagaraParticleBuffer& OrientBuf = MatchedState.Buffers[0];
		if (OrientBuf.Rotations.IsValidIndex(AbsIdx))
		{
			const FVector& Normal = OrientBuf.Rotations[AbsIdx];
			if (!Normal.IsNearlyZero())
			{
				P.Procedural.Orientation.FieldNormal = FVector3f(Normal);
			}
		}
	}

	// Typed re-init: sets Params, arms deferred placement, hides, and runs the async init
	// chain. FinalizeGalaxyPlacement positions and unhides once Ready.
	Galaxy->ReInit(P, FTransform(ParticlePos));

	UE_LOG(LogTemp, Log,
		TEXT("AUniverseActor::SpawnGalaxyFromPool - inert=%d node=(%.1f,%.1f,%.1f) extent=%.1f "
			"particlePos=(%.1f,%.1f,%.1f) particleExtent=%.3f unitScale(const)=%.6e derivedExtent=%.6e seed=%d"),
		PM->NumInert(AGalaxyActor::StaticClass()),
		InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent,
		ParticlePos.X, ParticlePos.Y, ParticlePos.Z, ParticleExtent,
		P.UnitScale, P.Extent, P.Seed);
}

void AUniverseActor::FinalizeGalaxyPlacement(AGalaxyActor* Galaxy)
{
	SVO_GT_SCOPE("Universe::FinalizeGalaxyPlacement");
	if (!Galaxy || !Galaxy->bPendingPlacement) return;

	const FVector SpawnLoc = ComputeChildSpawnLocation(Galaxy->PendingNodeCenter, Galaxy->Params.UnitScale);
	Galaxy->SetActorLocation(SpawnLoc);

	// NO SetActorRotation HERE, deliberately. Orientation lives in the density field, not on
	// the actor transform. Rotating the actor looked cheaper -- the proxy and the Niagara
	// components both inherit it for free -- but it also rotates the octree, the tier grids and
	// VirtualTraversal, and the streaming scheme assumes galaxy-local and world axes coincide:
	// UpdateSpawnScan assigns VirtualTraversal, a WORLD delta, straight into a LOCAL position,
	// and SpawnScan adds a LOCAL octree centre to a world location unrotated.
	Galaxy->SetActorHiddenInGame(false);

	Galaxy->VirtualTraversal = CurrentFrameOfReferenceLocation - SpawnLoc;
	Galaxy->LastPushedVirtualTraversal = Galaxy->VirtualTraversal;
	Galaxy->LastFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;
	Galaxy->CurrentFrameOfReferenceLocation = CurrentFrameOfReferenceLocation;

	Galaxy->bPendingPlacement = false;

	UE_LOG(LogTemp, Log, TEXT("=== FinalizeGalaxyPlacement ==="));
	UE_LOG(LogTemp, Log, TEXT("  Galaxy: spawnLoc=(%.1f, %.1f, %.1f) VT=(%.1f, %.1f, %.1f) playerPos=(%.1f, %.1f, %.1f)"),
		SpawnLoc.X, SpawnLoc.Y, SpawnLoc.Z,
		Galaxy->VirtualTraversal.X, Galaxy->VirtualTraversal.Y, Galaxy->VirtualTraversal.Z,
		CurrentFrameOfReferenceLocation.X, CurrentFrameOfReferenceLocation.Y, CurrentFrameOfReferenceLocation.Z);
}

void AUniverseActor::ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode)
{
	SVO_GT_SCOPE("Universe::ReturnGalaxyToPool");
	if (!InNode.IsValid()) return;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	TWeakObjectPtr<AGalaxyActor> WeakGalaxy;
	if (!SpawnedGalaxies.RemoveAndCopyValue(InNode, WeakGalaxy)) return;
	AGalaxyActor* Galaxy = WeakGalaxy.Get();
	if (!Galaxy) return;
	UE_LOG(LogTemp, Log, TEXT("Returning galaxy to pool for node seed: %d"), InNode->Data.Seed);

	// Abort signal for an in-flight async init chain, checked between phases and inside
	// InitializeTier via GetLiveState.
	Galaxy->InitializationState = ELifecycleState::Pooling;

	if (!Galaxy->bInitInProgress.load())
	{
		// FAST PATH: no init chain in flight, the normal case. Race-free on the GT, since the
		// flag is raised on the GT in Initialize() before the chain dispatches, so a false read
		// here means the chain fully exited or never ran.
		Galaxy->ResetForPool();
		FinishGalaxyPoolReturn(WeakGalaxy);
		return;
	}

	// DEFERRED RETURN: the init chain still owns the tier buffers -- its generation
	// ParallelFors write Buffers/SlotCounts and never raise bUpdateInProgress, so
	// ResetForPool's transition-drain would free live arrays under the workers. We also cannot
	// wait on the GAME THREAD: the chain rendezvouses with the GT for Niagara component spawns,
	// so a GT spin deadlocks. Park a worker to wait it out, then tear down on the GT.
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakGalaxy]()
		{
			while (true)
			{
				AGalaxyActor* G = WeakGalaxy.Get();
				if (!G) return;
				if (!G->bInitInProgress.load()) break;
				FPlatformProcess::Sleep(0.001f);
			}
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakGalaxy]()
				{
					AUniverseActor* Self = WeakThis.Get();
					AGalaxyActor* G = WeakGalaxy.Get();
					if (!Self || !G) return;
					G->ResetForPool();
					Self->FinishGalaxyPoolReturn(WeakGalaxy);
				});
		});
}

void AUniverseActor::FinishGalaxyPoolReturn(TWeakObjectPtr<AGalaxyActor> WeakGalaxy)
{
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, WeakGalaxy]()
		{
			AGalaxyActor* AsyncGalaxy = WeakGalaxy.Get();
			if (!AsyncGalaxy || !AsyncGalaxy->Octree.IsValid()) return;
			double StartTime = FPlatformTime::Seconds();
			AsyncGalaxy->Octree->bIsResetting.store(true);
			FPlatformProcess::Yield();  // Let in-flight octree ops see the flag and bail.
			// Build the fresh tree in a LOCAL; the MEMBER swap happens on the game thread
			// below, and the old tree dies when its last shared ref drops.
			TSharedPtr<FOctree> FreshTree = MakeShared<FOctree>(AsyncGalaxy->Params.Extent);
			UE_LOG(LogTemp, Log, TEXT("AGalaxyActor::Flushing Octree took: %.3f seconds"), FPlatformTime::Seconds() - StartTime);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, WeakGalaxy, FreshTree]()
				{
					AUniverseActor* Self = WeakThis.Get();
					AGalaxyActor* InnerGalaxy = WeakGalaxy.Get();
					if (!InnerGalaxy) return;

					// THE OLD TREE IS HANDED OFF, NOT DROPPED HERE -- the same reasoning as the
					// rebase swap in CheckOctreeBounds. Assigning over InnerGalaxy->Octree
					// releases the last reference, and FOctreeNode owns its children through
					// TSharedPtr, so that release destroys every node recursively and
					// synchronously on whichever thread performs it. A galaxy's tree carries
					// its star systems, so this is a full teardown on one frame.
					TSharedPtr<FOctree> OldTree = InnerGalaxy->Octree;

					InnerGalaxy->Octree = FreshTree;   // swap on game thread

					AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
						[OldTree = MoveTemp(OldTree)]() mutable
						{
							OldTree.Reset();
						});

					if (Self) { if (UActorPoolManager* PM = Self->GetPoolManager()) PM->ReturnPrepared(InnerGalaxy); }
				});
		});
}
#pragma endregion

#pragma region Public Octree Queries
TArray<TSharedPtr<FOctreeNode>> AUniverseActor::GetNodesByScreenSpace(const FVector& InCenter, double InScreenSpaceThreshold, int32 InTypeId) const
{
	if (!Octree.IsValid()) return {};
	return Octree->GetNodesByScreenSpace(InCenter, InScreenSpaceThreshold, -1, -1, InTypeId);
}
#pragma endregion

#pragma region Spawn Range Scanning
void AUniverseActor::RequestScan()
{
	SVO_GT_SCOPE("Universe::RequestScan");
	if (InitializationState != ELifecycleState::Ready) return;
	if (!Octree.IsValid()) return;
	if (bSpawnScanInProgress.load()) return;

	const double Now = FPlatformTime::Seconds();
	if (Now - LastScanDispatchTime < SpawnScanInterval) return;
	LastScanDispatchTime = Now;

	bSpawnScanInProgress.store(true);
	const FVector LocalPlayerPos = VirtualTraversal;
	// Snapshot the tree ref ON THE GAME THREAD (one atomic ref-count bump). The rebase
	// completion swaps this->Octree on the GT, so reading the member from the worker would be a
	// torn read; the snapshot also keeps the pre-rebase tree alive until this scan finishes.
	TSharedPtr<FOctree> TreeSnapshot = Octree;
	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, LocalPlayerPos, TreeSnapshot]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self || !TreeSnapshot.IsValid()) return;
			const TArray<TSharedPtr<FOctreeNode>> NearbyArray = TreeSnapshot->GetNodesByScreenSpace(LocalPlayerPos, Self->SpawnScreenSpaceThreshold);
			AsyncTask(ENamedThreads::GameThread, [WeakThis, NearbyArray, TreeSnapshot]()
				{
					AUniverseActor* InnerSelf = WeakThis.Get();
					if (!InnerSelf) return;
					InnerSelf->bSpawnScanInProgress.store(false);
					// A rebase swapped the tree while this scan was in flight: these nodes
					// belong to the retired tree and would diff against the remapped tracking
					// as all-new or all-exited. Drop them; the next interval rescans.
					if (InnerSelf->Octree != TreeSnapshot) return;
					InnerSelf->PendingScanResults = NearbyArray;
					InnerSelf->bHasPendingScanResults = true;
				});
		});
}

void AUniverseActor::DetermineAndDispatchScan()
{
	SVO_GT_SCOPE("Universe::DetermineAndDispatchScan");
	if (InitializationState != ELifecycleState::Ready) return;

	// Walk deepest-first: star systems -> galaxies -> universe, one scan dispatched per tick.
	for (auto& GalaxyPair : SpawnedGalaxies)
	{
		AGalaxyActor* Galaxy = GalaxyPair.Value.Get();
		if (!Galaxy || Galaxy->InitializationState != ELifecycleState::Ready)
			continue;

		for (auto& SystemPair : Galaxy->SpawnedStarSystems)
		{
			AStarSystemActor* System = SystemPair.Value.Get();
			if (!System || System->InitializationState != ELifecycleState::Ready)
				continue;

			if (System->IsPlayerInsideBounds())
			{
				System->RequestScan();
				return;  // Short-circuit: deepest level found
			}
		}

		if (Galaxy->IsPlayerInsideBounds())
		{
			Galaxy->RequestScan();
			return;  // Short-circuit: galaxy level
		}
	}

	// Player isn't inside any child - scan at universe level
	RequestScan();
}

void AUniverseActor::ProcessPendingScanResults()
{
	SVO_GT_SCOPE("Universe::ProcessPendingScanResults");
	if (!bHasPendingScanResults) return;
	bHasPendingScanResults = false;

	TSet<TSharedPtr<FOctreeNode>> NearbySet(PendingScanResults);
	PendingScanResults.Empty();

	for (const TSharedPtr<FOctreeNode>& Node : NearbySet)
	{
		// Log on first view-entry only (avoids per-scan spam on retries).
		if (!TrackedSpawnNodes.Contains(Node))
			LogSpawnNodeEnter(Node);

		// Gate the SPAWN on whether it actually spawned, not on whether we have seen it: a
		// first attempt that early-returns must retry next scan.
		if (!SpawnedGalaxies.Contains(Node))
			SpawnGalaxyFromPool(Node);

		if (bDebugDrawSpawnNodes) DebugDrawSpawnNode(Node);
	}

	// Despawn on view-exit. ReturnGalaxyToPool no-ops on a node that never spawned.
	TSet<TSharedPtr<FOctreeNode>> Exited = TrackedSpawnNodes.Difference(NearbySet);
	for (const TSharedPtr<FOctreeNode>& Node : Exited)
	{
		LogSpawnNodeExit(Node);
		ReturnGalaxyToPool(Node);
	}
	TrackedSpawnNodes = MoveTemp(NearbySet);
}

void AUniverseActor::LogSpawnNodeEnter(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const int32 Seed = InNode->Data.Seed;
	const int32 AbsIdx = InNode->Data.ParticleIndex;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan ENTER - node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d seed=%d bufIdx=%d scale=%.3f tier=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, Seed, AbsIdx, InNode->Data.ScaleFactor, InNode->Data.TypeId);
	if (bLogSpawnEnterExitBuffers)
	{
		// Use TypeId (set by InsertParticleIntoOctree) to identify the source tier.
		const FParticleTierConfig* Config = nullptr;
		const FParticleTierState* State = nullptr;
		const TCHAR* TierLabel = TEXT("unknown");
		switch (InNode->Data.TypeId)
		{
		case 0: Config = &LargeTierConfig; State = &LargeTierState; TierLabel = TEXT("coarse"); break;
		case 1: Config = &MidTierConfig;    State = &MidTierState;    TierLabel = TEXT("mid");    break;
		case 2: Config = &SmallTierConfig;  State = &SmallTierState;  TierLabel = TEXT("small");  break;
		default: break;
		}
		if (Config && State && State->Buffers.Num() > 0 && AbsIdx >= 0 &&
			!State->bUpdateInProgress.load()) // single-buffer read guard (GT)
		{
			const int32 SlotId = AbsIdx / Config->SlotCapacity;
			const FNiagaraParticleBuffer& Buf = State->Buffers[0];
			const int32 Start = SlotId * Config->SlotCapacity;
			// BOUNDED BY THE ARRAY, not by the config, for the same reason as
			// SpawnGalaxyFromPool: the slot index comes from a node and the extent from a
			// buffer.
			const int32 End = FMath::Min(Start + Config->SlotCapacity, Buf.Extents.Num());
			int32 LiveCount = 0;
			for (int32 i = Start; i < End; ++i)
			{
				if (Buf.Extents[i] > 0.0f) ++LiveCount;
			}
			UE_LOG(LogTemp, Log, TEXT("  %s slot %d: %d live particles of %d capacity"), TierLabel, SlotId, LiveCount, Config->SlotCapacity);
		}
	}
}

void AUniverseActor::LogSpawnNodeExit(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	UE_LOG(LogTemp, Log, TEXT("AUniverseActor::SpawnScan EXIT  - node center=(%.1f, %.1f, %.1f) extent=%.2f depth=%d seed=%d"), InNode->Center.X, InNode->Center.Y, InNode->Center.Z, InNode->Extent, InNode->Depth, InNode->Data.Seed);
}

void AUniverseActor::DebugDrawSpawnNode(const TSharedPtr<FOctreeNode>& InNode) const
{
	if (!InNode.IsValid()) return;
	const UWorld* World = GetWorld();
	if (!World) return;
	const FVector NodeCenterWorld = GetActorLocation() + InNode->Center - VirtualTraversal;
	const FVector BoxExtent(InNode->Extent);
	DrawDebugBox(World, NodeCenterWorld, BoxExtent, FColor::Green, false, SpawnScanInterval, 0, 2000.0f);
}
#pragma endregion

#pragma region Octree Bounds Check
void AUniverseActor::CheckOctreeBounds()
{
	SVO_GT_SCOPE("Universe::CheckOctreeBounds");
	if (!Octree.IsValid() || bRebaseInProgress.load()) return;

	// Measure against the tree's ACTUAL centre, not the origin: after a rebase the root is
	// centred on the player, so an origin-relative test would re-fire every tick.
	const FVector TreeCenter = Octree->Root.IsValid() ? Octree->Root->Center : FVector::ZeroVector;
	const double  Margin = Octree->Extent * 0.75;
	const FVector Delta = VirtualTraversal - TreeCenter;

	if (FMath::Abs(Delta.X) <= Margin &&
		FMath::Abs(Delta.Y) <= Margin &&
		FMath::Abs(Delta.Z) <= Margin)
		return;

	// Don't rebase while any tier owns its back buffer / state.
	if (LargeTierState.bUpdateInProgress.load() ||
		MidTierState.bUpdateInProgress.load() ||
		SmallTierState.bUpdateInProgress.load())
		return;

	bRebaseInProgress.store(true);
	const FVector RebaseOrigin = VirtualTraversal;
	const double  TreeExtent = GetPersistentTreeExtent();

	// THE WHOLE OPERATION IS TIMED IN THREE PARTS, because they fail differently and only one
	// is a frame cost. The BUILD runs off the game thread but holds every tier frozen, so its
	// cost is streaming latency; the SWAP and the RELEASE are frame time directly.
	const double RebaseStart = FPlatformTime::Seconds();

	TWeakObjectPtr<AUniverseActor> WeakThis(this);
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [WeakThis, RebaseOrigin, TreeExtent, RebaseStart]()
		{
			AUniverseActor* Self = WeakThis.Get();
			if (!Self) return;

			// Build + populate the new tree in a LOCAL ptr off the game thread, then hand it
			// back for the swap. This removes the torn-read race the original had, which
			// assigned Self->Octree on a worker while GT readers could be mid-copy.
			TSharedPtr<FOctree> NewTree = MakeShared<FOctree>(TreeExtent, RebaseOrigin);

			FTierStreamingContext Ctx = Self->BuildStreamingContext();
			Ctx.Octree = NewTree;   // insert into the new tree, not the live one

			// Tiers are guaranteed idle for this whole task: CheckOctreeBounds refused to start
			// the rebase while any tier had a transition in flight, and UpdateTier gates on
			// bRebaseInProgress, so no new transition can start until the swap completes.
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->LargeTierConfig,
				Self->LargeTierState);
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->MidTierConfig,
				Self->MidTierState);
			FTierStreamingSystem::InsertTierIntoOctree(Ctx, Self->SmallTierConfig,
				Self->SmallTierState);

			const double BuildSeconds = FPlatformTime::Seconds() - RebaseStart;

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NewTree, RebaseOrigin, BuildSeconds]()
				{
					AUniverseActor* S = WeakThis.Get();
					if (!S) return;

					const double SwapStart = FPlatformTime::Seconds();

					// REMAP live spawn bookkeeping onto the new tree BEFORE clearing
					// bRebaseInProgress. SpawnedGalaxies and TrackedSpawnNodes are keyed on
					// node IDENTITY and every node instance just changed -- without this remap
					// the first post-rebase scan sees all-new pointers, despawns EVERY live
					// galaxy and respawns it from the pool. Seed survives, so counterparts are
					// matched by the node-captured particle position plus seed.
					auto FindCounterpart = [&NewTree](const TSharedPtr<FOctreeNode>& Old) -> TSharedPtr<FOctreeNode>
						{
							if (!Old.IsValid()) return nullptr;
							// Descend by the CAPTURED particle position, set at insert and
							// immutable, not the old node centre: the new lattice is offset by
							// the rebase origin, so the two can land in different nodes.
							const FVector P = (Old->Data.ParticleExtent > 0.0f)
								? Old->Data.ParticlePosition
								: Old->Center;
							TSharedPtr<FOctreeNode> Candidate = NewTree->FindNodeAtPosition(P, Old->Depth);
							return (Candidate.IsValid() && Candidate->Data.Seed == Old->Data.Seed)
								? Candidate : nullptr;
						};

					// THE REMAP HAS NO OTHER OBSERVER, and a miss is silent by design: the
					// entry keeps its old key and the next scan re-inits one galaxy. A remap
					// that has stopped matching is a full respawn of every live galaxy at once.
					int32 GalaxyHits = 0;
					int32 TrackedHits = 0;

					TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> RemappedGalaxies;
					RemappedGalaxies.Reserve(S->SpawnedGalaxies.Num());
					for (auto& Pair : S->SpawnedGalaxies)
					{
						TSharedPtr<FOctreeNode> NewNode = FindCounterpart(Pair.Key);
						// Fall back to the old key on a miss: that one galaxy despawns on the
						// next scan, and nothing worse.
						GalaxyHits += NewNode.IsValid() ? 1 : 0;
						RemappedGalaxies.Add(NewNode.IsValid() ? NewNode : Pair.Key, Pair.Value);
					}

					TSet<TSharedPtr<FOctreeNode>> RemappedTracked;
					RemappedTracked.Reserve(S->TrackedSpawnNodes.Num());
					for (const TSharedPtr<FOctreeNode>& Old : S->TrackedSpawnNodes)
					{
						TSharedPtr<FOctreeNode> NewNode = FindCounterpart(Old);
						TrackedHits += NewNode.IsValid() ? 1 : 0;
						RemappedTracked.Add(NewNode.IsValid() ? NewNode : Old);
					}

					const int32 GalaxyTotal = S->SpawnedGalaxies.Num();
					const int32 TrackedTotal = S->TrackedSpawnNodes.Num();

					S->SpawnedGalaxies = MoveTemp(RemappedGalaxies);
					S->TrackedSpawnNodes = MoveTemp(RemappedTracked);

					// Drop results from any scan snapshotted against the old tree -- diffing
					// old-tree nodes against the remapped set would re-introduce the churn this
					// remap removes.
					S->bHasPendingScanResults = false;
					S->PendingScanResults.Empty();

					// THE OLD TREE IS HANDED OFF, NOT DROPPED HERE. FOctreeNode owns its
					// children through TSharedPtr, so releasing the last reference to the root
					// destroys the entire tree recursively and synchronously, wherever that
					// happens -- at the shipped tier capacities, tens of thousands of frees in
					// one frame on the game thread, for a tree nothing will read again.
					// Releasing it on a worker moves that cost off the frame; the refcount is
					// atomic and FOctreeNode owns no UObject, so there is no thread affinity.
					// IT MUST OUTLIVE THE REMAP ABOVE: entries that missed still hold old
					// nodes.
					TSharedPtr<FOctree> OldTree = S->Octree;

					S->Octree = NewTree;                  // swap on game thread
					S->bRebaseInProgress.store(false);

					const double SwapMs = (FPlatformTime::Seconds() - SwapStart) * 1000.0;

					AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
						[OldTree = MoveTemp(OldTree)]() mutable
						{
							OldTree.Reset();
						});

					// ONE LINE PER REBASE, at Warning, because a rebase is rare and every term
					// here is invisible otherwise. A hit rate below 100% is the interesting
					// one.
					UE_LOG(LogTemp, Warning,
						TEXT("AUniverseActor::RebaseOctree -> (%.1f, %.1f, %.1f); build %.1f ms ")
						TEXT("(tiers frozen), swap %.2f ms, galaxies remapped %d/%d, tracked ")
						TEXT("%d/%d."),
						RebaseOrigin.X, RebaseOrigin.Y, RebaseOrigin.Z,
						BuildSeconds * 1000.0, SwapMs,
						GalaxyHits, GalaxyTotal, TrackedHits, TrackedTotal);
				});
		});
}
#pragma endregion
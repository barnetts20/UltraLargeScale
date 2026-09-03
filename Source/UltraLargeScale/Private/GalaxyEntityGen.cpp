// GalaxyEntityGen.cpp

#include "GalaxyEntityGen.h"

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "GlobalShader.h"
#include "ShaderParameterUtils.h"
#include "RenderingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "Engine/Texture.h"
#include "TextureResource.h"

IMPLEMENT_GLOBAL_SHADER(FGalaxyEntityGenCS,
	"/UltraLargeScale/Private/GalaxyEntityGen.usf", "MainCS", SF_Compute);


namespace GalaxyEntityGen
{

	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest)
	{
		const int32 NumCells = InCells.Num();
		if (NumCells == 0 || InEntityCapacity <= 0)
		{
			return;
		}

		// WRITTEN AND NEVER READ: both blocking entry points hand AwaitReadback the same counts
		// as arguments, so these three carry the dispatch's shape for a reader and nothing
		// else.
		OutRequest->NumCells = NumCells;
		OutRequest->EntityCapacity = InEntityCapacity;

		// Snapshot everything the render thread will need. Nothing below may touch InParams or
		// InTierParams after this: the game thread may mutate them the instant this returns.
		const FGalaxyProceduralParams D = InParams.Procedural;
		const double InvUnit = 1.0 / FMath::Max(InParams.UnitScale, UE_DOUBLE_SMALL_NUMBER);

		const float InvGalaxyExtent =
			static_cast<float>(1.0 / FMath::Max(static_cast<double>(InParams.Extent), 1e-9));

		const float PlaceSpawnExponent = InTierParams.SpawnExponent;
		const float PlaceExtentMin = static_cast<float>(InTierParams.MinScale * InvUnit);
		const float PlaceExtentMax = static_cast<float>(InTierParams.MaxScale * InvUnit);
		const float PlaceExtentExponent = InTierParams.ExtentExponent;

		// The tier's calibrated constant: not a budget and not a ceiling, and there is no
		// authored candidate number anywhere in this path.
		const float BudgetScale = FMath::Max(InBudgetScale, 0.0f);

		// The anchor, and the pad that turns a probed peak into an envelope.
		//
		// THE PAD COVERS THE SAMPLING MISS, the one place per-cell normalisation stops
		// cancelling. Acceptance is min(density / envelope, 1) ^ g: below the envelope it
		// divides out exactly, above it the ratio saturates and the cell -- having drawn
		// candidates against E^g -- pays QUADRATICALLY while capping what it keeps. A short
		// envelope is a probe set that missed the cell's peak, so it happens where a cell holds
		// the most density range: the core and the arms under-deliver while the voids around
		// them do not, giving cell-shaped holes that read as a placement or hash fault long
		// before a sampling one. RAISING THIS DOES NOT REMOVE THE CASE -- reach for
		// GALAXY_ENTITYGEN_PROBE_ROUNDS, linear where this is exponential.
		const float BudgetAnchor = kBudgetAnchor;
		constexpr float EnvelopePad = 2.0f;

		// The UObject is captured, NOT dereferenced here. GetResource() on a background worker
		// is a UObject access off the game thread: it can run against an object mid-collection,
		// and texture streaming can swap the resource pointer between the read and the use.
		// Weak, and FOUR OF THEM, not expiring as a group, so a complete set at Dispatch says
		// nothing about the moment the marshalled lambda runs.
		TWeakObjectPtr<UTexture> WeakWarpDisc(InFieldTextures.WarpDisc);
		TWeakObjectPtr<UTexture> WeakWarpHalo(InFieldTextures.WarpHalo);
		TWeakObjectPtr<UTexture> WeakNoiseDisc(InFieldTextures.NoiseDisc);
		TWeakObjectPtr<UTexture> WeakNoiseHalo(InFieldTextures.NoiseHalo);

		// Allocated HERE, on the calling thread. The caller polls the moment Dispatch returns,
		// and readbacks created inside the render command stay null until it reaches the
		// lambda.
		OutRequest->Readback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("GalaxyEntityGenReadback"));
		OutRequest->CountReadback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("GalaxyEntityGenCountReadback"));

		FRHIGPUBufferReadback* EntityReadback = OutRequest->Readback.Get();
		FRHIGPUBufferReadback* CountReadbackPtr = OutRequest->CountReadback.Get();

		// ENQUEUED FROM THE GAME THREAD, not the calling worker: ENQUEUE_RENDER_COMMAND from a
		// background task can execute inline there, and RDG then runs on a thread carrying no
		// rendering task tag (the FTaskTagScope ensure). EVERYTHING THE RENDER COMMAND NEEDS
		// GOES IN ONE SHARED PAYLOAD, captured explicitly by value in both lambdas. DO NOT
		// REPLACE IT WITH NESTED [=] CAPTURES: across two thread hops it is possible to move
		// from the same parameter twice, and the second move yields an empty array.
		struct FDispatchPayload
		{
			TArray<FGalaxyGenCell> Cells;
			FGalaxyProceduralParams Density;
			TWeakObjectPtr<UTexture> WarpDisc;
			TWeakObjectPtr<UTexture> WarpHalo;
			TWeakObjectPtr<UTexture> NoiseDisc;
			TWeakObjectPtr<UTexture> NoiseHalo;
			int32 EntityCapacity = 0;
			int32 KeySeed = 0;
			float BudgetScale = 0.0f;
			bool bCalibrateOnly = false;
			float BudgetAnchor = 1.0f;
			// Overwritten from the constant above before the dispatch is enqueued. Calibration
			// and generation MUST be handed the same pad -- the mass is defined through the
			// envelope, so two would solve the constant against one field and spend it on
			// another.
			float EnvelopePad = 0.0f;
			float InvGalaxyExtent = 0.0f;
			float PlaceSpawnExponent = 0.0f;
			float PlaceExtentMin = 0.0f;
			float PlaceExtentMax = 0.0f;
			float PlaceExtentExponent = 0.0f;
		};

		TSharedRef<FDispatchPayload, ESPMode::ThreadSafe> Payload =
			MakeShared<FDispatchPayload, ESPMode::ThreadSafe>();

		Payload->Cells = MoveTemp(InCells);
		Payload->Density = D;
		Payload->WarpDisc = WeakWarpDisc;
		Payload->WarpHalo = WeakWarpHalo;
		Payload->NoiseDisc = WeakNoiseDisc;
		Payload->NoiseHalo = WeakNoiseHalo;
		Payload->EntityCapacity = InEntityCapacity;
		Payload->KeySeed = InKeySeed;
		Payload->BudgetScale = BudgetScale;
		Payload->bCalibrateOnly = bInCalibrateOnly;
		Payload->BudgetAnchor = BudgetAnchor;
		Payload->EnvelopePad = EnvelopePad;
		Payload->InvGalaxyExtent = InvGalaxyExtent;
		Payload->PlaceSpawnExponent = PlaceSpawnExponent;
		Payload->PlaceExtentMin = PlaceExtentMin;
		Payload->PlaceExtentMax = PlaceExtentMax;
		Payload->PlaceExtentExponent = PlaceExtentExponent;

		auto EnqueueOnRenderThread = [Payload, OutRequest, EntityReadback, CountReadbackPtr]() mutable
			{
				// On the game thread now, so the UObjects are safe to touch. Resolved as a SET,
				// all-or-nothing: each GetResource() can independently come back null.
				UTexture* WarpDiscTex = Payload->WarpDisc.Get();
				UTexture* WarpHaloTex = Payload->WarpHalo.Get();
				UTexture* NoiseDiscTex = Payload->NoiseDisc.Get();
				UTexture* NoiseHaloTex = Payload->NoiseHalo.Get();

				FTextureResource* WarpDiscRes = WarpDiscTex ? WarpDiscTex->GetResource() : nullptr;
				FTextureResource* WarpHaloRes = WarpHaloTex ? WarpHaloTex->GetResource() : nullptr;
				FTextureResource* NoiseDiscRes = NoiseDiscTex ? NoiseDiscTex->GetResource() : nullptr;
				FTextureResource* NoiseHaloRes = NoiseHaloTex ? NoiseHaloTex->GetResource() : nullptr;

				if (!WarpDiscRes || !WarpHaloRes || !NoiseDiscRes || !NoiseHaloRes)
				{
					// Nothing will ever land -- say so, or the worker waits out the whole
					// timeout for a copy never enqueued. NO PARTIAL BIND: three bound and one
					// null is the case that most looks like it worked.
					UE_LOG(LogTemp, Warning,
						TEXT("GalaxyEntityGen: aborting dispatch -- field textures unresolved ")
						TEXT("(WarpDisc %s, WarpHalo %s, NoiseDisc %s, NoiseHalo %s). Placement ")
						TEXT("is GPU-only and samples all four; nothing will be placed."),
						WarpDiscRes ? TEXT("ok") : TEXT("NULL"),
						WarpHaloRes ? TEXT("ok") : TEXT("NULL"),
						NoiseDiscRes ? TEXT("ok") : TEXT("NULL"),
						NoiseHaloRes ? TEXT("ok") : TEXT("NULL"));

					OutRequest->bAborted = true;
					return;
				}

				ENQUEUE_RENDER_COMMAND(GalaxyEntityGenDispatch)(
					[Payload, OutRequest, EntityReadback, CountReadbackPtr,
					WarpDiscRes, WarpHaloRes, NoiseDiscRes, NoiseHaloRes]
					(FRHICommandListImmediate& RHICmdList) mutable
					{
						// Named explicitly rather than reached through a capture chain.
						const TArray<FGalaxyGenCell>& Cells = Payload->Cells;
						const int32 Total = Payload->EntityCapacity;

						const int32 InKeySeed = Payload->KeySeed;
						// EVERY RHI HANDLE. A resource can exist with a null TextureRHI while
						// its mips stream in, and the four assets stream independently.
						if (!WarpDiscRes->TextureRHI || !WarpHaloRes->TextureRHI
							|| !NoiseDiscRes->TextureRHI || !NoiseHaloRes->TextureRHI)
						{
							OutRequest->bAborted = true;
							return;
						}

						FRDGBuilder GraphBuilder(RHICmdList);

						// --- cells in --- Explicit sizes and CopyData, so RDG owns the bytes
						// rather than reading the array at execute time.
						FRDGBufferRef CellBuffer = CreateStructuredBuffer(
							GraphBuilder, TEXT("GalaxyGenCells"),
							sizeof(FGalaxyGenCell), Cells.Num(),
							Cells.GetData(), Cells.Num() * sizeof(FGalaxyGenCell),
							ERDGInitialDataFlags::None);

						// --- entities out --- SourceCopy is REQUIRED and neither factory sets
						// it. AddEnqueueCopyPass reads the buffer on the copy queue, and
						// without the flag Lock returns a short mapping, so the memcpy runs off
						// the end -- a crash with correct-looking sizes on both sides.
						FRDGBufferDesc EntityDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(FUintVector4), Total * 2);
						EntityDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							EntityDesc, TEXT("GalaxyGenEntities"));

						// FIVE counters per cell PLUS ONE global, the entity append cursor. The
						// element count comes from the same expression the copy and the
						// readback use, because writing it three times is what let it drift --
						// and a mismatch survives, since RDG POOLS BUFFERS BY SIZE. Zeroed
						// before the dispatch, or the atomics accumulate across frames.
						const int32 CountElements = CountElementsFor(Cells.Num());

						FRDGBufferDesc CountDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), CountElements);
						CountDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							CountDesc, TEXT("GalaxyGenCounts"));

						// FILLED ONCE AS A VALUE, then copied into a FRESH allocation per pass.
						// FComputeShaderUtils::AddPass runs ClearUnusedGraphResources, which
						// NULLS every parameter the bound shader does not reference: the probe
						// permutation never touches OutEntities, so one shared struct lets the
						// first AddPass clear it and the third die on a missing parameter.
						FGalaxyEntityGenCS::FParameters Common;

						// PF_R32G32B32A32_UINT, matching RWBuffer<uint4>: a format mismatch
						// against the shader declaration is undefined, not an error.
						Common.OutEntities =
							GraphBuilder.CreateUAV(EntityBuffer, PF_R32G32B32A32_UINT);
						Common.InCells = GraphBuilder.CreateSRV(CellBuffer);
						// PF_R32_UINT so the UAV has a FORMAT, which RDG's clear path needs:
						// over a formatless UAV, AddClearUAVPass leaves the counters at
						// whatever the allocator handed back, InterlockedAdd returns a huge
						// index, and the cell writes nothing while looking entirely healthy.
						Common.OutCounts = GraphBuilder.CreateUAV(CountBuffer, PF_R32_UINT);

						AddClearUAVPass(GraphBuilder, Common.OutCounts, 0u);

						Common.NumCells = static_cast<uint32>(Cells.Num());
						Common.BudgetScale = Payload->BudgetScale;
						Common.BudgetAnchor = Payload->BudgetAnchor;
						Common.EnvelopePad = Payload->EnvelopePad;
						Common.EntityCapacity = static_cast<uint32>(Total);
						Common.KeySeed = InKeySeed;
						Common.InvGalaxyExtent = Payload->InvGalaxyExtent;

						Common.PlaceSpawnExponent = Payload->PlaceSpawnExponent;
						Common.PlaceExtentMin = Payload->PlaceExtentMin;
						Common.PlaceExtentMax = Payload->PlaceExtentMax;
						Common.PlaceExtentExponent = Payload->PlaceExtentExponent;

						Common.WarpTexDisc = WarpDiscRes->TextureRHI;
						Common.WarpTexHalo = WarpHaloRes->TextureRHI;
						Common.NoiseTexDisc = NoiseDiscRes->TextureRHI;
						Common.NoiseTexHalo = NoiseHaloRes->TextureRHI;

						// THE SAME STATE FOR ALL FOUR, written out rather than hoisted:
						// TStaticSamplerState::GetRHI returns one cached object per template
						// argument set, so this is four reads of one pointer. MUST MATCH THE
						// MATERIAL, whose pins carry whatever addressing each ASSET was saved
						// with -- clamp there gives the render a different field from this one.
						Common.WarpTexDiscSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.WarpTexHaloSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.NoiseTexDiscSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.NoiseTexHaloSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs, exactly as the material's Custom node passes them
						// -- the two places these values are marshalled, and both break
						// together.
						Payload->Density.FillShaderParameters(Common);

						// ONE GROUP PER CELL for the two per-cell passes, not threads divided
						// by group size: the group IS the unit of work, since a cell has to be
						// culled and enveloped by a reduction across its probes first.
						//
						// WRAPPED ACROSS TWO DIMENSIONS. A dispatch is limited to 65535 groups
						// per dimension and a subdivided calibration grid runs well past it --
						// the small tier alone is a hundred and fifty thousand cells. Exceeding
						// it is an ensure inside RDG followed by no dispatch at all, so the
						// readback times out and the tier reports a failure naming nothing.
						const int32 MaxGroupsPerDim = FMath::Max(
							GRHIGlobals.MaxDispatchThreadGroupsPerDimension.X, 1);

						const int32 GroupsX = FMath::Min(Cells.Num(), MaxGroupsPerDim);
						const int32 GroupsY = FMath::DivideAndRoundUp(Cells.Num(), GroupsX);

						const FIntVector CellGroups(GroupsX, GroupsY, 1);

						Common.DispatchGroupsX = static_cast<uint32>(GroupsX);

						// Two dimensions carry 65535^2 cells, which is four billion. If this
						// ever fires the cell count is the problem, not the layout.
						if (GroupsY > MaxGroupsPerDim)
						{
							UE_LOG(LogTemp, Warning,
								TEXT("GalaxyEntityGen: %d cells needs %d x %d groups, past the ")
								TEXT("dispatch limit of %d per dimension. Reduce the tier's ")
								TEXT("GenerationSubdivision or its GridDepth."),
								Cells.Num(), GroupsX, GroupsY, MaxGroupsPerDim);

							OutRequest->bAborted = true;
							return;
						}

						// TWO PASSES, ONE GRAPH, NO READBACK BETWEEN THEM: they share a UAV and
						// RDG orders them on it, so each cell's envelope crosses from probe to
						// generate entirely on the GPU. AsyncCompute for both, being pure
						// producers writing buffers nothing else in the frame reads.
						constexpr ERDGPassFlags PassFlags = ERDGPassFlags::AsyncCompute;

						auto AddGenPass = [&GraphBuilder, &Common, PassFlags](
							int32 InPass, const TCHAR* InName, const FIntVector& InGroups)
							{
								// One allocation per pass. See the note on Common above:
								// AddPass clears the parameters its shader does not use, so
								// reusing one struct destroys it for whichever pass runs later.
								FGalaxyEntityGenCS::FParameters* P =
									GraphBuilder.AllocParameters<FGalaxyEntityGenCS::FParameters>();

								*P = Common;

								FGalaxyEntityGenCS::FPermutationDomain Permutation;
								Permutation.Set<FGalaxyEntityGenCS::FPassDim>(InPass);

								TShaderMapRef<FGalaxyEntityGenCS> Shader(
									GetGlobalShaderMap(GMaxRHIFeatureLevel), Permutation);

								FComputeShaderUtils::AddPass(
									GraphBuilder, RDG_EVENT_NAME("%s", InName),
									PassFlags, Shader, P, InGroups);
							};

						// Probe always: it produces the per-cell envelope generation rejects
						// against, and the per-cell mass calibration is solved from.
						AddGenPass(FGalaxyEntityGenCS::PassProbe,
							TEXT("GalaxyEntityGen.Probe"), CellGroups);

						// Calibration stops here: it wants the masses and nothing else, and the
						// reduction over them happens on the CPU.
						if (!Payload->bCalibrateOnly)
						{
							AddGenPass(FGalaxyEntityGenCS::PassGenerate,
								TEXT("GalaxyEntityGen.Generate"), CellGroups);
						}

						// --- readback --- Copy on the GPU, fence, and leave. NEVER block here:
						// a synchronous read stalls the render thread for the full pipeline
						// depth.
						AddEnqueueCopyPass(GraphBuilder, EntityReadback, EntityBuffer,
							static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut));

						AddEnqueueCopyPass(GraphBuilder, CountReadbackPtr, CountBuffer,
							static_cast<uint32>(CountElements) * sizeof(uint32));

						GraphBuilder.Execute();

						// Only now can the waiting worker meaningfully poll the fences.
						OutRequest->bSubmitted = true;
					});
			};

		// InitializeTier issues its batch inline; UpdateTier runs on a background worker, so
		// its batch is marshalled and the enqueue happens a hop later.
		const bool bInline = IsInGameThread();

		if (bInline)
		{
			EnqueueOnRenderThread();
		}
		else
		{
			AsyncTask(ENamedThreads::GameThread, MoveTemp(EnqueueOnRenderThread));
		}
	}

	/** Shared tail of the two blocking entry points: enqueue the staging copy on the
	 *  render thread and wait for it.
	 *
	 *  Lock() maps through the RHI and crashes anywhere but the render thread, so the
	 *  worker -- which may poll the fence safely -- hands the copy over and waits again
	 *  rather than reading it directly. Marshalled through the game thread for the same
	 *  reason the dispatch is: ENQUEUE_RENDER_COMMAND from a background task can run
	 *  inline on that task, which puts RHI work back on the thread that must not do it. */
	static bool AwaitReadback(
		TSharedRef<FGalaxyEntityGenRequest> Request,
		int32 InTotal,
		int32 InNumCells,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts)
	{
		const double Deadline = FPlatformTime::Seconds() + ReadbackTimeoutSeconds;

		while (!Request->IsReady())
		{
			if (FPlatformTime::Seconds() > Deadline)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("GalaxyEntityGen: readback timed out after %.2fs; these slots get nothing."),
					ReadbackTimeoutSeconds);
				return false;
			}

			FPlatformProcess::Sleep(0.0005f);
		}

		if (Request->bAborted)
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Req = Request;

		auto EnqueueCopy = [Req, InTotal, InNumCells]() mutable
			{
				ENQUEUE_RENDER_COMMAND(GalaxyEntityGenCopy)(
					[Req, InTotal, InNumCells](FRHICommandListImmediate&) mutable
					{
						const uint32 Bytes =
							static_cast<uint32>(InTotal) * sizeof(FGalaxyEntityOut);
						const uint32 CountBytes =
							static_cast<uint32>(CountElementsFor(InNumCells)) * sizeof(uint32);

						const FGalaxyEntityOut* Src =
							static_cast<const FGalaxyEntityOut*>(Req->Readback->Lock(Bytes));

						if (Src)
						{
							Req->Entities.SetNumUninitialized(InTotal);
							FMemory::Memcpy(Req->Entities.GetData(), Src, Bytes);
						}
						Req->Readback->Unlock();

						const uint32* CountSrc =
							static_cast<const uint32*>(Req->CountReadback->Lock(CountBytes));

						if (CountSrc)
						{
							Req->Counts.SetNumUninitialized(CountElementsFor(InNumCells));
							FMemory::Memcpy(Req->Counts.GetData(), CountSrc, CountBytes);
						}
						Req->CountReadback->Unlock();

						Req->bCopied = true;
					});
			};

		if (IsInGameThread())
		{
			EnqueueCopy();
		}
		else
		{
			AsyncTask(ENamedThreads::GameThread, MoveTemp(EnqueueCopy));
		}

		const double CopyDeadline = FPlatformTime::Seconds() + ReadbackTimeoutSeconds;

		while (!Request->bCopied)
		{
			if (FPlatformTime::Seconds() > CopyDeadline)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("GalaxyEntityGen: staging copy timed out after %.2fs; these slots get nothing."),
					ReadbackTimeoutSeconds);
				return false;
			}

			FPlatformProcess::Sleep(0.0005f);
		}

		if (Request->Entities.Num() != InTotal
			|| Request->Counts.Num() != CountElementsFor(InNumCells))
		{
			return false;
		}

		OutEntities = MoveTemp(Request->Entities);
		OutCounts = MoveTemp(Request->Counts);

		Request->Readback.Reset();
		Request->CountReadback.Reset();

		return true;
	}

	bool CalibrateBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		TArray<float>& OutCellMass)
	{
		OutCellMass.Reset();

		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("CalibrateBlocking must run on a background thread"));
			return false;
		}

		// ALL FOUR OR NONE -- see FGalaxyFieldTextures. Calibrating against a partial set fits
		// budgets to a field the material does not draw, and those budgets are cached and
		// reused.
		if (InCells.Num() == 0 || !InFieldTextures.IsComplete())
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		// A token entity buffer. The generate pass never runs here, so nothing writes it -- but
		// RDG still needs a bound, non-empty UAV, and the readback still copies it.
		constexpr int32 TokenCapacity = 64;

		Dispatch(InParams, InTierParams, InCells, TokenCapacity, InKeySeed,
			InFieldTextures, 0.0f, true, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		TArray<FGalaxyEntityOut> Entities;
		TArray<uint32> Counts;

		if (!AwaitReadback(Request, TokenCapacity, InCells.Num(), Entities, Counts))
		{
			return false;
		}

		// THE PER-CELL MASSES ARE ALREADY IN THE READBACK: the probe pass writes one per cell
		// at [i*5+4] and the copy brings the whole counter buffer across, so the reduction
		// needs no GPU pass -- and reducing on the CPU lets the caller take a max of PER-PARENT
		// SUMS, which no single thread group can see.
		OutCellMass.SetNumUninitialized(InCells.Num());

		for (int32 i = 0; i < InCells.Num(); ++i)
		{
			const uint32 Bits = Counts[i * CountersPerCell + 4];
			const float Mass = *reinterpret_cast<const float*>(&Bits);

			// A cell that probed nothing is zero, which is ordinary; anything not finite and
			// non-negative is not, and would poison every cell's budget at once.
			OutCellMass[i] = FMath::IsFinite(Mass) ? FMath::Max(Mass, 0.0f) : 0.0f;
		}

		return true;
	}

	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		const FGalaxyFieldTextures& InFieldTextures,
		float InBudgetScale,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts)
	{
		// Guarding the thread rather than trusting the comment: on the game or render thread
		// this waits on work it is itself preventing, and the hang has no stack that points
		// here.
		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("GenerateBatchBlocking must run on a background thread"));
			return false;
		}

		if (InCells.Num() == 0 || InEntityCapacity <= 0 || !InFieldTextures.IsComplete())
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		Dispatch(InParams, InTierParams, InCells, InEntityCapacity, InKeySeed,
			InFieldTextures, InBudgetScale, false, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		if (!AwaitReadback(Request, InEntityCapacity, InCells.Num(), OutEntities, OutCounts))
		{
			return false;
		}

		// NOTHING IS CLAMPED HERE: no cell owns a run in a shared buffer, so there is no width
		// to clip a per-cell count against and the counts are pure diagnostics. The global
		// cursor deliberately over-counts past capacity, being the true total accepted;
		// clamping would make an overflow report exactly full.
		return true;
	}

} // namespace GalaxyEntityGen
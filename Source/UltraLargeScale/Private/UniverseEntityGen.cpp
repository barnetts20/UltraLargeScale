// UniverseEntityGen.cpp

#include "UniverseEntityGen.h"

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "GlobalShader.h"
#include "ShaderParameterUtils.h"
#include "RenderingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "Engine/Texture.h"
#include "TextureResource.h"

IMPLEMENT_GLOBAL_SHADER(FUniverseEntityGenCS,
	"/UltraLargeScale/Private/UniverseEntityGen.usf", "MainCS", SF_Compute);


namespace UniverseEntityGen
{

	void Dispatch(
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		TArray<FUniverseGenCell> InCells,
		int32 InEntityCapacity,
		int32 InNumSlots,
		int32 InSlotCapacity,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FUniverseEntityGenRequest> OutRequest)
	{
		const int32 NumCells = InCells.Num();
		if (NumCells == 0 || InEntityCapacity <= 0)
		{
			return;
		}

		// WRITTEN AND NEVER READ: both blocking entry points hand AwaitReadback the same counts
		// as arguments, so these carry the dispatch's shape for a reader and nothing else.
		OutRequest->NumCells = NumCells;
		OutRequest->NumSlots = InNumSlots;
		OutRequest->EntityCapacity = InEntityCapacity;

		// Snapshot everything the render thread will need. Nothing below may touch InParams or
		// InTierParams after this: the game thread may mutate them the instant this returns.
		const FUniverseDensityParams D = InParams.DensityParams;
		const double InvUnit = 1.0 / FMath::Max(InParams.UnitScale, UE_DOUBLE_SMALL_NUMBER);

		// SUPPLIED, not derived from InParams.Extent. The field's normalized frame is the RAY
		// MARCH PROXY, whose half extent is the Large tier's neighbourhood rather than the
		// sector extent -- see AUniverseActor::GetVolumetricProxyExtent.
		const float InvFieldExtent = InInvFieldExtent;

		const float PlaceSpawnExponent = InTierParams.SpawnExponent;
		const float PlaceExtentMin = static_cast<float>(InTierParams.MinScale * InvUnit);
		const float PlaceExtentMax = static_cast<float>(InTierParams.MaxScale * InvUnit);
		const float PlaceExtentExponent = InTierParams.ExtentExponent;

		// The tier's calibrated constant: not a budget and not a ceiling, and no authored
		// candidate number exists anywhere in this path.
		const float BudgetScale = FMath::Max(InBudgetScale, 0.0f);

		// The anchor, and the pad that turns a probed peak into an envelope.
		//
		// The pad covers the SAMPLING miss only: both probes and acceptance run
		// SampleAtPosition in the same dispatch, so the only error left is the finite probe
		// count. It matters more here than in the galaxy -- a cosmic web's peaks are the
		// surfaces BETWEEN nodes, so a thin filament crossing a cell can fall between all
		// fifty-six jittered probes. Counter [3] against [2] says how often it does.
		//
		// THE ERROR IS ASYMMETRIC, so the pad leans generous. Over-estimating costs candidates
		// and nothing else, budget scaling as envelope^g against acceptance (d/envelope)^g so
		// the two cancel; under-estimating does not, since everything above the envelope
		// saturates and the surplus is clipped by slot capacity. It cannot simply be raised
		// until clipping stops -- at g = 8 a pad of 2 is 256 times the candidates.
		const float BudgetAnchor = kBudgetAnchor;
		constexpr float EnvelopePad = 1.25f;

		// The UObject is captured, NOT dereferenced here. GetResource() on a background worker
		// is a UObject access off the game thread: it can run against an object mid-collection,
		// and texture streaming can swap the resource pointer between the read and the use.
		// Weak, and FOUR OF THEM, each checked on the game thread below: they do not expire as
		// a group, so a complete set at Dispatch says nothing about when the lambda runs.
		TWeakObjectPtr<UTexture> WeakVarianceA(InFieldTextures.VarianceA);
		TWeakObjectPtr<UTexture> WeakVarianceB(InFieldTextures.VarianceB);
		TWeakObjectPtr<UTexture> WeakWarpLarge(InFieldTextures.WarpLarge);
		TWeakObjectPtr<UTexture> WeakWarpSmall(InFieldTextures.WarpSmall);

		// Allocated HERE, on the calling thread. The caller polls the moment Dispatch returns,
		// and readbacks created inside the render command stay null until it reaches the
		// lambda.
		OutRequest->Readback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("UniverseEntityGenReadback"));
		OutRequest->CountReadback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("UniverseEntityGenCountReadback"));

		FRHIGPUBufferReadback* EntityReadback = OutRequest->Readback.Get();
		FRHIGPUBufferReadback* CountReadbackPtr = OutRequest->CountReadback.Get();

		// ENQUEUED FROM THE GAME THREAD, not the calling worker: ENQUEUE_RENDER_COMMAND from a
		// background task can execute inline there, and RDG then runs on a thread carrying no
		// rendering task tag (the FTaskTagScope ensure). EVERYTHING THE RENDER COMMAND NEEDS
		// GOES IN ONE SHARED PAYLOAD, captured explicitly by value in both lambdas. DO NOT
		// REPLACE IT WITH NESTED [=] CAPTURES: across two thread hops it is possible to move
		// from the same parameter twice, and the second move yields an empty array -- zero
		// cells, nothing written, no error, buffers as cleared.
		struct FDispatchPayload
		{
			TArray<FUniverseGenCell> Cells;
			FUniverseDensityParams Density;
			TWeakObjectPtr<UTexture> VarianceA;
			TWeakObjectPtr<UTexture> VarianceB;
			TWeakObjectPtr<UTexture> WarpLarge;
			TWeakObjectPtr<UTexture> WarpSmall;
			int32 EntityCapacity = 0;
			int32 NumSlots = 0;
			int32 SlotCapacity = 0;
			int32 KeySeed = 0;
			int32 Seed = 0;
			float BudgetScale = 0.0f;
			bool bCalibrateOnly = false;
			float BudgetAnchor = 1.0f;
			float EnvelopePad = 1.0f;
			float InvFieldExtent = 0.0f;
			float PlaceSpawnExponent = 0.0f;
			float PlaceExtentMin = 0.0f;
			float PlaceExtentMax = 0.0f;
			float PlaceExtentExponent = 0.0f;
		};

		TSharedRef<FDispatchPayload, ESPMode::ThreadSafe> Payload =
			MakeShared<FDispatchPayload, ESPMode::ThreadSafe>();

		Payload->Cells = MoveTemp(InCells);
		Payload->Density = D;
		Payload->VarianceA = WeakVarianceA;
		Payload->VarianceB = WeakVarianceB;
		Payload->WarpLarge = WeakWarpLarge;
		Payload->WarpSmall = WeakWarpSmall;
		Payload->EntityCapacity = InEntityCapacity;
		Payload->NumSlots = InNumSlots;
		Payload->SlotCapacity = InSlotCapacity;
		Payload->KeySeed = InKeySeed;
		Payload->Seed = InParams.Seed;
		Payload->BudgetScale = BudgetScale;
		Payload->bCalibrateOnly = bInCalibrateOnly;
		Payload->BudgetAnchor = BudgetAnchor;
		Payload->EnvelopePad = EnvelopePad;
		Payload->InvFieldExtent = InvFieldExtent;
		Payload->PlaceSpawnExponent = PlaceSpawnExponent;
		Payload->PlaceExtentMin = PlaceExtentMin;
		Payload->PlaceExtentMax = PlaceExtentMax;
		Payload->PlaceExtentExponent = PlaceExtentExponent;

		auto EnqueueOnRenderThread = [Payload, OutRequest, EntityReadback, CountReadbackPtr]() mutable
			{
				// On the game thread now, so the UObjects are safe to touch. RESOLVED AS A SET,
				// all-or-nothing: each GetResource() can independently come back null.
				UTexture* VarianceATex = Payload->VarianceA.Get();
				UTexture* VarianceBTex = Payload->VarianceB.Get();
				UTexture* WarpLargeTex = Payload->WarpLarge.Get();
				UTexture* WarpSmallTex = Payload->WarpSmall.Get();

				FTextureResource* VarianceARes = VarianceATex ? VarianceATex->GetResource() : nullptr;
				FTextureResource* VarianceBRes = VarianceBTex ? VarianceBTex->GetResource() : nullptr;
				FTextureResource* WarpLargeRes = WarpLargeTex ? WarpLargeTex->GetResource() : nullptr;
				FTextureResource* WarpSmallRes = WarpSmallTex ? WarpSmallTex->GetResource() : nullptr;

				if (!VarianceARes || !VarianceBRes || !WarpLargeRes || !WarpSmallRes)
				{
					// Nothing will ever land. Say so, or the worker waits out the whole timeout
					// for a copy never enqueued. FATAL RATHER THAN DEGRADED, for ANY of the
					// four: with one missing the field still evaluates -- a missing variance
					// volume takes every regional axis to its midpoint -- and that is a
					// DIFFERENT FIELD from the one the material draws. NO PARTIAL BIND: three
					// resolved and the fourth null most looks like it worked.
					UE_LOG(LogTemp, Warning,
						TEXT("UniverseEntityGen: aborting dispatch -- field textures unresolved ")
						TEXT("(VarianceA %s, VarianceB %s, WarpLarge %s, WarpSmall %s). ")
						TEXT("Placement is GPU-only and samples all four; nothing will be placed."),
						VarianceARes ? TEXT("ok") : TEXT("NULL"),
						VarianceBRes ? TEXT("ok") : TEXT("NULL"),
						WarpLargeRes ? TEXT("ok") : TEXT("NULL"),
						WarpSmallRes ? TEXT("ok") : TEXT("NULL"));

					OutRequest->bAborted = true;
					return;
				}

				ENQUEUE_RENDER_COMMAND(UniverseEntityGenDispatch)(
					[Payload, OutRequest, EntityReadback, CountReadbackPtr,
					VarianceARes, VarianceBRes, WarpLargeRes, WarpSmallRes]
					(FRHICommandListImmediate& RHICmdList) mutable
					{
						// Named explicitly rather than reached through a capture chain.
						const TArray<FUniverseGenCell>& Cells = Payload->Cells;
						const int32 Total = Payload->EntityCapacity;

						const int32 InKeySeed = Payload->KeySeed;

						// EVERY RHI HANDLE, not just one. A resource can exist with a null
						// TextureRHI while its mips stream in, and the four assets stream
						// independently -- so the first frame after a sector loads is exactly
						// when three of four can be ready.
						if (!VarianceARes->TextureRHI || !VarianceBRes->TextureRHI
							|| !WarpLargeRes->TextureRHI || !WarpSmallRes->TextureRHI)
						{
							OutRequest->bAborted = true;
							return;
						}

						FRDGBuilder GraphBuilder(RHICmdList);

						// --- cells in --- Explicit sizes and CopyData, so RDG owns the bytes
						// rather than reading the array at execute time.
						FRDGBufferRef CellBuffer = CreateStructuredBuffer(
							GraphBuilder, TEXT("UniverseGenCells"),
							sizeof(FUniverseGenCell), Cells.Num(),
							Cells.GetData(), Cells.Num() * sizeof(FUniverseGenCell),
							ERDGInitialDataFlags::None);

						// --- entities out --- SourceCopy is REQUIRED and neither factory sets
						// it. AddEnqueueCopyPass reads the buffer on the copy queue, and
						// without the flag Lock returns a mapping shorter than the bytes asked
						// for, so the memcpy runs off the end -- a crash inside Memcpy with
						// correct-looking sizes on both sides.
						FRDGBufferDesc EntityDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(FUintVector4), Total * 2);
						EntityDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							EntityDesc, TEXT("UniverseGenEntities"));

						// FIVE counters per cell PLUS ONE global, the entity append cursor. The
						// element count comes from the same expression the copy and the
						// readback use, because writing it three times is what let it drift in
						// the galaxy path -- and a mismatch survives, since RDG POOLS BUFFERS
						// BY SIZE. Zeroed before the dispatch, or the atomics accumulate across
						// frames.
						const int32 CountElements =
							CountElementsFor(Cells.Num(), Payload->NumSlots);

						FRDGBufferDesc CountDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), CountElements);
						CountDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							CountDesc, TEXT("UniverseGenCounts"));

						// FILLED ONCE AS A VALUE, then copied into a FRESH allocation per pass.
						// FComputeShaderUtils::AddPass runs ClearUnusedGraphResources, which
						// NULLS every parameter the bound shader does not reference: the probe
						// permutation never touches OutEntities, so one shared struct lets the
						// first AddPass clear it and the second die on a missing parameter.
						FUniverseEntityGenCS::FParameters Common;

						// PF_R32G32B32A32_UINT, matching RWBuffer<uint4>: a format mismatch
						// against the shader declaration is undefined, not an error.
						Common.OutEntities =
							GraphBuilder.CreateUAV(EntityBuffer, PF_R32G32B32A32_UINT);
						Common.InCells = GraphBuilder.CreateSRV(CellBuffer);

						// PF_R32_UINT so the UAV has a FORMAT, which RDG's clear path needs:
						// over a formatless UAV, AddClearUAVPass leaves the counters at
						// whatever the allocator handed back and the cell writes nothing while
						// looking healthy.
						Common.OutCounts = GraphBuilder.CreateUAV(CountBuffer, PF_R32_UINT);

						AddClearUAVPass(GraphBuilder, Common.OutCounts, 0u);

						Common.NumCells = static_cast<uint32>(Cells.Num());
						Common.BudgetScale = Payload->BudgetScale;
						Common.BudgetAnchor = Payload->BudgetAnchor;
						Common.EnvelopePad = Payload->EnvelopePad;
						Common.EntityCapacity = static_cast<uint32>(Total);
						Common.NumSlots = static_cast<uint32>(Payload->NumSlots);
						Common.SlotCapacity = static_cast<uint32>(Payload->SlotCapacity);
						Common.KeySeed = InKeySeed;
						Common.InvFieldExtent = Payload->InvFieldExtent;

						Common.PlaceSpawnExponent = Payload->PlaceSpawnExponent;
						Common.PlaceExtentMin = Payload->PlaceExtentMin;
						Common.PlaceExtentMax = Payload->PlaceExtentMax;
						Common.PlaceExtentExponent = Payload->PlaceExtentExponent;

						Common.VarianceTexA = VarianceARes->TextureRHI;
						Common.VarianceTexB = VarianceBRes->TextureRHI;
						Common.WarpTexLarge = WarpLargeRes->TextureRHI;
						Common.WarpTexSmall = WarpSmallRes->TextureRHI;

						// AM_WRAP ON ALL THREE AXES, correctness rather than taste. Every one
						// of the field's UVs is built from a MASKED cell index, so it wraps
						// every 4096 cells, and the two sides of that wrap are the same texel
						// only if the sampler repeats -- clamp mirrors the seam into a wall of
						// constant warp. MUST MATCH THE MATERIAL'S SAMPLERS, whose pins carry
						// whatever addressing each ASSET was authored with; set all four assets
						// to wrap.
						Common.VarianceTexASampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.VarianceTexBSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.WarpTexLargeSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.WarpTexSmallSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs as the material's Custom node passes them, except
						// for A ZERO OFFSET: the proxy's offset is how far the field has
						// scrolled under the VIEW, while the field is static in caller space.
						Payload->Density.FillShaderParameters(
							Common, Payload->Seed, FUniverseFieldOffset());

						// ONE GROUP PER CELL for both per-cell passes, not threads divided by
						// group size: the group IS the unit of work, since a cell has to be
						// culled and enveloped by a reduction across its probes first. WRAPPED
						// ACROSS TWO DIMENSIONS, a dispatch being limited to 65535 groups per
						// dimension -- exceeding it is an ensure inside RDG followed by no
						// dispatch, so the readback times out naming nothing.
						const int32 MaxGroupsPerDim = FMath::Max(
							GRHIGlobals.MaxDispatchThreadGroupsPerDimension.X, 1);

						const int32 GroupsX = FMath::Min(Cells.Num(), MaxGroupsPerDim);
						const int32 GroupsY = FMath::DivideAndRoundUp(Cells.Num(), GroupsX);

						const FIntVector CellGroups(GroupsX, GroupsY, 1);

						Common.DispatchGroupsX = static_cast<uint32>(GroupsX);

						// Two dimensions carry 65535^2 cells, four billion; if this fires the
						// cell count is the problem, not the layout. NOTE: it returns with a
						// clear pass recorded and no Execute, abandoning the builder.
						if (GroupsY > MaxGroupsPerDim)
						{
							UE_LOG(LogTemp, Warning,
								TEXT("UniverseEntityGen: %d cells needs %d x %d groups, past ")
								TEXT("the dispatch limit of %d per dimension. Reduce the ")
								TEXT("tier's GenerationSubdivision or its GridDepth."),
								Cells.Num(), GroupsX, GroupsY, MaxGroupsPerDim);

							OutRequest->bAborted = true;
							return;
						}

						// TWO PASSES, ONE GRAPH, NO READBACK BETWEEN THEM: they share a UAV and
						// RDG orders them on it, so each cell's envelope crosses from probe to
						// generate entirely on the GPU. AsyncCompute for both, being pure
						// producers -- which matters more here than in the galaxy, a universe
						// field sample being a fifty-four candidate walk plus five fetches.
						constexpr ERDGPassFlags PassFlags = ERDGPassFlags::AsyncCompute;

						auto AddGenPass = [&GraphBuilder, &Common, PassFlags](
							int32 InPass, const TCHAR* InName, const FIntVector& InGroups)
							{
								// One allocation per pass. See the note on Common above:
								// AddPass clears parameters its shader does not use.
								FUniverseEntityGenCS::FParameters* P =
									GraphBuilder.AllocParameters<FUniverseEntityGenCS::FParameters>();

								*P = Common;

								FUniverseEntityGenCS::FPermutationDomain Permutation;
								Permutation.Set<FUniverseEntityGenCS::FPassDim>(InPass);

								TShaderMapRef<FUniverseEntityGenCS> Shader(
									GetGlobalShaderMap(GMaxRHIFeatureLevel), Permutation);

								FComputeShaderUtils::AddPass(
									GraphBuilder, RDG_EVENT_NAME("%s", InName),
									PassFlags, Shader, P, InGroups);
							};

						// Probe always: it produces the per-cell envelope generation rejects
						// against, and the per-cell mass calibration is solved from.
						AddGenPass(FUniverseEntityGenCS::PassProbe,
							TEXT("UniverseEntityGen.Probe"), CellGroups);

						// Calibration stops here: it wants the masses and nothing else, and the
						// reduction over them happens on the CPU.
						if (!Payload->bCalibrateOnly)
						{
							AddGenPass(FUniverseEntityGenCS::PassGenerate,
								TEXT("UniverseEntityGen.Generate"), CellGroups);
						}

						// --- readback --- Copy on the GPU, fence, and leave. NEVER block here:
						// a synchronous read stalls the render thread for the full pipeline
						// depth.
						AddEnqueueCopyPass(GraphBuilder, EntityReadback, EntityBuffer,
							static_cast<uint32>(Total) * sizeof(FUniverseEntityOut));

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

	/** Shared tail of the two blocking entry points: enqueue the staging copy on the render
	 *  thread and wait for it.
	 *
	 *  Lock() maps through the RHI and crashes anywhere but the render thread, so the worker
	 *  -- which may poll the fence safely -- hands the copy over and waits again rather than
	 *  reading it directly. Marshalled through the game thread for the same reason the
	 *  dispatch is: ENQUEUE_RENDER_COMMAND from a background task can run inline on that
	 *  task, which puts RHI work back on the thread that must not do it. */
	static bool AwaitReadback(
		TSharedRef<FUniverseEntityGenRequest> Request,
		int32 InTotal,
		int32 InNumCells,
		int32 InNumSlots,
		TArray<FUniverseEntityOut>& OutEntities,
		TArray<uint32>& OutCounts)
	{
		const double Deadline = FPlatformTime::Seconds() + ReadbackTimeoutSeconds;

		while (!Request->IsReady())
		{
			if (FPlatformTime::Seconds() > Deadline)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("UniverseEntityGen: readback timed out after %.2fs; these slots get nothing."),
					ReadbackTimeoutSeconds);
				return false;
			}

			FPlatformProcess::Sleep(0.0005f);
		}

		if (Request->bAborted)
		{
			return false;
		}

		TSharedRef<FUniverseEntityGenRequest> Req = Request;

		auto EnqueueCopy = [Req, InTotal, InNumCells, InNumSlots]() mutable
			{
				ENQUEUE_RENDER_COMMAND(UniverseEntityGenCopy)(
					[Req, InTotal, InNumCells, InNumSlots](FRHICommandListImmediate&) mutable
					{
						const uint32 Bytes =
							static_cast<uint32>(InTotal) * sizeof(FUniverseEntityOut);
						const uint32 CountBytes = static_cast<uint32>(
							CountElementsFor(InNumCells, InNumSlots)) * sizeof(uint32);

						const FUniverseEntityOut* Src =
							static_cast<const FUniverseEntityOut*>(Req->Readback->Lock(Bytes));

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
							Req->Counts.SetNumUninitialized(
								CountElementsFor(InNumCells, InNumSlots));
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
					TEXT("UniverseEntityGen: staging copy timed out after %.2fs; these slots get nothing."),
					ReadbackTimeoutSeconds);
				return false;
			}

			FPlatformProcess::Sleep(0.0005f);
		}

		if (Request->Entities.Num() != InTotal
			|| Request->Counts.Num() != CountElementsFor(InNumCells, InNumSlots))
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
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FUniverseGenCell>& InCells,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		TArray<float>& OutCellMass)
	{
		OutCellMass.Reset();

		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("CalibrateBlocking must run on a background thread"));
			return false;
		}

		// ALL FOUR OR NONE -- see FUniverseFieldTextures. Calibrating against a partial set
		// fits budgets to a field the material does not draw, and those budgets are cached and
		// reused.
		if (InCells.Num() == 0 || !InFieldTextures.IsComplete())
		{
			return false;
		}

		TSharedRef<FUniverseEntityGenRequest> Request = MakeShared<FUniverseEntityGenRequest>();

		// A token entity buffer. The generate pass never runs here, so nothing writes it -- but
		// RDG still needs a bound, non-empty UAV, and the readback still copies it.
		constexpr int32 TokenCapacity = 64;

		// One token slot: the generate pass never runs, so nothing claims one.
		Dispatch(InParams, InTierParams, InCells, TokenCapacity, 1, TokenCapacity,
			InKeySeed, InInvFieldExtent, InFieldTextures, 0.0f, true, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		TArray<FUniverseEntityOut> Entities;
		TArray<uint32> Counts;

		if (!AwaitReadback(Request, TokenCapacity, InCells.Num(), 1, Entities, Counts))
		{
			return false;
		}

		// THE PER-CELL MASSES ARE ALREADY IN THE READBACK: the probe pass writes one per cell
		// at [i*5+4] and the copy brings the whole counter buffer across, so reducing on the
		// CPU lets the caller take a max of PER-PARENT SUMS, which no thread group can see.
		OutCellMass.SetNumUninitialized(InCells.Num());

		for (int32 i = 0; i < InCells.Num(); ++i)
		{
			const uint32 Bits = Counts[i * CountersPerCell + 4];
			const float Mass = *reinterpret_cast<const float*>(&Bits);

			// A cell that probed nothing is zero, ordinary in a mostly-void web; anything not
			// finite and non-negative is not, and would poison every cell's budget at once.
			OutCellMass[i] = FMath::IsFinite(Mass) ? FMath::Max(Mass, 0.0f) : 0.0f;
		}

		return true;
	}

	bool GenerateBatchBlocking(
		const FUniverseParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FUniverseGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InNumSlots,
		int32 InSlotCapacity,
		int32 InKeySeed,
		float InInvFieldExtent,
		const FUniverseFieldTextures& InFieldTextures,
		float InBudgetScale,
		TArray<FUniverseEntityOut>& OutEntities,
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

		TSharedRef<FUniverseEntityGenRequest> Request = MakeShared<FUniverseEntityGenRequest>();

		Dispatch(InParams, InTierParams, InCells, InEntityCapacity, InNumSlots,
			InSlotCapacity, InKeySeed, InInvFieldExtent, InFieldTextures,
			InBudgetScale, false, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		if (!AwaitReadback(Request, InEntityCapacity, InCells.Num(), InNumSlots,
			OutEntities, OutCounts))
		{
			return false;
		}

		// NOTHING IS CLAMPED HERE. The shader caps each slot at SlotCapacity, so the global
		// cursor is the records actually WRITTEN while the per-slot counters carry the true
		// demand.
		return true;
	}

} // namespace UniverseEntityGen
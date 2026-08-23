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
		UTexture* InNoiseTexture,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest)
	{
		const int32 NumCells = InCells.Num();
		if (NumCells == 0 || InEntityCapacity <= 0)
		{
			return;
		}

		OutRequest->NumCells = NumCells;
		OutRequest->EntityCapacity = InEntityCapacity;

		// Snapshot everything the render thread will need. Nothing below may touch
		// InParams or InTierParams after this point: the game thread is free to mutate
		// them the instant this function returns, and a dangling reference into tier
		// params is the kind of race that only reproduces under streaming load.
		const FGalaxyDensityParams D = InParams.DensityParams;
		const double InvUnit = 1.0 / FMath::Max(InParams.UnitScale, UE_DOUBLE_SMALL_NUMBER);

		const float InvGalaxyExtent =
			static_cast<float>(1.0 / FMath::Max(static_cast<double>(InParams.Extent), 1e-9));

		const float PlaceCompression = D.SpawnCompression;
		const float PlaceSpawnExponent = InTierParams.SpawnExponent;
		const float PlaceExtentMin = static_cast<float>(InTierParams.MinScale * InvUnit);
		const float PlaceExtentMax = static_cast<float>(InTierParams.MaxScale * InvUnit);
		const float PlaceExtentExponent = InTierParams.ExtentExponent;

		// The tier's calibrated constant. Not a budget and not a ceiling -- there is no
		// authored candidate number anywhere in this path.
		const float BudgetScale = FMath::Max(InBudgetScale, 0.0f);

		// The anchor, and the pad that turns a probed peak into an envelope.
		//
		// The pad covers the SAMPLING miss only. It was 1.5 and had to cover a
		// systematic error worth more than twice that, because the probes ran the
		// analytic field on the CPU while acceptance ran the textured field on the GPU.
		// Both now run GalaxySample in the same dispatch, so only the finite probe count
		// is left to insure against.
		const float BudgetAnchor = FMath::Max(D.SpawnDensityReference, 1e-3f);
		constexpr float EnvelopePad = 1.15f;

		// The UObject is captured, NOT dereferenced here.
		//
		// GetResource() on a background worker is a UObject access off the game thread:
		// it can run against an object mid-collection, and the resource pointer itself
		// can be swapped by texture streaming between the read and the use. Resolving
		// it on the game thread -- where the object is guaranteed live and the pointer
		// stable -- removes both.
		//
		// A weak pointer, so a galaxy torn down while a batch is in flight resolves to
		// null and the dispatch bails, rather than the marshalled lambda keeping a
		// destroyed texture alive or dereferencing one.
		TWeakObjectPtr<UTexture> WeakNoise(InNoiseTexture);

		// Allocated HERE, on the calling thread, not inside the render command.
		//
		// The caller polls this request the moment Dispatch returns. Creating the
		// readbacks inside the render command left them null until the render thread
		// got round to the lambda, so the poll saw nothing, gave up, and fell back to
		// the CPU -- on nearly every batch. Only the COPY has to happen on the render
		// thread; the objects themselves are just handles.
		OutRequest->Readback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("GalaxyEntityGenReadback"));
		OutRequest->CountReadback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("GalaxyEntityGenCountReadback"));

		FRHIGPUBufferReadback* EntityReadback = OutRequest->Readback.Get();
		FRHIGPUBufferReadback* CountReadbackPtr = OutRequest->CountReadback.Get();

		// Enqueued from the GAME THREAD, not from the calling worker.
		//
		// ENQUEUE_RENDER_COMMAND from a background task can execute the command inline
		// on that task instead of handing it to the render thread, and RDG then runs
		// on a thread carrying no rendering task tag -- which is the
		// FTaskTagScope(EParallelRenderingThread) ensure. Marshalling first costs one
		// hop and removes the whole class of problem.
		//
		// Safe against deadlock because the game thread does not block on generation:
		// InitializeTier hands its rendezvous off through a TPromise and returns, so
		// the game thread keeps pumping while the worker waits on the readback fence.
		// EVERYTHING THE RENDER COMMAND NEEDS, IN ONE SHARED PAYLOAD.
		//
		// This replaces a chain of [=] captures, and the chain was the bug. The outer
		// lambda took Cells with MoveTemp(InCells); the inner lambda then said
		// MoveTemp(InCells) AGAIN -- moving a second time from a parameter the outer
		// capture had already emptied. The render command therefore ran with zero
		// cells and a garbage budget, computed a group count of zero, and dispatched
		// nothing. No error, no RDG complaint, buffers exactly as cleared: which is
		// every symptom this took a long time to find.
		//
		// Nested [=] across two thread hops is what made it possible to write and
		// impossible to see. A single shared payload captured explicitly by value in
		// both lambdas cannot express the same mistake -- there is one copy of the
		// data and both hops name it.
		struct FDispatchPayload
		{
			TArray<FGalaxyGenCell> Cells;
			FGalaxyDensityParams Density;
			TWeakObjectPtr<UTexture> Noise;
			int32 EntityCapacity = 0;
			int32 KeySeed = 0;
			float BudgetScale = 0.0f;
			int32 ProbeRounds = 1;
			bool bCalibrateOnly = false;
			float BudgetAnchor = 1.0f;
			float EnvelopePad = 1.0f;
			float InvGalaxyExtent = 0.0f;
			float PlaceCompression = 0.0f;
			float PlaceSpawnExponent = 0.0f;
			float PlaceExtentMin = 0.0f;
			float PlaceExtentMax = 0.0f;
			float PlaceExtentExponent = 0.0f;
		};

		TSharedRef<FDispatchPayload, ESPMode::ThreadSafe> Payload =
			MakeShared<FDispatchPayload, ESPMode::ThreadSafe>();

		Payload->Cells = MoveTemp(InCells);
		Payload->Density = D;
		Payload->Noise = WeakNoise;
		Payload->EntityCapacity = InEntityCapacity;
		Payload->KeySeed = InKeySeed;
		Payload->BudgetScale = BudgetScale;
		Payload->bCalibrateOnly = bInCalibrateOnly;

		// One round when generating a subcell; many when measuring the parent it came
		// from. The two are not sampling the same volume at the same scale.
		Payload->ProbeRounds = bInCalibrateOnly ? CalibrationProbeRounds : 1;
		Payload->BudgetAnchor = BudgetAnchor;
		Payload->EnvelopePad = EnvelopePad;
		Payload->InvGalaxyExtent = InvGalaxyExtent;
		Payload->PlaceCompression = PlaceCompression;
		Payload->PlaceSpawnExponent = PlaceSpawnExponent;
		Payload->PlaceExtentMin = PlaceExtentMin;
		Payload->PlaceExtentMax = PlaceExtentMax;
		Payload->PlaceExtentExponent = PlaceExtentExponent;

		auto EnqueueOnRenderThread = [Payload, OutRequest, EntityReadback, CountReadbackPtr]() mutable
			{
				// On the game thread now: safe to touch the UObject.
				UTexture* NoiseTexture = Payload->Noise.Get();
				FTextureResource* NoiseResource = NoiseTexture ? NoiseTexture->GetResource() : nullptr;

				if (!NoiseResource)
				{
					// Nothing will ever land. Say so, or the worker waits out the whole
					// timeout for a copy that was never enqueued.
					OutRequest->bAborted = true;
					return;
				}

				ENQUEUE_RENDER_COMMAND(GalaxyEntityGenDispatch)(
					[Payload, OutRequest, EntityReadback, CountReadbackPtr, NoiseResource]
					(FRHICommandListImmediate& RHICmdList) mutable
					{
						// Named explicitly rather than reached through a capture chain.
						const TArray<FGalaxyGenCell>& Cells = Payload->Cells;
						const int32 Total = Payload->EntityCapacity;

						const int32 InKeySeed = Payload->KeySeed;
						if (!NoiseResource->TextureRHI)
						{
							OutRequest->bAborted = true;
							return;
						}

						FRDGBuilder GraphBuilder(RHICmdList);

						// --- cells in ---
						// Explicit sizes and CopyData, so RDG owns the bytes rather than reading
						// the array at execute time. The array does outlive Execute() here, but
						// "does" and "is guaranteed to" are different things and this costs one
						// memcpy of a few kilobytes.
						FRDGBufferRef CellBuffer = CreateStructuredBuffer(
							GraphBuilder, TEXT("GalaxyGenCells"),
							sizeof(FGalaxyGenCell), Cells.Num(),
							Cells.GetData(), Cells.Num() * sizeof(FGalaxyGenCell),
							ERDGInitialDataFlags::None);

						// --- entities out ---
						// SourceCopy is REQUIRED and neither factory sets it.
						//
						// AddEnqueueCopyPass reads the buffer on the copy queue, and without
						// this usage flag the copy does not land the way it claims to: Lock
						// then returns a mapping shorter than the bytes asked for, and the
						// memcpy that follows runs off the end of it. The failure is a crash
						// inside Memcpy with correct-looking sizes on both sides, which sends
						// you hunting for a struct layout mismatch that is not there.
						// Typed float4 elements, three per entity: same bytes, a view the
						// pipeline has already shown it can write through.
						FRDGBufferDesc EntityDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(FVector4f), Total * 3);
						EntityDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							EntityDesc, TEXT("GalaxyGenEntities"));

						// FIVE counters per cell PLUS THREE globals -- the append cursor and the
						// two mass reductions. The element count is derived from the same
						// expression the copy and the readback use, because writing it out
						// three times is what let it drift: this desc kept four per cell while
						// the copy moved to five plus three, so the copy ran off the end of the
						// buffer and the shader's global writes landed outside the view.
						//
						// It survived for a long time because RDG POOLS BUFFERS BY SIZE. A
						// pooled allocation left over from a larger dispatch absorbed both,
						// and the failure only appeared once the pool returned one sized
						// exactly to this desc.
						//
						// Zeroed before the dispatch. Not cleared and the atomics accumulate
						// across frames, which reads as cells that fill once and are empty
						// ever after.
						const int32 CountElements = CountElementsFor(Cells.Num());

						FRDGBufferDesc CountDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), CountElements);
						CountDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							CountDesc, TEXT("GalaxyGenCounts"));

						// FILLED ONCE AS A VALUE, then copied into a FRESH allocation for each
						// pass. Not shared.
						//
						// FComputeShaderUtils::AddPass runs ClearUnusedGraphResources, which
						// NULLS every parameter the bound shader does not reference. The probe
						// permutation never touches OutEntities -- the generate branch is
						// compiled out -- so handing all three passes one struct let the first
						// AddPass clear it, and the third then died on "required shader
						// parameter FParameters::OutEntities was not set".
						//
						// Fresh allocations are also what RDG expects: it records each pass's
						// resource dependencies FROM its parameter struct, so a shared one
						// would conflate three passes' access patterns into one.
						FGalaxyEntityGenCS::FParameters Common;

						Common.OutEntities = GraphBuilder.CreateUAV(EntityBuffer, PF_A32B32G32R32F);
						Common.InCells = GraphBuilder.CreateSRV(CellBuffer);
						// PF_R32_UINT so the UAV has a FORMAT. A structured UAV has none, and RDG's
						// clear path needs one: AddClearUAVPass over a formatless UAV leaves the
						// counters at whatever the allocator handed back, InterlockedAdd then returns
						// a huge index, every entity fails the bounds test, and the cell writes
						// nothing while looking entirely healthy.
						Common.OutCounts = GraphBuilder.CreateUAV(CountBuffer, PF_R32_UINT);

						AddClearUAVPass(GraphBuilder, Common.OutCounts, 0u);

						Common.NumCells = static_cast<uint32>(Cells.Num());
						Common.BudgetScale = Payload->BudgetScale;
						Common.ProbeRounds =
							static_cast<uint32>(FMath::Max(Payload->ProbeRounds, 1));
						Common.BudgetAnchor = Payload->BudgetAnchor;
						Common.EnvelopePad = Payload->EnvelopePad;
						Common.EntityCapacity = static_cast<uint32>(Total);
						Common.KeySeed = InKeySeed;
						Common.InvGalaxyExtent = Payload->InvGalaxyExtent;

						Common.PlaceCompression = Payload->PlaceCompression;
						Common.PlaceSpawnExponent = Payload->PlaceSpawnExponent;
						Common.PlaceExtentMin = Payload->PlaceExtentMin;
						Common.PlaceExtentMax = Payload->PlaceExtentMax;
						Common.PlaceExtentExponent = Payload->PlaceExtentExponent;

						Common.NoiseTex = NoiseResource->TextureRHI;
						Common.NoiseTexSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs, exactly as the material's Custom node passes them.
						// This block and the Custom node body are the two places the same values
						// are marshalled; if a parameter is ever added to
						// MakeGalaxyDensityParams, both fail to compile, which is the intent.
						Payload->Density.FillShaderParameters(Common);

						// ONE GROUP PER CELL for the two per-cell passes. Not threads divided
						// by group size: the group IS the unit of work, because a cell has to
						// be culled and enveloped by a reduction across its probes before any
						// of its candidates exist.
						//
						// This also retires the failure that cost the most. The old sizing was
						// cells x budget / 64, so a cell array that arrived empty through a
						// capture chain gave a group count of ZERO -- which dispatches nothing
						// silently, with no error and buffers reading exactly as cleared. Here
						// the group count IS the cell count, and the guard at the top of
						// Dispatch already returned on an empty batch.
						const FIntVector CellGroups(Cells.Num(), 1, 1);

						// THREE PASSES, ONE GRAPH, NO READBACK BETWEEN THEM.
						//
						// They share a UAV and RDG orders them on that dependency, so the
						// budget crosses from probe to generate entirely on the GPU. The
						// reduce pass is what makes the candidate count derivable at all:
						// K = target / sum(mass) needs a total no single group can see.
						//
						// Flags, once, for all three:
						//
						// Compute rather than AsyncCompute until the path is proven. These are
						// pure producers -- they write buffers nothing else in the frame reads
						// -- so there is no barrier on the critical path and they should
						// overlap the graphics pipe. Async asserts where the platform or
						// configuration does not support it, and a crash inside a background
						// worker was a poor first result to debug.
						//
						// NeverCull, because a culled pass and a pass that ran and wrote
						// nothing are indistinguishable from the readback -- both give empty
						// buffers with no error anywhere.
						constexpr ERDGPassFlags PassFlags = ERDGPassFlags::AsyncCompute;

						auto AddGenPass = [&GraphBuilder, &Common, PassFlags](
							int32 InPass, const TCHAR* InName, const FIntVector& InGroups)
							{
								// One allocation per pass. See the note on Common above: AddPass
								// clears the parameters its shader does not use, so reusing a
								// single struct destroys it for whichever pass runs later.
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

						// Probe always: it produces the per-cell envelope and mass that both
						// of the other two consume.
						AddGenPass(FGalaxyEntityGenCS::PassProbe,
							TEXT("GalaxyEntityGen.Probe"), CellGroups);

						if (Payload->bCalibrateOnly)
						{
							// ONE group, and it must be one: the reduce walks every cell in a
							// fixed strided order and combines in ascending lane order, so the
							// results are bit-identical run to run. More groups would need an
							// atomic, and a float atomic lets scheduling decide the low bits of
							// the number every cell's budget is divided by.
							AddGenPass(FGalaxyEntityGenCS::PassReduce,
								TEXT("GalaxyEntityGen.Reduce"), FIntVector(1, 1, 1));
						}
						else
						{
							// No reduce. Generation needs one constant and it was calibrated
							// once, over the tier's whole grid, before any of this ran.
							AddGenPass(FGalaxyEntityGenCS::PassGenerate,
								TEXT("GalaxyEntityGen.Generate"), CellGroups);
						}

						// --- readback ---
						// Copy on the GPU, fence, and leave. NEVER block here: a synchronous
						// readback stalls the render thread for the full pipeline depth, and the
						// caller polls IsReady() over the following frames instead.
						AddEnqueueCopyPass(GraphBuilder, EntityReadback, EntityBuffer,
							static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut));

						AddEnqueueCopyPass(GraphBuilder, CountReadbackPtr, CountBuffer,
							static_cast<uint32>(CountElements) * sizeof(uint32));

						GraphBuilder.Execute();

						// Only now can the waiting worker meaningfully poll the fences.
						OutRequest->bSubmitted = true;
					});
			};

		// InitializeTier issues its batch inline; UpdateTier runs on a background
		// worker, so its batch is marshalled and the enqueue happens a hop later.
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
		UTexture* InNoiseTexture,
		float& OutTotalMass,
		float& OutMaxCellMass)
	{
		OutTotalMass = 0.0f;
		OutMaxCellMass = 0.0f;

		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("CalibrateBlocking must run on a background thread"));
			return false;
		}

		if (InCells.Num() == 0 || InNoiseTexture == nullptr)
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		// A token entity buffer. The generate pass never runs here, so nothing writes it
		// -- but RDG still needs a bound, non-empty UAV, and the readback still copies it.
		constexpr int32 TokenCapacity = 64;

		Dispatch(InParams, InTierParams, InCells, TokenCapacity, InKeySeed,
			InNoiseTexture, 0.0f, true, Request);

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

		const uint32 TotalBits = Counts[GlobalTotalMassIndex(InCells.Num())];
		const uint32 MaxBits = Counts[GlobalMaxMassIndex(InCells.Num())];

		OutTotalMass = *reinterpret_cast<const float*>(&TotalBits);
		OutMaxCellMass = *reinterpret_cast<const float*>(&MaxBits);

		return true;
	}

	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InEntityCapacity,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		float InBudgetScale,
		TArray<FGalaxyEntityOut>& OutEntities,
		TArray<uint32>& OutCounts)
	{
		// Guarding the thread rather than trusting the comment: called on the game or
		// render thread this waits on work it is itself preventing from running, and the
		// resulting hang has no stack that points here.
		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("GenerateBatchBlocking must run on a background thread"));
			return false;
		}

		if (InCells.Num() == 0 || InEntityCapacity <= 0 || InNoiseTexture == nullptr)
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		Dispatch(InParams, InTierParams, InCells, InEntityCapacity, InKeySeed,
			InNoiseTexture, InBudgetScale, false, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		if (!AwaitReadback(Request, InEntityCapacity, InCells.Num(), OutEntities, OutCounts))
		{
			return false;
		}

		// NOTHING IS CLAMPED HERE. The per-cell accepted counts used to be clipped to a
		// run width before the scatter could walk them; with a shared buffer no cell owns
		// a run, and the counts are pure diagnostics.
		//
		// The global cursor deliberately over-counts past capacity -- it is the true
		// total accepted, which is what the caller thins against. Clamping it would make
		// an overflowing dispatch report exactly full.
		return true;
	}

} // namespace GalaxyEntityGen
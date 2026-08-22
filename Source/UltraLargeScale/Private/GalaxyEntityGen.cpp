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
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		int32 InMaxCandidates,
		TSharedRef<FGalaxyEntityGenRequest> OutRequest)
	{
		const int32 NumCells = InCells.Num();
		if (NumCells == 0 || InSlotStride <= 0)
		{
			return;
		}

		OutRequest->NumCells = NumCells;
		OutRequest->SlotStride = InSlotStride;

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

		// The dispatch stride is the LARGEST per-cell budget, not the tier's nominal
		// one: cells may each want a different number, and threads past a cell's own
		// budget return immediately.
		const int32 CandidateBudget = FMath::Max(InMaxCandidates, 1);

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
			int32 SlotStride = 0;
			int32 KeySeed = 0;
			int32 CandidateBudget = 0;
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
		Payload->SlotStride = InSlotStride;
		Payload->KeySeed = InKeySeed;
		Payload->CandidateBudget = FMath::Max(InMaxCandidates, 1);
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
						const int32 InSlotStride = Payload->SlotStride;
						const int32 CandidateBudget = Payload->CandidateBudget;
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
						const int32 Total = InSlotStride * Cells.Num();

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

						// One counter per cell, zeroed before the dispatch. Not cleared and the
					// atomics accumulate across frames, which reads as cells that fill once
					// and are empty ever after.
						FRDGBufferDesc CountDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), Cells.Num() * 4);
						CountDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							CountDesc, TEXT("GalaxyGenCounts"));

						FGalaxyEntityGenCS::FParameters* P =
							GraphBuilder.AllocParameters<FGalaxyEntityGenCS::FParameters>();

						P->OutEntities = GraphBuilder.CreateUAV(EntityBuffer, PF_A32B32G32R32F);
						P->InCells = GraphBuilder.CreateSRV(CellBuffer);
						// PF_R32_UINT so the UAV has a FORMAT. A structured UAV has none, and RDG's
						// clear path needs one: AddClearUAVPass over a formatless UAV leaves the
						// counters at whatever the allocator handed back, InterlockedAdd then returns
						// a huge index, every entity fails the bounds test, and the cell writes
						// nothing while looking entirely healthy.
						P->OutCounts = GraphBuilder.CreateUAV(CountBuffer, PF_R32_UINT);

						AddClearUAVPass(GraphBuilder, P->OutCounts, 0u);

						P->NumCells = static_cast<uint32>(Cells.Num());
						P->CandidateBudget = static_cast<uint32>(CandidateBudget);
						P->SlotStride = static_cast<uint32>(InSlotStride);
						P->KeySeed = InKeySeed;
						P->InvGalaxyExtent = Payload->InvGalaxyExtent;

						P->PlaceCompression = Payload->PlaceCompression;
						P->PlaceSpawnExponent = Payload->PlaceSpawnExponent;
						P->PlaceExtentMin = Payload->PlaceExtentMin;
						P->PlaceExtentMax = Payload->PlaceExtentMax;
						P->PlaceExtentExponent = Payload->PlaceExtentExponent;

						P->NoiseTex = NoiseResource->TextureRHI;
						P->NoiseTexSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs, exactly as the material's Custom node passes them.
						// This block and the Custom node body are the two places the same values
						// are marshalled; if a parameter is ever added to
						// MakeGalaxyDensityParams, both fail to compile, which is the intent.
						Payload->Density.FillShaderParameters(*P);

						FGalaxyEntityGenCS::FPermutationDomain PermutationVector;
						TShaderMapRef<FGalaxyEntityGenCS> ComputeShader(
							GetGlobalShaderMap(GMaxRHIFeatureLevel), PermutationVector);

						const int32 ThreadCount = Cells.Num() * CandidateBudget;
						const FIntVector Groups(
							FMath::DivideAndRoundUp(ThreadCount,
								static_cast<int32>(FGalaxyEntityGenCS::ThreadGroupSize)), 1, 1);

						FComputeShaderUtils::AddPass(
							GraphBuilder,
							RDG_EVENT_NAME("GalaxyEntityGen"),
							// Compute, not AsyncCompute, until the path is proven. This is a pure
							// producer -- it writes a buffer nothing else in the frame reads -- so
							// there is no barrier for RDG to insert on the critical path and it
							// should overlap the graphics pipe. Async asserts where the platform or
							// the current configuration does not support it, and a crash inside a
							// background worker was a poor first result to debug. Switch it once
							// this is measuring.
							//
							// NeverCull, because a culled pass and a pass that ran and wrote nothing
							// are indistinguishable from the readback -- both give empty buffers
							// with no error anywhere. RDG should keep this alive on the strength of
							// the copy passes reading its UAVs; now that real data flows, try
							// dropping it.
							ERDGPassFlags::Compute | ERDGPassFlags::NeverCull,
							ComputeShader,
							P,
							Groups);

						// --- readback ---
						// Copy on the GPU, fence, and leave. NEVER block here: a synchronous
						// readback stalls the render thread for the full pipeline depth, and the
						// caller polls IsReady() over the following frames instead.
						AddEnqueueCopyPass(GraphBuilder, EntityReadback, EntityBuffer,
							static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut));

						AddEnqueueCopyPass(GraphBuilder, CountReadbackPtr, CountBuffer,
							static_cast<uint32>(Cells.Num()) * 4u * sizeof(uint32));

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

	bool GenerateBatchBlocking(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		const TArray<FGalaxyGenCell>& InCells,
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		int32 InMaxCandidates,
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

		if (InCells.Num() == 0 || InSlotStride <= 0 || InNoiseTexture == nullptr)
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		Dispatch(InParams, InTierParams, InCells, InSlotStride, InKeySeed,
			InNoiseTexture, InMaxCandidates, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

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

			// Yield rather than spin. This worker holds no lock the render thread needs,
			// so sleeping here cannot deadlock the fence it is waiting on.
			FPlatformProcess::Sleep(0.0005f);
		}

		if (Request->Failed())
		{
			return false;
		}

		// LOCK ON THE RENDER THREAD. Lock() maps the staging buffer through the RHI
		// and crashes anywhere else, so the worker -- which may poll the fence safely
		// -- hands the copy over and waits again rather than reading it directly.
		//
		// Marshalled through the game thread for the same reason the dispatch is:
		// ENQUEUE_RENDER_COMMAND from a background task can run inline on that task,
		// which puts RHI work back on the thread that must not do it.
		const int32 Total = InSlotStride * InCells.Num();
		const int32 NumCells = InCells.Num();

		TSharedRef<FGalaxyEntityGenRequest> Req = Request;

		auto EnqueueCopy = [Req, Total, NumCells]() mutable
			{
				ENQUEUE_RENDER_COMMAND(GalaxyEntityGenCopy)(
					[Req, Total, NumCells](FRHICommandListImmediate&) mutable
					{
						const uint32 Bytes = static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut);
						const uint32 CountBytes = static_cast<uint32>(NumCells) * 4u * sizeof(uint32);

						const FGalaxyEntityOut* Src =
							static_cast<const FGalaxyEntityOut*>(Req->Readback->Lock(Bytes));

						if (Src)
						{
							Req->Entities.SetNumUninitialized(Total);
							FMemory::Memcpy(Req->Entities.GetData(), Src, Bytes);
						}
						Req->Readback->Unlock();

						const uint32* CountSrc =
							static_cast<const uint32*>(Req->CountReadback->Lock(CountBytes));

						if (CountSrc)
						{
							Req->Counts.SetNumUninitialized(NumCells * 4);
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

		if (Request->Entities.Num() != Total || Request->Counts.Num() != NumCells * 4)
		{
			return false;
		}

		OutEntities = MoveTemp(Request->Entities);
		OutCounts = MoveTemp(Request->Counts);

		// Only slot 0 of each group is an accepted count; slots 1, 2 and 3 are the
		// thread tally, the max density and the Candidates value the shader read, and
		// clamping those would destroy the calibration.
		//
		// The shader's atomic counts EVERY acceptance, including the ones it then drops
		// for exceeding the run, so slot 0 is clamped to keep the scatter in bounds.
		for (int32 i = 0; i < InCells.Num(); ++i)
		{
			OutCounts[i * 4] = FMath::Min(OutCounts[i * 4], static_cast<uint32>(InSlotStride));
		}

		Request->Readback.Reset();
		Request->CountReadback.Reset();

		return true;
	}

} // namespace GalaxyEntityGen
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
	"/UltraLargeScale/GalaxyEntityGen.usf", "MainCS", SF_Compute);

void FGalaxyEntityGenRequest::Consume(TArray<FGalaxyEntityOut>& OutValid)
{
	OutValid.Reset();

	if (!IsReady())
	{
		return;
	}

	// The dispatch compacts per cell, so this concatenates the live runs rather than
	// filtering. Reading the counts is what makes that possible: without them the only
	// way to find the end of a run would be to scan the whole reserved width, which is
	// exactly the traffic the compaction removed.
	const uint32 CountBytes = static_cast<uint32>(NumCells) * sizeof(uint32);
	const uint32* Counts = static_cast<const uint32*>(CountReadback->Lock(CountBytes));

	const int32 Total = SlotStride * NumCells;
	const uint32 Bytes = static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut);
	const FGalaxyEntityOut* Src = static_cast<const FGalaxyEntityOut*>(Readback->Lock(Bytes));

	if (Counts && Src)
	{
		OutValid.Reserve(Total);

		for (int32 c = 0; c < NumCells; ++c)
		{
			const int32 Live = FMath::Min(static_cast<int32>(Counts[c]), SlotStride);

			for (int32 i = 0; i < Live; ++i)
			{
				OutValid.Add(Src[c * SlotStride + i]);
			}
		}
	}

	Readback->Unlock();
	CountReadback->Unlock();

	Readback.Reset();
	CountReadback.Reset();
}

namespace GalaxyEntityGen
{

	void Dispatch(
		const FGalaxyParams& InParams,
		const FTierParams& InTierParams,
		TArray<FGalaxyGenCell> InCells,
		int32 InSlotStride,
		int32 InKeySeed,
		UTexture* InNoiseTexture,
		bool bForceNoiseOff,
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

		const int32 CandidateBudget = FMath::Max(InTierParams.CandidateBudget, 1);

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
		auto EnqueueOnRenderThread = [=, Cells = MoveTemp(InCells)]() mutable
			{
				// On the game thread now: safe to touch the UObject.
				UTexture* NoiseTexture = WeakNoise.Get();
				FTextureResource* NoiseResource = NoiseTexture ? NoiseTexture->GetResource() : nullptr;

				if (!NoiseResource)
				{
					// Nothing will ever land. Say so, or the worker waits out the whole
					// timeout for a copy that was never enqueued.
					OutRequest->bAborted = true;
					return;
				}

				ENQUEUE_RENDER_COMMAND(GalaxyEntityGenDispatch)(
					[=, Cells = MoveTemp(InCells)](FRHICommandListImmediate& RHICmdList) mutable
					{
						if (!NoiseResource->TextureRHI)
						{
							OutRequest->bAborted = true;
							return;
						}

						FRDGBuilder GraphBuilder(RHICmdList);

						// --- cells in ---
						FRDGBufferRef CellBuffer = CreateStructuredBuffer(
							GraphBuilder, TEXT("GalaxyGenCells"), Cells);

						// --- entities out ---
						const int32 Total = InSlotStride * Cells.Num();

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							FRDGBufferDesc::CreateStructuredDesc(sizeof(FGalaxyEntityOut), Total),
							TEXT("GalaxyGenEntities"));

						// One counter per cell, zeroed before the dispatch. Not cleared and the
					// atomics accumulate across frames, which reads as cells that fill once
					// and are empty ever after.
						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Cells.Num()),
							TEXT("GalaxyGenCounts"));

						FGalaxyEntityGenCS::FParameters* P =
							GraphBuilder.AllocParameters<FGalaxyEntityGenCS::FParameters>();

						P->OutEntities = GraphBuilder.CreateUAV(EntityBuffer);
						P->InCells = GraphBuilder.CreateSRV(CellBuffer);
						P->OutCounts = GraphBuilder.CreateUAV(CountBuffer);

						AddClearUAVPass(GraphBuilder, P->OutCounts, 0u);

						P->NumCells = static_cast<uint32>(Cells.Num());
						P->CandidateBudget = static_cast<uint32>(CandidateBudget);
						P->SlotStride = static_cast<uint32>(InSlotStride);
						P->KeySeed = InKeySeed;
						P->InvGalaxyExtent = InvGalaxyExtent;

						P->PlaceCompression = PlaceCompression;
						P->PlaceSpawnExponent = PlaceSpawnExponent;
						P->PlaceExtentMin = PlaceExtentMin;
						P->PlaceExtentMax = PlaceExtentMax;
						P->PlaceExtentExponent = PlaceExtentExponent;

						P->NoiseTex = NoiseResource->TextureRHI;
						P->NoiseTexSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs, exactly as the material's Custom node passes them.
						// This block and the Custom node body are the two places the same values
						// are marshalled; if a parameter is ever added to
						// MakeGalaxyDensityParams, both fail to compile, which is the intent.
						D.FillShaderParameters(*P);

						// Step 2a: with noise off, GalaxySample degenerates to SampleAnalytic and
						// the GPU runs the identical function the CPU does, so the accepted sets
						// should match. Any difference at this setting is marshalling -- most
						// likely struct layout, which the static_asserts catch first -- rather
						// than the field.
						if (bForceNoiseOff)
						{
							P->InNoiseEnable = 0.0f;
						}

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
							// Async so the dispatch overlaps the graphics pipe instead of
							// serialising against it. It is a pure producer -- it writes a buffer
							// nothing else in the frame reads -- so there is no barrier for RDG
							// to insert on the critical path.
							// Compute, not AsyncCompute, until the path is proven. Async overlaps
						// the graphics pipe but asserts where the platform or the current
						// configuration does not support it, and a crash inside a background
						// worker is a poor first result. Switch it once this is measuring.
							ERDGPassFlags::Compute,
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
							static_cast<uint32>(Cells.Num()) * sizeof(uint32));

						GraphBuilder.Execute();

						// Only now can the waiting worker meaningfully poll the fences.
						OutRequest->bSubmitted = true;
					});
			};

		if (IsInGameThread())
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
		bool bForceNoiseOff,
		double InTimeoutSeconds,
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
			InNoiseTexture, bForceNoiseOff, Request);

		if (!Request->Readback.IsValid())
		{
			return false;
		}

		const double Deadline = FPlatformTime::Seconds() + InTimeoutSeconds;

		while (!Request->IsReady())
		{
			if (FPlatformTime::Seconds() > Deadline)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("GalaxyEntityGen: readback timed out after %.2fs; falling back to CPU generation."),
					InTimeoutSeconds);
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

		// The dispatch already compacted per cell, so this reads two buffers: the runs
		// themselves, and how much of each run is live. Reading only the entity buffer
		// would mean scanning a whole SlotCapacity-wide run per cell to find the end,
		// which is the traffic the compaction exists to avoid.
		const int32 Total = InSlotStride * InCells.Num();
		const uint32 Bytes = static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut);

		const FGalaxyEntityOut* Src =
			static_cast<const FGalaxyEntityOut*>(Request->Readback->Lock(Bytes));

		if (!Src)
		{
			Request->Readback->Unlock();
			return false;
		}

		OutEntities.SetNumUninitialized(Total);
		FMemory::Memcpy(OutEntities.GetData(), Src, Bytes);

		Request->Readback->Unlock();

		const uint32 CountBytes = static_cast<uint32>(InCells.Num()) * sizeof(uint32);
		const uint32* CountSrc =
			static_cast<const uint32*>(Request->CountReadback->Lock(CountBytes));

		if (!CountSrc)
		{
			Request->CountReadback->Unlock();
			return false;
		}

		OutCounts.SetNumUninitialized(InCells.Num());
		FMemory::Memcpy(OutCounts.GetData(), CountSrc, CountBytes);

		Request->CountReadback->Unlock();

		// The shader's atomic counts EVERY acceptance, including the ones it then drops
		// for exceeding the run. Clamping here rather than in the shader keeps the
		// count meaningful as "how many were accepted", which is worth knowing when a
		// cell is saturating its capacity.
		for (uint32& Count : OutCounts)
		{
			Count = FMath::Min(Count, static_cast<uint32>(InSlotStride));
		}

		Request->Readback.Reset();
		Request->CountReadback.Reset();

		return true;
	}

} // namespace GalaxyEntityGen
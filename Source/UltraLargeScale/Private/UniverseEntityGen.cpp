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
		UTexture* InNoiseTexture,
		float InBudgetScale,
		bool bInCalibrateOnly,
		TSharedRef<FUniverseEntityGenRequest> OutRequest)
	{
		const int32 NumCells = InCells.Num();
		if (NumCells == 0 || InEntityCapacity <= 0)
		{
			return;
		}

		OutRequest->NumCells = NumCells;
		OutRequest->NumSlots = InNumSlots;
		OutRequest->EntityCapacity = InEntityCapacity;

		// Snapshot everything the render thread will need. Nothing below may touch
		// InParams or InTierParams after this point: the game thread is free to mutate them
		// the instant this function returns, and a dangling reference into tier params is
		// the kind of race that only reproduces under streaming load.
		const FUniverseDensityParams D = InParams.DensityParams;
		const double InvUnit = 1.0 / FMath::Max(InParams.UnitScale, UE_DOUBLE_SMALL_NUMBER);

		// SUPPLIED, not derived from InParams.Extent. The field's normalized frame is the
		// RAY MARCH PROXY, whose half extent is the Large tier's neighbourhood rather than
		// the sector extent -- see AUniverseActor::GetVolumetricProxyExtent. Deriving it
		// here from a plausible-looking member is exactly how the render and placement would
		// end up sampling two different scalings of the same field.
		const float InvFieldExtent = InInvFieldExtent;

		const float PlaceSpawnExponent = InTierParams.SpawnExponent;
		const float PlaceExtentMin = static_cast<float>(InTierParams.MinScale * InvUnit);
		const float PlaceExtentMax = static_cast<float>(InTierParams.MaxScale * InvUnit);
		const float PlaceExtentExponent = InTierParams.ExtentExponent;

		// The tier's calibrated constant. Not a budget and not a ceiling -- there is no
		// authored candidate number anywhere in this path.
		const float BudgetScale = FMath::Max(InBudgetScale, 0.0f);

		// The anchor, and the pad that turns a probed peak into an envelope.
		//
		// The pad covers the SAMPLING miss only: both probes and acceptance run
		// SampleAtPosition in the same dispatch, so the only error left to insure against is
		// the finite probe count. It matters more here than in the galaxy -- a cosmic web's
		// peaks are the surfaces BETWEEN nodes, which sit at no particular place relative to
		// the generation grid, so a thin filament crossing a cell can fall between all
		// fifty-six jittered probes. Counter [3] against [2] says how often it does.
		const float BudgetAnchor = kBudgetAnchor;

		// THE ERROR IS ASYMMETRIC, so the pad leans generous. Over-estimating an envelope
		// costs candidates and nothing else -- the budget scales as envelope^g and
		// acceptance as (d/envelope)^g, so the two cancel and the accepted count is
		// unchanged. Under-estimating does not cancel: everything above the envelope
		// saturates at probability 1, the cell over-delivers against its budget, and the
		// surplus is then clipped by slot capacity rather than by the field.
		//
		// It cannot simply be raised until clipping stops, because the candidate cost is
		// exponential in the spawn exponent -- at g = 8 a pad of 2 is 256 times the
		// candidates. Probe count is the cheaper axis; see UNIVERSE_ENTITYGEN_PROBE_ROUNDS.
		constexpr float EnvelopePad = 1.25f;

		// The UObject is captured, NOT dereferenced here.
		//
		// GetResource() on a background worker is a UObject access off the game thread: it
		// can run against an object mid-collection, and the resource pointer itself can be
		// swapped by texture streaming between the read and the use. Resolving it on the
		// game thread -- where the object is guaranteed live and the pointer stable --
		// removes both.
		//
		// A weak pointer, so a sector torn down while a batch is in flight resolves to null
		// and the dispatch bails, rather than the marshalled lambda keeping a destroyed
		// texture alive or dereferencing one.
		TWeakObjectPtr<UTexture> WeakNoise(InNoiseTexture);

		// Allocated HERE, on the calling thread, not inside the render command.
		//
		// The caller polls this request the moment Dispatch returns. Creating the readbacks
		// inside the render command left them null until the render thread got round to the
		// lambda, so the poll saw nothing and gave up. Only the COPY has to happen on the
		// render thread; the objects themselves are just handles.
		OutRequest->Readback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("UniverseEntityGenReadback"));
		OutRequest->CountReadback = MakeUnique<FRHIGPUBufferReadback>(
			TEXT("UniverseEntityGenCountReadback"));

		FRHIGPUBufferReadback* EntityReadback = OutRequest->Readback.Get();
		FRHIGPUBufferReadback* CountReadbackPtr = OutRequest->CountReadback.Get();

		// ENQUEUED FROM THE GAME THREAD, not from the calling worker.
		//
		// ENQUEUE_RENDER_COMMAND from a background task can execute the command inline on
		// that task rather than handing it to the render thread, and RDG then runs on a
		// thread carrying no rendering task tag -- the FTaskTagScope ensure. Marshalling
		// first costs one hop and removes the whole class of problem.
		//
		// Safe against deadlock because the game thread does not block on generation:
		// InitializeTier hands its rendezvous off through a TPromise and returns, so the
		// game thread keeps pumping while the worker waits on the readback fence.
		//
		// EVERYTHING THE RENDER COMMAND NEEDS GOES IN ONE SHARED PAYLOAD, captured
		// explicitly by value in both lambdas. DO NOT REPLACE IT WITH NESTED [=] CAPTURES:
		// across two thread hops it is possible to move from the same parameter twice, and
		// the second move silently yields an empty array. The dispatch then runs with zero
		// cells, computes a group count of zero, and writes nothing -- with no error, no RDG
		// complaint, and buffers reading exactly as cleared.
		struct FDispatchPayload
		{
			TArray<FUniverseGenCell> Cells;
			FUniverseDensityParams Density;
			TWeakObjectPtr<UTexture> Noise;
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
		Payload->Noise = WeakNoise;
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
				// On the game thread now: safe to touch the UObject.
				UTexture* NoiseTexture = Payload->Noise.Get();
				FTextureResource* NoiseResource = NoiseTexture ? NoiseTexture->GetResource() : nullptr;

				if (!NoiseResource)
				{
					// Nothing will ever land. Say so, or the worker waits out the whole
					// timeout for a copy that was never enqueued.
					//
					// AND IT IS FATAL HERE RATHER THAN DEGRADED. With no texture the field
					// would still evaluate -- to the unwarped analytic web, with both region
					// fetches neutral -- which is a DIFFERENT FIELD from the one the material
					// draws. Placing against it would look like a plausible cosmic web whose
					// entities sit nowhere near the rendered filaments.
					OutRequest->bAborted = true;
					return;
				}

				ENQUEUE_RENDER_COMMAND(UniverseEntityGenDispatch)(
					[Payload, OutRequest, EntityReadback, CountReadbackPtr, NoiseResource]
					(FRHICommandListImmediate& RHICmdList) mutable
					{
						// Named explicitly rather than reached through a capture chain.
						const TArray<FUniverseGenCell>& Cells = Payload->Cells;
						const int32 Total = Payload->EntityCapacity;

						const int32 InKeySeed = Payload->KeySeed;
						if (!NoiseResource->TextureRHI)
						{
							OutRequest->bAborted = true;
							return;
						}

						FRDGBuilder GraphBuilder(RHICmdList);

						// --- cells in ---
						// Explicit sizes and CopyData, so RDG owns the bytes rather than
						// reading the array at execute time. The array does outlive Execute()
						// here, but "does" and "is guaranteed to" are different things and
						// this costs one memcpy of a few kilobytes.
						FRDGBufferRef CellBuffer = CreateStructuredBuffer(
							GraphBuilder, TEXT("UniverseGenCells"),
							sizeof(FUniverseGenCell), Cells.Num(),
							Cells.GetData(), Cells.Num() * sizeof(FUniverseGenCell),
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
						FRDGBufferDesc EntityDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(FUintVector4), Total * 2);
						EntityDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							EntityDesc, TEXT("UniverseGenEntities"));

						// FIVE counters per cell PLUS ONE global, the entity append cursor.
						// The element count is derived from the same expression the copy and
						// the readback use, because writing it out three times is what let it
						// drift in the galaxy path: that desc kept four per cell while the
						// copy moved on, so the copy ran off the end and the shader's global
						// write landed outside the view.
						//
						// It survived for a long time because RDG POOLS BUFFERS BY SIZE. A
						// pooled allocation left over from a larger dispatch absorbed both,
						// and the failure only appeared once the pool returned one sized
						// exactly to this desc.
						//
						// Zeroed before the dispatch. Not cleared and the atomics accumulate
						// across frames, which reads as cells that fill once and are empty
						// ever after.
						const int32 CountElements =
							CountElementsFor(Cells.Num(), Payload->NumSlots);

						FRDGBufferDesc CountDesc =
							FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), CountElements);
						CountDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef CountBuffer = GraphBuilder.CreateBuffer(
							CountDesc, TEXT("UniverseGenCounts"));

						// FILLED ONCE AS A VALUE, then copied into a FRESH allocation for each
						// pass. Not shared.
						//
						// FComputeShaderUtils::AddPass runs ClearUnusedGraphResources, which
						// NULLS every parameter the bound shader does not reference. The probe
						// permutation never touches OutEntities -- the generate branch is
						// compiled out -- so handing both passes one struct lets the first
						// AddPass clear it, and the second then dies on "required shader
						// parameter FParameters::OutEntities was not set".
						//
						// Fresh allocations are also what RDG expects: it records each pass's
						// resource dependencies FROM its parameter struct, so a shared one
						// would conflate two passes' access patterns into one.
						FUniverseEntityGenCS::FParameters Common;

						// PF_R32G32B32A32_UINT, matching RWBuffer<uint4>. The record is an
						// integer buffer carrying floats as bit patterns rather than the
						// reverse -- see FUniverseEntityOut. A format mismatch between the UAV
						// and the shader declaration is undefined rather than an error.
						Common.OutEntities =
							GraphBuilder.CreateUAV(EntityBuffer, PF_R32G32B32A32_UINT);
						Common.InCells = GraphBuilder.CreateSRV(CellBuffer);

						// PF_R32_UINT so the UAV has a FORMAT. A structured UAV has none, and
						// RDG's clear path needs one: AddClearUAVPass over a formatless UAV
						// leaves the counters at whatever the allocator handed back,
						// InterlockedAdd then returns a huge index, every entity fails the
						// bounds test, and the cell writes nothing while looking healthy.
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

						Common.NoiseTex = NoiseResource->TextureRHI;

						// AM_WRAP ON ALL THREE AXES, and it is correctness rather than taste.
						// The field's warp UV wraps every 4096 cells by masking the cell
						// index, and the two sides of that wrap are the same texel only if
						// the sampler repeats. Clamp would mirror the seam into a wall of
						// constant warp at the wrap boundary.
						Common.NoiseTexSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();

						// The field inputs, exactly as the material's Custom node passes them.
						//
						// A ZERO OFFSET. The proxy's offset is how far the field has scrolled
						// under the VIEW; the field itself is static in caller space, and the
						// traversal terms cancel out of the placement coordinate exactly. See
						// the note on InvFieldExtent in the .usf -- this is the one place the
						// two paths deliberately differ in what they pass.
						Payload->Density.FillShaderParameters(
							Common, Payload->Seed, FUniverseFieldOffset());

						// ONE GROUP PER CELL for both per-cell passes. Not threads divided by
						// group size: the group IS the unit of work, because a cell has to be
						// culled and enveloped by a reduction across its probes before any of
						// its candidates exist.
						//
						// WRAPPED ACROSS TWO DIMENSIONS. A dispatch is limited to 65535 groups
						// per dimension, and a subdivided calibration grid runs past it.
						// Exceeding it is an ensure inside RDG followed by no dispatch at all,
						// so the readback simply times out with nothing to say the cause was
						// the shape of the dispatch rather than the field.
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
								TEXT("UniverseEntityGen: %d cells needs %d x %d groups, past ")
								TEXT("the dispatch limit of %d per dimension. Reduce the ")
								TEXT("tier's GenerationSubdivision or its GridDepth."),
								Cells.Num(), GroupsX, GroupsY, MaxGroupsPerDim);

							OutRequest->bAborted = true;
							return;
						}

						// TWO PASSES, ONE GRAPH, NO READBACK BETWEEN THEM.
						//
						// They share a UAV and RDG orders them on that dependency, so each
						// cell's envelope crosses from probe to generate entirely on the GPU.
						//
						// AsyncCompute for both: these are pure producers, writing buffers
						// nothing else in the frame reads, so there is no barrier on the
						// critical path and they overlap the graphics pipe. That matters more
						// for this layer than the galaxy -- a universe field sample is a
						// fifty-four candidate walk plus five fetches, so the dispatch is long
						// enough to be worth hiding.
						constexpr ERDGPassFlags PassFlags = ERDGPassFlags::AsyncCompute;

						auto AddGenPass = [&GraphBuilder, &Common, PassFlags](
							int32 InPass, const TCHAR* InName, const FIntVector& InGroups)
							{
								// One allocation per pass. See the note on Common above:
								// AddPass clears the parameters its shader does not use, so
								// reusing a single struct destroys it for the later pass.
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

						// Calibration stops here. It wants the masses and nothing else, and
						// the reduction over them happens on the CPU -- the readback already
						// carries every one of them across.
						if (!Payload->bCalibrateOnly)
						{
							AddGenPass(FUniverseEntityGenCS::PassGenerate,
								TEXT("UniverseEntityGen.Generate"), CellGroups);
						}

						// --- readback ---
						// Copy on the GPU, fence, and leave. NEVER block here: a synchronous
						// readback stalls the render thread for the full pipeline depth, and
						// the caller polls IsReady() over the following frames instead.
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
		UTexture* InNoiseTexture,
		TArray<float>& OutCellMass)
	{
		OutCellMass.Reset();

		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("CalibrateBlocking must run on a background thread"));
			return false;
		}

		if (InCells.Num() == 0 || InNoiseTexture == nullptr)
		{
			return false;
		}

		TSharedRef<FUniverseEntityGenRequest> Request = MakeShared<FUniverseEntityGenRequest>();

		// A token entity buffer. The generate pass never runs here, so nothing writes it --
		// but RDG still needs a bound, non-empty UAV, and the readback still copies it.
		constexpr int32 TokenCapacity = 64;

		// One token slot: the generate pass never runs, so nothing claims one.
		Dispatch(InParams, InTierParams, InCells, TokenCapacity, 1, TokenCapacity,
			InKeySeed, InInvFieldExtent, InNoiseTexture, 0.0f, true, Request);

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

		// THE PER-CELL MASSES WERE ALWAYS IN THE READBACK. The probe pass writes one per cell
		// at [i*5+4] and the copy already brings the whole counter buffer across, so the
		// reduction needs no GPU pass -- and reducing on the CPU is what lets the caller take
		// a max of PER-PARENT SUMS, which no single thread group can see.
		//
		// Written as asuint by the shader; read back as the float bit pattern.
		OutCellMass.SetNumUninitialized(InCells.Num());

		for (int32 i = 0; i < InCells.Num(); ++i)
		{
			const uint32 Bits = Counts[i * CountersPerCell + 4];
			const float Mass = *reinterpret_cast<const float*>(&Bits);

			// A cell that probed nothing is zero, which is ordinary -- most of a cosmic web
			// is void. Anything that is not a finite non-negative number is not, and letting
			// one into the sum would poison every cell's budget at once.
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
		UTexture* InNoiseTexture,
		float InBudgetScale,
		TArray<FUniverseEntityOut>& OutEntities,
		TArray<uint32>& OutCounts)
	{
		// Guarding the thread rather than trusting the comment: called on the game or render
		// thread this waits on work it is itself preventing from running, and the resulting
		// hang has no stack that points here.
		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("GenerateBatchBlocking must run on a background thread"));
			return false;
		}

		if (InCells.Num() == 0 || InEntityCapacity <= 0 || InNoiseTexture == nullptr)
		{
			return false;
		}

		TSharedRef<FUniverseEntityGenRequest> Request = MakeShared<FUniverseEntityGenRequest>();

		Dispatch(InParams, InTierParams, InCells, InEntityCapacity, InNumSlots,
			InSlotCapacity, InKeySeed, InInvFieldExtent, InNoiseTexture,
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
		// cursor is now the number of records actually WRITTEN and the per-slot counters
		// carry the true demand. The caller reads the entity array against the former and
		// judges the calibration against the latter.
		return true;
	}

} // namespace UniverseEntityGen
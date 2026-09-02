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

		// WRITTEN AND NEVER READ, here and in the universe layer. Both blocking entry points
		// hand AwaitReadback the same counts as arguments instead, so these three carry the
		// dispatch's shape for a reader and nothing else. Kept because a request that cannot
		// describe itself is worse to debug than three unused ints, but a future consumer
		// should take them from here rather than adding a fourth argument.
		OutRequest->NumCells = NumCells;
		OutRequest->EntityCapacity = InEntityCapacity;

		// Snapshot everything the render thread will need. Nothing below may touch
		// InParams or InTierParams after this point: the game thread is free to mutate
		// them the instant this function returns, and a dangling reference into tier
		// params is the kind of race that only reproduces under streaming load.
		const FGalaxyProceduralParams D = InParams.Procedural;
		const double InvUnit = 1.0 / FMath::Max(InParams.UnitScale, UE_DOUBLE_SMALL_NUMBER);

		const float InvGalaxyExtent =
			static_cast<float>(1.0 / FMath::Max(static_cast<double>(InParams.Extent), 1e-9));

		const float PlaceSpawnExponent = InTierParams.SpawnExponent;
		const float PlaceExtentMin = static_cast<float>(InTierParams.MinScale * InvUnit);
		const float PlaceExtentMax = static_cast<float>(InTierParams.MaxScale * InvUnit);
		const float PlaceExtentExponent = InTierParams.ExtentExponent;

		// The tier's calibrated constant. Not a budget and not a ceiling -- there is no
		// authored candidate number anywhere in this path.
		const float BudgetScale = FMath::Max(InBudgetScale, 0.0f);

		// The anchor, and the pad that turns a probed peak into an envelope.
		//
		// THE PAD COVERS THE SAMPLING MISS, and it is the only insurance against the one
		// place the per-cell normalisation stops cancelling. Acceptance is
		// min(density / envelope, 1) ^ g: below the envelope the envelope divides out of the
		// result exactly, and a cell's count depends on that cell's field alone. Above it
		// the ratio saturates and the cancellation fails in the expensive direction -- the
		// cell drew its candidates against E^g, so a short envelope costs it candidates
		// QUADRATICALLY while capping the fraction it keeps.
		//
		// WHICH CELLS PAY IS THE WHOLE PROBLEM. A short envelope is a probe set that missed
		// the cell's peak, so it happens where a cell holds the most density range -- the
		// core and the arms. Those cells under-deliver while the void cells around them do
		// not, and the result is cell-shaped holes in the brightest regions with the
		// raymarch still drawing structure through them. It reads as a placement or hash
		// fault long before it reads as a sampling one.
		//
		// TWO, AND THE HISTORY IS THE ARGUMENT FOR IT. At 1.15 a quarter of live cells on the
		// Large tier reported a candidate denser than their own envelope; counter [3] against
		// [2] is the observer -- see GalaxyDataGenerator's envelope-exceeded count. Raising
		// it here, together with GenerationSubdivision 3, is what stopped the cell-shaped
		// holes in practice.
		//
		// IT DOES NOT REMOVE THE CASE. The mechanism is a probe set missing a cell's peak,
		// and no pad removes that -- it only makes the miss less likely to bite. Expect the
		// symptom back on an archetype that rolls a tighter core or thinner arms, and reach
		// for GALAXY_ENTITYGEN_PROBE_ROUNDS rather than this number when it returns: the
		// probe count is linear where this is exponential.
		//
		// IT COSTS CANDIDATES QUADRATICALLY and accepted entities not at all, since the
		// envelope cancels: a cell draws pad^g times as many candidates and keeps the same
		// number. Read the C/P ratio after changing it.
		const float BudgetAnchor = kBudgetAnchor;
		constexpr float EnvelopePad = 2.0f;

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
		// FOUR WEAK POINTERS, each checked on the game thread below. They do not expire as a
		// group -- four separate objects can be collected independently -- so "the set was
		// complete when Dispatch was called" says nothing about the moment the marshalled
		// lambda runs.
		TWeakObjectPtr<UTexture> WeakWarpDisc(InFieldTextures.WarpDisc);
		TWeakObjectPtr<UTexture> WeakWarpHalo(InFieldTextures.WarpHalo);
		TWeakObjectPtr<UTexture> WeakNoiseDisc(InFieldTextures.NoiseDisc);
		TWeakObjectPtr<UTexture> WeakNoiseHalo(InFieldTextures.NoiseHalo);

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
		// explicitly by value in both lambdas. DO NOT REPLACE IT WITH NESTED [=]
		// CAPTURES: across two thread hops it is possible to move from the same parameter
		// twice, and the second move silently yields an empty array. The dispatch then
		// runs with zero cells, computes a group count of zero, and writes nothing --
		// with no error, no RDG complaint, and buffers reading exactly as cleared.
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
			// Overwritten from the single constant above before the dispatch is enqueued;
			// this initialiser exists only so a payload built and abandoned carries a
			// harmless value. Calibration and generation MUST be handed the same pad --
			// the mass is defined through the envelope, so two pads would solve the tier's
			// constant against one field and spend it against another.
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
				// On the game thread now: safe to touch the UObjects. Resolved as a SET, and
				// the set is all-or-nothing -- each GetResource() is a UObject access that is
				// only legal here, and each can independently come back null.
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
					// Nothing will ever land. Say so, or the worker waits out the whole
					// timeout for a copy that was never enqueued.
					//
					// NO PARTIAL BIND. Binding the three that resolved and leaving the fourth
					// null is the worst available outcome: it is the case that most looks
					// like it worked.
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
							FRDGBufferDesc::CreateBufferDesc(sizeof(FUintVector4), Total * 2);
						EntityDesc.Usage |= EBufferUsageFlags::SourceCopy;

						FRDGBufferRef EntityBuffer = GraphBuilder.CreateBuffer(
							EntityDesc, TEXT("GalaxyGenEntities"));

						// FIVE counters per cell PLUS ONE global, the entity append cursor. The
						// element count is derived from the same expression the copy and the
						// readback use, because writing it out three times is what let it
						// drift: this desc kept four per cell while the copy moved on, so the
						// copy ran off the end of the buffer and the shader's global write
						// landed outside the view.
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

						// PF_R32G32B32A32_UINT, matching RWBuffer<uint4>. The record is an
						// integer buffer carrying floats as bit patterns rather than the
						// reverse -- see FGalaxyEntityOut. A format mismatch between the
						// UAV and the shader declaration is undefined rather than an error.
						Common.OutEntities =
							GraphBuilder.CreateUAV(EntityBuffer, PF_R32G32B32A32_UINT);
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

						// THE SAME STATE FOR ALL FOUR, written out rather than hoisted.
						// TStaticSamplerState::GetRHI returns one cached object per template
						// argument set, so this is four reads of one pointer, and keeping the
						// arguments visible per texture makes it a one-line edit when one of
						// them wants different addressing.
						//
						// MUST MATCH THE MATERIAL. The material's Custom node pins carry
						// whatever addressing each ASSET is saved with, so an asset saved with
						// clamp gives the render a different field from the one this dispatch
						// places against.
						Common.WarpTexDiscSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.WarpTexHaloSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.NoiseTexDiscSampler = TStaticSamplerState<
							SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
						Common.NoiseTexHaloSampler = TStaticSamplerState<
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
						// WRAPPED ACROSS TWO DIMENSIONS. A dispatch is limited to 65535 groups
						// per dimension, and a tier's subdivided calibration grid runs well
						// past that -- the small tier alone is a hundred and fifty thousand
						// cells. Exceeding it is an ensure inside RDG followed by no dispatch
						// at all, so the readback simply times out and the tier reports a
						// calibration failure with nothing to say the cause was the shape of
						// the dispatch rather than the field.
						//
						// The shader unwraps with GroupId.y * DispatchGroupsX + GroupId.x and
						// the existing NumCells bound discards the tail of the last row.
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

						// TWO PASSES, ONE GRAPH, NO READBACK BETWEEN THEM.
						//
						// They share a UAV and RDG orders them on that dependency, so each
						// cell's envelope crosses from probe to generate entirely on the GPU.
						//
						// AsyncCompute for both: these are pure producers, writing buffers
						// nothing else in the frame reads, so there is no barrier on the
						// critical path and they overlap the graphics pipe.
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

						// Probe always: it produces the per-cell envelope generation rejects
						// against, and the per-cell mass calibration is solved from.
						AddGenPass(FGalaxyEntityGenCS::PassProbe,
							TEXT("GalaxyEntityGen.Probe"), CellGroups);

						// Calibration stops here. It wants the masses and nothing else, and
						// the reduction over them happens on the CPU -- the readback already
						// carries every one of them across.
						if (!Payload->bCalibrateOnly)
						{
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
		const FGalaxyFieldTextures& InFieldTextures,
		TArray<float>& OutCellMass)
	{
		OutCellMass.Reset();

		if (IsInGameThread() || IsInRenderingThread())
		{
			ensureMsgf(false, TEXT("CalibrateBlocking must run on a background thread"));
			return false;
		}

		// ALL FOUR OR NONE -- see FGalaxyFieldTextures. Calibrating against a partial set
		// fits budgets to a field the material does not draw, and those budgets are cached
		// per tier and reused.
		if (InCells.Num() == 0 || !InFieldTextures.IsComplete())
		{
			return false;
		}

		TSharedRef<FGalaxyEntityGenRequest> Request = MakeShared<FGalaxyEntityGenRequest>();

		// A token entity buffer. The generate pass never runs here, so nothing writes it
		// -- but RDG still needs a bound, non-empty UAV, and the readback still copies it.
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

		// THE PER-CELL MASSES WERE ALWAYS IN THE READBACK. The probe pass writes one
		// per cell at [i*5+4] and the copy already brings the whole counter buffer
		// across, so the reduction needs no GPU pass -- and reducing on the CPU is what
		// lets the caller take a max of PER-PARENT SUMS, which no single thread group
		// can see.
		//
		// Written as asuint by the shader; read back as the float bit pattern.
		OutCellMass.SetNumUninitialized(InCells.Num());

		for (int32 i = 0; i < InCells.Num(); ++i)
		{
			const uint32 Bits = Counts[i * CountersPerCell + 4];
			const float Mass = *reinterpret_cast<const float*>(&Bits);

			// A cell that probed nothing is zero, which is ordinary. Anything that is
			// not a finite non-negative number is not, and letting one into the sum
			// would poison every cell's budget at once.
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
		// Guarding the thread rather than trusting the comment: called on the game or
		// render thread this waits on work it is itself preventing from running, and the
		// resulting hang has no stack that points here.
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

		// NOTHING IS CLAMPED HERE. No cell owns a run in a shared buffer, so there is no run
		// width to clip a per-cell count against -- the counts are pure diagnostics.
		//
		// The global cursor deliberately over-counts past capacity -- it is the true
		// total accepted, which is what the caller thins against. Clamping it would make
		// an overflowing dispatch report exactly full.
		return true;
	}

} // namespace GalaxyEntityGen
// GalaxyGenProbe.cpp

#include "GalaxyGenProbe.h"

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "Async/TaskGraphInterfaces.h"

IMPLEMENT_GLOBAL_SHADER(FGalaxyGenProbeCS,
	"/UltraLargeScale/Private/GalaxyGenProbe.usf", "MainCS", SF_Compute);

namespace GalaxyGenProbe
{

void Run()
{
	if (IsInGameThread() || IsInRenderingThread())
	{
		ensureMsgf(false, TEXT("GalaxyGenProbe::Run must be called from a background thread"));
		return;
	}

	// Shared with the render thread; the probe outlives this scope only through it.
	struct FProbeResult
	{
		TUniquePtr<FRHIGPUBufferReadback> Readback;
		std::atomic<bool> bSubmitted { false };
		std::atomic<bool> bCopied { false };
		uint32 Values[2] = { 0, 0 };
	};

	TSharedRef<FProbeResult, ESPMode::ThreadSafe> Result = MakeShared<FProbeResult, ESPMode::ThreadSafe>();

	Result->Readback = MakeUnique<FRHIGPUBufferReadback>(TEXT("GalaxyGenProbeReadback"));
	FRHIGPUBufferReadback* ReadbackPtr = Result->Readback.Get();

	auto Enqueue = [Result, ReadbackPtr]() mutable
	{
		ENQUEUE_RENDER_COMMAND(GalaxyGenProbeDispatch)(
			[Result, ReadbackPtr](FRHICommandListImmediate& RHICmdList) mutable
			{
				FRDGBuilder GraphBuilder(RHICmdList);

				FRDGBufferDesc Desc = FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 2);
				Desc.Usage |= EBufferUsageFlags::SourceCopy;

				FRDGBufferRef Buffer = GraphBuilder.CreateBuffer(Desc, TEXT("GalaxyGenProbeBuffer"));

				FGalaxyGenProbeCS::FParameters* P =
					GraphBuilder.AllocParameters<FGalaxyGenProbeCS::FParameters>();

				P->OutProbe = GraphBuilder.CreateUAV(Buffer, PF_R32_UINT);

				// Cleared to a value the shader never writes, so "the clear ran but the
				// shader did not" is distinguishable from "nothing ran at all".
				AddClearUAVPass(GraphBuilder, P->OutProbe, 7u);

				TShaderMapRef<FGalaxyGenProbeCS> ComputeShader(
					GetGlobalShaderMap(GMaxRHIFeatureLevel));

				FComputeShaderUtils::AddPass(
					GraphBuilder,
					RDG_EVENT_NAME("GalaxyGenProbe"),
					ERDGPassFlags::AsyncCompute,
					ComputeShader,
					P,
					FIntVector(1, 1, 1));

				AddEnqueueCopyPass(GraphBuilder, ReadbackPtr, Buffer, 2 * sizeof(uint32));

				GraphBuilder.Execute();

				Result->bSubmitted = true;
			});
	};

	if (IsInGameThread())
	{
		Enqueue();
	}
	else
	{
		AsyncTask(ENamedThreads::GameThread, MoveTemp(Enqueue));
	}

	const double Deadline = FPlatformTime::Seconds() + 5.0;

	while (!(Result->bSubmitted && ReadbackPtr->IsReady()))
	{
		if (FPlatformTime::Seconds() > Deadline)
		{
			UE_LOG(LogTemp, Error,
				TEXT("GalaxyGenProbe: timed out. submitted=%d -- the render command never ")
				TEXT("completed, so the problem is upstream of the shader entirely."),
				Result->bSubmitted ? 1 : 0);
			return;
		}
		FPlatformProcess::Sleep(0.0005f);
	}

	// Same marshalled lock the real path uses: Lock maps through the RHI and is
	// render-thread only.
	auto EnqueueCopy = [Result, ReadbackPtr]() mutable
	{
		ENQUEUE_RENDER_COMMAND(GalaxyGenProbeCopy)(
			[Result, ReadbackPtr](FRHICommandListImmediate&) mutable
			{
				const uint32* Src = static_cast<const uint32*>(ReadbackPtr->Lock(2 * sizeof(uint32)));
				if (Src)
				{
					Result->Values[0] = Src[0];
					Result->Values[1] = Src[1];
				}
				ReadbackPtr->Unlock();
				Result->bCopied = true;
			});
	};

	AsyncTask(ENamedThreads::GameThread, MoveTemp(EnqueueCopy));

	const double CopyDeadline = FPlatformTime::Seconds() + 5.0;
	while (!Result->bCopied)
	{
		if (FPlatformTime::Seconds() > CopyDeadline)
		{
			UE_LOG(LogTemp, Error, TEXT("GalaxyGenProbe: staging copy timed out."));
			return;
		}
		FPlatformProcess::Sleep(0.0005f);
	}

	if (Result->Values[0] == 1234)
	{
		UE_LOG(LogTemp, Display,
			TEXT("GalaxyGenProbe: PASS -- got %u, %u. Dispatch, UAV binding and readback ")
			TEXT("all work. The bug is inside GalaxyEntityGen, not the setup."),
			Result->Values[0], Result->Values[1]);
	}
	else if (Result->Values[0] == 7)
	{
		UE_LOG(LogTemp, Error,
			TEXT("GalaxyGenProbe: CLEAR RAN, SHADER DID NOT -- got %u, %u. The graph ")
			TEXT("executes and RDG writes to the buffer, but the compute dispatch does ")
			TEXT("not. Look at shader registration and the dispatch itself, not at ")
			TEXT("buffers or parameters."),
			Result->Values[0], Result->Values[1]);
	}
	else
	{
		UE_LOG(LogTemp, Error,
			TEXT("GalaxyGenProbe: NOTHING LANDED -- got %u, %u (expected 1234, or 7 if ")
			TEXT("only the clear ran). Neither the clear nor the shader reached this ")
			TEXT("buffer, so the readback or the graph is at fault rather than anything ")
			TEXT("shader-side."),
			Result->Values[0], Result->Values[1]);
	}
}

} // namespace GalaxyGenProbe

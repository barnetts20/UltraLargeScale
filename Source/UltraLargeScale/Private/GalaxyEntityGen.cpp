// GalaxyEntityGen.cpp

#include "GalaxyEntityGen.h"

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "GlobalShader.h"
#include "ShaderParameterUtils.h"
#include "RenderingThread.h"
#include "Engine/Texture.h"
#include "TextureResource.h"

IMPLEMENT_GLOBAL_SHADER(FGalaxyEntityGenCS,
	"/UltraLargeScale/GalaxyEntityGen.usf", "MainCS", SF_Compute);

void FGalaxyEntityGenRequest::Consume(TArray<FGalaxyEntityOut>& OutValid)
{
	OutValid.Reset();

	if (!Readback.IsValid() || !Readback->IsReady())
	{
		return;
	}

	const int32 Total = SlotStride * NumCells;
	const uint32 Bytes = static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut);

	const FGalaxyEntityOut* Src =
		static_cast<const FGalaxyEntityOut*>(Readback->Lock(Bytes));

	if (Src)
	{
		// Reserve for the whole buffer rather than the accepted count: acceptance is
		// a property of the field and is not known until it has been read.
		OutValid.Reserve(Total);

		for (int32 i = 0; i < Total; ++i)
		{
			if (Src[i].bValid != 0)
			{
				OutValid.Add(Src[i]);
			}
		}
	}

	Readback->Unlock();
	Readback.Reset();
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

		FTextureResource* NoiseResource =
			InNoiseTexture ? InNoiseTexture->GetResource() : nullptr;

		ENQUEUE_RENDER_COMMAND(GalaxyEntityGenDispatch)(
			[=, Cells = MoveTemp(InCells)](FRHICommandListImmediate& RHICmdList) mutable
			{
				if (!NoiseResource || !NoiseResource->TextureRHI)
				{
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

				FGalaxyEntityGenCS::FParameters* P =
					GraphBuilder.AllocParameters<FGalaxyEntityGenCS::FParameters>();

				P->OutEntities = GraphBuilder.CreateUAV(EntityBuffer);
				P->InCells = GraphBuilder.CreateSRV(CellBuffer);

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
					ERDGPassFlags::AsyncCompute,
					ComputeShader,
					P,
					Groups);

				// --- readback ---
				// Copy on the GPU, fence, and leave. NEVER block here: a synchronous
				// readback stalls the render thread for the full pipeline depth, and the
				// caller polls IsReady() over the following frames instead.
				OutRequest->Readback = MakeUnique<FRHIGPUBufferReadback>(
					TEXT("GalaxyEntityGenReadback"));

				AddEnqueueCopyPass(GraphBuilder, OutRequest->Readback.Get(), EntityBuffer,
					static_cast<uint32>(Total) * sizeof(FGalaxyEntityOut));

				GraphBuilder.Execute();
			});
	}

} // namespace GalaxyEntityGen
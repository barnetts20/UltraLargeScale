// GalaxyGenProbe.h
//
// The smallest compute dispatch that can prove the plumbing works, using this
// module's own shader path, parameter-struct style and marshalling -- but none of the
// field code. Testing those through GalaxyEntityGen means testing them through five
// hundred lines of density evaluation at the same time.
//
// Call Run() ONCE from a BACKGROUND thread; the top of
// GalaxyDataGenerator::GenerateTierBatchGPU behind a static bool is the natural
// place. It logs its own verdict.

#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"
#include "RHIGPUReadback.h"

#include <atomic>

class FGalaxyGenProbeCS : public FGlobalShader
{
public:
	DECLARE_GLOBAL_SHADER(FGalaxyGenProbeCS);
	SHADER_USE_PARAMETER_STRUCT(FGalaxyGenProbeCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutProbe)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
	{
		return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5);
	}
};

namespace GalaxyGenProbe
{
	/** Dispatches one thread, reads back two uints, logs the result. Background
	 *  thread only -- it blocks, exactly as the real path does. */
	void Run();
}
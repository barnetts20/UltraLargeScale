// Copyright Epic Games, Inc. All Rights Reserved.
#include "UltraLargeScale.h"
#include "Misc/MessageDialog.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "ShaderCore.h"

DEFINE_LOG_CATEGORY(LogSVOPerf);

#define LOCTEXT_NAMESPACE "FUltraLargeScaleModule"

void FUltraLargeScaleModule::StartupModule()
{
	// Map /UltraLargeScale/... in shader include paths onto <Plugin>/Shaders/...
	// so material Custom nodes can #include from this plugin.
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UltraLargeScale"));
	if (!Plugin.IsValid())
	{
		UE_LOG(LogSVOPerf, Error,
			TEXT("UltraLargeScale plugin not found; shader include path not registered."));
		return;
	}

	const FString ShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));

	// Registering the same virtual path twice asserts, so this is guarded for
	// live-coding reloads, which re-run StartupModule without unregistering.
	if (!AllShaderSourceDirectoryMappings().Contains(TEXT("/UltraLargeScale")))
	{
		AddShaderSourceDirectoryMapping(TEXT("/UltraLargeScale"), ShaderDir);
	}
}

void FUltraLargeScaleModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUltraLargeScaleModule, UltraLargeScale)
// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class svo : ModuleRules
{
	public svo(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        CMakeTarget.add(Target, this, "FastNoise", Path.Combine(this.ModuleDirectory, "../FastNoise2"), "-DFASTNOISE2_NOISETOOL=OFF", true);
        CppStandard = CppStandardVersion.Cpp20; //Clean up mutated cpp state
			
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"InputCore",
                "RHI",
				"RenderCore",
				"Renderer",
				"ApplicationCore",
                "Niagara",
				"Projects",
				"EnhancedInput"
				// ... add other public dependencies that you statically link with here ...
			}
		);
        PrivateDependencyModuleNames.AddRange(new string[]
		{
            "TraceLog",         // Add this for trace channels
            "RHICore"           // Add this if available in your UE5 version
		});
    }
}

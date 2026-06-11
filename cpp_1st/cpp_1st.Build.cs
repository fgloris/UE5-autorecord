// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class cpp_1st : ModuleRules
{
	public cpp_1st(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine",
			"InputCore", "EnhancedInput",
			"NavigationSystem", "AIModule",
			"Json", "JsonUtilities"
		});

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(new string[] { "UnrealEd" });
		}
	}
}

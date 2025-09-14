// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
#include <FOctreeNode.h>
#include "FastNoise/FastNoise.h"
#include "CoreMinimal.h"

/**
 * 
 */
struct SVO_API FPointData {
	FVector Position;
	int InsertDepth;
	FVoxelData Data;
	FInt64Vector GetInt64Position() {
		return FInt64Vector(FMath::RoundToInt64(Position.X), FMath::RoundToInt64(Position.Y), FMath::RoundToInt64(Position.Z));
	}
};

class SVO_API PointCloudGenerator
{
public:
	int Seed;
	int Count = 1000000;
	int Type = 1;
	int DepthRange = 8;
	int MinInsertionDepth = 1;
	int MaxInsertionDepth = 1;
	int InsertDepthOffset = 0;
	//Other implementations responsible for handling their internal configuration randomization
	PointCloudGenerator(int InSeed) : Seed(InSeed) {};
	virtual ~PointCloudGenerator() = default;

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) = 0;

	// Applies noise derivative based point shifting
	FInt64Vector ApplyNoiseDerivative(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Vector InSamplePosition, float& OutDensity);
	bool ApplyNoiseSelective(FastNoise::SmartNode<> InNoise, double InDensity, double InDomainScale, int64 InExtent, FVector InSamplePosition);
	
	FInt64Vector RotateCoordinate(FInt64Vector InCoordinate, FRotator InAxes);

	FVector RotateCoordinate(FVector InCoordinate, FRotator InRotation);

	FRotator Rotation = FRotator(0.0, 0.0, 0.0);
	FVector WarpAmount = FVector(1, 1, 1); // Controls how far we push points along the gradient

	//ddxddyddz constants
	const int GridSize = 3;
	const float OffsetPositionsX[27] = {
		-1,  0,  1,  -1,  0,  1,  -1,  0,  1,
		-1,  0,  1,  -1,  0,  1,  -1,  0,  1,
		-1,  0,  1,  -1,  0,  1,  -1,  0,  1
	};
	const float OffsetPositionsY[27] = {
		-1, -1, -1,  0, 0, 0,  1, 1, 1,
		-1, -1, -1,  0, 0, 0,  1, 1, 1,
		-1, -1, -1,  0, 0, 0,  1, 1, 1
	};
	const float OffsetPositionsZ[27] = {
		-1, -1, -1, -1, -1, -1, -1, -1, -1,
		 0,  0,  0,  0,  0,  0,  0,  0,  0,
		 1,  1,  1,  1,  1,  1,  1,  1,  1
	};
};

//Basic Generators (No Noise)
class SVO_API SimpleRandomGenerator : public PointCloudGenerator
{
public:
	SimpleRandomGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;
};
class SVO_API SimpleRandomNoiseGenerator : public PointCloudGenerator
{
public:
	SimpleRandomNoiseGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;
	//Node tree string from noise tool
	const char* EncodedTree = "EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==";
};
class SVO_API GlobularGenerator : public PointCloudGenerator
{
public:
	GlobularGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	double HorizontalExtent = 1;
	double VerticalExtent = 1;
	double Falloff = 2;
};
class SVO_API GlobularNoiseGenerator : public PointCloudGenerator
{
public:
	GlobularNoiseGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	double HorizontalExtent = 1;
	double VerticalExtent = 1;
	double Falloff = 2;

	const char* EncodedTree = "EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==";
};
class SVO_API SpiralGenerator : public PointCloudGenerator
{
public:
	SpiralGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	//Arm Coeff
	int NumArms = 4;
	double PitchAngle = 25;
	double ArmContrast = .5;
	double RadialFalloff = 3;
	double CenterScale = .05;

	//Jitter Coeff
	double HorizontalSpreadMin = 0.02;
	double HorizontalSpreadMax = .3;
	double VerticalSpreadMin = 0.02;
	double VerticalSpreadMax = .1;

	//Scale Coef
	double RadialDistance;
	double CenterDistance;
	double VerticalDistance;
	double HorizontalSpreadDistance;
	double VerticalSpreadDistance;
	double PitchAngleRadians;
	double MaxTheta;
};
class SVO_API SpiralNoiseGenerator : public PointCloudGenerator
{
public:
	SpiralNoiseGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	//Arm Coeff
	int NumArms = 4;
	double PitchAngle = 25;
	double ArmContrast = .5;
	double RadialFalloff = 3;
	double CenterScale = .05;

	//Jitter Coeff
	double HorizontalSpreadMin = 0.02;
	double HorizontalSpreadMax = .3;
	double VerticalSpreadMin = 0.02;
	double VerticalSpreadMax = .1;

	//Scale Coef
	double RadialDistance;
	double CenterDistance;
	double VerticalDistance;
	double HorizontalSpreadDistance;
	double VerticalSpreadDistance;
	double PitchAngleRadians;
	double MaxTheta;

	//TODO: Should randomize distortion formula
	//const char* EncodedTree = "DQAIAAAAAAAAQAcAAAAAAD8AAAAAAA==";
	const char* EncodedTree = "FwAAAAAAAACAPwAAgD8AAIC/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA";
	//const char* EncodedTree = "FwAAAAAAmpmZPwAAAAAAAIA/DwABAAAAAAAAQA0ACAAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAPwAAAAAA";
	//const char* EncodedTree = "DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==";
	//const char* EncodedTree = "FwAAAADAAACAPwAAgD8AAIC/EAAAAAA/DQAGAAAAAAAAQBcAAAAAAAAAgD8AAIC/AACAPwsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAEbABMAzcxMPg0AAwAAAAAAAEAIAAAAAAA/AAAAAAAAAAAAQA==";
	//const char* EncodedTree = "FwAAAAAAAACAPwAAgD8AAIC/DQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAA==";
};
class SVO_API BurstGenerator : public PointCloudGenerator
{
public:
	BurstGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;
};
class SVO_API BurstNoiseGenerator : public PointCloudGenerator
{
public:
	BurstNoiseGenerator(int InSeed) : PointCloudGenerator(InSeed) {};
	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	const char* EncodedTree = "EABSuD5ADQAIAAAAAAAAQAcAAAAAAD8AAAAAAAH//wAA";
};

/// <summary>
///	GALAXY GENERATION PARAM STRUCT
/// </summary>
struct SVO_API GalaxyParams {
	//Base Params - control overall sizes
	double GalaxyRatio = .33;
	double BulgeRatio = .75;
	double VoidRatio = .033;

	//Twist Params - Alters the behavior of the twist pass
	double TwistStrength = 4;
	double TwistCoreRadius = .01;
	double TwistCoreTwistExponent = 1;
	double TwistCoreStrength = 4;

	//Arm Params - changes arm appearance
	int ArmNumPoints = 75000; //Total arm points to generate
	int ArmNumArms = 4; //Number of arms
	int ArmClusters = 124; //Number of clusters per arm
	double ArmDepthBias = .5; //Smaller number = more large stars, larger number = more small stars, at 1 it will use a basic stellar size distribution table
	double ArmBaseDensity = 3; //Base density factor
	double ArmSpreadFactor = .25; //Base cluster scale
	double ArmClusterRadiusMin = .05; //Cluster scale ramp start  
	double ArmClusterRadiusMax = .3; //Cluster scale ramp end
	double ArmStartRatio = .5; //Where will the arm start relative to the bulge
	double ArmHeightRatio = .5; //Height squish
	double ArmIncoherence = 6; //Height value = more scattered arms
	double ArmRadialDensityExponent = 2; //Density falloff towards center
	double ArmRadialDensityMultiplier = 8; //Density multiplier as points are further from center
	double ArmRadialBaseDensity = .5; //Min radial density multiplier

	//Bulge Params - Controls the galactic bulge
	int BulgeNumPoints = 75000;
	double BulgeBaseDensity = 2;
	double BulgeDepthBias = .75;
	double BulgeRadiusScale = .33;
	double BulgeTruncationScale = 1;
	double BulgeAcceptanceExponent = 2;
	double BulgeJitter = .2;
	FVector BulgeAxisScale = FVector(1, 1, .6);

	//Cluster Params
	int ClusterNumPoints = 50000;
	int ClusterNumClusters = 128;
	FVector ClusterAxisScale = FVector(1, 1, 1);
	double ClusterSpreadFactor = .35;
	double ClusterMinScale = .3;
	double ClusterMaxScale = .7;
	double ClusterIncoherence = 3;
	double ClusterBaseDensity = 1;
	double ClusterDepthBias = 1;

	//Disc Params - Controls the non spiral disc
	int DiscNumPoints = 50000;
	double DiscBaseDensity = 1;
	double DiscDepthBias = 1;
	double DiscHeightRatio = .1;

	//Background Params - Controls the background halo 
	int BackgroundNumPoints = 50000;
	double BackgroundBaseDensity = 10;
	double BackgroundDepthBias = 1;
	double BackgroundHeightRatio = .8;

	//Volume Material Params
	FLinearColor VolumeAmbientColor = FLinearColor(1,1,1,1);
	FLinearColor VolumeCoolShift = FLinearColor(.2,.5,.8);
	FLinearColor VolumeHotShift = FLinearColor(.5, 1.5, 3);
	double VolumeHueVariance = .1;
	double VolumeHueVarianceScale = .5;
	double VolumeSaturationVariance = .1;
	double VolumeTemperatureInfluence = 32;
	double VolumeTemperatureScale = 1;
	double VolumeDensity = .5;
	double VolumeWarpAmount = .05;
	double VolumeWarpScale = .33;
	FString VolumeNoise = "/svo/VolumeTextures/VT_PerlinWorley_Balanced";
};

/// <summary>
/// PARAM FACTORY CONSTRUCTOR - DEFINES RANDOMIZATION RANGES AND ARCHTYPES
/// </summary>
class SVO_API GalaxyParamFactory {
public:

	int Seed = 666;
	//Galaxy archtypes
	GalaxyParams E0;
	GalaxyParams E3;
	GalaxyParams E5;
	GalaxyParams E7;
	GalaxyParams S0;
	GalaxyParams Sa;
	GalaxyParams Sb;
	GalaxyParams Sc;
	GalaxyParams SBa;
	GalaxyParams SBb;
	GalaxyParams SBc;
	GalaxyParams Irr;

	//Bounds
	GalaxyParams E0_Min;  GalaxyParams E0_Max;
	GalaxyParams E3_Min;  GalaxyParams E3_Max;
	GalaxyParams E5_Min;  GalaxyParams E5_Max;
	GalaxyParams E7_Min;  GalaxyParams E7_Max;
	GalaxyParams S0_Min;  GalaxyParams S0_Max;
	GalaxyParams Sa_Min;  GalaxyParams Sa_Max;
	GalaxyParams Sb_Min;  GalaxyParams Sb_Max;
	GalaxyParams Sc_Min;  GalaxyParams Sc_Max;
	GalaxyParams SBa_Min; GalaxyParams SBa_Max;
	GalaxyParams SBb_Min; GalaxyParams SBb_Max;
	GalaxyParams SBc_Min; GalaxyParams SBc_Max;
	GalaxyParams Irr_Min; GalaxyParams Irr_Max;

	//Stand alone volume material bounds
	GalaxyParams Volume_Min; GalaxyParams Volume_Max;

	TArray<TPair<float, GalaxyParams*>> GalaxyWeights = {{0.02f, &E0}, {0.04f, &E3}, {0.04f, &E5}, {0.03f, &E7}, {0.22f, &S0}, {0.15f, &Sa}, {0.15f, &Sb}, {0.20f, &Sc}, {0.04f, &SBa}, {0.04f, &SBb}, {0.03f, &SBc}, {0.04f, &Irr}};
	TArray<const char*> NoisePaths = { "/svo/VolumeTextures/VT_PerlinWorley_Balanced", "/svo/VolumeTextures/VT_Gradient_l5_256", "/svo/VolumeTextures/VT_Gradient_Turbulence_l5_256", "/svo/VolumeTextures/VT_Voronoi_l5_256" };

	GalaxyParamFactory() {
		#pragma region VolumeMaterialBounds
		Volume_Min.VolumeAmbientColor = FLinearColor(.3, .3, .3);
		Volume_Max.VolumeAmbientColor = FLinearColor(1.7, 1.7, 1.7);
		Volume_Min.VolumeCoolShift = FLinearColor(.01, .25, .5);
		Volume_Max.VolumeCoolShift = FLinearColor(.3, .7, 1);
		Volume_Min.VolumeDensity = .1;
		Volume_Max.VolumeDensity = 1.0;
		Volume_Min.VolumeHotShift = FLinearColor(.5, .25, .01);
		Volume_Max.VolumeHotShift = FLinearColor(1, .7, .3);
		Volume_Min.VolumeHueVariance = .01;
		Volume_Max.VolumeHueVariance = .25;
		Volume_Min.VolumeHueVarianceScale = .25;
		Volume_Max.VolumeHueVarianceScale = 1.75;
		Volume_Min.VolumeSaturationVariance = 0;
		Volume_Max.VolumeSaturationVariance = .5;
		Volume_Min.VolumeTemperatureScale = .1;
		Volume_Max.VolumeTemperatureScale = 10;
		Volume_Min.VolumeTemperatureInfluence = 8;
		Volume_Max.VolumeTemperatureInfluence = 48;
		Volume_Min.VolumeWarpAmount = .05;
		Volume_Max.VolumeWarpAmount = .15;
		Volume_Min.VolumeWarpScale = .2;
		Volume_Max.VolumeWarpScale = .8;
		#pragma endregion

		#pragma region E0 Archtype
		E0.ArmNumPoints = 0;
		E0.DiscNumPoints = 0;
		E0.BulgeNumPoints = 300000;
		E0.BulgeBaseDensity = 3;
		E0.BulgeDepthBias = .2;
		E0.BackgroundBaseDensity = 20;
		E0.BulgeAxisScale = FVector(1);
		E0.GalaxyRatio = .4;
		E0.BulgeRatio = 1;
		E0.BackgroundNumPoints = 100000;
		#pragma endregion
		#pragma region E0 Bounds
		E0_Min = E0;
		E0_Max = E0;
		E0_Min.GalaxyRatio = .2;
		E0_Max.GalaxyRatio = .5;
		E0_Min.BulgeAcceptanceExponent = 1.5;
		E0_Max.BulgeAcceptanceExponent = 2.5;
		E0_Min.BulgeAxisScale = FVector(.85, .85, .85);
		E0_Max.BulgeAxisScale = FVector(1, 1, 1);
		E0_Min.BulgeBaseDensity = 2;
		E0_Max.BulgeBaseDensity = 4;
		E0_Min.BulgeDepthBias = .6;
		E0_Max.BulgeDepthBias = .8;
		E0_Min.BulgeNumPoints = 200000;
		E0_Max.BulgeNumPoints = 400000;
		E0_Min.BulgeRadiusScale = .2;
		E0_Max.BulgeRadiusScale = .4;
		E0_Min.BulgeRatio = .8;
		E0_Max.BulgeRatio = 1.2;
		E0_Min.BulgeTruncationScale = .7;
		E0_Max.BulgeTruncationScale = 1.3;
		E0_Min.BulgeJitter = .2;
		E0_Max.BulgeJitter = .4;
		E0_Min.BackgroundNumPoints = 50000;
		E0_Max.BackgroundNumPoints = 200000;
		E0_Min.BackgroundBaseDensity = 15;
		E0_Max.BackgroundBaseDensity = 25;
		E0_Min.BackgroundDepthBias = .7;
		E0_Max.BackgroundDepthBias = 1.3;
		E0_Min.BackgroundHeightRatio = 1;
		E0_Max.BackgroundHeightRatio = 1;
		E0_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		E0_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		E0_Min.ClusterBaseDensity = 2;
		E0_Max.ClusterBaseDensity = 4;
		E0_Min.ClusterDepthBias = .1;
		E0_Max.ClusterDepthBias = .3;
		E0_Min.ClusterIncoherence = 3;
		E0_Max.ClusterIncoherence = 6;
		E0_Min.ClusterMaxScale = .8;
		E0_Max.ClusterMaxScale = .6;
		E0_Min.ClusterMinScale = .2;
		E0_Max.ClusterMinScale = .4;
		E0_Min.ClusterNumClusters = 12;
		E0_Max.ClusterNumClusters = 64;
		E0_Min.ClusterNumPoints = 100;
		E0_Max.ClusterNumPoints = 20000;
		E0_Min.ClusterSpreadFactor = .25;
		E0_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region E3 Archtype
		E3.ArmNumPoints = 0;
		E3.DiscNumPoints = 0;
		E3.BulgeNumPoints = 300000;
		E3.BulgeBaseDensity = 3;
		E3.BulgeDepthBias = .2;
		E3.BackgroundBaseDensity = 50;
		E3.BulgeAxisScale = FVector(1, 1, .7);
		E3.GalaxyRatio = .3;
		E3.BulgeRatio = 1.25;
		E3.BackgroundNumPoints = 100000;
		#pragma endregion
		#pragma region E3 Bounds
		E3_Min = E3;
		E3_Max = E3;
		E3_Min.GalaxyRatio = .2;
		E3_Max.GalaxyRatio = .5;
		E3_Min.BulgeAcceptanceExponent = 1.5;
		E3_Max.BulgeAcceptanceExponent = 2.5;
		E3_Min.BulgeAxisScale = FVector(.85, .85, .45);
		E3_Max.BulgeAxisScale = FVector(1, 1, .7);
		E3_Min.BulgeBaseDensity = 2;
		E3_Max.BulgeBaseDensity = 4;
		E3_Min.BulgeDepthBias = .6;
		E3_Max.BulgeDepthBias = .8;
		E3_Min.BulgeNumPoints = 200000;
		E3_Max.BulgeNumPoints = 400000;
		E3_Min.BulgeRadiusScale = .2;
		E3_Max.BulgeRadiusScale = .4;
		E3_Min.BulgeRatio = .8;
		E3_Max.BulgeRatio = 1.2;
		E3_Min.BulgeTruncationScale = .7;
		E3_Max.BulgeTruncationScale = 1.3;
		E3_Min.BulgeJitter = .2;
		E3_Max.BulgeJitter = .4;
		E3_Min.BackgroundNumPoints = 50000;
		E3_Max.BackgroundNumPoints = 200000;
		E3_Min.BackgroundBaseDensity = 15;
		E3_Max.BackgroundBaseDensity = 25;
		E3_Min.BackgroundDepthBias = .7;
		E3_Max.BackgroundDepthBias = 1.3;
		E3_Min.BackgroundHeightRatio = 1;
		E3_Max.BackgroundHeightRatio = 1;
		E3_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		E3_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		E3_Min.ClusterBaseDensity = 2;
		E3_Max.ClusterBaseDensity = 4;
		E3_Min.ClusterDepthBias = .1;
		E3_Max.ClusterDepthBias = .3;
		E3_Min.ClusterIncoherence = 3;
		E3_Max.ClusterIncoherence = 6;
		E3_Min.ClusterMaxScale = .8;
		E3_Max.ClusterMaxScale = .6;
		E3_Min.ClusterMinScale = .2;
		E3_Max.ClusterMinScale = .4;
		E3_Min.ClusterNumClusters = 12;
		E3_Max.ClusterNumClusters = 64;
		E3_Min.ClusterNumPoints = 100;
		E3_Max.ClusterNumPoints = 20000;
		E3_Min.ClusterSpreadFactor = .25;
		E3_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region E5 Archtype
		E5.ArmNumPoints = 0;
		E5.DiscNumPoints = 50000;
		E5.DiscHeightRatio = .3;
		E5.DiscDepthBias = 1.2;
		E5.DiscBaseDensity = 3;
		E5.BulgeNumPoints = 200000;
		E5.BulgeBaseDensity = 3;
		E5.BulgeDepthBias = .2;
		E5.BackgroundBaseDensity = 20;
		E5.BulgeAxisScale = FVector(1, 1, .7);
		E5.GalaxyRatio = .3;
		E5.BulgeRatio = 1.25;
		E5.BackgroundNumPoints = 100000;
		#pragma endregion
		#pragma region E5 Bounds
		E5_Min = E5;
		E5_Max = E5;
		E5_Min.GalaxyRatio = .2;
		E5_Max.GalaxyRatio = .5;
		E5_Min.DiscBaseDensity = 3;
		E5_Max.DiscBaseDensity = 6;
		E5_Min.DiscDepthBias = .8;
		E5_Max.DiscDepthBias = 1.2;
		E5_Min.DiscHeightRatio = .15;
		E5_Max.DiscHeightRatio = .4;
		E5_Min.DiscNumPoints = 25000;
		E5_Max.DiscNumPoints = 100000;
		E5_Min.BulgeAcceptanceExponent = 1.5;
		E5_Max.BulgeAcceptanceExponent = 2.5;
		E5_Min.BulgeAxisScale = FVector(.85, .85, .45);
		E5_Max.BulgeAxisScale = FVector(1, 1, .7);
		E5_Min.BulgeBaseDensity = 2;
		E5_Max.BulgeBaseDensity = 4;
		E5_Min.BulgeDepthBias = .6;
		E5_Max.BulgeDepthBias = .8;
		E5_Min.BulgeNumPoints = 200000;
		E5_Max.BulgeNumPoints = 400000;
		E5_Min.BulgeRadiusScale = .2;
		E5_Max.BulgeRadiusScale = .4;
		E5_Min.BulgeRatio = .8;
		E5_Max.BulgeRatio = 1.2;
		E5_Min.BulgeTruncationScale = .7;
		E5_Max.BulgeTruncationScale = 1.3;
		E5_Min.BulgeJitter = .2;
		E5_Max.BulgeJitter = .4;
		E5_Min.BackgroundNumPoints = 50000;
		E5_Max.BackgroundNumPoints = 200000;
		E5_Min.BackgroundBaseDensity = 15;
		E5_Max.BackgroundBaseDensity = 25;
		E5_Min.BackgroundDepthBias = .7;
		E5_Max.BackgroundDepthBias = 1.3;
		E5_Min.BackgroundHeightRatio = 1;
		E5_Max.BackgroundHeightRatio = 1;
		E5_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		E5_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		E5_Min.ClusterBaseDensity = 2;
		E5_Max.ClusterBaseDensity = 4;
		E5_Min.ClusterDepthBias = .1;
		E5_Max.ClusterDepthBias = .3;
		E5_Min.ClusterIncoherence = 3;
		E5_Max.ClusterIncoherence = 6;
		E5_Min.ClusterMaxScale = .8;
		E5_Max.ClusterMaxScale = .6;
		E5_Min.ClusterMinScale = .2;
		E5_Max.ClusterMinScale = .4;
		E5_Min.ClusterNumClusters = 12;
		E5_Max.ClusterNumClusters = 64;
		E5_Min.ClusterNumPoints = 100;
		E5_Max.ClusterNumPoints = 20000;
		E5_Min.ClusterSpreadFactor = .25;
		E5_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region E7 Archtype
		E7.ArmNumPoints = 0;
		E7.DiscNumPoints = 100000;
		E7.DiscDepthBias = .8;
		E7.DiscHeightRatio = .2;
		E7.DiscBaseDensity = 3;
		E7.BulgeNumPoints = 200000;
		E7.BulgeBaseDensity = 3;
		E7.BulgeDepthBias = .2;
		E7.BackgroundBaseDensity = 20;
		E7.BulgeAxisScale = FVector(1, 1, .6);
		E7.GalaxyRatio = .3;
		E7.BulgeRatio = 1.25;
		E7.BackgroundNumPoints = 100000;
		#pragma endregion
		#pragma region E7 Bounds
		E7_Min = E7;
		E7_Max = E7;
		E7_Min.GalaxyRatio = .2;
		E7_Max.GalaxyRatio = .5;
		E7_Min.DiscBaseDensity = 3;
		E7_Max.DiscBaseDensity = 6;
		E7_Min.DiscDepthBias = .8;
		E7_Max.DiscDepthBias = 1.2;
		E7_Min.DiscHeightRatio = .1;
		E7_Max.DiscHeightRatio = .25;
		E7_Min.DiscNumPoints = 75000;
		E7_Max.DiscNumPoints = 125000;
		E7_Min.BulgeAcceptanceExponent = 1.5;
		E7_Max.BulgeAcceptanceExponent = 2.5;
		E7_Min.BulgeAxisScale = FVector(.85, .85, .35);
		E7_Max.BulgeAxisScale = FVector(1, 1, .6);
		E7_Min.BulgeBaseDensity = 2;
		E7_Max.BulgeBaseDensity = 4;
		E7_Min.BulgeDepthBias = .6;
		E7_Max.BulgeDepthBias = .8;
		E7_Min.BulgeNumPoints = 150000;
		E7_Max.BulgeNumPoints = 300000;
		E7_Min.BulgeRadiusScale = .2;
		E7_Max.BulgeRadiusScale = .4;
		E7_Min.BulgeRatio = .8;
		E7_Max.BulgeRatio = 1.2;
		E7_Min.BulgeTruncationScale = .7;
		E7_Max.BulgeTruncationScale = 1.3;
		E7_Min.BulgeJitter = .2;
		E7_Max.BulgeJitter = .4;
		E7_Min.BackgroundNumPoints = 50000;
		E7_Max.BackgroundNumPoints = 150000;
		E7_Min.BackgroundBaseDensity = 15;
		E7_Max.BackgroundBaseDensity = 25;
		E7_Min.BackgroundDepthBias = .7;
		E7_Max.BackgroundDepthBias = 1.3;
		E7_Min.BackgroundHeightRatio = 1;
		E7_Max.BackgroundHeightRatio = 1;
		E7_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		E7_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		E7_Min.ClusterBaseDensity = 2;
		E7_Max.ClusterBaseDensity = 4;
		E7_Min.ClusterDepthBias = .1;
		E7_Max.ClusterDepthBias = .3;
		E7_Min.ClusterIncoherence = 3;
		E7_Max.ClusterIncoherence = 6;
		E7_Min.ClusterMaxScale = .8;
		E7_Max.ClusterMaxScale = .6;
		E7_Min.ClusterMinScale = .2;
		E7_Max.ClusterMinScale = .4;
		E7_Min.ClusterNumClusters = 12;
		E7_Max.ClusterNumClusters = 64;
		E7_Min.ClusterNumPoints = 100;
		E7_Max.ClusterNumPoints = 20000;
		E7_Min.ClusterSpreadFactor = .25;
		E7_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region S0 Archtype
		S0.ArmNumPoints = 0;
		S0.DiscNumPoints = 200000;
		S0.DiscDepthBias = .3;
		S0.DiscHeightRatio = .1;
		S0.BulgeNumPoints = 100000;
		S0.BulgeBaseDensity = 3;
		S0.BulgeDepthBias = .2;
		S0.BackgroundBaseDensity = 20;
		S0.BulgeAxisScale = FVector(1, 1, .5);
		S0.GalaxyRatio = .3;
		S0.BulgeRatio = 1;
		S0.BackgroundNumPoints = 100000;
		#pragma endregion
		#pragma region S0 Bounds
		S0_Min = S0;
		S0_Max = S0;
		S0_Min.GalaxyRatio = .2;
		S0_Max.GalaxyRatio = .5;
		S0_Min.DiscBaseDensity = 3;
		S0_Max.DiscBaseDensity = 6;
		S0_Min.DiscDepthBias = .8;
		S0_Max.DiscDepthBias = 1.2;
		S0_Min.DiscHeightRatio = .03;
		S0_Max.DiscHeightRatio = .1;
		S0_Min.DiscNumPoints = 150000;
		S0_Max.DiscNumPoints = 250000;
		S0_Min.BulgeAcceptanceExponent = 1.5;
		S0_Max.BulgeAcceptanceExponent = 2.5;
		S0_Min.BulgeAxisScale = FVector(.9, .9, .35);
		S0_Max.BulgeAxisScale = FVector(1, 1, .6);
		S0_Min.BulgeBaseDensity = 2;
		S0_Max.BulgeBaseDensity = 4;
		S0_Min.BulgeDepthBias = .6;
		S0_Max.BulgeDepthBias = .8;
		S0_Min.BulgeNumPoints = 50000;
		S0_Max.BulgeNumPoints = 150000;
		S0_Min.BulgeRadiusScale = .2;
		S0_Max.BulgeRadiusScale = .4;
		S0_Min.BulgeRatio = .8;
		S0_Max.BulgeRatio = 1.2;
		S0_Min.BulgeTruncationScale = .7;
		S0_Max.BulgeTruncationScale = 1.3;
		S0_Min.BulgeJitter = .2;
		S0_Max.BulgeJitter = .4;
		S0_Min.BackgroundNumPoints = 50000;
		S0_Max.BackgroundNumPoints = 150000;
		S0_Min.BackgroundBaseDensity = 15;
		S0_Max.BackgroundBaseDensity = 25;
		S0_Min.BackgroundDepthBias = .7;
		S0_Max.BackgroundDepthBias = 1.3;
		S0_Min.BackgroundHeightRatio = 1;
		S0_Max.BackgroundHeightRatio = 1;
		S0_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		S0_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		S0_Min.ClusterBaseDensity = 2;
		S0_Max.ClusterBaseDensity = 4;
		S0_Min.ClusterDepthBias = .1;
		S0_Max.ClusterDepthBias = .3;
		S0_Min.ClusterIncoherence = 3;
		S0_Max.ClusterIncoherence = 6;
		S0_Min.ClusterMaxScale = .8;
		S0_Max.ClusterMaxScale = .6;
		S0_Min.ClusterMinScale = .2;
		S0_Max.ClusterMinScale = .4;
		S0_Min.ClusterNumClusters = 12;
		S0_Max.ClusterNumClusters = 64;
		S0_Min.ClusterNumPoints = 100;
		S0_Max.ClusterNumPoints = 20000;
		S0_Min.ClusterSpreadFactor = .25;
		S0_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region Sa Archtype
		Sa.TwistStrength = 4;
		Sa.TwistCoreStrength = 4;
		Sa.ArmNumArms = 2;
		Sa.ArmClusterRadiusMin = .15;
		Sa.ArmClusterRadiusMax = .4;
		Sa.ArmIncoherence = 3;
		Sa.ArmStartRatio = 0;
		#pragma endregion
		#pragma region Sa Bounds
		Sa_Min = Sa;
		Sa_Max = Sa;
		Sa_Min.GalaxyRatio = .2;
		Sa_Max.GalaxyRatio = .5;
		Sa_Min.DiscBaseDensity = 8;
		Sa_Max.DiscBaseDensity = 16;
		Sa_Min.DiscDepthBias = .8;
		Sa_Max.DiscDepthBias = 1.2;
		Sa_Min.DiscHeightRatio = .03;
		Sa_Max.DiscHeightRatio = .1;
		Sa_Min.DiscNumPoints = 50000;
		Sa_Max.DiscNumPoints = 150000;
		Sa_Min.ArmBaseDensity = 1;
		Sa_Max.ArmBaseDensity = 4;
		Sa_Min.ArmClusterRadiusMax = .15;
		Sa_Max.ArmClusterRadiusMax = .45;
		Sa_Min.ArmClusterRadiusMin = .025;
		Sa_Max.ArmClusterRadiusMin = .1;
		Sa_Min.ArmClusters = 64;
		Sa_Max.ArmClusters = 256;
		Sa_Min.ArmDepthBias = .2;
		Sa_Max.ArmDepthBias = .6;
		Sa_Min.ArmHeightRatio = .15;
		Sa_Max.ArmHeightRatio = .5;
		Sa_Min.ArmIncoherence = 3;
		Sa_Max.ArmIncoherence = 8;
		Sa_Min.ArmNumArms = 2;
		Sa_Max.ArmNumArms = 2;
		Sa_Min.ArmNumPoints = 100000;
		Sa_Max.ArmNumPoints = 200000;
		Sa_Min.ArmRadialBaseDensity = .1;
		Sa_Max.ArmRadialBaseDensity = 1;
		Sa_Min.ArmRadialDensityExponent = 1;
		Sa_Max.ArmRadialDensityExponent = 3;
		Sa_Min.ArmRadialDensityMultiplier = 2;
		Sa_Max.ArmRadialDensityMultiplier = 12;
		Sa_Min.ArmSpreadFactor = .25;
		Sa_Max.ArmSpreadFactor = .5;
		Sa_Min.ArmStartRatio = .25;
		Sa_Max.ArmStartRatio = .5;
		Sa_Min.TwistCoreRadius = .005;
		Sa_Max.TwistCoreRadius = .03;
		Sa_Min.TwistCoreStrength = 2;
		Sa_Max.TwistCoreStrength = 8;
		Sa_Min.TwistCoreTwistExponent = .9;
		Sa_Max.TwistCoreTwistExponent = 1.1;
		Sa_Min.TwistStrength = 6;
		Sa_Max.TwistStrength = 16;
		Sa_Min.BulgeAcceptanceExponent = 1.5;
		Sa_Max.BulgeAcceptanceExponent = 2.5;
		Sa_Min.BulgeAxisScale = FVector(.9, .9, .35);
		Sa_Max.BulgeAxisScale = FVector(1, 1, .6);
		Sa_Min.BulgeBaseDensity = 2;
		Sa_Max.BulgeBaseDensity = 4;
		Sa_Min.BulgeDepthBias = .6;
		Sa_Max.BulgeDepthBias = .8;
		Sa_Min.BulgeNumPoints = 100000;
		Sa_Max.BulgeNumPoints = 200000;
		Sa_Min.BulgeRadiusScale = .2;
		Sa_Max.BulgeRadiusScale = .4;
		Sa_Min.BulgeRatio = .8;
		Sa_Max.BulgeRatio = 1.2;
		Sa_Min.BulgeTruncationScale = .7;
		Sa_Max.BulgeTruncationScale = 1.3;
		Sa_Min.BulgeJitter = .2;
		Sa_Max.BulgeJitter = .4;
		Sa_Min.BackgroundNumPoints = 50000;
		Sa_Max.BackgroundNumPoints = 150000;
		Sa_Min.BackgroundBaseDensity = 15;
		Sa_Max.BackgroundBaseDensity = 25;
		Sa_Min.BackgroundDepthBias = .7;
		Sa_Max.BackgroundDepthBias = 1.3;
		Sa_Min.BackgroundHeightRatio = 1;
		Sa_Max.BackgroundHeightRatio = 1;
		Sa_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		Sa_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		Sa_Min.ClusterBaseDensity = 2;
		Sa_Max.ClusterBaseDensity = 4;
		Sa_Min.ClusterDepthBias = .1;
		Sa_Max.ClusterDepthBias = .3;
		Sa_Min.ClusterIncoherence = 2;
		Sa_Max.ClusterIncoherence = 6;
		Sa_Min.ClusterMaxScale = .8;
		Sa_Max.ClusterMaxScale = .6;
		Sa_Min.ClusterMinScale = .2;
		Sa_Max.ClusterMinScale = .4;
		Sa_Min.ClusterNumClusters = 12;
		Sa_Max.ClusterNumClusters = 64;
		Sa_Min.ClusterNumPoints = 100;
		Sa_Max.ClusterNumPoints = 20000;
		Sa_Min.ClusterSpreadFactor = .25;
		Sa_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region Sb Archtype
		Sb.TwistStrength = 12;
		Sb.ArmNumArms = 2;
		Sb.ArmIncoherence = 6;
		Sb.ArmStartRatio = 0;
		#pragma endregion
		#pragma region Sb Bounds
		Sb_Min = Sb;
		Sb_Max = Sb;
		Sb_Min.GalaxyRatio = .2;
		Sb_Max.GalaxyRatio = .5;
		Sb_Min.DiscBaseDensity = 8;
		Sb_Max.DiscBaseDensity = 16;
		Sb_Min.DiscDepthBias = .8;
		Sb_Max.DiscDepthBias = 1.2;
		Sb_Min.DiscHeightRatio = .03;
		Sb_Max.DiscHeightRatio = .1;
		Sb_Min.DiscNumPoints = 50000;
		Sb_Max.DiscNumPoints = 150000;
		Sb_Min.ArmBaseDensity = 1;
		Sb_Max.ArmBaseDensity = 4;
		Sb_Min.ArmClusterRadiusMax = .15;
		Sb_Max.ArmClusterRadiusMax = .45;
		Sb_Min.ArmClusterRadiusMin = .025;
		Sb_Max.ArmClusterRadiusMin = .1;
		Sb_Min.ArmClusters = 64;
		Sb_Max.ArmClusters = 256;
		Sb_Min.ArmDepthBias = .2;
		Sb_Max.ArmDepthBias = .6;
		Sb_Min.ArmHeightRatio = .15;
		Sb_Max.ArmHeightRatio = .5;
		Sb_Min.ArmIncoherence = 5;
		Sb_Max.ArmIncoherence = 10;
		Sb_Min.ArmNumArms = 2;
		Sb_Max.ArmNumArms = 2;
		Sb_Min.ArmNumPoints = 100000;
		Sb_Max.ArmNumPoints = 200000;
		Sb_Min.ArmRadialBaseDensity = .1;
		Sb_Max.ArmRadialBaseDensity = 1;
		Sb_Min.ArmRadialDensityExponent = 1;
		Sb_Max.ArmRadialDensityExponent = 3;
		Sb_Min.ArmRadialDensityMultiplier = 2;
		Sb_Max.ArmRadialDensityMultiplier = 12;
		Sb_Min.ArmSpreadFactor = .25;
		Sb_Max.ArmSpreadFactor = .5;
		Sb_Min.ArmStartRatio = .25;
		Sb_Max.ArmStartRatio = .5;
		Sb_Min.TwistCoreRadius = .01;
		Sb_Max.TwistCoreRadius = .035;
		Sb_Min.TwistCoreStrength = 3;
		Sb_Max.TwistCoreStrength = 9;
		Sb_Min.TwistCoreTwistExponent = .9;
		Sb_Max.TwistCoreTwistExponent = 1.1;
		Sb_Min.TwistStrength = 16;
		Sb_Max.TwistStrength = 32;
		Sb_Min.BulgeAcceptanceExponent = 1.5;
		Sb_Max.BulgeAcceptanceExponent = 2.5;
		Sb_Min.BulgeAxisScale = FVector(.9, .9, .35);
		Sb_Max.BulgeAxisScale = FVector(1, 1, .6);
		Sb_Min.BulgeBaseDensity = 2;
		Sb_Max.BulgeBaseDensity = 4;
		Sb_Min.BulgeDepthBias = .6;
		Sb_Max.BulgeDepthBias = .8;
		Sb_Min.BulgeNumPoints = 100000;
		Sb_Max.BulgeNumPoints = 200000;
		Sb_Min.BulgeRadiusScale = .2;
		Sb_Max.BulgeRadiusScale = .4;
		Sb_Min.BulgeRatio = .8;
		Sb_Max.BulgeRatio = 1.2;
		Sb_Min.BulgeTruncationScale = .7;
		Sb_Max.BulgeTruncationScale = 1.3;
		Sb_Min.BulgeJitter = .2;
		Sb_Max.BulgeJitter = .4;
		Sb_Min.BackgroundNumPoints = 50000;
		Sb_Max.BackgroundNumPoints = 150000;
		Sb_Min.BackgroundBaseDensity = 15;
		Sb_Max.BackgroundBaseDensity = 25;
		Sb_Min.BackgroundDepthBias = .7;
		Sb_Max.BackgroundDepthBias = 1.3;
		Sb_Min.BackgroundHeightRatio = 1;
		Sb_Max.BackgroundHeightRatio = 1;
		Sb_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		Sb_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		Sb_Min.ClusterBaseDensity = 2;
		Sb_Max.ClusterBaseDensity = 4;
		Sb_Min.ClusterDepthBias = .1;
		Sb_Max.ClusterDepthBias = .3;
		Sb_Min.ClusterIncoherence = 2;
		Sb_Max.ClusterIncoherence = 6;
		Sb_Min.ClusterMaxScale = .8;
		Sb_Max.ClusterMaxScale = .6;
		Sb_Min.ClusterMinScale = .2;
		Sb_Max.ClusterMinScale = .4;
		Sb_Min.ClusterNumClusters = 12;
		Sb_Max.ClusterNumClusters = 64;
		Sb_Min.ClusterNumPoints = 100;
		Sb_Max.ClusterNumPoints = 20000;
		Sb_Min.ClusterSpreadFactor = .25;
		Sb_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region Sc Archtype
		Sc.TwistStrength = 8;
		Sc.ArmNumArms = 4;
		Sc.ArmIncoherence = 8;
		Sc.ArmStartRatio = 0;
		Sc.TwistCoreRadius = .02;
		#pragma endregion
		#pragma region Sc Bounds
		Sc_Min = Sc;
		Sc_Max = Sc;
		Sc_Min.GalaxyRatio = .2;
		Sc_Max.GalaxyRatio = .5;
		Sc_Min.DiscBaseDensity = 8;
		Sc_Max.DiscBaseDensity = 16;
		Sc_Min.DiscDepthBias = .8;
		Sc_Max.DiscDepthBias = 1.2;
		Sc_Min.DiscHeightRatio = .03;
		Sc_Max.DiscHeightRatio = .1;
		Sc_Min.DiscNumPoints = 50000;
		Sc_Max.DiscNumPoints = 100000;
		Sc_Min.ArmBaseDensity = 1;
		Sc_Max.ArmBaseDensity = 4;
		Sc_Min.ArmClusterRadiusMax = .15;
		Sc_Max.ArmClusterRadiusMax = .45;
		Sc_Min.ArmClusterRadiusMin = .025;
		Sc_Max.ArmClusterRadiusMin = .1;
		Sc_Min.ArmClusters = 64;
		Sc_Max.ArmClusters = 256;
		Sc_Min.ArmDepthBias = .2;
		Sc_Max.ArmDepthBias = .4;
		Sc_Min.ArmHeightRatio = .15;
		Sc_Max.ArmHeightRatio = .5;
		Sc_Min.ArmIncoherence = 2;
		Sc_Max.ArmIncoherence = 6;
		Sc_Min.ArmNumArms = 3;
		Sc_Max.ArmNumArms = 8;
		Sc_Min.ArmNumPoints = 200000;
		Sc_Max.ArmNumPoints = 300000;
		Sc_Min.ArmRadialBaseDensity = .1;
		Sc_Max.ArmRadialBaseDensity = 1;
		Sc_Min.ArmRadialDensityExponent = 1;
		Sc_Max.ArmRadialDensityExponent = 3;
		Sc_Min.ArmRadialDensityMultiplier = 2;
		Sc_Max.ArmRadialDensityMultiplier = 12;
		Sc_Min.ArmSpreadFactor = .25;
		Sc_Max.ArmSpreadFactor = .5;
		Sc_Min.ArmStartRatio = .25;
		Sc_Max.ArmStartRatio = .5;
		Sc_Min.TwistCoreRadius = .01;
		Sc_Max.TwistCoreRadius = .035;
		Sc_Min.TwistCoreStrength = 3;
		Sc_Max.TwistCoreStrength = 9;
		Sc_Min.TwistCoreTwistExponent = .9;
		Sc_Max.TwistCoreTwistExponent = 1.1;
		Sc_Min.TwistStrength = 4;
		Sc_Max.TwistStrength = 16;
		Sc_Min.BulgeAcceptanceExponent = 1.5;
		Sc_Max.BulgeAcceptanceExponent = 2.5;
		Sc_Min.BulgeAxisScale = FVector(.9, .9, .35);
		Sc_Max.BulgeAxisScale = FVector(1, 1, .6);
		Sc_Min.BulgeBaseDensity = 2;
		Sc_Max.BulgeBaseDensity = 4;
		Sc_Min.BulgeDepthBias = .3;
		Sc_Max.BulgeDepthBias = .6;
		Sc_Min.BulgeNumPoints = 200000;
		Sc_Max.BulgeNumPoints = 300000;
		Sc_Min.BulgeRadiusScale = .2;
		Sc_Max.BulgeRadiusScale = .4;
		Sc_Min.BulgeRatio = .8;
		Sc_Max.BulgeRatio = 1.2;
		Sc_Min.BulgeTruncationScale = .7;
		Sc_Max.BulgeTruncationScale = 1.3;
		Sc_Min.BulgeJitter = .2;
		Sc_Max.BulgeJitter = .4;
		Sc_Min.BackgroundNumPoints = 50000;
		Sc_Max.BackgroundNumPoints = 150000;
		Sc_Min.BackgroundBaseDensity = 15;
		Sc_Max.BackgroundBaseDensity = 25;
		Sc_Min.BackgroundDepthBias = .7;
		Sc_Max.BackgroundDepthBias = 1.3;
		Sc_Min.BackgroundHeightRatio = 1;
		Sc_Max.BackgroundHeightRatio = 1;
		Sc_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		Sc_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		Sc_Min.ClusterBaseDensity = 2;
		Sc_Max.ClusterBaseDensity = 4;
		Sc_Min.ClusterDepthBias = .1;
		Sc_Max.ClusterDepthBias = .3;
		Sc_Min.ClusterIncoherence = 3;
		Sc_Max.ClusterIncoherence = 6;
		Sc_Min.ClusterMaxScale = .8;
		Sc_Max.ClusterMaxScale = .6;
		Sc_Min.ClusterMinScale = .2;
		Sc_Max.ClusterMinScale = .4;
		Sc_Min.ClusterNumClusters = 12;
		Sc_Max.ClusterNumClusters = 64;
		Sc_Min.ClusterNumPoints = 100;
		Sc_Max.ClusterNumPoints = 20000;
		Sc_Min.ClusterSpreadFactor = .25;
		Sc_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region SBa Archtype
		SBa.BulgeRatio = .35;
		SBa.BulgeAxisScale = FVector(1, .3, .3);
		SBa.TwistStrength = 4;
		SBa.TwistCoreStrength = 0;
		SBa.ArmNumArms = 2;
		SBa.ArmClusterRadiusMin = .05;
		SBa.ArmClusterRadiusMax = .4;
		SBa.ArmIncoherence = 2;
		SBa.ArmHeightRatio = .5;
		SBa.ArmStartRatio = 0.0;
		#pragma endregion
		#pragma region SBa Bounds
		SBa_Min = SBa;
		SBa_Max = SBa;
		SBa_Min.GalaxyRatio = .2;
		SBa_Max.GalaxyRatio = .5;
		SBa_Min.DiscBaseDensity = 8;
		SBa_Max.DiscBaseDensity = 16;
		SBa_Min.DiscDepthBias = .8;
		SBa_Max.DiscDepthBias = 1.2;
		SBa_Min.DiscHeightRatio = .03;
		SBa_Max.DiscHeightRatio = .1;
		SBa_Min.DiscNumPoints = 50000;
		SBa_Max.DiscNumPoints = 150000;
		SBa_Min.ArmBaseDensity = 1;
		SBa_Max.ArmBaseDensity = 4;
		SBa_Min.ArmClusterRadiusMax = .15;
		SBa_Max.ArmClusterRadiusMax = .45;
		SBa_Min.ArmClusterRadiusMin = .025;
		SBa_Max.ArmClusterRadiusMin = .1;
		SBa_Min.ArmClusters = 64;
		SBa_Max.ArmClusters = 256;
		SBa_Min.ArmDepthBias = .2;
		SBa_Max.ArmDepthBias = .6;
		SBa_Min.ArmHeightRatio = .15;
		SBa_Max.ArmHeightRatio = .5;
		SBa_Min.ArmIncoherence = 3;
		SBa_Max.ArmIncoherence = 8;
		SBa_Min.ArmNumArms = 2;
		SBa_Max.ArmNumArms = 2;
		SBa_Min.ArmNumPoints = 100000;
		SBa_Max.ArmNumPoints = 200000;
		SBa_Min.ArmRadialBaseDensity = .1;
		SBa_Max.ArmRadialBaseDensity = 1;
		SBa_Min.ArmRadialDensityExponent = 1;
		SBa_Max.ArmRadialDensityExponent = 3;
		SBa_Min.ArmRadialDensityMultiplier = 2;
		SBa_Max.ArmRadialDensityMultiplier = 12;
		SBa_Min.ArmSpreadFactor = .25;
		SBa_Max.ArmSpreadFactor = .5;
		SBa_Min.ArmStartRatio = 0;
		SBa_Max.ArmStartRatio = 0.2;
		SBa_Min.TwistCoreRadius = .005;
		SBa_Max.TwistCoreRadius = 0;
		SBa_Min.TwistCoreStrength = 0;
		SBa_Max.TwistCoreStrength = 8;
		SBa_Min.TwistCoreTwistExponent = .9;
		SBa_Max.TwistCoreTwistExponent = 1.1;
		SBa_Min.TwistStrength = 3;
		SBa_Max.TwistStrength = 8;
		SBa_Min.BulgeAcceptanceExponent = 1.5;
		SBa_Max.BulgeAcceptanceExponent = 2.5;
		SBa_Min.BulgeAxisScale = FVector(.9, .9, .35);
		SBa_Max.BulgeAxisScale = FVector(1, 1, .6);
		SBa_Min.BulgeBaseDensity = 2;
		SBa_Max.BulgeBaseDensity = 4;
		SBa_Min.BulgeDepthBias = .6;
		SBa_Max.BulgeDepthBias = .8;
		SBa_Min.BulgeNumPoints = 100000;
		SBa_Max.BulgeNumPoints = 200000;
		SBa_Min.BulgeRadiusScale = .2;
		SBa_Max.BulgeRadiusScale = .4;
		SBa_Min.BulgeRatio = .8;
		SBa_Max.BulgeRatio = 1.2;
		SBa_Min.BulgeTruncationScale = .7;
		SBa_Max.BulgeTruncationScale = 1.3;
		SBa_Min.BulgeJitter = .2;
		SBa_Max.BulgeJitter = .4;
		SBa_Min.BackgroundNumPoints = 50000;
		SBa_Max.BackgroundNumPoints = 150000;
		SBa_Min.BackgroundBaseDensity = 15;
		SBa_Max.BackgroundBaseDensity = 25;
		SBa_Min.BackgroundDepthBias = .7;
		SBa_Max.BackgroundDepthBias = 1.3;
		SBa_Min.BackgroundHeightRatio = 1;
		SBa_Max.BackgroundHeightRatio = 1;
		SBa_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		SBa_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		SBa_Min.ClusterBaseDensity = 2;
		SBa_Max.ClusterBaseDensity = 4;
		SBa_Min.ClusterDepthBias = .1;
		SBa_Max.ClusterDepthBias = .3;
		SBa_Min.ClusterIncoherence = 2;
		SBa_Max.ClusterIncoherence = 6;
		SBa_Min.ClusterMaxScale = .8;
		SBa_Max.ClusterMaxScale = .6;
		SBa_Min.ClusterMinScale = .2;
		SBa_Max.ClusterMinScale = .4;
		SBa_Min.ClusterNumClusters = 12;
		SBa_Max.ClusterNumClusters = 64;
		SBa_Min.ClusterNumPoints = 100;
		SBa_Max.ClusterNumPoints = 20000;
		SBa_Min.ClusterSpreadFactor = .25;
		SBa_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region SBb Archtype
		SBb.BulgeAxisScale = FVector(1, .3, .3);
		SBb.TwistStrength = 6;
		SBb.TwistCoreStrength = 0;
		SBb.ArmNumArms = 2;
		SBb.ArmClusterRadiusMin = .05;
		SBb.ArmClusterRadiusMax = .25;
		SBb.ArmIncoherence = 8;
		SBb.ArmHeightRatio = .5;
		SBb.ArmStartRatio = 0;
		#pragma endregion
		#pragma region SBb Bounds
		SBb_Min = SBb;
		SBb_Max = SBb;
		SBb_Min.GalaxyRatio = .2;
		SBb_Max.GalaxyRatio = .5;
		SBb_Min.DiscBaseDensity = 8;
		SBb_Max.DiscBaseDensity = 16;
		SBb_Min.DiscDepthBias = .8;
		SBb_Max.DiscDepthBias = 1.2;
		SBb_Min.DiscHeightRatio = .03;
		SBb_Max.DiscHeightRatio = .1;
		SBb_Min.DiscNumPoints = 50000;
		SBb_Max.DiscNumPoints = 150000;
		SBb_Min.ArmBaseDensity = 1;
		SBb_Max.ArmBaseDensity = 4;
		SBb_Min.ArmClusterRadiusMax = .15;
		SBb_Max.ArmClusterRadiusMax = .45;
		SBb_Min.ArmClusterRadiusMin = .025;
		SBb_Max.ArmClusterRadiusMin = .1;
		SBb_Min.ArmClusters = 64;
		SBb_Max.ArmClusters = 256;
		SBb_Min.ArmDepthBias = .2;
		SBb_Max.ArmDepthBias = .6;
		SBb_Min.ArmHeightRatio = .15;
		SBb_Max.ArmHeightRatio = .5;
		SBb_Min.ArmIncoherence = 5;
		SBb_Max.ArmIncoherence = 10;
		SBb_Min.ArmNumArms = 2;
		SBb_Max.ArmNumArms = 2;
		SBb_Min.ArmNumPoints = 100000;
		SBb_Max.ArmNumPoints = 200000;
		SBb_Min.ArmRadialBaseDensity = .1;
		SBb_Max.ArmRadialBaseDensity = 1;
		SBb_Min.ArmRadialDensityExponent = 1;
		SBb_Max.ArmRadialDensityExponent = 3;
		SBb_Min.ArmRadialDensityMultiplier = 2;
		SBb_Max.ArmRadialDensityMultiplier = 12;
		SBb_Min.ArmSpreadFactor = .25;
		SBb_Max.ArmSpreadFactor = .5;
		SBb_Min.ArmStartRatio = 0;
		SBb_Max.ArmStartRatio = .1;
		SBb_Min.TwistCoreRadius = 01;
		SBb_Max.TwistCoreRadius = .035;
		SBb_Min.TwistCoreStrength = 0;
		SBb_Max.TwistCoreStrength = 0;
		SBb_Min.TwistCoreTwistExponent = .9;
		SBb_Max.TwistCoreTwistExponent = 1.1;
		SBb_Min.TwistStrength = 8;
		SBb_Max.TwistStrength = 16;
		SBb_Min.BulgeAcceptanceExponent = 1.5;
		SBb_Max.BulgeAcceptanceExponent = 2.5;
		SBb_Min.BulgeAxisScale = FVector(.9, .9, .35);
		SBb_Max.BulgeAxisScale = FVector(1, 1, .6);
		SBb_Min.BulgeBaseDensity = 2;
		SBb_Max.BulgeBaseDensity = 4;
		SBb_Min.BulgeDepthBias = .6;
		SBb_Max.BulgeDepthBias = .8;
		SBb_Min.BulgeNumPoints = 100000;
		SBb_Max.BulgeNumPoints = 200000;
		SBb_Min.BulgeRadiusScale = .2;
		SBb_Max.BulgeRadiusScale = .4;
		SBb_Min.BulgeRatio = .8;
		SBb_Max.BulgeRatio = 1.2;
		SBb_Min.BulgeTruncationScale = .7;
		SBb_Max.BulgeTruncationScale = 1.3;
		SBb_Min.BulgeJitter = .2;
		SBb_Max.BulgeJitter = .4;
		SBb_Min.BackgroundNumPoints = 50000;
		SBb_Max.BackgroundNumPoints = 150000;
		SBb_Min.BackgroundBaseDensity = 15;
		SBb_Max.BackgroundBaseDensity = 25;
		SBb_Min.BackgroundDepthBias = .7;
		SBb_Max.BackgroundDepthBias = 1.3;
		SBb_Min.BackgroundHeightRatio = 1;
		SBb_Max.BackgroundHeightRatio = 1;
		SBb_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		SBb_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		SBb_Min.ClusterBaseDensity = 2;
		SBb_Max.ClusterBaseDensity = 4;
		SBb_Min.ClusterDepthBias = .1;
		SBb_Max.ClusterDepthBias = .3;
		SBb_Min.ClusterIncoherence = 2;
		SBb_Max.ClusterIncoherence = 6;
		SBb_Min.ClusterMaxScale = .8;
		SBb_Max.ClusterMaxScale = .6;
		SBb_Min.ClusterMinScale = .2;
		SBb_Max.ClusterMinScale = .4;
		SBb_Min.ClusterNumClusters = 12;
		SBb_Max.ClusterNumClusters = 64;
		SBb_Min.ClusterNumPoints = 100;
		SBb_Max.ClusterNumPoints = 20000;
		SBb_Min.ClusterSpreadFactor = .25;
		SBb_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region SBc Archtype 
		SBc.BulgeAxisScale = FVector(1, 1, 1);
		SBc.TwistStrength = 12;
		SBc.TwistCoreStrength = 0;
		SBc.ArmNumArms = 2;
		SBc.ArmClusterRadiusMin = .05;
		SBc.ArmClusterRadiusMax = .3;
		SBc.ArmIncoherence = 8;
		SBc.ArmHeightRatio = .5;
		SBc.ArmStartRatio = 0;
		#pragma endregion
		#pragma region SBc Bounds
		SBc_Min = SBc;
		SBc_Max = SBc;
		SBc_Min.GalaxyRatio = .2;
		SBc_Max.GalaxyRatio = .5;
		SBc_Min.DiscBaseDensity = 8;
		SBc_Max.DiscBaseDensity = 16;
		SBc_Min.DiscDepthBias = .8;
		SBc_Max.DiscDepthBias = 1.2;
		SBc_Min.DiscHeightRatio = .03;
		SBc_Max.DiscHeightRatio = .1;
		SBc_Min.DiscNumPoints = 50000;
		SBc_Max.DiscNumPoints = 100000;
		SBc_Min.ArmBaseDensity = 1;
		SBc_Max.ArmBaseDensity = 4;
		SBc_Min.ArmClusterRadiusMax = .15;
		SBc_Max.ArmClusterRadiusMax = .45;
		SBc_Min.ArmClusterRadiusMin = .025;
		SBc_Max.ArmClusterRadiusMin = .1;
		SBc_Min.ArmClusters = 64;
		SBc_Max.ArmClusters = 256;
		SBc_Min.ArmDepthBias = .2;
		SBc_Max.ArmDepthBias = .6;
		SBc_Min.ArmHeightRatio = .15;
		SBc_Max.ArmHeightRatio = .5;
		SBc_Min.ArmIncoherence = 6;
		SBc_Max.ArmIncoherence = 12;
		SBc_Min.ArmNumArms = 2;
		SBc_Max.ArmNumArms = 2;
		SBc_Min.ArmNumPoints = 150000;
		SBc_Max.ArmNumPoints = 250000;
		SBc_Min.ArmRadialBaseDensity = .1;
		SBc_Max.ArmRadialBaseDensity = 1;
		SBc_Min.ArmRadialDensityExponent = 1;
		SBc_Max.ArmRadialDensityExponent = 3;
		SBc_Min.ArmRadialDensityMultiplier = 2;
		SBc_Max.ArmRadialDensityMultiplier = 12;
		SBc_Min.ArmSpreadFactor = .25;
		SBc_Max.ArmSpreadFactor = .5;
		SBc_Min.ArmStartRatio = 0;
		SBc_Max.ArmStartRatio = .1;
		SBc_Min.TwistCoreRadius = 01;
		SBc_Max.TwistCoreRadius = .035;
		SBc_Min.TwistCoreStrength = 0;
		SBc_Max.TwistCoreStrength = 0;
		SBc_Min.TwistCoreTwistExponent = .9;
		SBc_Max.TwistCoreTwistExponent = 1.1;
		SBc_Min.TwistStrength = 12;
		SBc_Max.TwistStrength = 18;
		SBc_Min.BulgeAcceptanceExponent = 1.5;
		SBc_Max.BulgeAcceptanceExponent = 2.5;
		SBc_Min.BulgeAxisScale = FVector(.9, .9, .35);
		SBc_Max.BulgeAxisScale = FVector(1, 1, .6);
		SBc_Min.BulgeBaseDensity = 2;
		SBc_Max.BulgeBaseDensity = 4;
		SBc_Min.BulgeDepthBias = .6;
		SBc_Max.BulgeDepthBias = .8;
		SBc_Min.BulgeNumPoints = 100000;
		SBc_Max.BulgeNumPoints = 150000;
		SBc_Min.BulgeRadiusScale = .2;
		SBc_Max.BulgeRadiusScale = .4;
		SBc_Min.BulgeRatio = .8;
		SBc_Max.BulgeRatio = 1.2;
		SBc_Min.BulgeTruncationScale = .7;
		SBc_Max.BulgeTruncationScale = 1.3;
		SBc_Min.BulgeJitter = .2;
		SBc_Max.BulgeJitter = .4;
		SBc_Min.BackgroundNumPoints = 50000;
		SBc_Max.BackgroundNumPoints = 150000;
		SBc_Min.BackgroundBaseDensity = 15;
		SBc_Max.BackgroundBaseDensity = 25;
		SBc_Min.BackgroundDepthBias = .7;
		SBc_Max.BackgroundDepthBias = 1.3;
		SBc_Min.BackgroundHeightRatio = 1;
		SBc_Max.BackgroundHeightRatio = 1;
		SBc_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		SBc_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		SBc_Min.ClusterBaseDensity = 2;
		SBc_Max.ClusterBaseDensity = 4;
		SBc_Min.ClusterDepthBias = .1;
		SBc_Max.ClusterDepthBias = .3;
		SBc_Min.ClusterIncoherence = 2;
		SBc_Max.ClusterIncoherence = 6;
		SBc_Min.ClusterMaxScale = .8;
		SBc_Max.ClusterMaxScale = .6;
		SBc_Min.ClusterMinScale = .2;
		SBc_Max.ClusterMinScale = .4;
		SBc_Min.ClusterNumClusters = 12;
		SBc_Max.ClusterNumClusters = 64;
		SBc_Min.ClusterNumPoints = 100;
		SBc_Max.ClusterNumPoints = 20000;
		SBc_Min.ClusterSpreadFactor = .25;
		SBc_Max.ClusterSpreadFactor = .45;
		#pragma endregion

		#pragma region Irr Archtype
		Irr.BulgeNumPoints = 0;
		Irr.ArmNumPoints = 0;
		Irr.DiscNumPoints = 0;
		Irr.BackgroundNumPoints = 100000;
		Irr.BackgroundBaseDensity = 0.5;
		Irr.ClusterNumPoints = 300000;
		Irr.ClusterIncoherence = 4;
		Irr.ClusterNumClusters = 128;
		Irr.ClusterDepthBias = .75;
		#pragma endregion
		#pragma region Irr Bounds
		Irr_Min = Irr;
		Irr_Max = Irr;
		Irr_Min.GalaxyRatio = .2;
		Irr_Max.GalaxyRatio = .5;
		Irr_Min.BulgeAcceptanceExponent = 1.5;
		Irr_Max.BulgeAcceptanceExponent = 2.5;
		Irr_Min.BulgeAxisScale = FVector(.85, .85, .85);
		Irr_Max.BulgeAxisScale = FVector(1, 1, 1);
		Irr_Min.BulgeBaseDensity = 1;
		Irr_Max.BulgeBaseDensity = 2;
		Irr_Min.BulgeDepthBias = .6;
		Irr_Max.BulgeDepthBias = .8;
		Irr_Min.BulgeNumPoints = 1000;
		Irr_Max.BulgeNumPoints = 5000;
		Irr_Min.BulgeRadiusScale = .2;
		Irr_Max.BulgeRadiusScale = .4;
		Irr_Min.BulgeRatio = .8;
		Irr_Max.BulgeRatio = 1.2;
		Irr_Min.BulgeTruncationScale = .7;
		Irr_Max.BulgeTruncationScale = 1.3;
		Irr_Min.BulgeJitter = .2;
		Irr_Max.BulgeJitter = .4;
		Irr_Min.BackgroundNumPoints = 150000;
		Irr_Max.BackgroundNumPoints = 250000;
		Irr_Min.BackgroundBaseDensity = 5;
		Irr_Max.BackgroundBaseDensity = 10;
		Irr_Min.BackgroundDepthBias = .7;
		Irr_Max.BackgroundDepthBias = 1.3;
		Irr_Min.BackgroundHeightRatio = 1;
		Irr_Max.BackgroundHeightRatio = 1;
		Irr_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
		Irr_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
		Irr_Min.ClusterBaseDensity = 3;
		Irr_Max.ClusterBaseDensity = 6;
		Irr_Min.ClusterDepthBias = .1;
		Irr_Max.ClusterDepthBias = .3;
		Irr_Min.ClusterIncoherence = 2;
		Irr_Max.ClusterIncoherence = 4;
		Irr_Min.ClusterMaxScale = .8;
		Irr_Max.ClusterMaxScale = .6;
		Irr_Min.ClusterMinScale = .2;
		Irr_Max.ClusterMinScale = .4;
		Irr_Min.ClusterNumClusters = 32;
		Irr_Max.ClusterNumClusters = 128;
		Irr_Min.ClusterNumPoints = 200000;
		Irr_Max.ClusterNumPoints = 400000;
		Irr_Min.ClusterSpreadFactor = .3;
		Irr_Max.ClusterSpreadFactor = .6;
		#pragma endregion
	};

	GalaxyParams GenerateParams();
	GalaxyParams BoundedRandomizeParams(GalaxyParams MinParams, GalaxyParams MaxParams);
	int SelectGalaxyTypeIndex();
};

/// <summary>
///GALAXY GENERATOR - GENERATES DATA FOR POPULATING A GALAXY
/// </summary>
class SVO_API GalaxyGenerator : public PointCloudGenerator {
public:
	GalaxyGenerator() : PointCloudGenerator(69) {};
	GalaxyGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	bool IsDestroying = false;
	double MaxRadius;
	double GalaxyRadius;
	double BulgeRadius;
	double VoidRadius;

	GalaxyParams GalaxyParams;

	TArray<FPointData> GeneratedData;
	
	// depth probabilities for depths 0..6 (sum = 1.0)
	static constexpr double DepthProb[7] = {
		0.7246688105348869,
		0.18633259810092362,
		0.060410870884541834,
		0.019585801723497227,
		0.006349910596145782,
		0.0020587038073948565,
		0.0005933043526099457
	};

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	void GenerateArms();
	void ApplyTwist();
	void GenerateBulge();
	void GenerateClusters();
	void GenerateDisc();
	void GenerateBackground();
	void GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity = 1, double InDepthBias = 1);
	int ChooseDepth(double InRandomSample, double InDepthBias);
	void MarkDestroying();
};

//TODO: BUILD UNIVERSE GENERATOR
struct SVO_API UniverseParams {
	int Count = 2000000;
	double Extent = 2147483648;
	const char* EncodedTree = "HgAdABcAAAAAAAAAgD8AAIA/AAAAABkAEAAAAAA/FwAAAAAAzczMPQAAAAAAAIA/HwAXAJqZmT4AAIA/AAAAAAAAgD8gAA0ABgAAAAAAAEALAAEAAAAAAAAAAQAAAAAAAAAAAACAPwAAAAA/AAAAAAAAmpmZPgAAAAAAAM3MzD0AAAAAPwAAAIA/ARsAJAACAAAA//8BAAAAAIA/AAAAgD8AAAAAAA==";
};
//class SVO_API UniverseParamFactory {};
class SVO_API UniverseGenerator : public PointCloudGenerator {
public:
	UniverseGenerator() : PointCloudGenerator(8647) {};
	UniverseGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	UniverseParams UniverseParams;
	// depth probabilities for depths 0..6 (sum = 1.0)
	static constexpr double DepthProb[7] = {0.25,0.35,0.20,0.10,0.05,0.04,0.01};

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	int ChooseDepth(double InRandomSample, double InDepthBias);

	TArray<FPointData> GeneratedData;
};
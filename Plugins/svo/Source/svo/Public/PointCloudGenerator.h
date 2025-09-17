// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <DataTypes.h>
#include <FOctreeNode.h>
#include "FastNoise/FastNoise.h"
#include "CoreMinimal.h"

/**
 * 
 */
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

	GalaxyParamFactory();

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
	const char* EncodedTree = "EwAAAEBAJAACAAAAEAAAAIA/FwAAAAAAAACAPwAAgD8AAAAAJAAEAAAADwABAAAAAAAAQAcAAAAAAD8AAAAAAAAAAIA/";
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
	void GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity = 1, double InDepthBias = 1);

	int ChooseDepth(double InRandomSample, double InDepthBias);

	TArray<FPointData> GeneratedData;
};
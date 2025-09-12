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
	FInt64Vector ApplyNoise(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Vector InSamplePosition, float& OutDensity);
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

struct SVO_API GalaxyParams {
	//Base Params - control overall sizes
	double GalaxyRatio = .33;
	double BulgeRatio = .75;
	double VoidRatio = .025;

	//Twist Params - Alters the behavior of the twist pass
	double TwistStrength = 4;
	double TwistCoreRadius = .01;
	double TwistCoreTwistExponent = 1;
	double TwistCoreStrength = 4;

	//Arm Params - changes arm appearance
	int ArmNumPoints = 300000; //Total arm points to generate
	int ArmNumArms = 4; //Number of arms
	int ArmClusters = 124; //Number of clusters per arm
	double ArmDepthBias = .5; //Smaller number = more large stars, larger number = more small stars, at 1 it will use a basic stellar size distribution table
	double ArmBaseDensity = 4; //Base density factor
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
	int BulgeNumPoints = 300000;
	double BulgeBaseDensity = 2;
	double BulgeDepthBias = .75;
	double BulgeRadiusScale = .33;
	double BulgeTruncationScale = 1;
	double BulgeAcceptanceExponent = 2;
	FVector BulgeAxisScale = FVector(1, 1, .6);

	//Cluster Params
	int ClusterNumPoints = 0;
	int ClusterNumClusters = 0;
	FVector ClusterAxisScale = FVector(1, 1, 1);
	double ClusterSpreadFactor = .25;
	double ClusterMinScale = .1;
	double ClusterMaxScale = .3;
	double ClusterIncoherence = 6;

	//Disc Params - Controls the non spiral disc
	int DiscNumPoints = 200000;
	double DiscBaseDensity = 20;
	double DiscDepthBias = 1;
	double DiscHeightRatio = .1;

	//Background Params - Controls the background halo 
	int BackgroundNumPoints = 200000;
	double BackgroundBaseDensity = 10;
	double BackgroundDepthBias = 1;
	double BackgroundHeightRatio = .8;
};

class SVO_API GalaxyGenerator : public PointCloudGenerator {
public:
	GalaxyGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

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

	//TODO: need optimization pass, should be using mostly parallel fors on background priority thread
	//No rejection repeats, just feed rejected to black hole

	void GenerateArms();
	void ApplyTwist();
	void GenerateBulge();
	void GenerateClusters();
	void GenerateDisc();
	void GenerateBackground();

	void GenerateCluster(FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity = 1, double InDepthBias = 1);
	int ChooseDepth(double InRandomSample, double InDepthBias);
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
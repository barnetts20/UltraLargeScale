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
	FInt64Coordinate ApplyNoise(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Coordinate InSamplePosition, float& OutDensity);
	FInt64Coordinate RotateCoordinate(FInt64Coordinate InCoordinate, FRotator InAxes);

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
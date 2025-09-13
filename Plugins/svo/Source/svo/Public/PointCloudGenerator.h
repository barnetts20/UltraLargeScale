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


/// <summary>
///	GALAXY GENERATION
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
	FLinearColor VolumeAmbientColor;		//DynamicMaterial->SetVectorParameterValue(FName("AmbientColor"), ParentColor);
	FLinearColor VolumeCoolShift;			//DynamicMaterial->SetVectorParameterValue(FName("CoolShift"), FLinearColor(Stream.FRandRange(0,6), Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), 1));
	FLinearColor VolumeHotShift;			//DynamicMaterial->SetVectorParameterValue(FName("HotShift"), FLinearColor(Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), Stream.FRandRange(0, 6), 1));
	double VolumeHueVariance;				//DynamicMaterial->SetScalarParameterValue(FName("HueVariance"), Stream.FRandRange(0,.5));
	double VolumeHueVarianceScale;			//DynamicMaterial->SetScalarParameterValue(FName("HueVarianceScale"), Stream.FRandRange(.5, 3));
	double VolumeSaturationVariance;		//DynamicMaterial->SetScalarParameterValue(FName("SaturationVariance"), Stream.FRandRange(0, .5));
	double VolumeTemperatureInfluence;		//DynamicMaterial->SetScalarParameterValue(FName("TemperatureInfluence"), Stream.FRandRange(2, 8));
	double VolumeTemepratureScale;			//DynamicMaterial->SetScalarParameterValue(FName("TemperatureScale"), Stream.FRandRange(1, 6));
	double VolumeDensity;					//DynamicMaterial->SetScalarParameterValue(FName("Density"), Stream.FRandRange(0.1, .5));
	double VolumeWarpAmount;				//DynamicMaterial->SetScalarParameterValue(FName("WarpAmount"), Stream.FRandRange(0.02, .15));
	double VolumeWarpScale;					//DynamicMaterial->SetScalarParameterValue(FName("WarpScale"), Stream.FRandRange(0.5, 2));
	UVolumeTexture* VolumeNoise;
};

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

	GalaxyParams E0_Min;
	GalaxyParams E3_Min;
	GalaxyParams E5_Min;
	GalaxyParams E7_Min;
	GalaxyParams S0_Min;
	GalaxyParams Sa_Min;
	GalaxyParams Sb_Min;
	GalaxyParams Sc_Min;
	GalaxyParams SBa_Min;
	GalaxyParams SBb_Min;
	GalaxyParams SBc_Min;
	GalaxyParams Irr_Min;

	GalaxyParams E0_Max;
	GalaxyParams E3_Max;
	GalaxyParams E5_Max;
	GalaxyParams E7_Max;
	GalaxyParams S0_Max;
	GalaxyParams Sa_Max;
	GalaxyParams Sb_Max;
	GalaxyParams Sc_Max;
	GalaxyParams SBa_Max;
	GalaxyParams SBb_Max;
	GalaxyParams SBc_Max;
	GalaxyParams Irr_Max;

	TArray<TPair<float, GalaxyParams*>> GalaxyWeights = {{0.02f, &E0}, {0.04f, &E3}, {0.04f, &E5}, {0.03f, &E7}, {0.22f, &S0}, {0.15f, &Sa}, {0.15f, &Sb}, {0.20f, &Sc}, {0.04f, &SBa}, {0.04f, &SBb}, {0.03f, &SBc}, {0.04f, &Irr}};

	GalaxyParamFactory() {
		E0.ArmNumPoints = 0;
		E0.DiscNumPoints = 0;
		E0.BulgeNumPoints = 300000;
		E0.BulgeBaseDensity = 3;
		E0.BulgeDepthBias = .2;
		E0.BackgroundBaseDensity = 20;
		E0.BulgeAxisScale = FVector(1);
		E0.GalaxyRatio = .4;
		E0.BulgeRatio = 1.5;
		E0.BackgroundNumPoints = 100000;
		E0_Min = E0;
		E0_Max = E0;
		//TODO: Bounds

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
		E3_Min = E3;
		E3_Max = E3;
		//TODO: Bounds

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
		E5_Min = E5;
		E5_Max = E5;
		//TODO: Bounds

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
		E7_Min = E7;
		E7_Max = E7;
		//TODO: Bounds

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
		S0.BulgeRatio = 1.25;
		S0.BackgroundNumPoints = 100000;
		S0_Min = S0;
		S0_Max = S0;
		//TODO: Bounds

		Sa.TwistStrength = 4;
		Sa.TwistCoreStrength = 4;
		Sa.ArmNumArms = 2;
		Sa.ArmClusterRadiusMin = .15;
		Sa.ArmClusterRadiusMax = .4;
		Sa.ArmIncoherence = 3;
		Sa.ArmStartRatio = 0;
		Sa_Min = Sa;
		Sa_Max = Sa;
		//TODO: Bounds

		Sb.TwistStrength = 12;
		Sb.ArmNumArms = 2;
		Sb.ArmIncoherence = 6;
		Sb.ArmStartRatio = 0;
		Sb_Min = Sb;
		Sb_Max = Sb;
		//TODO: Bounds

		Sc.TwistStrength = 8;
		Sc.ArmNumArms = 4;
		Sc.ArmIncoherence = 8;
		Sc.ArmStartRatio = 0;
		Sc.TwistCoreRadius = .02;
		Sc_Min = Sc;
		Sc_Max = Sc;
		//TODO: Bounds

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
		SBa_Min = SBa;
		SBa_Max = SBa;
		//TODO: Bounds

		SBb.BulgeAxisScale = FVector(1, .3, .3);
		SBb.TwistStrength = 6;
		SBb.TwistCoreStrength = 0;
		SBb.ArmNumArms = 2;
		SBb.ArmClusterRadiusMin = .05;
		SBb.ArmClusterRadiusMax = .25;
		SBb.ArmIncoherence = 8;
		SBb.ArmHeightRatio = .5;
		SBb.ArmStartRatio = 0;
		SBb_Min = SBb;
		SBb_Max = SBb;
		//TODO: Bounds

		SBc.BulgeAxisScale = FVector(1, 1, 1);
		SBc.TwistStrength = 12;
		SBc.TwistCoreStrength = 0;
		SBc.ArmNumArms = 2;
		SBc.ArmClusterRadiusMin = .05;
		SBc.ArmClusterRadiusMax = .3;
		SBc.ArmIncoherence = 8;
		SBc.ArmHeightRatio = .5;
		SBc.ArmStartRatio = 0;
		SBc_Min = SBc;
		SBc_Max = SBc;
		//TODO: Bounds

		Irr.BulgeNumPoints = 0;
		Irr.ArmNumPoints = 0;
		Irr.DiscNumPoints = 0;
		Irr.BackgroundNumPoints = 100000;
		Irr.BackgroundBaseDensity = 0.5;
		Irr.ClusterNumPoints = 300000;
		Irr.ClusterIncoherence = 4;
		Irr.ClusterNumClusters = 128;
		Irr.ClusterDepthBias = .75;
		Irr_Min = Irr;
		Irr_Max = Irr;
		//TODO: Bounds
	};

	GalaxyParams GenerateParams();
	GalaxyParams BoundedRandomizeParams(GalaxyParams MinParams, GalaxyParams MaxParams);
	int SelectGalaxyTypeIndex();
};

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
	void MarkDestroying();
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
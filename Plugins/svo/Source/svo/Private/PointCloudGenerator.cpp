#include "PointCloudGenerator.h"

#pragma region PointCloudGenerator
FInt64Vector PointCloudGenerator::ApplyNoiseDerivative(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Vector InSamplePosition, float& OutDensity)
{
	double ScaleFactor = InDomainScale / static_cast<double>(InExtent);
	//Noise scaled sample position
	double NX = static_cast<double>(InSamplePosition.X) * ScaleFactor;
	double NY = static_cast<double>(InSamplePosition.Y) * ScaleFactor;
	double NZ = static_cast<double>(InSamplePosition.Z) * ScaleFactor;

	float Output[27] = { 0 }; //Size 27 to house 3x3x3 
	//Sample noise 3x3x3 grid around the noise scaled sample position
	InNoise->GenPositionArray3D(
		Output,							//Sample output array 
		27,						//3x3x3 grid again
		OffsetPositionsX,		//Hard coded dx offsets
		OffsetPositionsY,		//Hard coded dy offsets
		OffsetPositionsZ,		//Hard coded dz offsets
		static_cast<float>(NX),	//Noise scaled sample position X
		static_cast<float>(NY),	//Noise scaled sample position Y
		static_cast<float>(NZ),	//Noise scaled sample position Z
		Seed							//Seed
	);

	// Compute gradients via central differences over the 3x3x3 Output grid
	double Dx = (Output[2] + Output[5] + Output[8] - Output[0] - Output[3] - Output[6]) / 6.0;
	double Dy = (Output[18] + Output[19] + Output[20] - Output[6] - Output[7] - Output[8]) / 6.0;
	double Dz = (Output[24] + Output[25] + Output[26] - Output[0] - Output[1] - Output[2]) / 6.0;

	//Warp noise scaled position with ddxddyddz
	double GX = NX + Dx * WarpAmount.X;
	double GY = NY + Dy * WarpAmount.Y;
	double GZ = NZ + Dz * WarpAmount.Z;

	//Convert back to octree scale
	int64 FX = FMath::RoundToInt64(GX / ScaleFactor);
	int64 FY = FMath::RoundToInt64(GY / ScaleFactor);
	int64 FZ = FMath::RoundToInt64(GZ / ScaleFactor);

	//Clamp to bounds
	FX = FMath::Clamp(FX, -InExtent, InExtent);
	FY = FMath::Clamp(FY, -InExtent, InExtent);
	FZ = FMath::Clamp(FZ, -InExtent, InExtent);

	//Back to int64 octree coordinates
	OutDensity = (Output[13] + 1)/2;
	auto InsertPosition = FInt64Vector(FX, FY, FZ);
	return InsertPosition;
}

bool PointCloudGenerator::ApplyNoiseSelective(FastNoise::SmartNode<> InNoise, double InDensity, double InDomainScale, int64 InExtent, FVector InSamplePosition)
{
	double ScaleFactor = InDomainScale / static_cast<double>(InExtent);
	double NX = static_cast<double>(InSamplePosition.X) * ScaleFactor;
	double NY = static_cast<double>(InSamplePosition.Y) * ScaleFactor;
	double NZ = static_cast<double>(InSamplePosition.Z) * ScaleFactor;
	return (InDensity <= InNoise->GenSingle3D(NX, NY, NZ, Seed + 80085));
}

FInt64Vector PointCloudGenerator::RotateCoordinate(FInt64Vector InCoordinate, FRotator InRotation)
{
	// Convert to FVector for rotation
	FVector Position = FVector(
		static_cast<double>(InCoordinate.X),
		static_cast<double>(InCoordinate.Y),
		static_cast<double>(InCoordinate.Z)
	);

	// Apply rotation around the origin
	FVector Rotated = Rotation.RotateVector(Position);

	// Convert back to FInt64Coordinate (rounded)
	return FInt64Vector(
		FMath::RoundToInt64(Rotated.X),
		FMath::RoundToInt64(Rotated.Y),
		FMath::RoundToInt64(Rotated.Z)
	);
}

FVector PointCloudGenerator::RotateCoordinate(FVector InCoordinate, FRotator InRotation)
{
	// Apply rotation around the origin
	return Rotation.RotateVector(InCoordinate);
}
#pragma endregion

#pragma region GalaxyGenerator
#pragma region GalaxyParamFactory
GalaxyParamFactory::GalaxyParamFactory() {
#pragma region VolumeMaterialBounds
	Volume_Min.VolumeAmbientColor = FLinearColor(.1, .1, .1);
	Volume_Max.VolumeAmbientColor = FLinearColor(1.5, 1.5, 1.5);
	Volume_Min.VolumeCoolShift = FLinearColor(1, 1, 1);
	Volume_Max.VolumeCoolShift = FLinearColor(25, 25, 25);
	Volume_Min.VolumeDensity = .1;
	Volume_Max.VolumeDensity = .5;
	Volume_Min.VolumeHotShift = FLinearColor(.5, .25, .01);
	Volume_Max.VolumeHotShift = FLinearColor(1, .7, .3);
	Volume_Min.VolumeHueVariance = .01;
	Volume_Max.VolumeHueVariance = .25;
	Volume_Min.VolumeHueVarianceScale = .25;
	Volume_Max.VolumeHueVarianceScale = 1.75;
	Volume_Min.VolumeSaturationVariance = 0;
	Volume_Max.VolumeSaturationVariance = .5;
	Volume_Min.VolumeTemperatureScale = 3;
	Volume_Max.VolumeTemperatureScale = 10;
	Volume_Min.VolumeTemperatureInfluence = 8;
	Volume_Max.VolumeTemperatureInfluence = 48;
	Volume_Min.VolumeWarpAmount = .03;
	Volume_Max.VolumeWarpAmount = .1;
	Volume_Min.VolumeWarpScale = .1;
	Volume_Max.VolumeWarpScale = .5;
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
	E0_Min.BulgeDepthBias = .8;
	E0_Max.BulgeDepthBias = 1;
	E0_Min.BulgeNumPoints = 150000;
	E0_Max.BulgeNumPoints = 250000;
	E0_Min.BulgeRadiusScale = .2;
	E0_Max.BulgeRadiusScale = .4;
	E0_Min.BulgeRatio = .8;
	E0_Max.BulgeRatio = 1.2;
	E0_Min.BulgeTruncationScale = .7;
	E0_Max.BulgeTruncationScale = 1.3;
	E0_Min.BulgeJitter = .2;
	E0_Max.BulgeJitter = .4;
	E0_Min.BackgroundNumPoints = 100000;
	E0_Max.BackgroundNumPoints = 150000;
	E0_Min.BackgroundBaseDensity = 15;
	E0_Max.BackgroundBaseDensity = 25;
	E0_Min.BackgroundDepthBias = .9;
	E0_Max.BackgroundDepthBias = 1.2;
	E0_Min.BackgroundHeightRatio = 1;
	E0_Max.BackgroundHeightRatio = 1;
	E0_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	E0_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	E0_Min.ClusterBaseDensity = 2;
	E0_Max.ClusterBaseDensity = 4;
	E0_Min.ClusterDepthBias = .5;
	E0_Max.ClusterDepthBias = .8;
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
	E3.BulgeNumPoints = 150000;
	E3.BulgeBaseDensity = 3;
	E3.BulgeDepthBias = .5;
	E3.BackgroundBaseDensity = 50;
	E3.BulgeAxisScale = FVector(1, 1, .7);
	E3.GalaxyRatio = .3;
	E3.BulgeRatio = 1.25;
	E3.BackgroundNumPoints = 50000;
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
	E3_Min.BulgeDepthBias = .8;
	E3_Max.BulgeDepthBias = 1;
	E3_Min.BulgeNumPoints = 150000;
	E3_Max.BulgeNumPoints = 250000;
	E3_Min.BulgeRadiusScale = .2;
	E3_Max.BulgeRadiusScale = .4;
	E3_Min.BulgeRatio = .8;
	E3_Max.BulgeRatio = 1.2;
	E3_Min.BulgeTruncationScale = .7;
	E3_Max.BulgeTruncationScale = 1.3;
	E3_Min.BulgeJitter = .2;
	E3_Max.BulgeJitter = .4;
	E3_Min.BackgroundNumPoints = 100000;
	E3_Max.BackgroundNumPoints = 150000;
	E3_Min.BackgroundBaseDensity = 15;
	E3_Max.BackgroundBaseDensity = 25;
	E3_Min.BackgroundDepthBias = .9;
	E3_Max.BackgroundDepthBias = 1.2;
	E3_Min.BackgroundHeightRatio = 1;
	E3_Max.BackgroundHeightRatio = 1;
	E3_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	E3_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	E3_Min.ClusterBaseDensity = 2;
	E3_Max.ClusterBaseDensity = 4;
	E3_Min.ClusterDepthBias = .5;
	E3_Max.ClusterDepthBias = .8;
	E3_Min.ClusterIncoherence = 3;
	E3_Max.ClusterIncoherence = 6;
	E3_Min.ClusterMaxScale = .8;
	E3_Max.ClusterMaxScale = .6;
	E3_Min.ClusterMinScale = .2;
	E3_Max.ClusterMinScale = .4;
	E3_Min.ClusterNumClusters = 12;
	E3_Max.ClusterNumClusters = 64;
	E3_Min.ClusterNumPoints = 100;
	E3_Max.ClusterNumPoints = 10000;
	E3_Min.ClusterSpreadFactor = .25;
	E3_Max.ClusterSpreadFactor = .45;
#pragma endregion

#pragma region E5 Archtype
	E5.ArmNumPoints = 0;
	E5.DiscNumPoints = 50000;
	E5.DiscHeightRatio = .3;
	E5.DiscDepthBias = 1;
	E5.DiscBaseDensity = 1;
	E5.BulgeNumPoints = 100000;
	E5.BulgeBaseDensity = 2;
	E5.BulgeDepthBias = .4;
	E5.BackgroundBaseDensity = 10;
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
	E5_Min.DiscBaseDensity = 1;
	E5_Max.DiscBaseDensity = 4;
	E5_Min.DiscDepthBias = .8;
	E5_Max.DiscDepthBias = 1;
	E5_Min.DiscHeightRatio = .15;
	E5_Max.DiscHeightRatio = .4;
	E5_Min.DiscNumPoints = 50000;
	E5_Max.DiscNumPoints = 100000;
	E5_Min.BulgeAcceptanceExponent = 1.5;
	E5_Max.BulgeAcceptanceExponent = 2.5;
	E5_Min.BulgeAxisScale = FVector(.85, .85, .45);
	E5_Max.BulgeAxisScale = FVector(1, 1, .7);
	E5_Min.BulgeBaseDensity = 1;
	E5_Max.BulgeBaseDensity = 4;
	E5_Min.BulgeDepthBias = .6;
	E5_Max.BulgeDepthBias = .8;
	E5_Min.BulgeNumPoints = 150000;
	E5_Max.BulgeNumPoints = 250000;
	E5_Min.BulgeRadiusScale = .2;
	E5_Max.BulgeRadiusScale = .4;
	E5_Min.BulgeRatio = .8;
	E5_Max.BulgeRatio = 1.2;
	E5_Min.BulgeTruncationScale = .7;
	E5_Max.BulgeTruncationScale = 1.3;
	E5_Min.BulgeJitter = .2;
	E5_Max.BulgeJitter = .4;
	E5_Min.BackgroundNumPoints = 100000;
	E5_Max.BackgroundNumPoints = 150000;
	E5_Min.BackgroundBaseDensity = 15;
	E5_Max.BackgroundBaseDensity = 25;
	E5_Min.BackgroundDepthBias = .9;
	E5_Max.BackgroundDepthBias = 1.2;
	E5_Min.BackgroundHeightRatio = 1;
	E5_Max.BackgroundHeightRatio = 1;
	E5_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	E5_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	E5_Min.ClusterBaseDensity = 1;
	E5_Max.ClusterBaseDensity = 4;
	E5_Min.ClusterDepthBias = .5;
	E5_Max.ClusterDepthBias = .8;
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
	E7.DiscDepthBias = 1;
	E7.DiscHeightRatio = .2;
	E7.DiscBaseDensity = 2;
	E7.BulgeNumPoints = 100000;
	E7.BulgeBaseDensity = 3;
	E7.BulgeDepthBias = .5;
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
	E7_Min.DiscBaseDensity = 1;
	E7_Max.DiscBaseDensity = 5;
	E7_Min.DiscDepthBias = .8;
	E7_Max.DiscDepthBias = 1;
	E7_Min.DiscHeightRatio = .1;
	E7_Max.DiscHeightRatio = .25;
	E7_Min.DiscNumPoints = 150000;
	E7_Max.DiscNumPoints = 200000;
	E7_Min.BulgeAcceptanceExponent = 1.5;
	E7_Max.BulgeAcceptanceExponent = 2.5;
	E7_Min.BulgeAxisScale = FVector(.85, .85, .35);
	E7_Max.BulgeAxisScale = FVector(1, 1, .6);
	E7_Min.BulgeBaseDensity = 1;
	E7_Max.BulgeBaseDensity = 4;
	E7_Min.BulgeDepthBias = .6;
	E7_Max.BulgeDepthBias = .8;
	E7_Min.BulgeNumPoints = 150000;
	E7_Max.BulgeNumPoints = 250000;
	E7_Min.BulgeRadiusScale = .2;
	E7_Max.BulgeRadiusScale = .4;
	E7_Min.BulgeRatio = .8;
	E7_Max.BulgeRatio = 1.2;
	E7_Min.BulgeTruncationScale = .7;
	E7_Max.BulgeTruncationScale = 1.3;
	E7_Min.BulgeJitter = .2;
	E7_Max.BulgeJitter = .4;
	E7_Min.BackgroundNumPoints = 100000;
	E7_Max.BackgroundNumPoints = 150000;
	E7_Min.BackgroundBaseDensity = 15;
	E7_Max.BackgroundBaseDensity = 25;
	E7_Min.BackgroundDepthBias = .9;
	E7_Max.BackgroundDepthBias = 1.2;
	E7_Min.BackgroundHeightRatio = 1;
	E7_Max.BackgroundHeightRatio = 1;
	E7_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	E7_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	E7_Min.ClusterBaseDensity = 1;
	E7_Max.ClusterBaseDensity = 4;
	E7_Min.ClusterDepthBias = .5;
	E7_Max.ClusterDepthBias = .8;
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
	S0.DiscNumPoints = 100000;
	S0.DiscDepthBias = .8;
	S0.DiscHeightRatio = .1;
	S0.BulgeNumPoints = 100000;
	S0.BulgeBaseDensity = 3;
	S0.BulgeDepthBias = .5;
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
	S0_Min.DiscBaseDensity = 2;
	S0_Max.DiscBaseDensity = 5;
	S0_Min.DiscDepthBias = .8;
	S0_Max.DiscDepthBias = 1;
	S0_Min.DiscHeightRatio = .03;
	S0_Max.DiscHeightRatio = .1;
	S0_Min.DiscNumPoints = 150000;
	S0_Max.DiscNumPoints = 200000;
	S0_Min.BulgeAcceptanceExponent = 1.5;
	S0_Max.BulgeAcceptanceExponent = 2.5;
	S0_Min.BulgeAxisScale = FVector(.9, .9, .35);
	S0_Max.BulgeAxisScale = FVector(1, 1, .6);
	S0_Min.BulgeBaseDensity = 2;
	S0_Max.BulgeBaseDensity = 4;
	S0_Min.BulgeDepthBias = .6;
	S0_Max.BulgeDepthBias = .8;
	S0_Min.BulgeNumPoints = 150000;
	S0_Max.BulgeNumPoints = 200000;
	S0_Min.BulgeRadiusScale = .2;
	S0_Max.BulgeRadiusScale = .4;
	S0_Min.BulgeRatio = .8;
	S0_Max.BulgeRatio = 1.2;
	S0_Min.BulgeTruncationScale = .7;
	S0_Max.BulgeTruncationScale = 1.3;
	S0_Min.BulgeJitter = .2;
	S0_Max.BulgeJitter = .4;
	S0_Min.BackgroundNumPoints = 100000;
	S0_Max.BackgroundNumPoints = 150000;
	S0_Min.BackgroundBaseDensity = 15;
	S0_Max.BackgroundBaseDensity = 25;
	S0_Min.BackgroundDepthBias = .9;
	S0_Max.BackgroundDepthBias = 1.2;
	S0_Min.BackgroundHeightRatio = 1;
	S0_Max.BackgroundHeightRatio = 1;
	S0_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	S0_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	S0_Min.ClusterBaseDensity = 2;
	S0_Max.ClusterBaseDensity = 4;
	S0_Min.ClusterDepthBias = .5;
	S0_Max.ClusterDepthBias = .8;
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
	Sa.ArmSpreadMax = 3;
	Sa.ArmStartRatio = 0;
#pragma endregion
#pragma region Sa Bounds
	Sa_Min = Sa;
	Sa_Max = Sa;
	Sa_Min.GalaxyRatio = .2;
	Sa_Max.GalaxyRatio = .5;
	Sa_Min.DiscBaseDensity = 2;
	Sa_Max.DiscBaseDensity = 6;
	Sa_Min.DiscDepthBias = .8;
	Sa_Max.DiscDepthBias = 1;
	Sa_Min.DiscHeightRatio = .03;
	Sa_Max.DiscHeightRatio = .1;
	Sa_Min.DiscNumPoints = 100000;
	Sa_Max.DiscNumPoints = 150000;
	Sa_Min.ArmBaseDensity = 1;
	Sa_Max.ArmBaseDensity = 4;
	Sa_Min.ArmClusterRadiusMin = .025;
	Sa_Max.ArmClusterRadiusMin = .1;
	Sa_Min.ArmClusterRadiusMax = .15;
	Sa_Max.ArmClusterRadiusMax = .45;
	Sa_Min.ArmSpreadMin = .05;
	Sa_Max.ArmSpreadMin = .15;
	Sa_Min.ArmSpreadMax = .2;
	Sa_Max.ArmSpreadMax = .5;
	Sa_Min.ArmProgressExponent = .5;
	Sa_Max.ArmProgressExponent = 2;
	Sa_Min.ArmClusters = 128;
	Sa_Max.ArmClusters = 256;
	Sa_Min.ArmDepthBias = .4;
	Sa_Max.ArmDepthBias = .6;
	Sa_Min.ArmHeightRatio = .2;
	Sa_Max.ArmHeightRatio = .8;
	Sa_Min.ArmNumArms = 2;
	Sa_Max.ArmNumArms = 2;
	Sa_Min.ArmNumPoints = 125000;
	Sa_Max.ArmNumPoints = 200000;
	Sa_Min.ArmRadialDensityMin = .1;
	Sa_Max.ArmRadialDensityMin = 1;
	Sa_Min.ArmRadialDensityExponent = 1;
	Sa_Max.ArmRadialDensityExponent = 3;
	Sa_Min.ArmRadialDensityMax = 2;
	Sa_Max.ArmRadialDensityMax = 12;
	Sa_Min.ArmStartRatio = .25;
	Sa_Max.ArmStartRatio = .5;
	Sa_Min.TwistCoreRadius = .01;
	Sa_Max.TwistCoreRadius = .1;
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
	Sa_Min.BulgeNumPoints = 150000;
	Sa_Max.BulgeNumPoints = 200000;
	Sa_Min.BulgeRadiusScale = .2;
	Sa_Max.BulgeRadiusScale = .4;
	Sa_Min.BulgeRatio = .8;
	Sa_Max.BulgeRatio = 1.2;
	Sa_Min.BulgeTruncationScale = .7;
	Sa_Max.BulgeTruncationScale = 1.3;
	Sa_Min.BulgeJitter = .2;
	Sa_Max.BulgeJitter = .4;
	Sa_Min.BackgroundNumPoints = 100000;
	Sa_Max.BackgroundNumPoints = 150000;
	Sa_Min.BackgroundBaseDensity = 15;
	Sa_Max.BackgroundBaseDensity = 25;
	Sa_Min.BackgroundDepthBias = .9;
	Sa_Max.BackgroundDepthBias = 1.2;
	Sa_Min.BackgroundHeightRatio = 1;
	Sa_Max.BackgroundHeightRatio = 1;
	Sa_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	Sa_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	Sa_Min.ClusterBaseDensity = 2;
	Sa_Max.ClusterBaseDensity = 4;
	Sa_Min.ClusterDepthBias = .5;
	Sa_Max.ClusterDepthBias = .8;
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
	Sb.ArmSpreadMax = 6;
	Sb.ArmStartRatio = 0;
#pragma endregion
#pragma region Sb Bounds
	Sb_Min = Sb;
	Sb_Max = Sb;
	Sb_Min.GalaxyRatio = .2;
	Sb_Max.GalaxyRatio = .5;
	Sb_Min.DiscBaseDensity = 2;
	Sb_Max.DiscBaseDensity = 6;
	Sb_Min.DiscDepthBias = .8;
	Sb_Max.DiscDepthBias = 1;
	Sb_Min.DiscHeightRatio = .03;
	Sb_Max.DiscHeightRatio = .1;
	Sb_Min.DiscNumPoints = 100000;
	Sb_Max.DiscNumPoints = 150000;
	Sb_Min.ArmBaseDensity = 1;
	Sb_Max.ArmBaseDensity = 4;
	Sb_Min.ArmClusterRadiusMin = .025;
	Sb_Max.ArmClusterRadiusMin = .1;
	Sb_Min.ArmClusterRadiusMax = .15;
	Sb_Max.ArmClusterRadiusMax = .45;
	Sb_Min.ArmSpreadMin = .05;
	Sb_Max.ArmSpreadMin = .15;
	Sb_Min.ArmSpreadMax = .2;
	Sb_Max.ArmSpreadMax = .5;
	Sb_Min.ArmProgressExponent = .5;
	Sb_Max.ArmProgressExponent = 2;
	Sb_Min.ArmClusters = 128;
	Sb_Max.ArmClusters = 256;
	Sb_Min.ArmDepthBias = .4;
	Sb_Max.ArmDepthBias = .6;
	Sb_Min.ArmHeightRatio = .2;
	Sb_Max.ArmHeightRatio = .8;
	Sb_Min.ArmNumArms = 2;
	Sb_Max.ArmNumArms = 2;
	Sb_Min.ArmNumPoints = 150000;
	Sb_Max.ArmNumPoints = 200000;
	Sb_Min.ArmRadialDensityMin = .1;
	Sb_Max.ArmRadialDensityMin = 1;
	Sb_Min.ArmRadialDensityExponent = 1;
	Sb_Max.ArmRadialDensityExponent = 3;
	Sb_Min.ArmRadialDensityMax = 2;
	Sb_Max.ArmRadialDensityMax = 12;
	Sb_Min.ArmStartRatio = .25;
	Sb_Max.ArmStartRatio = .5;
	Sb_Min.TwistCoreRadius = .01;
	Sb_Max.TwistCoreRadius = .1;
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
	Sb_Min.BulgeNumPoints = 150000;
	Sb_Max.BulgeNumPoints = 200000;
	Sb_Min.BulgeRadiusScale = .2;
	Sb_Max.BulgeRadiusScale = .4;
	Sb_Min.BulgeRatio = .8;
	Sb_Max.BulgeRatio = 1.2;
	Sb_Min.BulgeTruncationScale = .7;
	Sb_Max.BulgeTruncationScale = 1.3;
	Sb_Min.BulgeJitter = .2;
	Sb_Max.BulgeJitter = .4;
	Sb_Min.BackgroundNumPoints = 100000;
	Sb_Max.BackgroundNumPoints = 150000;
	Sb_Min.BackgroundBaseDensity = 15;
	Sb_Max.BackgroundBaseDensity = 25;
	Sb_Min.BackgroundDepthBias = .9;
	Sb_Max.BackgroundDepthBias = 1.2;
	Sb_Min.BackgroundHeightRatio = 1;
	Sb_Max.BackgroundHeightRatio = 1;
	Sb_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	Sb_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	Sb_Min.ClusterBaseDensity = 2;
	Sb_Max.ClusterBaseDensity = 4;
	Sb_Min.ClusterDepthBias = .5;
	Sb_Max.ClusterDepthBias = .8;
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
	Sc.ArmSpreadMax = 8;
	Sc.ArmStartRatio = 0;
	Sc.TwistCoreRadius = .02;
#pragma endregion
#pragma region Sc Bounds
	Sc_Min = Sc;
	Sc_Max = Sc;
	Sc_Min.GalaxyRatio = .2;
	Sc_Max.GalaxyRatio = .5;
	Sc_Min.DiscBaseDensity = 2;
	Sc_Max.DiscBaseDensity = 6;
	Sc_Min.DiscDepthBias = .8;
	Sc_Max.DiscDepthBias = 1;
	Sc_Min.DiscHeightRatio = .03;
	Sc_Max.DiscHeightRatio = .1;
	Sc_Min.DiscNumPoints = 100000;
	Sc_Max.DiscNumPoints = 150000;
	Sc_Min.ArmBaseDensity = 1;
	Sc_Max.ArmBaseDensity = 4;
	Sc_Min.ArmClusterRadiusMin = .025;
	Sc_Max.ArmClusterRadiusMin = .1;
	Sc_Min.ArmClusterRadiusMax = .15;
	Sc_Max.ArmClusterRadiusMax = .45;
	Sc_Min.ArmSpreadMin = .05;
	Sc_Max.ArmSpreadMin = .15;
	Sc_Min.ArmSpreadMax = .2;
	Sc_Max.ArmSpreadMax = .5;
	Sc_Min.ArmProgressExponent = .5;
	Sc_Max.ArmProgressExponent = 2;
	Sc_Min.ArmClusters = 128;
	Sc_Max.ArmClusters = 256;
	Sc_Min.ArmDepthBias = .4;
	Sc_Max.ArmDepthBias = .6;
	Sc_Min.ArmHeightRatio = .2;
	Sc_Max.ArmHeightRatio = .8;
	Sc_Min.ArmNumArms = 3;
	Sc_Max.ArmNumArms = 8;
	Sc_Min.ArmNumPoints = 150000;
	Sc_Max.ArmNumPoints = 250000;
	Sc_Min.ArmRadialDensityMin = .1;
	Sc_Max.ArmRadialDensityMin = 1;
	Sc_Min.ArmRadialDensityExponent = 1;
	Sc_Max.ArmRadialDensityExponent = 3;
	Sc_Min.ArmRadialDensityMax = 2;
	Sc_Max.ArmRadialDensityMax = 12;
	Sc_Min.ArmStartRatio = .25;
	Sc_Max.ArmStartRatio = .5;
	Sc_Min.TwistCoreRadius = .01;
	Sc_Max.TwistCoreRadius = .1;
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
	Sc_Min.BulgeDepthBias = .6;
	Sc_Max.BulgeDepthBias = .8;
	Sc_Min.BulgeNumPoints = 150000;
	Sc_Max.BulgeNumPoints = 250000;
	Sc_Min.BulgeRadiusScale = .2;
	Sc_Max.BulgeRadiusScale = .4;
	Sc_Min.BulgeRatio = .8;
	Sc_Max.BulgeRatio = 1.2;
	Sc_Min.BulgeTruncationScale = .7;
	Sc_Max.BulgeTruncationScale = 1.3;
	Sc_Min.BulgeJitter = .2;
	Sc_Max.BulgeJitter = .4;
	Sc_Min.BackgroundNumPoints = 100000;
	Sc_Max.BackgroundNumPoints = 150000;
	Sc_Min.BackgroundBaseDensity = 15;
	Sc_Max.BackgroundBaseDensity = 25;
	Sc_Min.BackgroundDepthBias = .9;
	Sc_Max.BackgroundDepthBias = 1.2;
	Sc_Min.BackgroundHeightRatio = 1;
	Sc_Max.BackgroundHeightRatio = 1;
	Sc_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	Sc_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	Sc_Min.ClusterBaseDensity = 2;
	Sc_Max.ClusterBaseDensity = 4;
	Sc_Min.ClusterDepthBias = .5;
	Sc_Max.ClusterDepthBias = .8;
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
	SBa.ArmSpreadMax = 2;
	SBa.ArmHeightRatio = .5;
	SBa.ArmStartRatio = 0.0;
#pragma endregion
#pragma region SBa Bounds
	SBa_Min = SBa;
	SBa_Max = SBa;
	SBa_Min.GalaxyRatio = .2;
	SBa_Max.GalaxyRatio = .5;
	SBa_Min.DiscBaseDensity = 2;
	SBa_Max.DiscBaseDensity = 6;
	SBa_Min.DiscDepthBias = .8;
	SBa_Max.DiscDepthBias = 1;
	SBa_Min.DiscHeightRatio = .03;
	SBa_Max.DiscHeightRatio = .1;
	SBa_Min.DiscNumPoints = 100000;
	SBa_Max.DiscNumPoints = 150000;
	SBa_Min.ArmBaseDensity = 1;
	SBa_Max.ArmBaseDensity = 4;
	SBa_Min.ArmClusterRadiusMin = .025;
	SBa_Max.ArmClusterRadiusMin = .1;
	SBa_Min.ArmClusterRadiusMax = .15;
	SBa_Max.ArmClusterRadiusMax = .45;
	SBa_Min.ArmSpreadMin = .05;
	SBa_Max.ArmSpreadMin = .15;
	SBa_Min.ArmSpreadMax = .2;
	SBa_Max.ArmSpreadMax = .5;
	SBa_Min.ArmProgressExponent = .5;
	SBa_Max.ArmProgressExponent = 2;
	SBa_Min.ArmClusters = 128;
	SBa_Max.ArmClusters = 256;
	SBa_Min.ArmDepthBias = .4;
	SBa_Max.ArmDepthBias = .6;
	SBa_Min.ArmHeightRatio = .2;
	SBa_Max.ArmHeightRatio = .8;
	SBa_Min.ArmNumArms = 2;
	SBa_Max.ArmNumArms = 2;
	SBa_Min.ArmNumPoints = 150000;
	SBa_Max.ArmNumPoints = 250000;
	SBa_Min.ArmRadialDensityMin = .1;
	SBa_Max.ArmRadialDensityMin = 1;
	SBa_Min.ArmRadialDensityExponent = 1;
	SBa_Max.ArmRadialDensityExponent = 3;
	SBa_Min.ArmRadialDensityMax = 2;
	SBa_Max.ArmRadialDensityMax = 12;
	SBa_Min.ArmStartRatio = 0;
	SBa_Max.ArmStartRatio = 0.2;
	SBa_Min.TwistCoreRadius = .01;
	SBa_Max.TwistCoreRadius = 0.1;
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
	SBa_Min.BulgeNumPoints = 150000;
	SBa_Max.BulgeNumPoints = 250000;
	SBa_Min.BulgeRadiusScale = .2;
	SBa_Max.BulgeRadiusScale = .4;
	SBa_Min.BulgeRatio = .8;
	SBa_Max.BulgeRatio = 1.2;
	SBa_Min.BulgeTruncationScale = .7;
	SBa_Max.BulgeTruncationScale = 1.3;
	SBa_Min.BulgeJitter = .2;
	SBa_Max.BulgeJitter = .4;
	SBa_Min.BackgroundNumPoints = 100000;
	SBa_Max.BackgroundNumPoints = 150000;
	SBa_Min.BackgroundBaseDensity = 15;
	SBa_Max.BackgroundBaseDensity = 25;
	SBa_Min.BackgroundDepthBias = .9;
	SBa_Max.BackgroundDepthBias = 1.2;
	SBa_Min.BackgroundHeightRatio = 1;
	SBa_Max.BackgroundHeightRatio = 1;
	SBa_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	SBa_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	SBa_Min.ClusterBaseDensity = 2;
	SBa_Max.ClusterBaseDensity = 4;
	SBa_Min.ClusterDepthBias = .5;
	SBa_Max.ClusterDepthBias = .8;
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
	SBb.ArmSpreadMax = 8;
	SBb.ArmHeightRatio = .5;
	SBb.ArmStartRatio = 0;
#pragma endregion
#pragma region SBb Bounds
	SBb_Min = SBb;
	SBb_Max = SBb;
	SBb_Min.GalaxyRatio = .2;
	SBb_Max.GalaxyRatio = .5;
	SBb_Min.DiscBaseDensity = 2;
	SBb_Max.DiscBaseDensity = 6;
	SBb_Min.DiscDepthBias = .8;
	SBb_Max.DiscDepthBias = 1;
	SBb_Min.DiscHeightRatio = .03;
	SBb_Max.DiscHeightRatio = .1;
	SBb_Min.DiscNumPoints = 100000;
	SBb_Max.DiscNumPoints = 150000;
	SBb_Min.ArmBaseDensity = 1;
	SBb_Max.ArmBaseDensity = 4;
	SBb_Min.ArmClusterRadiusMin = .025;
	SBb_Max.ArmClusterRadiusMin = .1;
	SBb_Min.ArmClusterRadiusMax = .15;
	SBb_Max.ArmClusterRadiusMax = .45;
	SBb_Min.ArmSpreadMin = .05;
	SBb_Max.ArmSpreadMin = .15;
	SBb_Min.ArmSpreadMax = .2;
	SBb_Max.ArmSpreadMax = .5;
	SBb_Min.ArmProgressExponent = .5;
	SBb_Max.ArmProgressExponent = 2;
	SBb_Min.ArmClusters = 128;
	SBb_Max.ArmClusters = 256;
	SBb_Min.ArmDepthBias = .4;
	SBb_Max.ArmDepthBias = .6;
	SBb_Min.ArmHeightRatio = .2;
	SBb_Max.ArmHeightRatio = .8;
	SBb_Min.ArmNumArms = 2;
	SBb_Max.ArmNumArms = 2;
	SBb_Min.ArmNumPoints = 150000;
	SBb_Max.ArmNumPoints = 250000;
	SBb_Min.ArmRadialDensityMin = .1;
	SBb_Max.ArmRadialDensityMin = 1;
	SBb_Min.ArmRadialDensityExponent = 1;
	SBb_Max.ArmRadialDensityExponent = 3;
	SBb_Min.ArmRadialDensityMax = 2;
	SBb_Max.ArmRadialDensityMax = 12;
	SBb_Min.ArmStartRatio = 0;
	SBb_Max.ArmStartRatio = .1;
	SBb_Min.TwistCoreRadius = 01;
	SBb_Max.TwistCoreRadius = .1;
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
	SBb_Min.BulgeNumPoints = 150000;
	SBb_Max.BulgeNumPoints = 250000;
	SBb_Min.BulgeRadiusScale = .2;
	SBb_Max.BulgeRadiusScale = .4;
	SBb_Min.BulgeRatio = .8;
	SBb_Max.BulgeRatio = 1.2;
	SBb_Min.BulgeTruncationScale = .7;
	SBb_Max.BulgeTruncationScale = 1.3;
	SBb_Min.BulgeJitter = .2;
	SBb_Max.BulgeJitter = .4;
	SBb_Min.BackgroundNumPoints = 100000;
	SBb_Max.BackgroundNumPoints = 150000;
	SBb_Min.BackgroundBaseDensity = 15;
	SBb_Max.BackgroundBaseDensity = 25;
	SBb_Min.BackgroundDepthBias = .9;
	SBb_Max.BackgroundDepthBias = 1.2;
	SBb_Min.BackgroundHeightRatio = 1;
	SBb_Max.BackgroundHeightRatio = 1;
	SBb_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	SBb_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	SBb_Min.ClusterBaseDensity = 2;
	SBb_Max.ClusterBaseDensity = 4;
	SBb_Min.ClusterDepthBias = .5;
	SBb_Max.ClusterDepthBias = .8;
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
	SBc.ArmSpreadMax = 8;
	SBc.ArmHeightRatio = .5;
	SBc.ArmStartRatio = 0;
#pragma endregion
#pragma region SBc Bounds
	SBc_Min = SBc;
	SBc_Max = SBc;
	SBc_Min.GalaxyRatio = .2;
	SBc_Max.GalaxyRatio = .5;
	SBc_Min.DiscBaseDensity = 2;
	SBc_Max.DiscBaseDensity = 4;
	SBc_Min.DiscDepthBias = .8;
	SBc_Max.DiscDepthBias = 1;
	SBc_Min.DiscHeightRatio = .03;
	SBc_Max.DiscHeightRatio = .1;
	SBc_Min.DiscNumPoints = 100000;
	SBc_Max.DiscNumPoints = 150000;
	SBc_Min.ArmBaseDensity = 2;
	SBc_Max.ArmBaseDensity = 4;
	SBc_Min.ArmClusterRadiusMin = .025;
	SBc_Max.ArmClusterRadiusMin = .1;
	SBc_Min.ArmClusterRadiusMax = .15;
	SBc_Max.ArmClusterRadiusMax = .45;
	SBc_Min.ArmSpreadMin = .05;
	SBc_Max.ArmSpreadMin = .15;
	SBc_Min.ArmSpreadMax = .2;
	SBc_Max.ArmSpreadMax = .5;
	SBc_Min.ArmProgressExponent = .5;
	SBc_Max.ArmProgressExponent = 2;
	SBc_Min.ArmClusters = 128;
	SBc_Max.ArmClusters = 256;
	SBc_Min.ArmDepthBias = .4;
	SBc_Max.ArmDepthBias = .6;
	SBc_Min.ArmHeightRatio = .2;
	SBc_Max.ArmHeightRatio = .8;
	SBc_Min.ArmNumArms = 2;
	SBc_Max.ArmNumArms = 2;
	SBc_Min.ArmNumPoints = 150000;
	SBc_Max.ArmNumPoints = 250000;
	SBc_Min.ArmRadialDensityMin = .1;
	SBc_Max.ArmRadialDensityMin = 1;
	SBc_Min.ArmRadialDensityExponent = 1;
	SBc_Max.ArmRadialDensityExponent = 3;
	SBc_Min.ArmRadialDensityMax = 2;
	SBc_Max.ArmRadialDensityMax = 12;
	SBc_Min.ArmStartRatio = 0;
	SBc_Max.ArmStartRatio = .1;
	SBc_Min.TwistCoreRadius = 01;
	SBc_Max.TwistCoreRadius = .1;
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
	SBc_Min.BulgeNumPoints = 150000;
	SBc_Max.BulgeNumPoints = 250000;
	SBc_Min.BulgeRadiusScale = .2;
	SBc_Max.BulgeRadiusScale = .4;
	SBc_Min.BulgeRatio = .8;
	SBc_Max.BulgeRatio = 1.2;
	SBc_Min.BulgeTruncationScale = .7;
	SBc_Max.BulgeTruncationScale = 1.3;
	SBc_Min.BulgeJitter = .2;
	SBc_Max.BulgeJitter = .4;
	SBc_Min.BackgroundNumPoints = 100000;
	SBc_Max.BackgroundNumPoints = 150000;
	SBc_Min.BackgroundBaseDensity = 15;
	SBc_Max.BackgroundBaseDensity = 25;
	SBc_Min.BackgroundDepthBias = .9;
	SBc_Max.BackgroundDepthBias = 1.2;
	SBc_Min.BackgroundHeightRatio = 1;
	SBc_Max.BackgroundHeightRatio = 1;
	SBc_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	SBc_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	SBc_Min.ClusterBaseDensity = 2;
	SBc_Max.ClusterBaseDensity = 4;
	SBc_Min.ClusterDepthBias = .5;
	SBc_Max.ClusterDepthBias = .8;
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
	Irr_Min.BulgeAxisScale = FVector(.85, .85, .85);
	Irr_Max.BulgeAxisScale = FVector(1, 1, 1);
	Irr_Min.BulgeNumPoints = 0;
	Irr_Max.BulgeNumPoints = 0;
	Irr_Min.BulgeRadiusScale = .2;
	Irr_Max.BulgeRadiusScale = .4;
	Irr_Min.BulgeRatio = .8;
	Irr_Max.BulgeRatio = 1.2;
	Irr_Min.BackgroundNumPoints = 100000;
	Irr_Max.BackgroundNumPoints = 150000;
	Irr_Min.BackgroundBaseDensity = 5;
	Irr_Max.BackgroundBaseDensity = 10;
	Irr_Min.BackgroundDepthBias = .9;
	Irr_Max.BackgroundDepthBias = 1.2;
	Irr_Min.BackgroundHeightRatio = 1;
	Irr_Max.BackgroundHeightRatio = 1;
	Irr_Min.ClusterAxisScale = E0_Min.BulgeAxisScale;
	Irr_Max.ClusterAxisScale = E0_Max.BulgeAxisScale;
	Irr_Min.ClusterBaseDensity = 3;
	Irr_Max.ClusterBaseDensity = 6;
	Irr_Min.ClusterDepthBias = .4;
	Irr_Max.ClusterDepthBias = .6;
	Irr_Min.ClusterIncoherence = 2;
	Irr_Max.ClusterIncoherence = 4;
	Irr_Min.ClusterMaxScale = .8;
	Irr_Max.ClusterMaxScale = .6;
	Irr_Min.ClusterMinScale = .2;
	Irr_Max.ClusterMinScale = .4;
	Irr_Min.ClusterNumClusters = 64;
	Irr_Max.ClusterNumClusters = 256;
	Irr_Min.ClusterNumPoints = 200000;
	Irr_Max.ClusterNumPoints = 300000;
	Irr_Min.ClusterSpreadFactor = .3;
	Irr_Max.ClusterSpreadFactor = .6;
#pragma endregion
};

GalaxyParams GalaxyParamFactory::GenerateParams()
{
	// 0  1  2  3  4  5  6  7  8   9   10  11
	// E0 E3 E5 E7 S0 Sa Sb Sc SBa SBb SBc Irr
	int TypeIndex = SelectGalaxyTypeIndex();

	GalaxyParams Params;// = *GalaxyWeights[TypeIndex].Value;
	switch (TypeIndex) {
	case 0: // E0
		Params = BoundedRandomizeParams(E0_Min, E0_Max);
		break;
	case 1: // E3
		Params = BoundedRandomizeParams(E3_Min, E3_Max);
		break;
	case 2: // E5
		Params = BoundedRandomizeParams(E5_Min, E5_Max);
		break;
	case 3: // E7
		Params = BoundedRandomizeParams(E7_Min, E7_Max);
		break;
	case 4: // S0
		Params = BoundedRandomizeParams(S0_Min, S0_Max);
		break;
	case 5: // Sa
		Params = BoundedRandomizeParams(Sa_Min, Sa_Max);
		break;
	case 6: // Sb 
		Params = BoundedRandomizeParams(Sb_Min, Sb_Max);
		break;
	case 7: // Sc
		Params = BoundedRandomizeParams(Sc_Min, Sc_Max);
		break;
	case 8: // SBa
		Params = BoundedRandomizeParams(SBa_Min, SBa_Max);
		break;
	case 9: // SBb
		Params = BoundedRandomizeParams(SBb_Min, SBb_Max);
		break;
	case 10: // SBc
		Params = BoundedRandomizeParams(SBc_Min, SBc_Max);
		break;
	case 11: // Irr
		Params = BoundedRandomizeParams(Irr_Min, Irr_Max);
		break;
	}

	return Params;
}

GalaxyParams GalaxyParamFactory::BoundedRandomizeParams(GalaxyParams MinParams, GalaxyParams MaxParams) {
	FRandomStream Stream(Seed + 666);
	GalaxyParams Params;

	//Arm
	Params.ArmBaseDensity = Stream.FRandRange(MinParams.ArmBaseDensity, MaxParams.ArmBaseDensity);
	Params.ArmClusterRadiusMax = Stream.FRandRange(MinParams.ArmClusterRadiusMax, MaxParams.ArmClusterRadiusMax);
	Params.ArmClusterRadiusMin = Stream.FRandRange(MinParams.ArmClusterRadiusMin, MaxParams.ArmClusterRadiusMin);
	Params.ArmClusters = Stream.RandRange(MinParams.ArmClusters, MaxParams.ArmClusters);
	Params.ArmDepthBias = Stream.FRandRange(MinParams.ArmDepthBias, MaxParams.ArmDepthBias);
	Params.ArmHeightRatio = Stream.FRandRange(MinParams.ArmHeightRatio, MaxParams.ArmHeightRatio);
	Params.ArmSpreadMax = Stream.FRandRange(MinParams.ArmSpreadMax, MaxParams.ArmSpreadMax);
	Params.ArmNumArms = Stream.RandRange(MinParams.ArmNumArms, MaxParams.ArmNumArms);
	Params.ArmNumPoints = Stream.RandRange(MinParams.ArmNumPoints, MaxParams.ArmNumPoints);
	Params.ArmRadialDensityExponent = Stream.FRandRange(MinParams.ArmRadialDensityExponent, MaxParams.ArmRadialDensityExponent);
	Params.ArmRadialDensityMax = Stream.FRandRange(MinParams.ArmRadialDensityMax, MaxParams.ArmRadialDensityMax);
	Params.ArmSpreadMin = Stream.FRandRange(MinParams.ArmSpreadMin, MaxParams.ArmSpreadMin);
	Params.ArmStartRatio = Stream.FRandRange(MinParams.ArmStartRatio, MaxParams.ArmStartRatio);

	//Background
	Params.BackgroundBaseDensity = Stream.FRandRange(MinParams.BackgroundBaseDensity, MaxParams.BackgroundBaseDensity);
	Params.BackgroundDepthBias = Stream.FRandRange(MinParams.BackgroundDepthBias, MaxParams.BackgroundDepthBias);
	Params.BackgroundHeightRatio = Stream.FRandRange(MinParams.BackgroundHeightRatio, MaxParams.BackgroundHeightRatio);
	Params.BackgroundNumPoints = Stream.RandRange(MinParams.BackgroundNumPoints, MaxParams.BackgroundNumPoints);

	//Bulge
	Params.BulgeAcceptanceExponent = Stream.FRandRange(MinParams.BulgeAcceptanceExponent, MaxParams.BulgeAcceptanceExponent);
	Params.BulgeAxisScale = FVector(Stream.FRandRange(MinParams.BulgeAxisScale.X, MaxParams.BulgeAxisScale.X), Stream.FRandRange(MinParams.BulgeAxisScale.Y, MaxParams.BulgeAxisScale.Y), Stream.FRandRange(MinParams.BulgeAxisScale.Z, MaxParams.BulgeAxisScale.Z));
	Params.BulgeBaseDensity = Stream.FRandRange(MinParams.BulgeBaseDensity, MaxParams.BulgeBaseDensity);
	Params.BulgeDepthBias = Stream.FRandRange(MinParams.BulgeDepthBias, MaxParams.BulgeDepthBias);
	Params.BulgeNumPoints = Stream.RandRange(MinParams.BulgeNumPoints, MaxParams.BulgeNumPoints);
	Params.BulgeRadiusScale = Stream.FRandRange(MinParams.BulgeRadiusScale, MaxParams.BulgeRadiusScale);
	Params.BulgeRatio = Stream.FRandRange(MinParams.BulgeRatio, MaxParams.BulgeRatio);
	Params.BulgeTruncationScale = Stream.FRandRange(MinParams.BulgeTruncationScale, MaxParams.BulgeTruncationScale);

	//ere
	//Cluster
	Params.ClusterAxisScale = FVector(Stream.FRandRange(MinParams.ClusterAxisScale.X, MaxParams.ClusterAxisScale.X), Stream.FRandRange(MinParams.ClusterAxisScale.Y, MaxParams.ClusterAxisScale.Y), Stream.FRandRange(MinParams.ClusterAxisScale.Z, MaxParams.ClusterAxisScale.Z));
	Params.ClusterBaseDensity = Stream.FRandRange(MinParams.ClusterBaseDensity, MaxParams.ClusterBaseDensity);
	Params.ClusterDepthBias = Stream.FRandRange(MinParams.ClusterDepthBias, MaxParams.ClusterDepthBias);
	Params.ClusterIncoherence = Stream.FRandRange(MinParams.ClusterIncoherence, MaxParams.ClusterIncoherence);
	Params.ClusterMaxScale = Stream.FRandRange(MinParams.ClusterMaxScale, MaxParams.ClusterMaxScale);
	Params.ClusterMinScale = Stream.FRandRange(MinParams.ClusterMinScale, MaxParams.ClusterMinScale);
	Params.ClusterNumClusters = Stream.RandRange(MinParams.ClusterNumClusters, MaxParams.ClusterNumClusters);
	Params.ClusterNumPoints = Stream.RandRange(MinParams.ClusterNumPoints, MaxParams.ClusterNumPoints);
	Params.ClusterSpreadFactor = Stream.FRandRange(MinParams.ClusterSpreadFactor, MaxParams.ClusterSpreadFactor);

	//Disc
	Params.DiscBaseDensity = Stream.FRandRange(MinParams.DiscBaseDensity, MaxParams.DiscBaseDensity);
	Params.DiscDepthBias = Stream.FRandRange(MinParams.DiscDepthBias, MaxParams.DiscDepthBias);
	Params.DiscHeightRatio = Stream.FRandRange(MinParams.DiscHeightRatio, MaxParams.DiscHeightRatio);
	Params.DiscNumPoints = Stream.RandRange(MinParams.DiscNumPoints, MaxParams.DiscNumPoints);

	//Twist
	Params.TwistCoreRadius = Stream.FRandRange(MinParams.TwistCoreRadius, MaxParams.TwistCoreRadius);
	Params.TwistCoreStrength = Stream.FRandRange(MinParams.TwistCoreStrength, MaxParams.TwistCoreStrength);
	Params.TwistCoreTwistExponent = Stream.FRandRange(MinParams.TwistCoreTwistExponent, MaxParams.TwistCoreTwistExponent);
	Params.TwistStrength = Stream.FRandRange(MinParams.TwistStrength, MaxParams.TwistStrength);

	//Void
	Params.VoidRatio = Stream.FRandRange(MinParams.VoidRatio, MaxParams.VoidRatio);


	//Volume
	Params.VolumeAmbientColor = FLinearColor(
		Stream.FRandRange(Volume_Min.VolumeAmbientColor.R, Volume_Max.VolumeAmbientColor.R),
		Stream.FRandRange(Volume_Min.VolumeAmbientColor.G, Volume_Max.VolumeAmbientColor.G),
		Stream.FRandRange(Volume_Min.VolumeAmbientColor.B, Volume_Max.VolumeAmbientColor.B)
	);
	Params.VolumeCoolShift = FLinearColor(
		Stream.FRandRange(Volume_Min.VolumeCoolShift.R, Volume_Max.VolumeCoolShift.R),
		Stream.FRandRange(Volume_Min.VolumeCoolShift.G, Volume_Max.VolumeCoolShift.G),
		Stream.FRandRange(Volume_Min.VolumeCoolShift.B, Volume_Max.VolumeCoolShift.B)
	);
	Params.VolumeHotShift = FLinearColor(
		Stream.FRandRange(Volume_Min.VolumeHotShift.R, Volume_Max.VolumeHotShift.R),
		Stream.FRandRange(Volume_Min.VolumeHotShift.G, Volume_Max.VolumeHotShift.G),
		Stream.FRandRange(Volume_Min.VolumeHotShift.B, Volume_Max.VolumeHotShift.B)
	);
	Params.VolumeNoise = NoisePaths[Stream.RandRange(0, 3)];
	Params.VolumeDensity = Stream.FRandRange(Volume_Min.VolumeDensity, Volume_Max.VolumeDensity);
	Params.VolumeHueVariance = Stream.FRandRange(Volume_Min.VolumeHueVariance, Volume_Max.VolumeHueVariance);
	Params.VolumeHueVarianceScale = Stream.FRandRange(Volume_Min.VolumeHueVarianceScale, Volume_Max.VolumeHueVarianceScale);
	Params.VolumeSaturationVariance = Stream.FRandRange(Volume_Min.VolumeSaturationVariance, Volume_Max.VolumeSaturationVariance);
	Params.VolumeTemperatureScale = Stream.FRandRange(Volume_Min.VolumeTemperatureScale, Volume_Max.VolumeTemperatureScale);
	Params.VolumeTemperatureInfluence = Stream.FRandRange(Volume_Min.VolumeTemperatureInfluence, Volume_Max.VolumeTemperatureInfluence);
	Params.VolumeWarpAmount = Stream.FRandRange(Volume_Min.VolumeWarpAmount, Volume_Max.VolumeWarpAmount);
	Params.VolumeWarpScale = Stream.FRandRange(Volume_Min.VolumeWarpScale, Volume_Max.VolumeWarpScale);

	return Params;
}

int GalaxyParamFactory::SelectGalaxyTypeIndex()
{
	FRandomStream Stream(Seed + 69);
	float RandomValue = Stream.FRand();
	float CumulativeWeight = 0.0f;

	for (int i = 0; i < GalaxyWeights.Num(); i++)
	{
		CumulativeWeight += GalaxyWeights[i].Key;
		if (RandomValue <= CumulativeWeight)
		{
			return i;
		}
	}

	return 7; // Fallback to most common type
}
#pragma endregion
#pragma region GalaxyGenerator
void GalaxyGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{	 
	//Glob - E0 E3 E5 E7 S0
	//Spiral - Sa Sb Sc
	//Spiral barred - SBa SBb SBc
	//Irregular - Irr
	// 0  1  2  3  4  5  6  7  8   9   10  11
	// E0 E3 E5 E7 S0 Sa Sb Sc SBa SBb SBc Irr

	MaxInsertionDepth = FMath::Max(1, InOctree->MaxDepth - InsertDepthOffset);
	MinInsertionDepth = FMath::Max(1, MaxInsertionDepth - DepthRange);

	MaxRadius = InOctree->Extent;
	GalaxyRadius = MaxRadius * GalaxyParams.GalaxyRatio; //Galaxy proper radius
	BulgeRadius = GalaxyRadius * GalaxyParams.BulgeRatio; //Central bulge radius
	VoidRadius = BulgeRadius * GalaxyParams.VoidRatio; //Central void

	GenerateArms();
	ApplyTwist();
	GenerateClusters();
	GenerateBulge();
	GenerateDisc();
	GenerateBackground();
	ApplyRotation();

	//TODO: Might be worth while to create a specific gas distribution and explicitly insert it at the desired depth for the volume texture

	const float VoidRadiusSquared = VoidRadius * VoidRadius;

	FPointData BlackHole;
	BlackHole.Position = FVector::ZeroVector;
	BlackHole.Data.ObjectId = INT32_MAX;
	BlackHole.Data.TypeId = 1;

	FCriticalSection BlackHoleMutex;
	TArray<FPointData> FinalData;
	ParallelFor(GeneratedData.Num(), [this, InOctree, &BlackHole, &BlackHoleMutex, VoidRadiusSquared](int32 i){
		FPointData InsertData = GeneratedData[i];
		//This step checks for stars that are within the black holes consumption radius. 
		//Their data gets accumulated into the black hole, and the original star is configured to be ignored in bulk insert.
		if (InsertData.Position.SizeSquared() < VoidRadiusSquared)
		{
			FScopeLock Lock(&BlackHoleMutex);
			GeneratedData[i].Data.TypeId = -1;
			GeneratedData[i].Position = FVector::ZeroVector;
			double DensityWeight = FMath::Pow(2.0, InOctree->MaxDepth - InsertData.InsertDepth);
			BlackHole.Data.Density += InsertData.Data.Density * DensityWeight;
			BlackHole.Data.Composition += InsertData.Data.Composition * BlackHole.Data.Density;
		}
	});

	BlackHole.InsertDepth = MinInsertionDepth - 3; //TODO: Calculate approx radius based on end density
	BlackHole.Data.Density = 1; //TODO: This should be allowed to be the accumulated density but it currently effects gas accumulation too much
	GeneratedData.Add(BlackHole);
}

void GalaxyGenerator::GenerateBulge()
{
	//double StartTime = FPlatformTime::Seconds();

	const int32 NumPoints = GalaxyParams.BulgeNumPoints;
	GeneratedData.Reserve(GeneratedData.Num() + NumPoints);

	// Pre-allocate space for thread-safe insertion
	const int32 StartIndex = GeneratedData.Num();
	GeneratedData.AddUninitialized(NumPoints);

	// Cache frequently used values
	const FVector AxisBounds = GalaxyParams.BulgeAxisScale * BulgeRadius;
	const double a = BulgeRadius * GalaxyParams.BulgeRadiusScale;
	const double SoftTruncationRadius = BulgeRadius * GalaxyParams.BulgeTruncationScale;
	const double TruncationZone = MaxRadius - SoftTruncationRadius;
	const double AxisScaleX = AxisBounds.X / BulgeRadius;
	const double AxisScaleY = AxisBounds.Y / BulgeRadius;
	const double AxisScaleZ = AxisBounds.Z / BulgeRadius;
	const FVector AxisScale = AxisBounds / BulgeRadius;

	// Create per-thread random streams
	int32 NumThreads = FPlatformMisc::NumberOfCores();
	TArray<FRandomStream> ThreadStreams;
	ThreadStreams.SetNumUninitialized(NumThreads);
	for (int32 t = 0; t < NumThreads; t++) {
		ThreadStreams[t].Initialize(Seed + 99 + t * 997); // unique per thread
	}

	ParallelFor(NumPoints, [&](int32 i)
		{
			// Each thread gets its own random stream with unique seed
			int32 ThreadIdx = FPlatformTLS::GetCurrentThreadId() % NumThreads;
			FRandomStream& Stream = ThreadStreams[ThreadIdx];

			FPointData& InsertData = GeneratedData[StartIndex + i];
			InsertData.Data.ObjectId = i;
			InsertData.Data.TypeId = 1;
			InsertData.Data.Density = Stream.FRandRange(0.5f, 1.5f) * GalaxyParams.BulgeBaseDensity;
			InsertData.Data.Composition = Stream.GetUnitVector();
			InsertData.InsertDepth = ChooseDepth(Stream.FRand(), GalaxyParams.BulgeDepthBias);

			// 1. Sample Hernquist radius
			// double sqrtu = FMath::Sqrt(FMath::Clamp(Stream.FRand(), KINDA_SMALL_NUMBER, 1.0 - KINDA_SMALL_NUMBER));
			// double r = (a * sqrtu) / (1.0 - sqrtu);
			// Approximate f(u) = sqrt(u)/(1-sqrt(u)) with a 5th-degree poly
			double u = Stream.FRand();
			double f = u * (1.0 + u * (0.5 + u * (0.333 + u * 0.25)));
			double r = a * f;
			// 2. Apply soft truncation with probabilistic acceptance
			if (r > SoftTruncationRadius)
			{
				if (r > MaxRadius) // Failed points get fed to the black hole
				{
					InsertData.Position = FVector::ZeroVector;
					return;
				}

				// Exponential falloff probability
				double acceptanceProbability = FMath::Exp(-3.0 * FMath::Pow(((r - SoftTruncationRadius) / TruncationZone), GalaxyParams.BulgeAcceptanceExponent));
				if (Stream.FRand() > acceptanceProbability) // Failed points get fed to the black hole
				{
					InsertData.Position = FVector::ZeroVector;
					return;
				}
			}

			// 3. Calculate final position
			InsertData.Position = Stream.GetUnitVector() * r * AxisScale + (Stream.GetUnitVector() * Stream.FRand() * BulgeRadius * GalaxyParams.BulgeJitter);
		});

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Generate Bulge duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::GenerateClusters()
{
	//double StartTime = FPlatformTime::Seconds();

	// Exit if no clusters or points are requested to avoid division by zero.
	if (GalaxyParams.ClusterNumClusters <= 0 || GalaxyParams.ClusterNumPoints <= 0)
	{
		return;
	}

	const int PointsPerCluster = GalaxyParams.ClusterNumPoints / GalaxyParams.ClusterNumClusters;
	if (PointsPerCluster <= 0)
	{
		return;
	}

	// Use a unique seed for this generation pass for reproducible randomness.
	FRandomStream Stream(Seed + 2024);

	for (int i = 0; i < GalaxyParams.ClusterNumClusters; i++)
	{
		// 1. Determine the cluster's central position.
		// We generate a random point within a unit sphere and then scale it.
		// Using a cubic root on the random sample ensures a uniform spatial distribution.
		const double u = Stream.FRand();
		const double Dist = GalaxyRadius * FMath::Pow(u, 1.0 / 3.0);

		// Get a random isotropic direction.
		const double z = Stream.FRandRange(-1.0, 1.0);
		const double phi = Stream.FRandRange(0.0, 2.0 * PI);
		const double xy = FMath::Sqrt(1.0 - z * z);
		const FVector Dir(xy * FMath::Cos(phi), xy * FMath::Sin(phi), z);

		FVector Center = Dir * Dist;

		// Apply axis scaling to shape the overall irregular galaxy (e.g., making it flatter or more elongated).
		Center.X *= GalaxyParams.ClusterAxisScale.X;
		Center.Y *= GalaxyParams.ClusterAxisScale.Y;
		Center.Z *= GalaxyParams.ClusterAxisScale.Z;

		// 2. Determine the cluster's radius.
		const double DistFromOrigin = Center.Length();
		const double NormalizedDist = FMath::Clamp(DistFromOrigin / GalaxyRadius, 0.0, 1.0);

		// Interpolate between the min and max scale based on the cluster's distance from the galactic center.
		const double BaseRadiusScale = FMath::Lerp(GalaxyParams.ClusterMinScale, GalaxyParams.ClusterMaxScale, NormalizedDist);
		const double ClusterRadiusVal = GalaxyRadius * BaseRadiusScale;

		FVector Radius(
			ClusterRadiusVal * GalaxyParams.ClusterSpreadFactor,
			ClusterRadiusVal * GalaxyParams.ClusterSpreadFactor,
			ClusterRadiusVal * GalaxyParams.ClusterSpreadFactor
		);

		// 3. Apply jitter (incoherence) to the final position for a more natural look.
		// This is done after calculating the radius to prevent jitter from affecting cluster size.
		const FVector Jitter(
			Stream.FRandRange(-Radius.X, Radius.X) * GalaxyParams.ClusterIncoherence,
			Stream.FRandRange(-Radius.Y, Radius.Y) * GalaxyParams.ClusterIncoherence,
			Stream.FRandRange(-Radius.Z, Radius.Z) * GalaxyParams.ClusterIncoherence
		);
		Center += Jitter;

		// 5. Generate the actual cluster of points.
		GenerateCluster(i + 987654, Center, Radius, PointsPerCluster, GalaxyParams.ClusterBaseDensity, GalaxyParams.ClusterDepthBias);
	}

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Generate Cluster duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::GenerateArms()
{
	//double StartTime = FPlatformTime::Seconds();

	int StarsPerCluster = (GalaxyParams.ArmNumPoints / GalaxyParams.ArmNumArms) / GalaxyParams.ArmClusters; // should have some degree of randomization
	double MinScale = GalaxyRadius * GalaxyParams.ArmClusterRadiusMin;
	double MaxScale = GalaxyRadius * GalaxyParams.ArmClusterRadiusMax;
	double MinJitter = GalaxyRadius * GalaxyParams.ArmSpreadMin;
	double MaxJitter = GalaxyRadius * GalaxyParams.ArmSpreadMax;

	double MinScaleVariance = .5;
	double MaxScaleVariance = 1.5;
	double MinJitterVariance = .5;
	double MaxJitterVariance = 1.5;

	double ProgressExponent = GalaxyParams.ArmProgressExponent;

	FRandomStream Stream(Seed + 1337);
	for (int ArmIndex = 0; ArmIndex < GalaxyParams.ArmNumArms; ArmIndex++)
	{
		double BaseAngle = (2.0 * PI * ArmIndex) / GalaxyParams.ArmNumArms;
		FVector ArmDir(FMath::Cos(BaseAngle), FMath::Sin(BaseAngle), 0);
		for (int c = 0; c < GalaxyParams.ArmClusters; c++)
		{
			double Dist = FMath::Clamp(-GalaxyRadius * FMath::Loge(1.0 - Stream.FRand()), FMath::Max(0,BulgeRadius * GalaxyParams.ArmStartRatio), GalaxyRadius);
			double Progress = Dist / GalaxyRadius;
			double ExpProgress = FMath::Pow(Progress, ProgressExponent);
			FVector Center = ArmDir * Dist + FMath::Lerp(MinJitter, MaxJitter, ExpProgress) * Stream.GetUnitVector() * Stream.FRandRange(MinJitterVariance, MaxJitterVariance);
			double ArmWidth = FMath::Lerp(MinScale,MaxScale, ExpProgress) * Stream.FRandRange(MinScaleVariance, MaxScaleVariance);
			
			FVector Radius(
				ArmWidth,
				ArmWidth,
				ArmWidth * GalaxyParams.ArmHeightRatio
			);

			double DensityCoef = FMath::Lerp(GalaxyParams.ArmRadialDensityMin, GalaxyParams.ArmRadialDensityMax, FMath::Pow(Progress, GalaxyParams.ArmRadialDensityExponent)); //todo: May need a look
			GenerateCluster(ArmIndex + c + 123, Center, Radius, StarsPerCluster, GalaxyParams.ArmBaseDensity * DensityCoef, GalaxyParams.ArmDepthBias);
		}
	}

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Generate Arms duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::ApplyTwist()
{
	//double StartTime = FPlatformTime::Seconds();

	ParallelFor(GeneratedData.Num(), [&](int32 i)
		{
			FVector P = GeneratedData[i].Position;
			//double rXY = (P * FVector(1,1,0)).Length();
			double rXY = P.Length();
			if (rXY < KINDA_SMALL_NUMBER) return;
			double theta = FMath::Atan2(P.Y, P.X);
			double normalizedRadius = FMath::Clamp(rXY / MaxRadius, 0.0, 1.0);
			double baseDelta = GalaxyParams.TwistStrength * normalizedRadius;
			double coreBoost = GalaxyParams.TwistCoreStrength * -FMath::Exp((GalaxyParams.TwistCoreRadius / FMath::Max(FMath::Pow(normalizedRadius, GalaxyParams.TwistCoreTwistExponent), 1e-4)));// Need this to only ramp up on an inner well defined part of the spiral
			double deltaTheta = baseDelta + coreBoost;
			double newTheta = theta + deltaTheta;

			GeneratedData[i].Position.X = rXY * FMath::Cos(newTheta);
			GeneratedData[i].Position.Y = rXY * FMath::Sin(newTheta);
		}, EParallelForFlags::BackgroundPriority);

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Apply Twist duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::GenerateDisc()
{
	//double StartTime = FPlatformTime::Seconds();
	
	GenerateCluster(Seed + 999, FVector::ZeroVector, FVector(GalaxyRadius, GalaxyRadius, GalaxyRadius * GalaxyParams.DiscHeightRatio), GalaxyParams.DiscNumPoints, GalaxyParams.DiscBaseDensity, GalaxyParams.DiscDepthBias);
	
	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Generate Disc duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::GenerateBackground()
{
	//double StartTime = FPlatformTime::Seconds();

	GenerateCluster(Seed + 123456789, FVector::ZeroVector, FVector(MaxRadius, MaxRadius, MaxRadius * GalaxyParams.BackgroundHeightRatio), GalaxyParams.BackgroundNumPoints, GalaxyParams.BackgroundBaseDensity, GalaxyParams.BackgroundDepthBias);

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Generate Background duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::ApplyRotation() {
	//double StartTime = FPlatformTime::Seconds();
	
	ParallelFor(GeneratedData.Num(), [&](int32 i)
		{
			GeneratedData[i].Position = RotateCoordinate(GeneratedData[i].Position, Rotation);
		}
	);

	//double TotalDuration = FPlatformTime::Seconds() - StartTime;
	//UE_LOG(LogTemp, Log, TEXT("GalaxyGenerator::Apply Rotation duration: %.3f seconds "), TotalDuration);
}

void GalaxyGenerator::GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity, double InDepthBias) //add falloff or curve param
{
	int32 StartIndex = GeneratedData.Num();
	GeneratedData.AddUninitialized(InCount);

	ParallelFor(InCount, [&](int32 i)
		{
			FRandomStream Stream(InSeed ^ (StartIndex + i)); // unique seed per point

			auto Gaussian = [&](FRandomStream& Rand)
				{
					double U1 = FMath::Max(Rand.FRand(), KINDA_SMALL_NUMBER);
					double U2 = Rand.FRand();
					double R = FMath::Sqrt(-2.0 * FMath::Loge(U1));
					double Theta = 2.0 * PI * U2;
					return R * FMath::Cos(Theta);
				};

			FVector Offset(
				Gaussian(Stream) * InClusterRadius.X,
				Gaussian(Stream) * InClusterRadius.Y,
				Gaussian(Stream) * InClusterRadius.Z
			);

			FVector P = InClusterCenter + Offset;
			double size = P.Length();

			FPointData InsertData;
			InsertData.Position = (size < MaxRadius ? P : FVector::ZeroVector);
			InsertData.InsertDepth = ChooseDepth(Stream.FRand(), InDepthBias);
			InsertData.Data = FVoxelData(InBaseDensity * Stream.FRandRange(.5, 1.5), Stream.GetUnitVector(), i + StartIndex, 1);

			GeneratedData[StartIndex + i] = InsertData;
		}, EParallelForFlags::BackgroundPriority);
}

int GalaxyGenerator::ChooseDepth(double InRandomSample, double InDepthBias)
{
	double biasedSample = FMath::Clamp(FMath::Pow(InRandomSample, InDepthBias),0,1);
	double cumulative = 0.0;
	int chosenDepth = 6;
	for (int d = 0; d < 7; ++d)
	{
		cumulative += DepthProb[d];
		if (biasedSample <= cumulative)
		{
			chosenDepth = d;
			break;
		}
	}
	return FMath::Clamp(MaxInsertionDepth - chosenDepth, MinInsertionDepth, MaxInsertionDepth);
}
#pragma endregion
#pragma endregion

#pragma region UniverseGenerator
//BEGIN UNIVERSE GENERATOR
void UniverseGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	MaxInsertionDepth = FMath::Max(1, InOctree->MaxDepth - InsertDepthOffset);
	MinInsertionDepth = FMath::Max(1, MaxInsertionDepth - DepthRange);

	GeneratedData.Empty();

	// Pre-allocate with extra space to avoid race conditions
	GeneratedData.AddUninitialized(UniverseParams.Count); // Double size for safety
	auto LocalNoise = FastNoise::NewFromEncodedNodeTree(UniverseParams.EncodedTree);

	ParallelFor(UniverseParams.Count, [&](int32 i) {
		FRandomStream LocalStream(Seed + 4713 + i * 10007);
		bool PointInserted = false;
		while (!PointInserted) {
			FVector PointCenter(
				LocalStream.FRandRange(-UniverseParams.Extent, UniverseParams.Extent),
				LocalStream.FRandRange(-UniverseParams.Extent, UniverseParams.Extent),
				LocalStream.FRandRange(-UniverseParams.Extent, UniverseParams.Extent)
			);

			const double ScaleFactor = 1.0 / static_cast<double>(UniverseParams.Extent);
			const double NX = PointCenter.X * ScaleFactor;
			const double NY = PointCenter.Y * ScaleFactor;
			const double NZ = PointCenter.Z * ScaleFactor;


			const double RadialWeight = FMath::Pow(1.0 - FMath::Clamp(PointCenter.Size() / UniverseParams.Extent, 0.0, 1.0), .3);

			const double NoiseVal = LocalStream.FRand();
			const double NoiseSample = FMath::Pow(LocalNoise->GenSingle3D(NX, NY, NZ, Seed), 2.0) * RadialWeight;

			if (NoiseVal >= NoiseSample)
			{
				continue; // Reject this point
			}

			FPointData InsertData;
			InsertData.Data = FVoxelData(LocalStream.FRandRange(0.5, 1.5), LocalStream.GetUnitVector(), i, 1);
			InsertData.Position = PointCenter;
			InsertData.InsertDepth = ChooseDepth(LocalStream.FRand(), 1 - NoiseSample * 0.9);
			InsertData.Data.ObjectId = i;
			GeneratedData[i] = InsertData;
			PointInserted = true;
		}
	}, EParallelForFlags::BackgroundPriority);
}

void UniverseGenerator::GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity, double InDepthBias) //add falloff or curve param
{
	int32 StartIndex = GeneratedData.Num();
	GeneratedData.AddUninitialized(InCount);

	ParallelFor(InCount, [&](int32 i)
		{
			FRandomStream Stream(InSeed ^ (StartIndex + i)); // unique seed per point

			auto Gaussian = [&](FRandomStream& Rand)
				{
					double U1 = FMath::Max(Rand.FRand(), KINDA_SMALL_NUMBER);
					double U2 = Rand.FRand();
					double R = FMath::Sqrt(-2.0 * FMath::Loge(U1));
					double Theta = 2.0 * PI * U2;
					return R * FMath::Cos(Theta);
				};

			FVector Offset(
				Gaussian(Stream) * InClusterRadius.X,
				Gaussian(Stream) * InClusterRadius.Y,
				Gaussian(Stream) * InClusterRadius.Z
			);

			FVector P = InClusterCenter + Offset;
			double size = P.Length();

			FPointData InsertData;
			InsertData.Position = (size < UniverseParams.Extent ? P : FVector::ZeroVector);
			InsertData.InsertDepth = ChooseDepth(Stream.FRand(), InDepthBias);
			InsertData.Data = FVoxelData(InBaseDensity * Stream.FRandRange(.5, 1.5), FVector(Stream.FRandRange(.2,1.2), Stream.FRandRange(.2, 1.2), Stream.FRandRange(.2, 1.2)), i + StartIndex, 1);

			GeneratedData[StartIndex + i] = InsertData;
		}, EParallelForFlags::BackgroundPriority);
}

int UniverseGenerator::ChooseDepth(double InRandomSample, double InDepthBias)
{
	double biasedSample = FMath::Clamp(FMath::Pow(InRandomSample, InDepthBias), 0, 1);
	double cumulative = 0.0;
	int chosenDepth = 6;
	for (int d = 0; d < 7; ++d)
	{
		cumulative += DepthProb[d];
		if (biasedSample <= cumulative)
		{
			chosenDepth = d;
			break;
		}
	}
	return FMath::Clamp(MaxInsertionDepth - chosenDepth, MinInsertionDepth, MaxInsertionDepth);
}
//END UNIVERSE GENERATOR
#pragma endregion

#pragma region ExampleGenerators
void SimpleRandomGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	int64 Extent = InOctree->Extent;
	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(InOctree->MaxDepth - DepthRange, 1);

	TArray<FInt64Vector> InsertPositions;
	TArray<int32> InsertDepths;
	TArray<FVoxelData> InsertPayloads;
	ParallelFor(Count, [this, InOctree](int32 i)
		{
			// Generate unique stream per point
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			// Generate random coordinates
			int64 X = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
			int64 Y = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
			int64 Z = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));

			auto InsertPosition = FInt64Vector(X, Y, Z);
			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			auto InsertData = FVoxelData(Stream.FRand(), Stream.GetUnitVector(), i, Type); //For now just placing the index in ObectId, will probably use it to map to object types

			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		});
}

void SimpleRandomNoiseGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(1, InOctree->MaxDepth - 8);

	//Noise from encoded node tree, use Fast Noise tool to generate encoded tree strings
	auto Noise = FastNoise::NewFromEncodedNodeTree(EncodedTree);

	ParallelFor(Count, [this, InOctree, Noise](int32 i)
		{
			//Base Position Step
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);
			int64 X = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
			int64 Y = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
			int64 Z = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));

			float OutDensity;
			FInt64Vector InsertPosition = ApplyNoiseDerivative(Noise, 1, InOctree->Extent, FInt64Vector(X, Y, Z), OutDensity);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			auto InsertData = FVoxelData(OutDensity, Stream.GetUnitVector(), i, Type);
			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		});
}

void GlobularGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(InOctree->MaxDepth - DepthRange, 1);

	ParallelFor(Count, [this, InOctree](int32 i)
		{
			// Generate unique stream per point
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			// Generate random coordinates
			FVector AxisScale = FVector(HorizontalExtent, HorizontalExtent, VerticalExtent);
			FVector Direction = Stream.GetUnitVector();
			double Distance = FMath::Pow(Stream.FRand(), Falloff) * InOctree->Extent;

			auto InsertVector = AxisScale * Direction * Distance;
			FInt64Vector InsertPosition = FInt64Vector(FMath::RoundToInt64(InsertVector.X), FMath::RoundToInt64(InsertVector.Y), FMath::RoundToInt64(InsertVector.Z));
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			auto InsertDepth = Stream.FRandRange(MinInsertionDepth, MaxInsertionDepth);
			auto InsertData = FVoxelData(Stream.FRand(), Stream.GetUnitVector(), i, Type);

			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		});
}

void GlobularNoiseGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;
	FRandomStream Stream(Seed);
	MaxInsertionDepth = FMath::Max(InOctree->MaxDepth - InsertDepthOffset, 1);
	MinInsertionDepth = FMath::Max(MaxInsertionDepth - DepthRange, 1); //Do not insert above level 1
	auto Noise = FastNoise::NewFromEncodedNodeTree(EncodedTree);

	ParallelFor(Count, [this, InOctree, Noise](int32 i)
		{
			//for (int i = 0; i < Count; i++)
			//{
					// Generate unique stream per point
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			// Generate random coordinates
			FVector AxisScale = FVector(HorizontalExtent, HorizontalExtent, VerticalExtent) * .5;
			FVector Direction = Stream.GetUnitVector();
			double Distance = FMath::Pow(Stream.FRand(), Falloff) * InOctree->Extent;
			auto InsertCoeff = AxisScale * Direction;
			auto InsertVector = InsertCoeff * Distance;

			float OutDensity;
			FInt64Vector InsertPosition = FInt64Vector(FMath::RoundToInt64(InsertVector.X), FMath::RoundToInt64(InsertVector.Y), FMath::RoundToInt64(InsertVector.Z));
			InsertPosition = ApplyNoiseDerivative(Noise, 1, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);

			FLinearColor Color = FLinearColor::MakeRandomSeededColor(i);
			FVector Composition(Color.R, Color.G, Color.B);

			auto InsertData = FVoxelData(FMath::Max(HorizontalExtent * .8, InsertPosition.Size()), Composition, i, Type);
			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		}
	);
}

void SpiralGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;
	//FRandomStream Stream(Seed);
	int64 Extent = InOctree->Extent;

	// Proceduralize parameters before entering population loop
	HorizontalSpreadDistance = InOctree->Extent * HorizontalSpreadMax;
	VerticalSpreadDistance = InOctree->Extent * VerticalSpreadMax;
	RadialDistance = InOctree->Extent - HorizontalSpreadDistance; //Radius
	CenterDistance = CenterScale * RadialDistance; //Distance from center spiral begins
	PitchAngleRadians = FMath::DegreesToRadians(PitchAngle);
	MaxTheta = FMath::Loge(RadialDistance / CenterDistance) / FMath::Tan(PitchAngleRadians);

	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(InOctree->MaxDepth - DepthRange, 1); //Do not insert above level 1

	ParallelFor(Count, [this, InOctree](int32 i)
		{
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			int32 ArmIndex = i % NumArms;
			double ArmPhaseOffset = (2.0 * PI * ArmIndex) / NumArms;
			double Lambda = 1; // tweak to tune central density
			double T = (1 - FMath::Exp(-Stream.FRand() / Lambda)) / (1 - FMath::Exp(-1.0 / Lambda));
			double BaseTheta = T * MaxTheta;

			// Spiral equation
			double Radius = CenterDistance * FMath::Exp(BaseTheta * FMath::Tan(PitchAngleRadians));

			// Arm perturbation
			double PerturbStrength = ArmContrast * 0.5;
			double Perturb = PerturbStrength * FMath::Sin(BaseTheta * 2.0);

			double FinalTheta = BaseTheta + ArmPhaseOffset + Perturb;

			double X = Radius * FMath::Cos(FinalTheta);
			double Y = Radius * FMath::Sin(FinalTheta);
			double Z = 0;

			FVector JitterDirection = Stream.GetUnitVector();
			double HorizontalFalloff = Stream.FRand() * FMath::Lerp(HorizontalSpreadMin, HorizontalSpreadMax, T);
			double VerticalFalloff = Stream.FRand() * FMath::Lerp(VerticalSpreadMin, VerticalSpreadMax, T);

			FVector JitterOffset;
			JitterOffset.X = JitterDirection.X * InOctree->Extent * HorizontalFalloff;
			JitterOffset.Y = JitterDirection.Y * InOctree->Extent * HorizontalFalloff;
			JitterOffset.Z = JitterDirection.Z * InOctree->Extent * VerticalFalloff;

			X += JitterOffset.X;
			Y += JitterOffset.Y;
			Z += JitterOffset.Z;

			FInt64Vector InsertPosition(
				FMath::RoundToInt64(X),
				FMath::RoundToInt64(Y),
				FMath::RoundToInt64(Z)
			);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			auto InsertDepth = FMath::Lerp(MinInsertionDepth, MaxInsertionDepth, Stream.FRand());//  Stream.FRandRange(MinInsertionDepth, MaxInsertionDepth); //If a different scale distribution is wanted can change the way depth is randomized
			auto InsertData = FVoxelData(Stream.FRand() * FMath::Pow(FMath::Max(T - .3, 0.000001), 1), Stream.GetUnitVector().GetAbs(), i, Type);
			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		}, EParallelForFlags::BackgroundPriority);
}

void SpiralNoiseGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;
	FRandomStream Stream(Seed);
	int64 Extent = InOctree->Extent;

	// Proceduralize parameters before entering population loop
	MaxInsertionDepth = FMath::Max(InOctree->MaxDepth - InsertDepthOffset, 1);
	MinInsertionDepth = FMath::Max(MaxInsertionDepth - DepthRange, 1); //Do not insert above level 1
	HorizontalSpreadDistance = InOctree->Extent * HorizontalSpreadMax;
	VerticalSpreadDistance = InOctree->Extent * VerticalSpreadMax;
	RadialDistance = InOctree->Extent - HorizontalSpreadDistance; //Radius
	CenterDistance = CenterScale * RadialDistance; //Distance from center spiral begins
	PitchAngleRadians = FMath::DegreesToRadians(PitchAngle);
	MaxTheta = FMath::Loge(RadialDistance / CenterDistance) / FMath::Tan(PitchAngleRadians);

	TArray<FInt64Vector> InsertPositions;
	TArray<int32> InsertDepths;
	TArray<FVoxelData> InsertPayloads;
	InsertPositions.SetNumZeroed(Count);
	InsertDepths.SetNumZeroed(Count);
	InsertPayloads.SetNumZeroed(Count);

	auto Noise = FastNoise::NewFromEncodedNodeTree(EncodedTree);
	ParallelFor(Count, [this, &InsertPositions, &InsertDepths, &InsertPayloads, Extent, Noise, InOctree](int32 i)
		{
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			int32 ArmIndex = i % NumArms;
			double ArmPhaseOffset = (2.0 * PI * ArmIndex) / NumArms;
			double Lambda = 1; // tweak to tune central density
			double T = (1 - FMath::Exp(-Stream.FRand() / Lambda)) / (1 - FMath::Exp(-1.0 / Lambda));
			double BaseTheta = T * MaxTheta;

			// Spiral equation
			double Radius = CenterDistance * FMath::Exp(BaseTheta * FMath::Tan(PitchAngleRadians));

			// Arm perturbation
			double PerturbStrength = ArmContrast * 0.5;
			double Perturb = PerturbStrength * FMath::Sin(BaseTheta * 2.0);

			double FinalTheta = BaseTheta + ArmPhaseOffset + Perturb;

			double X = Radius * FMath::Cos(FinalTheta);
			double Y = Radius * FMath::Sin(FinalTheta);
			double Z = 0;

			FVector JitterDirection = Stream.GetUnitVector();
			double HorizontalFalloff = Stream.FRand() * FMath::Lerp(HorizontalSpreadMin, HorizontalSpreadMax, T);
			double VerticalFalloff = Stream.FRand() * FMath::Lerp(VerticalSpreadMin, VerticalSpreadMax, T);

			FVector JitterOffset;
			JitterOffset.X = JitterDirection.X * Extent * HorizontalFalloff;
			JitterOffset.Y = JitterDirection.Y * Extent * HorizontalFalloff;
			JitterOffset.Z = JitterDirection.Z * Extent * VerticalFalloff;

			X += JitterOffset.X;
			Y += JitterOffset.Y;
			Z += JitterOffset.Z;

			float OutDensity;
			FInt64Vector InsertPosition(
				FMath::RoundToInt64(X),
				FMath::RoundToInt64(Y),
				FMath::RoundToInt64(Z)
			);
			InsertPosition = ApplyNoiseDerivative(Noise, 2, Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);
			FLinearColor Color = FLinearColor::MakeRandomSeededColor(i);
			FVector Composition(Color.R, Color.G, Color.B);
			InOctree->InsertPosition(InsertPosition, Stream.RandRange(MinInsertionDepth, MaxInsertionDepth), FVoxelData(Stream.FRand() * FMath::Pow(FMath::Max(T, 0.000001), 6), Composition, i, Type));
		}
	, EParallelForFlags::BackgroundPriority);
}

void BurstGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	int64 Extent = InOctree->Extent;
	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(1, InOctree->MaxDepth - DepthRange);

	// Setup FastNoise2 graph
	auto Noise = FastNoise::New<FastNoise::CellularDistance>();
	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Noise);
	SeedOffset->SetOffset(Seed);

	// Domain scaling consistent with extent
	double OriginalScale = 2;
	double DomainScale = OriginalScale / static_cast<double>(Extent);

	ParallelFor(Count, [this, InOctree, SeedOffset, DomainScale](int32 i)
		{
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			// Sample a uniform random point
			FVector3d UniformPoint;
			UniformPoint.X = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);
			UniformPoint.Y = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);
			UniformPoint.Z = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);

			// Normalize for noise input
			double NX = UniformPoint.X * DomainScale;
			double NY = UniformPoint.Y * DomainScale;
			double NZ = UniformPoint.Z * DomainScale;

			// Evaluate noise and remap to [0,1] with emphasis
			double NoiseValue = FMath::Pow(SeedOffset->GenSingle3D(NX, NY, NZ, 0), 3);

			// Warp point toward noise "center" (here we use origin, but could sample gradient or add attractors later)
			FVector3d WarpedPoint = FMath::Lerp(UniformPoint, FVector3d::ZeroVector, 1.0 - NoiseValue);

			// Snap to voxel
			auto Coord = FInt64Vector(
				FMath::RoundToInt64(WarpedPoint.X),
				FMath::RoundToInt64(WarpedPoint.Y),
				FMath::RoundToInt64(WarpedPoint.Z)
			);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			FVoxelData Data(NoiseValue, Stream.GetUnitVector(), i, Type);
			Coord = RotateCoordinate(Coord, Rotation);
			InOctree->InsertPosition(Coord, InsertDepth, Data);
		});
}

void BurstNoiseGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	int64 Extent = InOctree->Extent;
	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(1, InOctree->MaxDepth - DepthRange);

	// Setup FastNoise2 graph
	auto Noise = FastNoise::New<FastNoise::CellularDistance>();
	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Noise);
	SeedOffset->SetOffset(Seed);

	// Domain scaling consistent with extent
	double OriginalScale = 1;
	double DomainScale = OriginalScale / static_cast<double>(Extent);

	auto DistortionNoise = FastNoise::NewFromEncodedNodeTree(EncodedTree);

	ParallelFor(Count, [this, InOctree, SeedOffset, DomainScale, Noise, DistortionNoise](int32 i)
		{
			int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
			FRandomStream Stream(HashedSeed);

			// Sample a uniform random point
			FVector3d UniformPoint;
			UniformPoint.X = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);
			UniformPoint.Y = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);
			UniformPoint.Z = Stream.FRandRange(-InOctree->Extent, InOctree->Extent);

			// Normalize for noise input
			double NX = UniformPoint.X * DomainScale;
			double NY = UniformPoint.Y * DomainScale;
			double NZ = UniformPoint.Z * DomainScale;

			// Evaluate noise and remap to [0,1] with emphasis
			double NoiseValue = FMath::Pow(SeedOffset->GenSingle3D(NX, NY, NZ, 0), 3);

			// Warp point toward noise "center" (here we use origin, but could sample gradient or add attractors later)
			FVector3d WarpedPoint = FMath::Lerp(UniformPoint, FVector3d::ZeroVector, 1.0 - NoiseValue);

			// Snap to voxel
			float OutDensity;
			auto InsertPosition = FInt64Vector(
				FMath::RoundToInt64(WarpedPoint.X),
				FMath::RoundToInt64(WarpedPoint.Y),
				FMath::RoundToInt64(WarpedPoint.Z)
			);
			InsertPosition = ApplyNoiseDerivative(DistortionNoise, 6, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = MaxInsertionDepth;// GalaxyGenerator::ChooseDepth(Stream.FRand());
			FVoxelData Data(OutDensity, Stream.GetUnitVector(), i, Type);

			InOctree->InsertPosition(InsertPosition, InsertDepth, Data);
		});
}
#pragma endregion
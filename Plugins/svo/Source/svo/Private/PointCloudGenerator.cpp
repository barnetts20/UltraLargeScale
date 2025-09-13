// Fill out your copyright notice in the Description page of Project Settings.


#include "PointCloudGenerator.h"

//Applies noise derivative offset to a sample position
FInt64Vector PointCloudGenerator::ApplyNoise(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Vector InSamplePosition, float& OutDensity)
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

//Example Generators
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
			FInt64Vector InsertPosition = ApplyNoise(Noise, 1, InOctree->Extent, FInt64Vector(X, Y, Z), OutDensity);

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
			InsertPosition = ApplyNoise(Noise, 1, InOctree->Extent, InsertPosition, OutDensity);
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
		InsertPosition = ApplyNoise(Noise, 2, Extent, InsertPosition, OutDensity);
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
			InsertPosition = ApplyNoise(DistortionNoise, 6, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = MaxInsertionDepth;// GalaxyGenerator::ChooseDepth(Stream.FRand());
			FVoxelData Data(OutDensity, Stream.GetUnitVector(), i, Type);

			InOctree->InsertPosition(InsertPosition, InsertDepth, Data);
		});
}

//BEGIN GALAXY GENERATOR
//Param Factory - Randomization Bounds are controlled here
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
	Params.ArmIncoherence = Stream.FRandRange(MinParams.ArmIncoherence, MaxParams.ArmIncoherence);
	Params.ArmNumArms = Stream.RandRange(MinParams.ArmNumArms, MaxParams.ArmNumArms);
	Params.ArmNumPoints = Stream.RandRange(MinParams.ArmNumPoints, MaxParams.ArmNumPoints);
	Params.ArmRadialDensityExponent = Stream.FRandRange(MinParams.ArmRadialDensityExponent, MaxParams.ArmRadialDensityExponent);
	Params.ArmRadialDensityMultiplier = Stream.FRandRange(MinParams.ArmRadialDensityMultiplier, MaxParams.ArmRadialDensityMultiplier);
	Params.ArmSpreadFactor = Stream.FRandRange(MinParams.ArmSpreadFactor, MaxParams.ArmSpreadFactor);
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

//Galaxy Generator - Generates and inserts the data
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

	//Can tune these to create different classes maybe
	GenerateArms();
	//if (IsDestroying) return;
	ApplyTwist();
	//if (IsDestroying) return;
	GenerateClusters();
	//if (IsDestroying) return;
	GenerateBulge();
	//if (IsDestroying) return;
	GenerateDisc();
	//if (IsDestroying) return;
	GenerateBackground();
	//if (IsDestroying) return;

	//TODO: May want to apply noise to some components
	//TODO: Might be worth while to create a specific gas distribution and explicitly insert it at the desired depth for the volume texture

	//Accumulate Zero Vectors and Void stars into black hole
	FVoxelData BlackHole;
	BlackHole.ObjectId = INT32_MAX;
	BlackHole.TypeId = 3;
	
	ParallelFor(GeneratedData.Num(), [this, InOctree, &BlackHole](int32 i)
	{
			FRandomStream Stream(i);
			FPointData InsertData = GeneratedData[i];

			if (InsertData.Position.Size() < VoidRadius) { //Any insert failures or nodes within void radius get fed to the black hole
				BlackHole.Density += InsertData.Data.Density;
				BlackHole.Composition += InsertData.Data.Composition;
			}
			else {
				InsertData.Position = RotateCoordinate(InsertData.Position, Rotation);
				InOctree->InsertPosition(InsertData.GetInt64Position(), InsertData.InsertDepth, InsertData.Data); //Need to make insert accummulate density for repeated inserts, and then we can use the zero vectors to populate the black hole
			}
	});
	BlackHole.Density = 1; // Should be accumulated below but that affects gas distribution, hard setting for now, can switch back if we do an explicit gas population stage

	//TODO: calculate black hole size based on solar mass density and figure out the depth, for now inserting at min depth
	InOctree->InsertPosition(FInt64Vector::ZeroValue, MinInsertionDepth, BlackHole); //Should insert at different depth for black hole
}
void GalaxyGenerator::GenerateBulge()
{
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

	ParallelFor(NumPoints, [&](int32 i)
		{
			// Each thread gets its own random stream with unique seed
			FRandomStream Stream((Seed + 99) * 1664525U + i * 1013904223U);

			FPointData& InsertData = GeneratedData[StartIndex + i];
			InsertData.Data.ObjectId = i;
			InsertData.Data.TypeId = 1;
			InsertData.Data.Density = Stream.FRandRange(0.5f, 1.5f) * GalaxyParams.BulgeBaseDensity;
			InsertData.Data.Composition = Stream.GetUnitVector();
			InsertData.InsertDepth = ChooseDepth(Stream.FRand(), GalaxyParams.BulgeDepthBias);

			// 1. Sample Hernquist radius
			double sqrtu = FMath::Sqrt(FMath::Clamp(Stream.FRand(), KINDA_SMALL_NUMBER, 1.0 - KINDA_SMALL_NUMBER));
			double r = (a * sqrtu) / (1.0 - sqrtu);

			// 2. Apply soft truncation with probabilistic acceptance
			if (r > SoftTruncationRadius)
			{
				if (r > MaxRadius) // Failed points get fed to the black hole
				{
					InsertData.Position = FVector::ZeroVector;
					return;
				}

				// Soft truncation zone - probabilistic acceptance
				double excessRadius = r - SoftTruncationRadius;
				double falloffFactor = excessRadius / TruncationZone; // 0 to 1

				// Exponential falloff probability
				double acceptanceProbability = FMath::Exp(-3.0 * FMath::Pow(falloffFactor, GalaxyParams.BulgeAcceptanceExponent));
				if (Stream.FRand() > acceptanceProbability) // Failed points get fed to the black hole
				{
					InsertData.Position = FVector::ZeroVector;
					return;
				}
			}

			// 3. Sample isotropic direction
			double z = Stream.FRandRange(-1.0, 1.0);
			double phi = Stream.FRandRange(0.0, 2.0 * PI);
			double xy = FMath::Sqrt(1.0 - z * z);
			FVector dir(xy * FMath::Cos(phi), xy * FMath::Sin(phi), z);

			// 4. Scale per axis
			FVector P;
			P.X = dir.X * r * AxisScaleX;
			P.Y = dir.Y * r * AxisScaleY;
			P.Z = dir.Z * r * AxisScaleZ;

			P += Stream.GetUnitVector() * Stream.FRand() * BulgeRadius * GalaxyParams.BulgeJitter;

			InsertData.Position = P;
		});
}
void GalaxyGenerator::GenerateClusters()
{
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
}
void GalaxyGenerator::GenerateArms()
{
	int StarsPerCluster = (GalaxyParams.ArmNumPoints / GalaxyParams.ArmNumArms) / GalaxyParams.ArmClusters;
	double Rd = GalaxyRadius;
	double BaseWidth = GalaxyRadius * GalaxyParams.ArmClusterRadiusMin;
	double WidthScale = GalaxyRadius * GalaxyParams.ArmClusterRadiusMax;
	FRandomStream Stream(Seed + 1337);
	for (int ArmIndex = 0; ArmIndex < GalaxyParams.ArmNumArms; ArmIndex++)
	{
		double BaseAngle = (2.0 * PI * ArmIndex) / GalaxyParams.ArmNumArms;
		FVector ArmDir(FMath::Cos(BaseAngle), FMath::Sin(BaseAngle), 0);
		for (int c = 0; c < GalaxyParams.ArmClusters; c++)
		{
			double u = Stream.FRand();
			double Dist = -Rd * FMath::Loge(1.0 - u);
			Dist = FMath::Clamp(Dist, BulgeRadius * GalaxyParams.ArmStartRatio, GalaxyRadius);
			Dist = FMath::Clamp(Dist, 0, GalaxyRadius);
			FVector Center = ArmDir * Dist;
			double ArmWidth = BaseWidth + WidthScale * (Dist / GalaxyRadius);
			FVector Radius(
				ArmWidth * GalaxyParams.ArmSpreadFactor,
				ArmWidth * GalaxyParams.ArmSpreadFactor,
				ArmWidth * GalaxyParams.ArmHeightRatio * GalaxyParams.ArmSpreadFactor
			);
			FVector Jitter(
				Stream.FRandRange(-Radius.X, Radius.X) * GalaxyParams.ArmIncoherence,
				Stream.FRandRange(-Radius.Y, Radius.Y) * GalaxyParams.ArmIncoherence,
				Stream.FRandRange(-Radius.Z, Radius.Z) * GalaxyParams.ArmIncoherence
			);
			Center += Jitter;
			double DensityCoef = FMath::Pow(Center.Length() / GalaxyRadius, GalaxyParams.ArmRadialDensityExponent) * GalaxyParams.ArmRadialDensityMultiplier + GalaxyParams.ArmRadialBaseDensity;
			GenerateCluster(ArmIndex + c + 123, Center, Radius, StarsPerCluster, GalaxyParams.ArmBaseDensity * DensityCoef, GalaxyParams.ArmDepthBias);
		}
	}
}
void GalaxyGenerator::ApplyTwist()
{
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
}
void GalaxyGenerator::GenerateDisc()
{
	GenerateCluster(Seed + 999, FVector::ZeroVector, FVector(GalaxyRadius, GalaxyRadius, GalaxyRadius * GalaxyParams.DiscHeightRatio), GalaxyParams.DiscNumPoints, GalaxyParams.DiscBaseDensity, GalaxyParams.DiscDepthBias);
}
void GalaxyGenerator::GenerateBackground()
{
	GenerateCluster(Seed + 123456789, FVector::ZeroVector, FVector(MaxRadius, MaxRadius, MaxRadius * GalaxyParams.BackgroundHeightRatio), GalaxyParams.BackgroundNumPoints, GalaxyParams.BackgroundBaseDensity, GalaxyParams.BackgroundDepthBias);
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
			InsertData.Data = FVoxelData(InBaseDensity * Stream.FRandRange(.5, 1.5), Stream.GetUnitVector(), i, 1);

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
	return FMath::Clamp(MaxInsertionDepth-chosenDepth, MinInsertionDepth, MaxInsertionDepth);
}
void GalaxyGenerator::MarkDestroying()
{
	IsDestroying = true;
}
//END GALAXY GENERATOR

//BEGIN UNIVERSE GENERATOR
//TODO: IMPLEMENT
//END UNIVERSE GENERATOR



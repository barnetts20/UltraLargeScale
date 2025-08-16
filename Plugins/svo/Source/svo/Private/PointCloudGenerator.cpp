// Fill out your copyright notice in the Description page of Project Settings.


#include "PointCloudGenerator.h"

//Applies noise derivative offset to a sample position
FInt64Coordinate PointCloudGenerator::ApplyNoise(FastNoise::SmartNode<> InNoise, double InDomainScale, int64 InExtent, FInt64Coordinate InSamplePosition, float& OutDensity)
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
	auto InsertPosition = FInt64Coordinate(FX, FY, FZ);
	return InsertPosition;
}
FInt64Coordinate PointCloudGenerator::RotateCoordinate(FInt64Coordinate InCoordinate, FRotator InRotation)
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
	return FInt64Coordinate(
		FMath::RoundToInt64(Rotated.X),
		FMath::RoundToInt64(Rotated.Y),
		FMath::RoundToInt64(Rotated.Z)
	);
}

void SimpleRandomGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	if (!InOctree) return;

	int64 Extent = InOctree->Extent;
	MaxInsertionDepth = InOctree->MaxDepth;
	MinInsertionDepth = FMath::Max(InOctree->MaxDepth - DepthRange, 1);

	ParallelFor(Count, [this, InOctree](int32 i)
	{
		// Generate unique stream per point
		int32 HashedSeed = FCrc::MemCrc32(&i, sizeof(i), Seed);
		FRandomStream Stream(HashedSeed);

		// Generate random coordinates
		int64 X = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
		int64 Y = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));
		int64 Z = FMath::RoundToInt64(Stream.FRandRange(-InOctree->Extent, InOctree->Extent));

		auto InsertPosition = FInt64Coordinate(X, Y, Z);
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
			FInt64Coordinate InsertPosition = ApplyNoise(Noise, 1, InOctree->Extent, FInt64Coordinate(X, Y, Z), OutDensity);

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
		FInt64Coordinate InsertPosition = FInt64Coordinate(FMath::RoundToInt64(InsertVector.X), FMath::RoundToInt64(InsertVector.Y), FMath::RoundToInt64(InsertVector.Z));
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
			FInt64Coordinate InsertPosition = FInt64Coordinate(FMath::RoundToInt64(InsertVector.X), FMath::RoundToInt64(InsertVector.Y), FMath::RoundToInt64(InsertVector.Z));
			InsertPosition = ApplyNoise(Noise, 1, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			auto InsertData = FVoxelData(FMath::Pow(InsertCoeff.Length(), 3.0), Stream.GetUnitVector().GetAbs(), i, Type); // May need to add some way to control typing. for instance galaxy tree could contain stars, blackholes, gas all at different depths
			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		});
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

		FInt64Coordinate InsertPosition(
			FMath::RoundToInt64(X), 
			FMath::RoundToInt64(Y), 
			FMath::RoundToInt64(Z)
		);
		InsertPosition = RotateCoordinate(InsertPosition, Rotation);

		auto InsertDepth = FMath::Lerp(MinInsertionDepth, MaxInsertionDepth, Stream.FRand());//  Stream.FRandRange(MinInsertionDepth, MaxInsertionDepth); //If a different scale distribution is wanted can change the way depth is randomized
		auto InsertData = FVoxelData(Stream.FRand() * FMath::Pow(FMath::Max(T - .3, 0.000001), 1), Stream.GetUnitVector().GetAbs(), i, Type);
		InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
	});
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

	auto Noise = FastNoise::NewFromEncodedNodeTree(EncodedTree);
	ParallelFor(Count, [this, InOctree, Noise](int32 i)
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

			float OutDensity;
			FInt64Coordinate InsertPosition(
				FMath::RoundToInt64(X),
				FMath::RoundToInt64(Y),
				FMath::RoundToInt64(Z)
			);
			InsertPosition = ApplyNoise(Noise, 2, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			auto InsertData = FVoxelData(Stream.FRand() * FMath::Pow(FMath::Max(T, 0.000001), 6), Stream.GetUnitVector(), i, Type);
			InOctree->InsertPosition(InsertPosition, InsertDepth, InsertData);
		});
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
		auto Coord = FInt64Coordinate(
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
			auto InsertPosition = FInt64Coordinate(
				FMath::RoundToInt64(WarpedPoint.X),
				FMath::RoundToInt64(WarpedPoint.Y),
				FMath::RoundToInt64(WarpedPoint.Z)
			);
			InsertPosition = ApplyNoise(DistortionNoise, 6, InOctree->Extent, InsertPosition, OutDensity);
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			FVoxelData Data(OutDensity, Stream.GetUnitVector(), i, Type);

			InOctree->InsertPosition(InsertPosition, InsertDepth, Data);
		});
}



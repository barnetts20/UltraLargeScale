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
	//for(int i = 0; i < Count; i++)
	//{
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
		//InsertPositions[i] = InsertPosition;
		//InsertDepths[i] = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
		//InsertPayloads[i] = FVoxelData(Stream.FRand() * FMath::Pow(FMath::Max(T, 0.000001), 6), Stream.GetUnitVector(), i, Type);
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

			int32 InsertDepth = Stream.RandRange(MinInsertionDepth, MaxInsertionDepth);
			FVoxelData Data(OutDensity, Stream.GetUnitVector(), i, Type);

			InOctree->InsertPosition(InsertPosition, InsertDepth, Data);
		});
}

void GalaxyGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	//TODO: Proceduralize
	 
	//Randomize class and use for parameter bounds? 
	//Glob - E0 E3 E5 E7 S0
	//Spiral - Sa Sb Sc
	//Spiral barred - SBa SBb SBc

	MaxInsertionDepth = FMath::Max(1, InOctree->MaxDepth - InsertDepthOffset);
	MinInsertionDepth = FMath::Max(1, MaxInsertionDepth - DepthRange);

	MaxRadius = InOctree->Extent;
	GalaxyRadius = MaxRadius * .3; //Galaxy proper radius
	BulgeRadius = GalaxyRadius * .3; //Central bulge radius
	VoidRadius = BulgeRadius * .1; //Central void
	NumArms = 2;
	double TwistFactor = 8; // Need work unifying original functionality with center spiral tightening

	int NumDiscPoints = 100000;
	int NumArmPoints = 300000;
	int NumBulgePoints = 300000;
	int NumBackgroundPoints = 100000;

	//Can tune these to create different classes maybe
	GenerateArms(GeneratedPoints, NumArmPoints);
	ApplyTwist(GeneratedPoints, TwistFactor);
	GenerateBulge(GeneratedPoints, NumBulgePoints);
	GenerateDisc(GeneratedPoints, NumDiscPoints);
	GenerateBackground(GeneratedPoints, NumBackgroundPoints);

	//May want to apply noise to some components

	//Might be worth while to create a specific gas distribution and insert it at the desired depth for the volume texture

	ParallelFor(GeneratedPoints.Num(), [this, InOctree](int32 i)
	{
			FRandomStream Stream(i);
			auto InsertPosition = FInt64Vector(
				FMath::RoundToInt64(GeneratedPoints[i].X),
				FMath::RoundToInt64(GeneratedPoints[i].Y),
				FMath::RoundToInt64(GeneratedPoints[i].Z)
			);
			FVoxelData Data(1, FVector(1, 1, 1), i, (InsertPosition != FInt64Vector(0,0,0) ? 1 : 2)); 
			InsertPosition = RotateCoordinate(InsertPosition, Rotation);
			InOctree->InsertPosition(InsertPosition, Stream.RandRange(MinInsertionDepth, MaxInsertionDepth), Data); //Need to make insert accummulate density for repeated inserts, and then we can use the zero vectors to populate the black hole
	});
}

void GalaxyGenerator::GenerateCluster(TArray<FVector>& InGeneratedPoints, FVector InClusterCenter, FVector InClusterRadius, int InCount)
{
	int32 StartIndex = InGeneratedPoints.Num();
	InGeneratedPoints.AddUninitialized(InCount);

	ParallelFor(InCount, [&](int32 i)
		{
			FRandomStream Stream(Seed ^ (StartIndex + i)); // unique seed per point

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

			// Bounds + void checks
			//TODO Calc size once
			if (P.Size() > MaxRadius || P.Size() < VoidRadius)
			{
				// If invalid, push to center (or mark)
				InGeneratedPoints[StartIndex + i] = FVector::ZeroVector;
			}
			else
			{
				InGeneratedPoints[StartIndex + i] = P;
			}
		}, EParallelForFlags::BackgroundPriority);
}

void GalaxyGenerator::GenerateBulge(TArray<FVector>& InGeneratedPoints, int NumBulgePoints)
{
	int NumClusters = 8;  // tweakable
	int StarsPerCluster = NumBulgePoints / NumClusters;
	FRandomStream Stream(Seed + 42);

	for (int c = 0; c < NumClusters; c++)
	{
		// Pick a cluster center near the origin
		FVector ClusterCenter(
			Stream.FRandRange(-BulgeRadius * .5, BulgeRadius * .5),
			Stream.FRandRange(-BulgeRadius * .5, BulgeRadius * .5),
			Stream.FRandRange(-BulgeRadius * .25, BulgeRadius * .25)
		);

		// Cluster radius (slightly varied, smaller than bulge)
		FVector ClusterRadius(
			BulgeRadius * Stream.FRandRange(0.3, .5),
			BulgeRadius * Stream.FRandRange(0.3, .5),
			BulgeRadius * Stream.FRandRange(0.3, .5)
		);

		GenerateCluster(InGeneratedPoints, ClusterCenter, ClusterRadius, StarsPerCluster);
	}
}

void GalaxyGenerator::GenerateDisc(TArray<FVector>& InGeneratedPoints, int NumDiskPoints)
{
	FVector Center(0, 0, 0);
	FVector Radius(GalaxyRadius, GalaxyRadius, GalaxyRadius * 0.1); // wide + thin

	GenerateCluster(InGeneratedPoints, Center, Radius, NumDiskPoints);
}

void GalaxyGenerator::GenerateArms(TArray<FVector>& InGeneratedPoints, int NumArmPoints)
{
	int ClustersPerArm = 60; // tweakable
	int StarsPerCluster = (NumArmPoints / NumArms) / ClustersPerArm; // tweakable
	float ArmSpread = 0.3f;   // jitter around arm line

	for (int ArmIndex = 0; ArmIndex < NumArms; ArmIndex++)
	{
		// Base angle for this arm
		double Angle = (2.0 * PI * ArmIndex) / NumArms;
		FVector ArmDir(FMath::Cos(Angle), FMath::Sin(Angle), 0);

		for (int c = 0; c < ClustersPerArm; c++)
		{
			// Radius along the arm line
			double T = (double)c / (ClustersPerArm - 1); // 0 → 1
			//double Dist = FMath::Lerp(VoidRadius + .5 * ArmSpread * GalaxyRadius, GalaxyRadius, T);
			double Dist = FMath::Lerp(BulgeRadius * .5, GalaxyRadius, T);

			// Center of this cluster
			FVector Center = ArmDir * Dist;

			// Cluster radius grows with distance
			FVector Radius(
				Dist * ArmSpread,
				Dist * ArmSpread,
				Dist * ArmSpread * .2 // thin in Z
			);

			// Add jitter so arms aren’t too perfect
			FVector Jitter(
				FMath::FRandRange(-Radius.X * 0.5, Radius.X * 0.5),
				FMath::FRandRange(-Radius.Y * 0.5, Radius.Y * 0.5),
				FMath::FRandRange(-Radius.Z * 0.5, Radius.Z * 0.5)
			);

			Center += Jitter;

			GenerateCluster(InGeneratedPoints, Center, Radius, StarsPerCluster);
		}
	}
}

void GalaxyGenerator::ApplyTwist(TArray<FVector>& InGeneratedPoints, double InTwistStrength)
{
	double BaseTwistStrength = InTwistStrength;
	double CoreRadius = 0.1;           // fraction of galaxy radius considered core
	double MaxCoreTwist = 6.0;         // extra radians of twist near center

	ParallelFor(InGeneratedPoints.Num(), [&](int32 i)
		{
			FVector P = InGeneratedPoints[i];

			double rXY = P.Length();// FMath::Sqrt(P.X * P.X + P.Y * P.Y);
			if (rXY < KINDA_SMALL_NUMBER)
				return;

			double theta = FMath::Atan2(P.Y, P.X);
			double normalizedRadius = FMath::Clamp(rXY / GalaxyRadius, 0.0, 1.0);

			// Normal outer twist
			double baseDelta = BaseTwistStrength * normalizedRadius;

			// Extra inner twist: smooth saturating function
			double coreBoost = MaxCoreTwist * (1.0 - FMath::Exp((CoreRadius / (normalizedRadius + 1e-4))));

			// Total twist
			double deltaTheta = baseDelta + coreBoost;

			double newTheta = theta + deltaTheta;

			InGeneratedPoints[i].X = rXY * FMath::Cos(newTheta);
			InGeneratedPoints[i].Y = rXY * FMath::Sin(newTheta);

		}, EParallelForFlags::BackgroundPriority);
}

void GalaxyGenerator::GenerateBackground(TArray<FVector>& InGeneratedPoints, int NumBackgroundPoints)
{
	GenerateCluster(InGeneratedPoints, FVector(0), FVector(MaxRadius, MaxRadius, MaxRadius * .6), NumBackgroundPoints);
}

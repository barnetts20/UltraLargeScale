#include "UniverseDataGenerator.h"

#pragma region UniverseGenerator
//BEGIN UNIVERSE GENERATOR
void UniverseDataGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	GeneratedData.Empty();

	// Pre-allocate with extra space to avoid race conditions
	GeneratedData.AddUninitialized(Params.Count);
	auto LocalNoise = FastNoise::NewFromEncodedNodeTree(Params.EncodedTree);

	ParallelFor(Params.Count, [&](int32 i) {
		FRandomStream LocalStream(Seed + 4713 + i * 10007);
		bool PointInserted = false;
		while (!PointInserted) {
			FVector PointCenter(
				LocalStream.FRandRange(-Params.Extent, Params.Extent),
				LocalStream.FRandRange(-Params.Extent, Params.Extent),
				LocalStream.FRandRange(-Params.Extent, Params.Extent)
			);

			const double ScaleFactor = 1.0 / static_cast<double>(Params.Extent);
			const double NX = PointCenter.X * ScaleFactor;
			const double NY = PointCenter.Y * ScaleFactor;
			const double NZ = PointCenter.Z * ScaleFactor;


			//const double RadialWeight = FMath::Pow(1.0 - FMath::Clamp(PointCenter.Size() / Params.Extent, 0.0, 1.0), .3);

			const double NoiseVal = LocalStream.FRand();
			const double NoiseSample = FMath::Pow(LocalNoise->GenSingle3D(NX, NY, NZ, Seed), 2.0);// *RadialWeight;

			if (NoiseVal >= NoiseSample)
			{
				continue; // Reject this point
			}

			double scale = FPointData::SampleScaleFromDistribution(Params.MinGalaxyScale, Params.MaxGalaxyScale, NoiseVal, Params.ScaleDistributionCurve);
			FPointData InsertData = FPointData::MakePointDataFromWorldScale(scale, Params.UnitScale, Params.Extent);
			InsertData.Data.GasDensity = LocalStream.FRandRange(0.5, 1.5);
			InsertData.Data.Composition = LocalStream.GetUnitVector();
			InsertData.Data.ObjectId = i;
			InsertData.Data.TypeId = 1;
			
			PointCenter *= FVector(
				1.0 + LocalStream.FRandRange(-Params.Jitter, Params.Jitter),
				1.0 + LocalStream.FRandRange(-Params.Jitter, Params.Jitter),
				1.0 + LocalStream.FRandRange(-Params.Jitter, Params.Jitter)
			);

			InsertData.SetPosition(PointCenter);

			GeneratedData[i] = InsertData;
			PointInserted = true;
		}
		}, EParallelForFlags::BackgroundPriority);
}
//END UNIVERSE GENERATOR
#pragma endregion

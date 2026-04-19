#include "UniverseDataGenerator.h"

#pragma region UniverseGenerator
//BEGIN UNIVERSE GENERATOR

namespace
{
	// Shared generation worker. Density sampling is abstracted via a callable so
	// the two public entry points can supply either the legacy octree path or
	// the new FDensityVolume path without duplicating the generation loop.
	template <typename TDensitySampler>
	void GenerateDataInternal(
		UniverseDataGenerator& Self,
		TDensitySampler&& SampleDensity)
	{
		Self.GeneratedData.Empty();

		// Pre-allocate with extra space to avoid race conditions
		Self.GeneratedData.AddUninitialized(Self.Params.Count);
		auto LocalNoise = FastNoise::NewFromEncodedNodeTree(Self.Params.EncodedTree);

		const int32 LocalSeed = Self.Seed;
		const FUniverseParams& LocalParams = Self.Params;

		ParallelFor(LocalParams.Count, [&](int32 i) {
			FRandomStream LocalStream(LocalSeed + 4713 + i * 10007);
			bool PointInserted = false;
			while (!PointInserted) {
				FVector PointCenter(
					LocalStream.FRandRange(-LocalParams.Extent, LocalParams.Extent),
					LocalStream.FRandRange(-LocalParams.Extent, LocalParams.Extent),
					LocalStream.FRandRange(-LocalParams.Extent, LocalParams.Extent)
				);

				const float areaDensity = SampleDensity(PointCenter);
				if (LocalStream.FRand() > areaDensity) continue;

				const double ScaleFactor = 1.0 / static_cast<double>(LocalParams.Extent);
				const double NX = PointCenter.X * ScaleFactor;
				const double NY = PointCenter.Y * ScaleFactor;
				const double NZ = PointCenter.Z * ScaleFactor;


				//const double RadialWeight = FMath::Pow(1.0 - FMath::Clamp(PointCenter.Size() / LocalParams.Extent, 0.0, 1.0), .3);

				const double NoiseVal = LocalStream.FRand();
				const double NoiseSample = FMath::Pow(LocalNoise->GenSingle3D(NX, NY, NZ, LocalSeed), 2.0);// *RadialWeight;

				if (NoiseVal >= NoiseSample)
				{
					continue; // Reject this point
				}

				double scale = FPointData::SampleScaleFromDistribution(LocalParams.MinClusterScale, LocalParams.MaxClusterScale, NoiseVal, LocalParams.ScaleDistributionCurve);
				FPointData InsertData = FPointData::MakePointDataFromWorldScale(scale, LocalParams.UnitScale, LocalParams.Extent);
				InsertData.Data.Density = LocalStream.FRandRange(0.5, 1.5);
				InsertData.Data.Composition = LocalStream.GetUnitVector();
				InsertData.Data.ObjectId = i;
				InsertData.Data.TypeId = 1;

				PointCenter *= FVector(
					1.0 + LocalStream.FRandRange(-LocalParams.Jitter, LocalParams.Jitter),
					1.0 + LocalStream.FRandRange(-LocalParams.Jitter, LocalParams.Jitter),
					1.0 + LocalStream.FRandRange(-LocalParams.Jitter, LocalParams.Jitter)
				);

				InsertData.SetPosition(PointCenter);

				Self.GeneratedData[i] = InsertData;
				PointInserted = true;
			}
			}, EParallelForFlags::BackgroundPriority);
	}
}

// Legacy override required by PointCloudGenerator base class. The octree is no
// longer the authoritative density source — callers on this path run with a
// uniform-density fallback (every candidate accepted at the density gate). A
// warning fires once per call so stale call sites stay visible.
//
// ASectorActor uses the FDensityVolume overload below; this path is only hit
// by legacy AUniverseActor which had no density data written to its octree
// anyway (SampleDensityAtPosition returned 0.0f → rejected everything).
void UniverseDataGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	UE_LOG(LogTemp, Warning,
		TEXT("UniverseDataGenerator::GenerateData(FOctree) called — legacy octree density path is deprecated. ")
		TEXT("Falling back to uniform density. Migrate caller to the FDensityVolume overload."));

	GenerateDataInternal(*this, [](const FVector& /*Pos*/) -> float
		{
			return 1.0f; // Uniform acceptance — no octree density read
		});
}

// Preferred entry point. Density is sampled directly from the CPU-side BGRA8
// volume buffer via FDensityVolume (alpha channel = gas density, matching
// SampleNoiseToVolume's default output channel).
void UniverseDataGenerator::GenerateData(const FDensityVolume& InDensityVolume)
{
	checkf(InDensityVolume.IsValid(),
		TEXT("UniverseDataGenerator::GenerateData: FDensityVolume is not valid"));

	GenerateDataInternal(*this, [&InDensityVolume](const FVector& Pos) -> float
		{
			return InDensityVolume.SampleDensityAtLocalPos(Pos, 3);
		});
}

//END UNIVERSE GENERATOR
#pragma endregion
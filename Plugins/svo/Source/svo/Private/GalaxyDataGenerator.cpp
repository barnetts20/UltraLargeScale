// GalaxyDataGenerator.cpp
// Refactored to mirror UniverseDataGenerator architecture.
// Legacy code (GalaxyParamFactory, old GalaxyDataGenerator) is preserved
// in a disabled #if 0 block at the bottom of this file.

#include "GalaxyDataGenerator.h"

#pragma region Signed Distance Fields

float GalaxyDataGenerator::SampleArmSDF(const FVector& InNormPos, double rXY) const
{
	// =====================================================================
	// Arm SDF: distance from the nearest spiral arm centerline.
	//
	// Returns unsigned distance. The density remap (core/envelope
	// thresholds) is applied in SampleDensity.
	//
	// To get a true perpendicular distance to the arm curve:
	//   1. Un-twist to find angular distance to nearest straight arm.
	//   2. Compute the closest point on the arm in polar coords.
	//   3. Get Euclidean 3D distance from query point to that closest point.
	//
	// This avoids the arc-length-at-radius approach which distorts
	// the distance metric at different radii.
	// =====================================================================

	const double discR = (double)Params.DiscRadius;
	const double armStart = (double)Params.ArmStartRadius * discR;
	const int32 N = FMath::Max(Params.ArmCount, 1);

	if (rXY < 1e-6 || N <= 0)
		return 10.0f;

	if (rXY > discR)
		return static_cast<float>(rXY - discR + 1.0);

	// --- Un-twist to find angular offset from nearest arm ---
	const double normalizedR = rXY / discR;
	const double baseTwist = (double)Params.ArmTwistStrength * normalizedR;
	const double coreBoost = (double)Params.ArmCoreTwistStrength *
		-FMath::Exp((double)Params.ArmCoreTwistRadius /
			FMath::Max(normalizedR, 1e-4));
	const double twistAngle = baseTwist + coreBoost;

	const double theta = FMath::Atan2(InNormPos.Y, InNormPos.X);
	const double untwistedTheta = theta - twistAngle;

	// Angular distance to nearest arm
	const double armSpacing = 2.0 * PI / N;
	double angDist = FMath::Fmod(untwistedTheta, armSpacing);
	if (angDist < 0.0) angDist += armSpacing;
	if (angDist > armSpacing * 0.5) angDist -= armSpacing;

	// --- Find the closest point ON the arm curve ---
	// The nearest arm in un-twisted space is at angle 0 (by construction
	// of angDist). In twisted space at this radius, that arm passes through
	// angle (theta - angDist). The closest point on the arm at this radius
	// is at (rXY, theta - angDist, z=0) in cylindrical coords.
	//
	// Convert both the query point and the arm point to Cartesian and
	// compute Euclidean distance. This gives a true perpendicular-ish
	// distance that is consistent in all directions.
	const double armTheta = theta - angDist; // angle where the arm actually is
	const double armX = rXY * FMath::Cos(armTheta);
	const double armY = rXY * FMath::Sin(armTheta);
	// Arm centerline is in the z=0 plane
	const double dx = InNormPos.X - armX;
	const double dy = InNormPos.Y - armY;
	const double dz = InNormPos.Z; // arm is at z=0

	double dist = FMath::Sqrt(dx * dx + dy * dy + dz * dz);

	// Fade in from arm start radius
	if (rXY < armStart)
	{
		dist += (armStart - rXY);
	}
	else if (rXY < armStart + 0.05)
	{
		const double blend = (rXY - armStart) / 0.05;
		const double smooth = blend * blend * (3.0 - 2.0 * blend);
		dist = FMath::Lerp(dist + 0.05, dist, smooth);
	}

	return static_cast<float>(dist);
}

float GalaxyDataGenerator::SampleBulgeSDF(const FVector& InNormPos) const
{
	// Signed distance to the bulge ellipsoid. Positive inside.
	const double squashedZ = InNormPos.Z / FMath::Max((double)Params.BulgeVerticalSquash, 0.01);
	const double rBulge = FMath::Sqrt(
		InNormPos.X * InNormPos.X +
		InNormPos.Y * InNormPos.Y +
		squashedZ * squashedZ);

	return static_cast<float>((double)Params.BulgeCutoffRadius - rBulge);
}

float GalaxyDataGenerator::SampleDiscSDF(const FVector& InNormPos, double rXY, double absZ) const
{
	// Signed distance to the disc volume (flat cylinder). Positive inside.
	const double discR = (double)Params.DiscRadius;
	const double h = FMath::Max(discR * (double)Params.DiscHeightRatio, 1e-6);

	// Distance to the radial boundary
	const double radialDist = discR - rXY;
	// Distance to the vertical boundary
	const double verticalDist = h - absZ;

	// For a box/cylinder SDF: inside = min of all boundary distances,
	// outside = distance to nearest edge
	if (radialDist > 0.0 && verticalDist > 0.0)
	{
		// Inside: positive, distance to nearest wall
		return static_cast<float>(FMath::Min(radialDist, verticalDist));
	}
	else
	{
		// Outside: negative, distance to nearest edge
		const double rOut = FMath::Max(-radialDist, 0.0);
		const double vOut = FMath::Max(-verticalDist, 0.0);
		return static_cast<float>(-FMath::Sqrt(rOut * rOut + vOut * vOut));
	}
}

#pragma endregion

#pragma region Density Sampling

float GalaxyDataGenerator::SampleDensity(const FVector& InNormPos) const
{
	// =====================================================================
	// SDF-based density evaluation.
	//
	// 1. Evaluate arm SDF (unsigned distance from arm centerline).
	// 2. Remap through core/envelope thresholds to [0, 1] density:
	//      dist <= ArmCoreThickness       → peak density
	//      dist in [core, envelope]       → smoothstep falloff
	//      dist > ArmEnvelopeThickness    → zero
	// 3. (Future) Composite with bulge/disc SDFs via max.
	// 4. Add background halo.
	// =====================================================================

	const double px = InNormPos.X;
	const double py = InNormPos.Y;
	const double pz = InNormPos.Z;
	const double rXY = FMath::Sqrt(px * px + py * py);
	const double absZ = FMath::Abs(pz);

	// --- Arm density from unsigned distance ---
	const float ArmDist = SampleArmSDF(InNormPos, rXY);

	const double core = FMath::Max((double)Params.ArmCoreThickness, 0.0);
	const double envelope = FMath::Max((double)Params.ArmEnvelopeThickness, core + 1e-6);

	double ArmDensity = 0.0;
	if (ArmDist <= core)
	{
		// Inside the core — full peak density
		ArmDensity = (double)Params.SDFPeakDensity;
	}
	else if (ArmDist < envelope)
	{
		// In the falloff zone — smoothstep from peak to zero
		const double t = (ArmDist - core) / (envelope - core);
		const double smooth = t * t * (3.0 - 2.0 * t); // smoothstep 0→1
		ArmDensity = (double)Params.SDFPeakDensity * (1.0 - smooth);
	}
	// else: beyond envelope — density stays 0

	double Density = ArmDensity;

	// --- Background halo (not SDF-based) ---
	if (Params.BackgroundDensity > 0.0f)
	{
		const double squashedZ = pz / FMath::Max((double)Params.BackgroundVerticalSquash, 0.01);
		const double rBg = FMath::Sqrt(px * px + py * py + squashedZ * squashedZ);
		const double cutoff = (double)Params.BackgroundCutoffRadius;

		if (rBg < cutoff)
		{
			const double fadeStart = (double)Params.BackgroundFadeStart * cutoff;
			double Fade = 1.0;
			if (rBg > fadeStart)
			{
				const double t2 = (rBg - fadeStart) / (cutoff - fadeStart);
				Fade = 1.0 - t2 * t2 * (3.0 - 2.0 * t2);
			}
			Density += (double)Params.BackgroundDensity * Fade;
		}
	}

	return FMath::Clamp(static_cast<float>(Density), 0.0f, 1.0f);
}

void GalaxyDataGenerator::SampleDensityBatch(
	float* OutDensity,
	int32 InCount,
	const float* InX,
	const float* InY,
	const float* InZ) const
{
	for (int32 i = 0; i < InCount; ++i)
	{
		OutDensity[i] = SampleDensity(FVector(InX[i], InY[i], InZ[i]));
	}
}

#pragma endregion

#pragma region Noise Composition

void GalaxyDataGenerator::Initialize()
{
	// Build the encoded tree for future use (currently unused � C++ path active).
	DensityNoise = BuildNoise();
}

FastNoise::SmartNode<> GalaxyDataGenerator::BuildNoise() const
{
	// -----------------------------------------------------------------------
	// Encoded noise tree � kept for future FastNoise swap-in.
	// Currently the C++ SampleDensity path is used instead.
	// When ready to switch: replace SampleDensityBatch calls with
	// DensityNoise->GenPositionArray3D and SampleNoiseVolume with
	// FVolumeTextureUtils::SampleNoiseToVolume.
	// -----------------------------------------------------------------------

	auto Graph = FastNoise::NewFromEncodedNodeTree(Params.EncodedTree);

	return Graph;
}

TArray<uint8> GalaxyDataGenerator::SampleNoiseVolume(int InNoiseResolution) const
{
	// -----------------------------------------------------------------------
	// Custom volume sampler using the C++ SampleDensity function.
	// Mirrors FVolumeTextureUtils::SampleNoiseToVolume layout:
	//   - BGRA8, 4 bytes per voxel
	//   - Linear [z][y][x] order
	//   - Density written to channel 3 (alpha)
	// -----------------------------------------------------------------------

	const int BytesPerVoxel = 4;
	const int64 TotalBytes = (int64)InNoiseResolution * InNoiseResolution * InNoiseResolution * BytesPerVoxel;

	TArray<uint8> TextureData;
	TextureData.SetNumZeroed(TotalBytes);

	const double VoxelSize = 2.0 / InNoiseResolution;  // In normalized [-1, 1] space

	ParallelFor(InNoiseResolution, [&](int z)
		{
			const double nz = -1.0 + (z + 0.5) * VoxelSize;

			for (int y = 0; y < InNoiseResolution; ++y)
			{
				const double ny = -1.0 + (y + 0.5) * VoxelSize;

				for (int x = 0; x < InNoiseResolution; ++x)
				{
					const double nx = -1.0 + (x + 0.5) * VoxelSize;

					float Density = SampleDensity(FVector(nx, ny, nz));
					Density = FMath::Pow(Density, Params.NoisePower);
					uint8 DensityByte = static_cast<uint8>(FMath::Clamp(Density * 255.0f, 0.0f, 255.0f));

					const int64 Idx = ((int64)z * InNoiseResolution * InNoiseResolution
						+ (int64)y * InNoiseResolution + x) * BytesPerVoxel;
					TextureData[Idx + 3] = DensityByte;  // Alpha channel
				}
			}
		}, EParallelForFlags::BackgroundPriority);

	return TextureData;
}

#pragma endregion

#pragma region Particle Count Derivation
// DeriveLargeParticleCount removed — was never called. Large tier particle
// count is effectively determined by SlotCapacity * acceptance rate from the
// density rejection sampling, which is the correct behavior.
#pragma endregion

#pragma region Tier Generation Callbacks

void GalaxyDataGenerator::GenerateTierNode(
	const FIntVector& InCoord,
	int32 InSlotIndex,
	FNiagaraParticleBuffer& InBuffer,
	const FVector& InNodeCenter,
	double InCellExtent,
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	int32& OutSlotCount) const
{
	// -----------------------------------------------------------------------
	// Three-phase batched generation matching UniverseDataGenerator pattern:
	//   1. Generate candidate positions + normalized noise coords.
	//   2. Batch density evaluation via C++ SampleDensityBatch.
	//   3. Walk results, rejection-gate, write accepted to slot buffers.
	//
	// For the large tier, InNodeCenter is ZeroVector and InCellExtent is
	// Params.Extent, so candidates span the full galaxy volume.
	// For mid/small tiers, candidates are local to the cell.
	// -----------------------------------------------------------------------

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed + InSeedOffset, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = InBuffer.SlotCapacity;
	const double InvExtent = 1.0 / static_cast<double>(Params.Extent);

	// --- Phase 1: generate candidates + normalized noise coords ---
	TArray<FVector> CandidatePositions;
	TArray<float> NoiseX, NoiseY, NoiseZ;
	CandidatePositions.SetNumUninitialized(NumCandidates);
	NoiseX.SetNumUninitialized(NumCandidates);
	NoiseY.SetNumUninitialized(NumCandidates);
	NoiseZ.SetNumUninitialized(NumCandidates);

	for (int32 i = 0; i < NumCandidates; ++i)
	{
		FVector Candidate(
			Stream.FRandRange(-InCellExtent, InCellExtent),
			Stream.FRandRange(-InCellExtent, InCellExtent),
			Stream.FRandRange(-InCellExtent, InCellExtent));
		Candidate += InNodeCenter;
		CandidatePositions[i] = Candidate;

		// Normalize to [-1, 1] noise space relative to galaxy center.
		// Galaxy is self-contained so no cross-cell snapping is needed.
		NoiseX[i] = static_cast<float>(Candidate.X * InvExtent);
		NoiseY[i] = static_cast<float>(Candidate.Y * InvExtent);
		NoiseZ[i] = static_cast<float>(Candidate.Z * InvExtent);
	}

	// --- Phase 2: batch density evaluation ---
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	SampleDensityBatch(
		NoiseOut.GetData(), NumCandidates,
		NoiseX.GetData(), NoiseY.GetData(), NoiseZ.GetData());

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// --- Phase 3: accept/reject + write to slot ---
	int32 ActualCount = 0;
	auto dCurve = InTierParams.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		const float Density = FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			InTierParams.MinScale, InTierParams.MaxScale,
			ScaleSample, InTierParams.ScaleDistribution);
		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = static_cast<double>(Params.Extent) / static_cast<double>(1 << PointData.InsertDepth);
		const float FinalExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		const FVector CompVec = Stream.GetUnitVector();

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Extents[Idx] = FinalExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	InBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);
	OutSlotCount = ActualCount;
}

#pragma endregion


// ============================================================================
// LEGACY CODE � preserved for Phase E spiral density field reference
// ============================================================================
// The code below is the original galaxy generation system. It is compiled
// out via #if 0 but kept in this file so that the arm, twist, bulge, disc,
// cluster, and background generation logic is easily accessible when
// implementing the spiral density field in Phase E.
//
// To re-enable for reference/testing, change #if 0 to #if 1.
// ============================================================================

#if 0 // ---- BEGIN LEGACY CODE (disabled, preserved for reference) ----

#include "PointCloudGenerator.h"

#pragma region LegacyGalaxyParamFactory
LegacyGalaxyParamFactory::LegacyGalaxyParamFactory() {
#pragma region VolumeMaterialBounds
	Volume_Min.VolumeAmbientColor = FLinearColor(.1, .1, .1);
	Volume_Max.VolumeAmbientColor = FLinearColor(1.2, 1.2, 1.2);
	Volume_Min.VolumeCoolShift = FLinearColor(1, 1, 1);
	Volume_Max.VolumeCoolShift = FLinearColor(20, 20, 20);
	Volume_Min.VolumeDensity = .1;
	Volume_Max.VolumeDensity = .5;
	Volume_Min.VolumeHotShift = FLinearColor(.5, .25, .01);
	Volume_Max.VolumeHotShift = FLinearColor(1, .7, .3);
	Volume_Min.VolumeHueVariance = .01;
	Volume_Max.VolumeHueVariance = .25;
	Volume_Min.VolumeHueVarianceScale = .25;
	Volume_Max.VolumeHueVarianceScale = 1.75;
	Volume_Min.VolumeSaturationVariance = 0;
	Volume_Max.VolumeSaturationVariance = .9;
	Volume_Min.VolumeTemperatureScale = 3;
	Volume_Max.VolumeTemperatureScale = 10;
	Volume_Min.VolumeTemperatureInfluence = 8;
	Volume_Max.VolumeTemperatureInfluence = 48;
	Volume_Min.VolumeWarpAmount = .03;
	Volume_Max.VolumeWarpAmount = .1;
	Volume_Min.VolumeWarpScale = .1;
	Volume_Max.VolumeWarpScale = .5;
#pragma endregion

	// --- All archetype definitions (E0, E3, E5, E7, S0, Sa, Sb, Sc, SBa, SBb, SBc, Irr) ---
	// --- and their min/max bounds would go here ---
	// --- See original GalaxyDataGenerator.cpp for full archetype definitions ---
	// [ARCHETYPE DEFINITIONS OMITTED FOR BREVITY � full original preserved in git history]
};

FLegacyGalaxyParams LegacyGalaxyParamFactory::GenerateParams()
{
	int TypeIndex = SelectGalaxyTypeIndex();
	FLegacyGalaxyParams Params;
	// [Switch statement over all archetypes � see original]
	return Params;
}

FLegacyGalaxyParams LegacyGalaxyParamFactory::BoundedRandomizeParams(FLegacyGalaxyParams MinParams, FLegacyGalaxyParams MaxParams)
{
	FRandomStream Stream(Seed);
	FLegacyGalaxyParams Params;
	// [Full bounded randomization � see original]
	return Params;
}

int LegacyGalaxyParamFactory::SelectGalaxyTypeIndex()
{
	FRandomStream Stream(Seed + 69);
	float RandomValue = Stream.FRand();
	float CumulativeWeight = 0.0f;
	for (int i = 0; i < GalaxyWeights.Num(); i++)
	{
		CumulativeWeight += GalaxyWeights[i].Key;
		if (RandomValue <= CumulativeWeight) return i;
	}
	return 7;
}
#pragma endregion

#pragma region LegacyGalaxyDataGenerator
void LegacyGalaxyDataGenerator::GenerateData(TSharedPtr<FOctree> InOctree)
{
	GeneratedData.SetNum(0);
	Params.GalaxyRadius = Params.Extent * Params.GalaxyRatio;
	Params.BulgeRadius = Params.Extent * Params.BulgeRatio;
	Params.VoidRadius = Params.BulgeRadius * Params.VoidRatio;

	GenerateArms();
	ApplyTwist();
	GenerateClusters();
	GenerateBulge();
	GenerateDisc();
	GenerateBackground();
	ApplyRotation();

	const float VoidRadiusSquared = Params.VoidRadius * Params.VoidRadius;
	FPointData BlackHole;
	BlackHole.SetPosition(FVector::ZeroVector);
	BlackHole.Data.ObjectId = INT32_MAX;
	BlackHole.Data.TypeId = 1;

	FCriticalSection BlackHoleMutex;
	ParallelFor(GeneratedData.Num(), [this, InOctree, &BlackHole, &BlackHoleMutex, VoidRadiusSquared](int32 i) {
		FPointData InsertData = GeneratedData[i];
		if (InsertData.GetPosition().SizeSquared() < VoidRadiusSquared)
		{
			FScopeLock Lock(&BlackHoleMutex);
			GeneratedData[i].Data.TypeId = -1;
			GeneratedData[i].SetPosition(FVector::ZeroVector);
			double DensityWeight = FMath::Pow(2.0, InOctree->MaxDepth - InsertData.InsertDepth);
			BlackHole.Data.ScaleFactor += InsertData.Data.ScaleFactor;
			BlackHole.Data.Density += InsertData.Data.Density * DensityWeight;
			BlackHole.Data.Composition += InsertData.Data.Composition * BlackHole.Data.Density;
		}
		});

	BlackHole.InsertDepth = MinInsertionDepth - 3;
	BlackHole.Data.ScaleFactor = 1;
	BlackHole.Data.Density = 1;
}

void LegacyGalaxyDataGenerator::GenerateBulge()
{
	double StartTime = FPlatformTime::Seconds();
	const int32 NumPoints = Params.BulgeNumPoints;
	GeneratedData.Reserve(GeneratedData.Num() + NumPoints);
	const int32 StartIndex = GeneratedData.Num();
	GeneratedData.AddUninitialized(NumPoints);

	const FVector AxisBounds = Params.BulgeAxisScale * Params.BulgeRadius;
	const double a = Params.BulgeRadius * Params.BulgeRadiusScale;
	const double SoftTruncationRadius = Params.BulgeRadius * Params.BulgeTruncationScale;
	const double TruncationZone = Params.Extent - SoftTruncationRadius;
	const FVector AxisScale = AxisBounds / Params.BulgeRadius;

	ParallelFor(NumPoints, [&](int32 i)
		{
			FRandomStream Stream(Seed + 99 + i * 997);
			FPointData& InsertData = GeneratedData[StartIndex + i];
			InsertData.Data.ObjectId = i;
			InsertData.Data.TypeId = 1;

			bool PointInserted = false;
			while (!PointInserted) {
				double u = Stream.FRand();
				double f = u * (1.0 + u * (0.5 + u * (0.333 + u * 0.25)));
				double r = a * f;

				if (r > SoftTruncationRadius)
				{
					if (r > Params.Extent) continue;
					double acceptanceProbability = FMath::Exp(-3.0 * FMath::Pow(((r - SoftTruncationRadius) / TruncationZone), Params.BulgeAcceptanceExponent));
					if (Stream.FRand() > acceptanceProbability) continue;
				}

				double scale = FPointData::SampleScaleFromDistribution(Params.MinStarSystemScale, Params.MaxStarSystemScale, Stream.FRand(), Params.ScaleDistributionCurve);
				FPointData idata = FPointData::MakePointDataFromWorldScale(scale, Params.UnitScale, Params.Extent);

				InsertData.Data.ScaleFactor = idata.Data.ScaleFactor;
				InsertData.Data.Density = Stream.FRandRange(0.5f, 1.5f) * Params.BulgeBaseDensity;
				InsertData.Data.Composition = Stream.GetUnitVector();
				InsertData.InsertDepth = idata.InsertDepth;
				InsertData.SetPosition(Stream.GetUnitVector() * r * AxisScale + (Stream.GetUnitVector() * Stream.FRand() * Params.BulgeRadius * Params.BulgeJitter));
				PointInserted = true;
			}
		});

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Generate Bulge duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::GenerateClusters()
{
	double StartTime = FPlatformTime::Seconds();
	if (Params.ClusterNumClusters <= 0 || Params.ClusterNumPoints <= 0) return;
	const int PointsPerCluster = Params.ClusterNumPoints / Params.ClusterNumClusters;
	if (PointsPerCluster <= 0) return;

	FRandomStream Stream(Seed + 2024);
	for (int i = 0; i < Params.ClusterNumClusters; i++)
	{
		const double u = Stream.FRand();
		const double Dist = Params.GalaxyRadius * FMath::Pow(u, 1.0 / 3.0);
		const double z = Stream.FRandRange(-1.0, 1.0);
		const double phi = Stream.FRandRange(0.0, 2.0 * PI);
		const double xy = FMath::Sqrt(1.0 - z * z);
		const FVector Dir(xy * FMath::Cos(phi), xy * FMath::Sin(phi), z);
		FVector Center = Dir * Dist;
		Center.X *= Params.ClusterAxisScale.X;
		Center.Y *= Params.ClusterAxisScale.Y;
		Center.Z *= Params.ClusterAxisScale.Z;

		const double DistFromOrigin = Center.Length();
		const double NormalizedDist = FMath::Clamp(DistFromOrigin / Params.GalaxyRadius, 0.0, 1.0);
		const double BaseRadiusScale = FMath::Lerp(Params.ClusterMinScale, Params.ClusterMaxScale, NormalizedDist);
		const double ClusterRadiusVal = Params.GalaxyRadius * BaseRadiusScale;
		FVector Radius(ClusterRadiusVal * Params.ClusterSpreadFactor, ClusterRadiusVal * Params.ClusterSpreadFactor, ClusterRadiusVal * Params.ClusterSpreadFactor);

		const FVector Jitter(
			Stream.FRandRange(-Radius.X, Radius.X) * Params.ClusterIncoherence,
			Stream.FRandRange(-Radius.Y, Radius.Y) * Params.ClusterIncoherence,
			Stream.FRandRange(-Radius.Z, Radius.Z) * Params.ClusterIncoherence);
		Center += Jitter;

		GenerateCluster(i + 987654, Center, Radius, PointsPerCluster, Params.ClusterBaseDensity, Params.ClusterDepthBias);
	}

	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Generate Clusters duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::GenerateArms()
{
	double StartTime = FPlatformTime::Seconds();
	int StarsPerCluster = (Params.ArmNumPoints / Params.ArmNumArms) / Params.ArmClusters;
	double MinScale = Params.GalaxyRadius * Params.ArmClusterRadiusMin;
	double MaxScale = Params.GalaxyRadius * Params.ArmClusterRadiusMax;
	double MinJitter = Params.GalaxyRadius * Params.ArmSpreadMin;
	double MaxJitter = Params.GalaxyRadius * Params.ArmSpreadMax;
	double MinScaleVariance = .5;
	double MaxScaleVariance = 1.5;
	double MinJitterVariance = .5;
	double MaxJitterVariance = 1.5;
	double ProgressExponent = Params.ArmProgressExponent;

	FRandomStream Stream(Seed + 1337);
	for (int ArmIndex = 0; ArmIndex < Params.ArmNumArms; ArmIndex++)
	{
		double BaseAngle = (2.0 * PI * ArmIndex) / Params.ArmNumArms;
		FVector ArmDir(FMath::Cos(BaseAngle), FMath::Sin(BaseAngle), 0);
		for (int c = 0; c < Params.ArmClusters; c++)
		{
			double Dist = FMath::Lerp(FMath::Max(0, Params.BulgeRadius * Params.ArmStartRatio), Params.GalaxyRadius, -FMath::Loge(1.0 - Stream.FRand()));
			double Progress = Dist / Params.GalaxyRadius;
			double ExpProgress = FMath::Pow(Progress, ProgressExponent);
			FVector Center = ArmDir * Dist + FMath::Lerp(MinJitter, MaxJitter, ExpProgress) * Stream.GetUnitVector() * Stream.FRandRange(MinJitterVariance, MaxJitterVariance);
			double ArmWidth = FMath::Lerp(MinScale, MaxScale, ExpProgress) * Stream.FRandRange(MinScaleVariance, MaxScaleVariance);
			FVector Radius(ArmWidth, ArmWidth, ArmWidth * Params.ArmHeightRatio);
			double DensityCoef = FMath::Lerp(Params.ArmRadialDensityMin, Params.ArmRadialDensityMax, FMath::Pow(Progress, Params.ArmRadialDensityExponent));
			GenerateCluster(ArmIndex + c + 123, Center, Radius, StarsPerCluster, Params.ArmBaseDensity * DensityCoef, Params.ArmDepthBias);
		}
	}
	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Generate Arms duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::ApplyTwist()
{
	double StartTime = FPlatformTime::Seconds();
	ParallelFor(GeneratedData.Num(), [&](int32 i)
		{
			FVector P = GeneratedData[i].GetPosition();
			double rXY = P.Length();
			if (rXY < KINDA_SMALL_NUMBER) return;
			double theta = FMath::Atan2(P.Y, P.X);
			double normalizedRadius = FMath::Clamp(rXY / Params.Extent, 0.0, 1.0);
			double baseDelta = Params.TwistStrength * normalizedRadius;
			double coreBoost = Params.TwistCoreStrength * -FMath::Exp((Params.TwistCoreRadius / FMath::Max(FMath::Pow(normalizedRadius, Params.TwistCoreTwistExponent), 1e-4)));
			double deltaTheta = baseDelta + coreBoost;
			double newTheta = theta + deltaTheta;
			GeneratedData[i].SetPosition(FVector(rXY * FMath::Cos(newTheta), rXY * FMath::Sin(newTheta), GeneratedData[i].GetPosition().Z));
		}, EParallelForFlags::BackgroundPriority);
	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Apply Twist duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::GenerateDisc()
{
	double StartTime = FPlatformTime::Seconds();
	GenerateCluster(Seed + 999, FVector::ZeroVector, FVector(Params.GalaxyRadius, Params.GalaxyRadius, Params.GalaxyRadius * Params.DiscHeightRatio), Params.DiscNumPoints, Params.DiscBaseDensity, Params.DiscDepthBias);
	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Generate Disc duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::GenerateBackground()
{
	double StartTime = FPlatformTime::Seconds();
	GenerateCluster(Seed + 123456789, FVector::ZeroVector, FVector(Params.Extent, Params.Extent, Params.Extent * Params.BackgroundHeightRatio), Params.BackgroundNumPoints, Params.BackgroundBaseDensity, Params.BackgroundDepthBias);
	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Generate Background duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::ApplyRotation()
{
	double StartTime = FPlatformTime::Seconds();
	ParallelFor(GeneratedData.Num(), [&](int32 i)
		{
			GeneratedData[i].SetPosition(RotateCoordinate(GeneratedData[i].GetPosition(), Params.Rotation));
		});
	double TotalDuration = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("LegacyGalaxyGenerator::Apply Rotation duration: %.3f seconds "), TotalDuration);
}

void LegacyGalaxyDataGenerator::GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity, double InDepthBias)
{
	int32 StartIndex = GeneratedData.Num();
	GeneratedData.AddUninitialized(InCount);

	ParallelFor(InCount, [&](int32 i)
		{
			FRandomStream Stream(InSeed ^ (StartIndex + i));
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
				Gaussian(Stream) * InClusterRadius.Z);

			FVector P = InClusterCenter + Offset;
			double size = P.Length();

			double scale = FPointData::SampleScaleFromDistribution(Params.MinStarSystemScale, Params.MaxStarSystemScale, Stream.FRand(), Params.ScaleDistributionCurve);
			FPointData InsertData = FPointData::MakePointDataFromWorldScale(scale, Params.UnitScale, Params.Extent);
			InsertData.SetPosition(size < Params.Extent ? P : FVector::ZeroVector);
			InsertData.Data.Density = InBaseDensity * Stream.FRandRange(.5, 1.5);
			InsertData.Data.Composition = Stream.GetUnitVector();
			InsertData.Data.ObjectId = i + StartIndex;
			InsertData.Data.TypeId = 1;
			GeneratedData[StartIndex + i] = InsertData;
		}, EParallelForFlags::BackgroundPriority);
}
#pragma endregion

#endif // ---- END LEGACY CODE ----
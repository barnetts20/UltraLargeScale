// GalaxyDataGenerator.cpp
// Spiral density field, SDF evaluation, and tier generation callbacks.

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
	// Finds the closest arm point AT THE SAME RADIUS as the query point
	// (preserves spiral structure). Vertical distance is scaled by
	// DiscHeightRatio so the arm tube matches the disc's aspect ratio
	// and the falloff is isotropic in the arm's local frame.
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

	// --- Find the closest point on the arm at this radius ---
	const double armTheta = theta - angDist;
	const double armX = rXY * FMath::Cos(armTheta);
	const double armY = rXY * FMath::Sin(armTheta);

	const double dx = InNormPos.X - armX;
	const double dy = InNormPos.Y - armY;

	// In-plane distance (XY chord to nearest arm point at same radius)
	const double xyDist = FMath::Sqrt(dx * dx + dy * dy);

	// Radial progress along the arm: 0 at inner edge, 1 at disc rim
	const double tRadial = FMath::Clamp(
		(rXY - armStart) / FMath::Max(discR - armStart, 1e-6), 0.0, 1.0);

	// Vertical squash lerps from inner to outer value along the arm
	const double squash = FMath::Lerp(
		(double)Params.ArmVerticalSquash, (double)Params.ArmVerticalSquashOuter, tRadial);
	const double scaledZ = InNormPos.Z * squash;

	double dist = FMath::Sqrt(xyDist * xyDist + scaledZ * scaledZ);

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

	// Radial progress along the arm (same t as in SampleArmSDF)
	const double discR = (double)Params.DiscRadius;
	const double armStart = (double)Params.ArmStartRadius * discR;
	const double tRadial = FMath::Clamp(
		(rXY - armStart) / FMath::Max(discR - armStart, 1e-6), 0.0, 1.0);

	// Radial growth factor: envelope/core widen as the arm extends outward
	const double growthFactor = FMath::Lerp(1.0, (double)Params.ArmRadialGrowth, tRadial);

	// Scale core and envelope by growth
	const double core = FMath::Max((double)Params.ArmCoreThickness * growthFactor, 0.0);
	const double envelope = FMath::Max((double)Params.ArmEnvelopeThickness * growthFactor, core + 1e-6);

	// Peak density drops with growth, controlled by falloff exponent.
	// Exponent 1.0 = full inverse, 0.5 = sqrt, 0.0 = no drop.
	const double densityScale = FMath::Pow(growthFactor, (double)Params.ArmDensityFalloffExponent);
	const double peakDensity = (double)Params.SDFPeakDensity / FMath::Max(densityScale, 1e-6);

	double ArmDensity = 0.0;
	if (ArmDist <= core)
	{
		ArmDensity = peakDensity;
	}
	else if (ArmDist < envelope)
	{
		const double t = (ArmDist - core) / (envelope - core);
		const double smooth = t * t * (3.0 - 2.0 * t);
		ArmDensity = peakDensity * (1.0 - smooth);
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

	// --- Bounds fade — prevent hard transitions at volume edges ---
	// Spherical distance for a natural round falloff. Hard zero beyond r=1.0
	// to prevent density leaking into the cube corners.
	const double rBounds = FMath::Sqrt(px * px + py * py + pz * pz);
	if (rBounds >= 1.0)
		return 0.0f;

	const double fadeStart = (double)Params.BoundsFadeStart;
	if (rBounds > fadeStart)
	{
		const double t = (rBounds - fadeStart) / (1.0 - fadeStart);
		const double fade = 1.0 - t * t * (3.0 - 2.0 * t);
		Density *= fade;
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
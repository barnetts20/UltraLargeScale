// GalaxyDataGenerator.cpp
// Spiral density field, SDF evaluation, and tier generation callbacks.

#include "GalaxyDataGenerator.h"

#pragma region Signed Distance Fields

float GalaxyDataGenerator::SampleArmSDF(const FVector& InNormPos, double rXY) const
{
	// =====================================================================
	// Arm SDF: unsigned distance from the nearest spiral arm centerline.
	//
	// Returns unsigned distance. The density remap (core/envelope
	// thresholds) is applied in SampleDensity.
	//
	// Finds the closest arm point AT THE SAME RADIUS as the query point
	// (preserves spiral structure). Vertical distance is scaled by
	// ArmVerticalSquash (lerped from inner to outer along the arm) so
	// the arm tube cross-section appears round when viewed edge-on.
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
	const double blendWidth = FMath::Max((double)Params.ArmStartBlendWidth, 1e-6);
	if (rXY < armStart)
	{
		dist += (armStart - rXY);
	}
	else if (rXY < armStart + blendWidth)
	{
		const double blend = (rXY - armStart) / blendWidth;
		const double smooth = blend * blend * (3.0 - 2.0 * blend);
		dist = FMath::Lerp(dist + blendWidth, dist, smooth);
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

float GalaxyDataGenerator::SampleBulgeDensity(const FVector& InNormPos) const
{
	// =====================================================================
	// Hernquist bulge density in oblate (vertically squashed) coordinates.
	//
	// Profile: density(r) = BulgePeakDensity * (a / r_cutoff) *
	//                       (a / r) / (1 + r/a)^3
	//
	// Normalised so that density(r = BulgeScaleRadius) = BulgePeakDensity
	// by evaluating the Hernquist formula at r = a and using that as the
	// reference peak. This gives stable [0, 1] output regardless of how
	// BulgeScaleRadius is tuned.
	//
	// Vertical squash: Z is divided by BulgeVerticalSquash before computing
	// the ellipsoidal radius, producing an oblate shape without distorting
	// the radial profile shape.
	//
	// Hard cutoff at BulgeCutoffRadius prevents the 1/r^4 Hernquist tail
	// from leaking density into the disc and arm regions.
	// =====================================================================

	if (Params.BulgePeakDensity <= 0.0f)
		return 0.0f;

	const double a = FMath::Max((double)Params.BulgeScaleRadius, 1e-6);
	const double cutoff = FMath::Max((double)Params.BulgeCutoffRadius, a);

	// Oblate radius
	const double squashedZ = InNormPos.Z / FMath::Max((double)Params.BulgeVerticalSquash, 1e-4);
	const double r = FMath::Sqrt(
		InNormPos.X * InNormPos.X +
		InNormPos.Y * InNormPos.Y +
		squashedZ * squashedZ);

	// Hard cutoff
	if (r >= cutoff)
		return 0.0f;

	// Hernquist profile: rho(r) = 1 / (r/a * (1 + r/a)^3)
	// Evaluated at r = a to get the reference value for normalisation:
	//   rho(a) = 1 / (1 * 2^3) = 1/8
	// So: normalised_density = 8 * rho(r) / rho_max, where rho_max = rho(r_min)
	// Rather than deal with the singularity at r=0, we clamp r to a small
	// fraction of a and let BulgePeakDensity scale the output.
	const double rClamped = FMath::Max(r, a * 0.01);
	const double rOverA = rClamped / a;
	const double hernquist = 1.0 / (rOverA * FMath::Pow(1.0 + rOverA, 3.0));

	// Normalise: hernquist(a) = 1/8, so multiply by 8 to get 1.0 at r = a.
	// Beyond r = a density is < 1, at r < a it rises above 1 — clamp at the end.
	const double normalised = hernquist * 8.0;

	// Smooth fade to zero approaching the cutoff radius (avoids a hard density
	// cliff at BulgeCutoffRadius when compositing with disc/arms).
	const double fadeStart = cutoff * 0.75;
	double fade = 1.0;
	if (r > fadeStart)
	{
		const double t = (r - fadeStart) / (cutoff - fadeStart);
		fade = 1.0 - t * t * (3.0 - 2.0 * t);
	}

	return FMath::Clamp(
		static_cast<float>((double)Params.BulgePeakDensity * normalised * fade),
		0.0f, 1.0f);
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

float GalaxyDataGenerator::SampleDiscDensity(double rXY, double absZ) const
{
	// =====================================================================
	// Analytic disc density — separable exponential profile.
	//
	// Radial:   exp(-rXY / scaleLength)
	//   scaleLength = DiscRadius * DiscRadialScaleLength
	//   Smaller DiscRadialScaleLength = tighter, nucleus-concentrated disc.
	//   Larger  DiscRadialScaleLength = more diffuse, extended disc.
	//
	// Vertical: exp(-(|z| / h)^DiscVerticalFalloff)
	//   h = DiscRadius * DiscHeightRatio
	//   DiscVerticalFalloff = 1.0 → exponential / isothermal sheet
	//   DiscVerticalFalloff = 2.0 → Gaussian (softer for thick disc)
	//
	// Hard cutoff at the disc cylinder boundary (rXY > DiscRadius or
	// absZ > h) prevents leakage into the arm/bulge regions above/below.
	// =====================================================================

	if (Params.DiscBaseDensity <= 0.0f)
		return 0.0f;

	const double discR = (double)Params.DiscRadius;
	const double h = discR * FMath::Max((double)Params.DiscHeightRatio, 1e-6);
	const double scaleL = discR * FMath::Max((double)Params.DiscRadialScaleLength, 1e-6);

	// Hard boundary — outside cylinder contributes nothing
	if (rXY >= discR || absZ >= h)
		return 0.0f;

	// Radial: exponential decay from center
	const double radialProfile = FMath::Exp(-rXY / scaleL);

	// Vertical: exp(-(|z|/h)^falloff)
	// Clamp zNorm to [0,1] — absZ < h is guaranteed above, but float precision guard
	const double zNorm = FMath::Min(absZ / h, 1.0);
	const double vExp = FMath::Max((double)Params.DiscVerticalFalloff, 0.1);
	const double verticalProfile = FMath::Exp(-FMath::Pow(zNorm, vExp));

	return static_cast<float>((double)Params.DiscBaseDensity * radialProfile * verticalProfile);
}

#pragma endregion

#pragma region Density Sampling

float GalaxyDataGenerator::SampleDensity(const FVector& InNormPos) const
{
	// =====================================================================
	// Composite density evaluation.
	//
	// Active layers (max-blended — each fills space the others don't):
	//   1. Arms  — SDF distance -> core/envelope remap -> scaled peak density
	//   2. Disc  — Analytic exponential radial x vertical profile
	//   3. Bulge — Hernquist profile in oblate coordinates
	//
	// Zeroed layers (re-enable during compositing phase):
	//   4. BG    — Additive halo (BackgroundDensity = 0)
	//
	// Global bounds fade: spherical smoothstep to zero at r = 1.0.
	// =====================================================================

	const double px = InNormPos.X;
	const double py = InNormPos.Y;
	const double pz = InNormPos.Z;

	// --- Bounds check (early out) ---
	// Hard zero beyond unit sphere — saves all SDF work for cube corners.
	const double rBounds = FMath::Sqrt(px * px + py * py + pz * pz);
	if (rBounds >= 1.0)
		return 0.0f;

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

	// --- Disc density ---
	// Max-blend: disc fills the inter-arm gaps without amplifying density
	// where the arms are already strong.
	const double DiscDensity = (double)SampleDiscDensity(rXY, absZ);

	// --- Bulge density ---
	// Max-blend: concentrated central mass, fades before the disc/arm region.
	const double BulgeDensity = (double)SampleBulgeDensity(InNormPos);
	double BlendSharpness = 6; //TODO: LIFT THIS OUT INTO CONFIG
	double Density = SmoothMax(ArmDensity, DiscDensity, BulgeDensity, BlendSharpness);// FMath::Max(ArmDensity, FMath::Max(DiscDensity, BulgeDensity));

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

	// --- Bounds fade — spherical smoothstep to zero approaching r=1.0 ---
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

// ============================================================================
//  Large Tier — SDF-culled grid generation
// ============================================================================

TArray<GalaxyDataGenerator::FActiveLargeTierCell> GalaxyDataGenerator::CollectActiveLargeTierCells() const
{
	// -----------------------------------------------------------------------
	// Subdivide [-Extent, +Extent]^3 into a uniform grid at LargeTierCullDepth.
	// For each cell, evaluate SampleDensity at all 8 corners (in normalized
	// [-1, 1] space). If every corner returns zero, the cell is entirely
	// outside all SDF envelopes (arms, disc, bulge, background) and is
	// skipped. Any cell with at least one corner > 0 is kept.
	//
	// We intentionally test corners rather than the cell center so that cells
	// straddling an envelope boundary are never wrongly discarded. A cell
	// whose center happens to fall in the gap between two arms but whose
	// corner clips an arm edge will still be included — candidates generated
	// inside it will simply fail the per-candidate density rejection gate as
	// normal, at very low cost.
	// -----------------------------------------------------------------------

	const int32 CullDepth = FMath::Max(Params.LargeTierCullDepth, 1);
	const int32 GridSide = 1 << CullDepth;          // cells per axis
	const double FullExtent = static_cast<double>(Params.Extent);
	const double CellFull = (2.0 * FullExtent) / static_cast<double>(GridSide);
	const double HalfCell = CellFull * 0.5;
	const double InvExtent = 1.0 / FullExtent;

	// 8 corner offsets in cell-local space (±HalfCell on each axis)
	static const FVector CornerOffsets[8] =
	{
		FVector(-1, -1, -1), FVector(1, -1, -1),
		FVector(-1,  1, -1), FVector(1,  1, -1),
		FVector(-1, -1,  1), FVector(1, -1,  1),
		FVector(-1,  1,  1), FVector(1,  1,  1),
	};

	const int32 TotalCells = GridSide * GridSide * GridSide;
	TArray<FActiveLargeTierCell> ActiveCells;
	ActiveCells.Reserve(TotalCells / 4); // rough estimate; arms ~ 25% fill

	for (int32 iz = 0; iz < GridSide; ++iz)
	{
		for (int32 iy = 0; iy < GridSide; ++iy)
		{
			for (int32 ix = 0; ix < GridSide; ++ix)
			{
				// Cell center in galaxy-local space
				const FVector Center(
					-FullExtent + HalfCell + static_cast<double>(ix) * CellFull,
					-FullExtent + HalfCell + static_cast<double>(iy) * CellFull,
					-FullExtent + HalfCell + static_cast<double>(iz) * CellFull);

				// Test all 8 corners in normalized space.
				// Early-out as soon as any corner has non-zero density —
				// the cell is active and we don't need to check the rest.
				bool bAnyActive = false;
				for (int32 c = 0; c < 8; ++c)
				{
					const FVector CornerWorld = Center + CornerOffsets[c] * HalfCell;
					const FVector CornerNorm = CornerWorld * InvExtent;

					// Hard bounds check: SampleDensity already returns 0
					// beyond the unit sphere, but an explicit check here
					// lets us skip the full SDF evaluation for corners
					// that are clearly outside the disc extents.
					if (FMath::Abs(CornerNorm.X) > 1.0 ||
						FMath::Abs(CornerNorm.Y) > 1.0 ||
						FMath::Abs(CornerNorm.Z) > 1.0)
					{
						continue; // This corner is outside — try next
					}

					if (SampleDensity(CornerNorm) > 0.0f)
					{
						bAnyActive = true;
						break;
					}
				}

				if (bAnyActive)
				{
					FActiveLargeTierCell Cell;
					Cell.Center = Center;
					Cell.HalfExt = HalfCell;
					Cell.GridCoord = FIntVector(ix, iy, iz);
					ActiveCells.Add(Cell);
				}
			}
		}
	}

	UE_LOG(LogTemp, Verbose,
		TEXT("GalaxyDataGenerator::CollectActiveLargeTierCells — depth=%d grid=%d^3 total=%d active=%d (%.1f%%)"),
		CullDepth, GridSide, TotalCells, ActiveCells.Num(),
		TotalCells > 0 ? 100.0f * static_cast<float>(ActiveCells.Num()) / static_cast<float>(TotalCells) : 0.0f);

	return ActiveCells;
}

void GalaxyDataGenerator::GenerateLargeTierSlot(
	int32 InSlotIndex,
	FNiagaraParticleBuffer& InBuffer,
	int32& OutSlotCount) const
{
	// -----------------------------------------------------------------------
	// Collect SDF-active cells, then distribute the slot's candidate budget
	// across them. Each active cell is assigned:
	//
	//   CandidatesPerCell = ceil(SlotCapacity / ActiveCellCount)
	//
	// so all cells get equal sampling density. The total is capped at
	// SlotCapacity to prevent overrun. Cells use distinct seed streams
	// derived from their grid coordinate so results are stable regardless
	// of active-cell ordering.
	//
	// Writing is sequential into the slot region: each cell appends to the
	// running write cursor (BufferStart + TotalAccepted), stopping when the
	// slot is full. Dead padding is applied once at the end.
	// -----------------------------------------------------------------------

	const TArray<FActiveLargeTierCell> ActiveCells = CollectActiveLargeTierCells();

	const int32 SlotCapacity = InBuffer.SlotCapacity;
	const int32 BufferStart = InSlotIndex * SlotCapacity;
	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);

	if (ActiveCells.Num() == 0)
	{
		InBuffer.PadSlotDead(InSlotIndex, 0, DeadPos);
		OutSlotCount = 0;
		UE_LOG(LogTemp, Warning, TEXT("GalaxyDataGenerator::GenerateLargeTierSlot — no active cells found (check SDF params)"));
		return;
	}

	// Candidates per cell: divide budget evenly, rounding up so we don't
	// under-fill when ActiveCellCount doesn't divide evenly. The cap at
	// SlotCapacity ensures we never overrun even with rounding.
	const int32 CandPerCell = FMath::Max(1,
		FMath::DivideAndRoundUp(SlotCapacity, ActiveCells.Num()));

	const double InvExtent = 1.0 / static_cast<double>(Params.Extent);
	auto dCurve = Params.LargeTier.DensityResponse.GetRichCurveConst();

	int32 TotalAccepted = 0;

	for (const FActiveLargeTierCell& Cell : ActiveCells)
	{
		if (TotalAccepted >= SlotCapacity) break;

		// Clamp candidates for this cell so we never exceed the remaining budget.
		const int32 NumCandidates = FMath::Min(CandPerCell, SlotCapacity - TotalAccepted);

		// Stable seed: hash the grid coordinate so each cell's stream is
		// independent of the active-cell ordering and of InSlotIndex.
		const int32 CoordHash = HashCombine(
			HashCombine(GetTypeHash(Cell.GridCoord.X), GetTypeHash(Cell.GridCoord.Y)),
			GetTypeHash(Cell.GridCoord.Z));
		// Seed offset 0 matches the existing large tier convention.
		const int32 CellSeed = HashCombine(Params.Seed, CoordHash);
		FRandomStream Stream(CellSeed);

		// --- Phase 1: generate candidates ---
		TArray<FVector> CandidatePositions;
		TArray<float> NoiseX, NoiseY, NoiseZ;
		CandidatePositions.SetNumUninitialized(NumCandidates);
		NoiseX.SetNumUninitialized(NumCandidates);
		NoiseY.SetNumUninitialized(NumCandidates);
		NoiseZ.SetNumUninitialized(NumCandidates);

		for (int32 i = 0; i < NumCandidates; ++i)
		{
			const FVector Candidate(
				Cell.Center.X + Stream.FRandRange(-Cell.HalfExt, Cell.HalfExt),
				Cell.Center.Y + Stream.FRandRange(-Cell.HalfExt, Cell.HalfExt),
				Cell.Center.Z + Stream.FRandRange(-Cell.HalfExt, Cell.HalfExt));
			CandidatePositions[i] = Candidate;
			NoiseX[i] = static_cast<float>(Candidate.X * InvExtent);
			NoiseY[i] = static_cast<float>(Candidate.Y * InvExtent);
			NoiseZ[i] = static_cast<float>(Candidate.Z * InvExtent);
		}

		// --- Phase 2: batch density evaluation ---
		TArray<float> NoiseOut;
		NoiseOut.SetNumUninitialized(NumCandidates);
		SampleDensityBatch(NoiseOut.GetData(), NumCandidates,
			NoiseX.GetData(), NoiseY.GetData(), NoiseZ.GetData());

		// --- Phase 3: accept/reject + write ---
		for (int32 i = 0; i < NumCandidates; ++i)
		{
			if (TotalAccepted >= SlotCapacity) break;

			const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
			const float Density = FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f);
			if (Stream.FRand() > Density) continue;

			const float ScaleSample = Stream.FRand();
			const double Scale = FPointData::SampleScaleFromDistribution(
				Params.LargeTier.MinScale, Params.LargeTier.MaxScale,
				ScaleSample, Params.LargeTier.ScaleDistribution);
			FPointData PointData = FPointData::MakePointDataFromWorldScale(
				Scale, Params.UnitScale, Params.Extent);
			const double ExtentAtDepth = static_cast<double>(Params.Extent)
				/ static_cast<double>(1 << PointData.InsertDepth);
			const float FinalExtent = static_cast<float>(
				ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

			const FVector CompVec = Stream.GetUnitVector();

			const int32 Idx = BufferStart + TotalAccepted;
			InBuffer.Positions[Idx] = CandidatePositions[i];
			InBuffer.Extents[Idx] = FinalExtent;
			InBuffer.Colors[Idx] = FLinearColor(
				FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

			++TotalAccepted;
		}
	}

	InBuffer.PadSlotDead(InSlotIndex, TotalAccepted, DeadPos);
	OutSlotCount = TotalAccepted;

	UE_LOG(LogTemp, Log,
		TEXT("GalaxyDataGenerator::GenerateLargeTierSlot — %d active cells, %d/%d particles accepted"),
		ActiveCells.Num(), TotalAccepted, SlotCapacity);
}

#pragma endregion
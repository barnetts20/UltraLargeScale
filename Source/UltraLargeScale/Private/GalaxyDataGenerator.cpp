// GalaxyDataGenerator.cpp
// Density evaluation and tier generation for the galaxy layer.
//
// The density field lives in Shaders/GalaxyDensityCore.ush and is compiled by BOTH
// the shader and this module, so star placement and the rendered gas are one function
// rather than two implementations kept in agreement by hand.

#include "GalaxyDataGenerator.h"

// SHIM FIRST, then the shared field compiled INSIDE the shim's namespace.
//
// The namespace is never opened. MSVC's <cmath> declares float overloads of
// sqrt/abs/exp/pow and friends at GLOBAL scope, which libstdc++ does not; opening the
// namespace would put both sets in the same scope and every call in the field would
// be a genuine tie. Compiling the field inside the namespace instead makes ordinary
// unqualified lookup find GalaxyHLSL::sqrt and stop, never reaching ::sqrt.
#include "GalaxyHLSLShim.h"

namespace GalaxyHLSL
{
#include "GalaxyDensityCore.ush"
}

using GalaxyHLSL::float3;
using GalaxyHLSL::float4;
using GalaxyHLSL::GalaxyDensityParams;
using GalaxyHLSL::MakeGalaxyDensityParams;

#pragma region Lifetime

// Out of line so GalaxyDensityParams is complete where TUniquePtr destroys it.
GalaxyDataGenerator::GalaxyDataGenerator() = default;
GalaxyDataGenerator::GalaxyDataGenerator(FGalaxyParams InParams) : Params(MoveTemp(InParams)) {}
GalaxyDataGenerator::~GalaxyDataGenerator() = default;
GalaxyDataGenerator::GalaxyDataGenerator(GalaxyDataGenerator&&) noexcept = default;
GalaxyDataGenerator& GalaxyDataGenerator::operator=(GalaxyDataGenerator&&) noexcept = default;

#pragma endregion

#pragma region Parameter Derivation

GalaxyHLSL::GalaxyDensityParams FGalaxyDensityParams::ToDerived() const
{
	// Pure packing. Every correlation -- arm width from the disc scale height, bulge
	// and void radii from the disc radius, arm growth from the disc flare -- is
	// resolved inside MakeGalaxyDensityParams, which the material also calls. That is
	// what lets this struct and the material's pin set be the same list of raw
	// values, so neither side can drift from the other.
	return MakeGalaxyDensityParams(
		float4(ArmRadius, DiscRadius, BulgeRadius, BackgroundRadius),
		float4(ArmVerticalRatio, DiscVerticalRatio, BulgeVerticalRatio, BackgroundVerticalRatio),
		float4(ArmDensity, DiscDensity, BulgeDensity, BackgroundDensity),
		float4(ArmNoiseAmount, DiscNoiseAmount, BulgeNoiseAmount, BackgroundNoiseAmount),
		float4(WarpAmountArms, WarpAmountDisc, WarpAmountBulge, WarpAmountBackground),
		float4(ArmAsymPitch, ArmAsymPhase, ArmAsymDensity, ArmAsymLength),
		float4(ArmPitchAngle, ArmPitchTightening, ArmPhaseOffset, HaloTwistInherit),
		float3(CentralVoidRadius, CentralVoidAmount, CentralVoidExponent),
		float4(NoiseDiscLateralScale, NoiseDiscVerticalScale, NoiseHaloLateralScale, NoiseHaloVerticalScale),
		float4(WarpDiscLateralScale, WarpDiscVerticalScale, WarpHaloLateralScale, WarpHaloVerticalScale),
		float4(NoiseChannelWeights.R, NoiseChannelWeights.G, NoiseChannelWeights.B, NoiseChannelWeights.A),
		float3(NoiseOffset.X, NoiseOffset.Y, NoiseOffset.Z),
		BoundsFadeStart,
		DiscScaleRatio,
		DiscVerticalFalloff,
		DiscFlare,
		DiscWarpAmplitude,
		DiscWarpPhase,
		DiscWarpTwist,
		DiscLopsidedAmount,
		DiscLopsidedPhase,
		ArmCount,
		ArmAsymSeed,
		ArmProfileExponent,
		ArmRadialGrowth,
		ArmHostFalloff,
		BulgeConcentration,
		BackgroundConcentration,
		NoiseOctaves,
		NoiseRidged,
		bEnableNoise ? 1.0f : 0.0f);
}

float FGalaxyDensityParams::ToSpawnProbability(float InDensity) const
{
	if (InDensity <= 0.0f)
	{
		return 0.0f;
	}

	const float Ratio = InDensity / FMath::Max(SpawnDensityReference, 0.001f);

	// Linear keeps star number density proportional to gas density: accepted count is
	// candidates * p, so only p ~ d gives a region twice as dense twice the stars.
	const float Linear = FMath::Min(Ratio, 1.0f);

	// Beer-Lambert deliberately breaks that proportionality. It is the same form the
	// renderer uses for alpha, so "looks opaque" and "certainly spawns" become the
	// same statement, and dense cores fill evenly rather than becoming a wall of
	// sprites while the faint halo spawns almost nothing.
	const float Compressed = 1.0f - FMath::Exp(-Ratio);

	return FMath::Clamp(
		FMath::Lerp(Linear, Compressed, FMath::Clamp(SpawnCompression, 0.0f, 1.0f)),
		0.0f, 1.0f);
}

#pragma endregion

#pragma region Density Sampling

float GalaxyDataGenerator::SampleDensity(const FVector& InNormPos) const
{
	if (!Derived)
	{
		return 0.0f;
	}

	// SampleAnalytic is non-const only because HLSL has no such qualifier; it reads
	// and returns. TUniquePtr::operator-> hands back a non-const pointee even from a
	// const method, so this needs no cast, and it stays safe under ParallelFor.
	return Derived->SampleAnalytic(float3(
		static_cast<float>(InNormPos.X),
		static_cast<float>(InNormPos.Y),
		static_cast<float>(InNormPos.Z)));
}

void GalaxyDataGenerator::SampleDensityBatch(
	float* OutDensity,
	int32 InCount,
	const float* InX,
	const float* InY,
	const float* InZ) const
{
	if (!Derived)
	{
		FMemory::Memzero(OutDensity, sizeof(float) * InCount);
		return;
	}

	GalaxyDensityParams& P = *Derived;
	for (int32 i = 0; i < InCount; ++i)
	{
		OutDensity[i] = P.SampleAnalytic(float3(InX[i], InY[i], InZ[i]));
	}
}

#pragma endregion

#pragma region Initialization

void GalaxyDataGenerator::Initialize()
{
	// Derived once. MakeGalaxyDensityParams runs 16 arm hashes, a tan and every
	// reciprocal; per-candidate it would dominate generation.
	Derived = MakeUnique<GalaxyDensityParams>(Params.DensityParams.ToDerived());

	// PARITY PROBE.
	// The shader runs this same derivation on the values PushDensityParams sends, so
	// if the render and the star placement disagree the cause is the INPUTS, not the
	// maths. Several property names survived the FGalaxyDensityParams rewrite --
	// DiscRadius, ArmVerticalRatio, ArmProfileExponent, ArmRadialGrowth, ArmCount,
	// ArmPitchAngle, ArmPitchTightening, ArmPhaseOffset, DiscVerticalFalloff,
	// BackgroundDensity, BoundsFadeStart, ArmAsym* -- and UE keeps the SERIALIZED
	// value for a name that still exists, so those silently carry old data while only
	// the genuinely new names fall through to the C++ defaults.
	//
	// Compare this against the material instance's parameter values. A mismatch on a
	// carried-over name is the expected failure.
	{
		const FGalaxyDensityParams& D = Params.DensityParams;
		UE_LOG(LogTemp, Log, TEXT("=== GalaxyDensity inputs (seed %d) ==="), Params.Seed);
		UE_LOG(LogTemp, Log, TEXT("  lateral   arm %.4f  disc %.4f  bulge %.4f  bg %.4f"),
			D.ArmRadius, D.DiscRadius, D.BulgeRadius, D.BackgroundRadius);
		UE_LOG(LogTemp, Log, TEXT("  vertical  arm %.4f  disc %.4f  bulge %.4f  bg %.4f"),
			D.ArmVerticalRatio, D.DiscVerticalRatio, D.BulgeVerticalRatio, D.BackgroundVerticalRatio);
		UE_LOG(LogTemp, Log, TEXT("  density   arm %.4f  disc %.4f  bulge %.4f  bg %.4f"),
			D.ArmDensity, D.DiscDensity, D.BulgeDensity, D.BackgroundDensity);
		UE_LOG(LogTemp, Log, TEXT("  arms      count %.2f  pitch %.2f  tighten %.2f  growth %.3f  host %.3f  profile %.3f"),
			D.ArmCount, D.ArmPitchAngle, D.ArmPitchTightening, D.ArmRadialGrowth, D.ArmHostFalloff, D.ArmProfileExponent);
		UE_LOG(LogTemp, Log, TEXT("  disc      flare %.3f  scaleRatio %.3f  vFalloff %.3f  boundsFade %.3f"),
			D.DiscFlare, D.DiscScaleRatio, D.DiscVerticalFalloff, D.BoundsFadeStart);

		UE_LOG(LogTemp, Log, TEXT("=== derived ==="));
		UE_LOG(LogTemp, Log, TEXT("  ArmWidth %.6f   ArmRadialGrowth %.3f   ArmN %d   K0 %.4f"),
			Derived->ArmWidth, Derived->ArmRadialGrowth, Derived->ArmN, Derived->ArmSpiralK0);
		UE_LOG(LogTemp, Log, TEXT("  OverPath  arm %.2f  disc %.2f  bulge %.2f  bg %.4f"),
			Derived->ArmDensityOverPath, Derived->DiscDensityOverPath,
			Derived->BulgeDensityOverPath, Derived->BackgroundDensityOverPath);

		// Half-heights at mid-disc, the two numbers the silhouette is made of.
		const float rn = 0.5f;
		const float edge = 1.0f - rn * rn;
		const float DiscH = Derived->DiscRadius * Derived->DiscHeightRatio
			* (1.0f + Derived->DiscFlare * rn) * FMath::Sqrt(edge);
		const float ArmH = Derived->ArmWidth
			* FMath::Lerp(1.0f, Derived->ArmRadialGrowth, rn) * FMath::Sqrt(edge)
			/ FMath::Max(Derived->InvArmVerticalRatio, 1e-6f);
		UE_LOG(LogTemp, Log, TEXT("  at rn=0.5: disc h %.5f   arm H %.5f   ratio %.2f"),
			DiscH, ArmH, DiscH > 1e-9f ? ArmH / DiscH : 0.0f);
	}

	// Kept for a future FastNoise swap-in; unused on the active path.
	DensityNoise = BuildNoise();
}

FastNoise::SmartNode<> GalaxyDataGenerator::BuildNoise() const
{
	return FastNoise::NewFromEncodedNodeTree(Params.EncodedTree);
}

TArray<uint8> GalaxyDataGenerator::SampleNoiseVolume(int InNoiseResolution) const
{
	// BGRA8, 4 bytes per voxel, linear [z][y][x] order, density in alpha.
	//
	// TRANSITIONAL, and it cannot represent the field faithfully: at 256 a voxel is
	// 0.0078 in normalized space against an arm half-width near 0.011, so arms are
	// under three voxels across. Gross structure reads, fine detail does not. That is
	// a property of the texture, not of the field.

	const int BytesPerVoxel = 4;
	const int64 TotalBytes = (int64)InNoiseResolution * InNoiseResolution * InNoiseResolution * BytesPerVoxel;

	TArray<uint8> TextureData;
	TextureData.SetNumZeroed(TotalBytes);

	const double VoxelSize = 2.0 / InNoiseResolution;  // normalized [-1, 1] space

	// The field is an unbounded optical depth and this channel is a byte. Without the
	// divide every voxel where density exceeds 1.0 -- the arms, the inner disc and
	// the entire bulge -- writes 255 and the bake is a solid blob.
	const float InvBakeRef = 1.0f / FMath::Max(Params.DensityParams.BakeDensityReference, 0.001f);
	const float BakePower = Params.DensityParams.MasterDensityPower;

	ParallelFor(InNoiseResolution, [&](int z)
		{
			const double nz = -1.0 + (z + 0.5) * VoxelSize;

			for (int y = 0; y < InNoiseResolution; ++y)
			{
				const double ny = -1.0 + (y + 0.5) * VoxelSize;

				for (int x = 0; x < InNoiseResolution; ++x)
				{
					const double nx = -1.0 + (x + 0.5) * VoxelSize;

					float Density = SampleDensity(FVector(nx, ny, nz)) * InvBakeRef;
					Density = FMath::Pow(FMath::Clamp(Density, 0.0f, 1.0f), BakePower);

					const uint8 DensityByte =
						static_cast<uint8>(FMath::Clamp(Density * 255.0f, 0.0f, 255.0f));

					const int64 Idx = ((int64)z * InNoiseResolution * InNoiseResolution
						+ (int64)y * InNoiseResolution + x) * BytesPerVoxel;
					TextureData[Idx + 3] = DensityByte;  // Alpha channel
				}
			}
		}, EParallelForFlags::BackgroundPriority);

	return TextureData;
}

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
	//   1. Generate candidate positions + normalized coords.
	//   2. Batch density evaluation via SampleDensityBatch.
	//   3. Walk results, rejection-gate, write accepted to slot buffers.
	//
	// For the large tier, InNodeCenter is ZeroVector and InCellExtent is
	// Params.Extent, so candidates span the full galaxy volume. For mid/small
	// tiers, candidates are local to the cell.
	// -----------------------------------------------------------------------

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed + InSeedOffset, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = InBuffer.SlotCapacity;
	const double InvExtent = 1.0 / static_cast<double>(Params.Extent);

	// Phase 1: generate candidates + normalized coords
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

		// Normalize to [-1, 1] relative to galaxy center. The galaxy is
		// self-contained so no cross-cell snapping is needed.
		NoiseX[i] = static_cast<float>(Candidate.X * InvExtent);
		NoiseY[i] = static_cast<float>(Candidate.Y * InvExtent);
		NoiseZ[i] = static_cast<float>(Candidate.Z * InvExtent);
	}

	// Phase 2: batch density evaluation
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	SampleDensityBatch(
		NoiseOut.GetData(), NumCandidates,
		NoiseX.GetData(), NoiseY.GetData(), NoiseZ.GetData());

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// Phase 3: accept/reject + write to slot
	int32 ActualCount = 0;
	auto dCurve = InTierParams.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		// The field is an optical depth, not a probability: it peaks in the
		// hundreds while most of the volume sits below 0.01. Clamping it to [0,1]
		// here would accept every candidate across the arms, the inner disc and
		// the bulge, flattening the distribution exactly where the structure is.
		const float RawDensity = Params.DensityParams.ToSpawnProbability(NoiseOut[i]);

		// Response curve gates spawning only; it now sees a value genuinely in
		// [0,1]. Raw density is used for particle sizing below and never feeds
		// the pseudovolume texture.
		const float SpawnDensity = (dCurve && dCurve->GetNumKeys() > 0)
			? FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f)
			: RawDensity;
		if (Stream.FRand() > SpawnDensity) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			InTierParams.MinScale, InTierParams.MaxScale,
			ScaleSample, InTierParams.ScaleDistribution);

		// AUTHORED SIZE IS TRUTH: convert the real-unit Scale through the
		// layer's constant UnitScale exactly once. Octree insert depth is
		// derived later, at insert time, by InsertParticleIntoOctree, so no
		// FPointData is needed here. Never reconstruct size from the quantized
		// depth: that would inflate every particle to its power-of-two node
		// extent and make sprite sizes octave-step with UnitScale.
		const float FinalExtent = static_cast<float>(Scale / Params.UnitScale);

		const FVector CompVec = Stream.GetUnitVector();

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Extents[Idx] = FinalExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	InBuffer.PadSlotDead(InSlotIndex, ActualCount);
	OutSlotCount = ActualCount;
}

#pragma endregion

#pragma region Large Tier Culling
//TODO: Check if this block should be shared
TArray<GalaxyDataGenerator::FActiveLargeTierCell> GalaxyDataGenerator::CollectActiveLargeTierCells() const
{
	// -----------------------------------------------------------------------
	// Subdivide [-Extent, +Extent]^3 into a uniform grid at LargeTierCullDepth and,
	// for each cell, find the PEAK density inside it. A cell whose peak is zero lies
	// entirely outside every layer's support and is skipped; the rest carry their
	// peak forward as a local rejection envelope.
	//
	// Sampling corners alone is not enough for either job. An arm is narrower than a
	// cell through most of the disc, so one can pass through a cell's interior
	// without reaching any vertex -- which discards a live cell outright, and
	// under-estimates the envelope of the cells it does keep. An under-estimated
	// envelope clips the peak, flattening exactly the structure the field describes.
	// A jittered interior set costs a few more evaluations in a prepass that runs
	// once per galaxy.
	//
	// The peak is padded because it remains an estimate from a finite sample. Erring
	// high costs acceptance rate; erring low costs fidelity, and only one of those is
	// recoverable.
	// -----------------------------------------------------------------------

	const int32 CullDepth = FMath::Max(Params.LargeTierCullDepth, 1);
	const int32 GridSide = 1 << CullDepth;          // cells per axis
	const double FullExtent = static_cast<double>(Params.Extent);
	const double CellFull = (2.0 * FullExtent) / static_cast<double>(GridSide);
	const double HalfCell = CellFull * 0.5;
	const double InvExtent = 1.0 / FullExtent;

	// 8 corner offsets in cell-local space (+/-HalfCell on each axis)
	static const FVector CornerOffsets[8] =
	{
		FVector(-1, -1, -1), FVector(1, -1, -1),
		FVector(-1,  1, -1), FVector(1,  1, -1),
		FVector(-1, -1,  1), FVector(1, -1,  1),
		FVector(-1,  1,  1), FVector(1,  1,  1),
	};

	// Interior probes per cell, on top of the 8 corners. Deterministic so the cell set
	// is reproducible for a given seed.
	const int32 InteriorProbes = 24;
	const float EnvelopePad = 1.5f;

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

				// No early-out: we need the PEAK, not merely "is anything here".
				float CellMax = 0.0f;

				for (int32 c = 0; c < 8; ++c)
				{
					const FVector CornerNorm = (Center + CornerOffsets[c] * HalfCell) * InvExtent;

					// Hard bounds check: SampleDensity already returns 0 beyond the
					// unit sphere, but an explicit check skips the full evaluation
					// for corners clearly outside the extents.
					if (FMath::Abs(CornerNorm.X) > 1.0 ||
						FMath::Abs(CornerNorm.Y) > 1.0 ||
						FMath::Abs(CornerNorm.Z) > 1.0)
					{
						continue;
					}

					CellMax = FMath::Max(CellMax, SampleDensity(CornerNorm));
				}

				// Interior probes. Seeded from the grid coordinate so the same cell
				// yields the same probes on every regeneration.
				const int32 CellHash = HashCombine(
					HashCombine(GetTypeHash(ix), GetTypeHash(iy)), GetTypeHash(iz));
				FRandomStream ProbeStream(HashCombine(Params.Seed, CellHash));

				for (int32 p = 0; p < InteriorProbes; ++p)
				{
					const FVector ProbeNorm = (Center + FVector(
						ProbeStream.FRandRange(-HalfCell, HalfCell),
						ProbeStream.FRandRange(-HalfCell, HalfCell),
						ProbeStream.FRandRange(-HalfCell, HalfCell))) * InvExtent;

					if (FMath::Abs(ProbeNorm.X) > 1.0 ||
						FMath::Abs(ProbeNorm.Y) > 1.0 ||
						FMath::Abs(ProbeNorm.Z) > 1.0)
					{
						continue;
					}

					CellMax = FMath::Max(CellMax, SampleDensity(ProbeNorm));
				}

				if (CellMax > 0.0f)
				{
					FActiveLargeTierCell Cell;
					Cell.Center = Center;
					Cell.HalfExt = HalfCell;
					Cell.GridCoord = FIntVector(ix, iy, iz);
					Cell.MaxDensity = CellMax * EnvelopePad;
					ActiveCells.Add(Cell);
				}
			}
		}
	}

	UE_LOG(LogTemp, Verbose,
		TEXT("GalaxyDataGenerator::CollectActiveLargeTierCells - depth=%d grid=%d^3 total=%d active=%d (%.1f%%)"),
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
	// Collect active cells, then distribute the slot's candidate budget across them
	// and rejection-sample inside each against ITS OWN peak density.
	//
	// The two halves have to be paired correctly or the distribution is wrong.
	// Accepted count per cell is budget x mean(d)/envelope, and what we want is
	// proportional to cell mass, i.e. to mean(d):
	//
	//   budget ~ max,  envelope = cellMax   -> accepted ~ mean          EXACT
	//   budget ~ mean, envelope = globalRef -> accepted ~ mean^2        1.7x biased
	//   budget ~ mean, envelope = cellMax   -> accepted ~ mean^2/max    3.9x biased
	//
	// So the budget weights on cell MAX, not cell mean. Weighting by mean while also
	// rejecting by density counts the same factor twice and over-concentrates in the
	// arms quadratically -- which is what the previous pairing did.
	//
	// A local envelope is also what makes this affordable: the field spans four
	// decades across the galaxy but a narrow band within one cell, so acceptance
	// rises from a fraction of a percent to roughly a quarter.
	//
	// Weights are RAW density, deliberately -- the allocation is proportional, so an
	// unbounded scale cancels in CellWeight/TotalWeight. Only the per-candidate gate
	// needs a probability, and it gets one by dividing by the cell's own envelope.
	//
	// Minimum of 1 candidate per active cell so none is silently skipped, and a final
	// clamp so the sum never exceeds SlotCapacity. Writing is sequential into the
	// slot region; dead padding applied once at the end.
	// -----------------------------------------------------------------------

	const TArray<FActiveLargeTierCell> ActiveCells = CollectActiveLargeTierCells();

	const int32 SlotCapacity = InBuffer.SlotCapacity;
	const int32 BufferStart = InSlotIndex * SlotCapacity;

	if (ActiveCells.Num() == 0)
	{
		InBuffer.PadSlotDead(InSlotIndex, 0);
		OutSlotCount = 0;
		UE_LOG(LogTemp, Warning, TEXT("GalaxyDataGenerator::GenerateLargeTierSlot - no active cells found (check density params)"));
		return;
	}

	// Budget weights: the cell's ENVELOPE, already computed by the collector. No
	// resampling needed, and pairing envelope-weighted budgets with envelope-relative
	// acceptance is what makes the result exactly proportional to the field.
	const double InvExtent = 1.0 / static_cast<double>(Params.Extent);

	TArray<float> CellWeights;
	CellWeights.SetNumUninitialized(ActiveCells.Num());
	float TotalWeight = 0.0f;

	for (int32 ci = 0; ci < ActiveCells.Num(); ++ci)
	{
		CellWeights[ci] = FMath::Max(ActiveCells[ci].MaxDensity, 1e-6f); // never starve a cell
		TotalWeight += CellWeights[ci];
	}

	// Allocate candidate counts proportional to weight
	TArray<int32> CandidateCounts;
	CandidateCounts.SetNumZeroed(ActiveCells.Num());
	int32 TotalAllocated = 0;

	for (int32 ci = 0; ci < ActiveCells.Num(); ++ci)
	{
		const int32 Count = FMath::Max(1,
			FMath::RoundToInt(static_cast<float>(SlotCapacity) * CellWeights[ci] / TotalWeight));
		CandidateCounts[ci] = Count;
		TotalAllocated += Count;
	}

	// Trim or top-up the last cell to hit exactly SlotCapacity.
	// Rounding errors are typically +/-1 per cell so the delta is small.
	const int32 Delta = SlotCapacity - TotalAllocated;
	CandidateCounts.Last() = FMath::Max(1, CandidateCounts.Last() + Delta);

	// Generate per cell
	auto dCurve = Params.LargeTier.DensityResponse.GetRichCurveConst();
	int32 TotalAccepted = 0;

	for (int32 ci = 0; ci < ActiveCells.Num(); ++ci)
	{
		if (TotalAccepted >= SlotCapacity) break;

		const FActiveLargeTierCell& Cell = ActiveCells[ci];
		const int32 NumCandidates = FMath::Min(CandidateCounts[ci], SlotCapacity - TotalAccepted);

		// Stable per-cell seed derived from grid coordinate.
		const int32 CoordHash = HashCombine(
			HashCombine(GetTypeHash(Cell.GridCoord.X), GetTypeHash(Cell.GridCoord.Y)),
			GetTypeHash(Cell.GridCoord.Z));
		FRandomStream Stream(HashCombine(Params.Seed, CoordHash));

		// Phase 1: candidates
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

		// Phase 2: batch density eval
		TArray<float> NoiseOut;
		NoiseOut.SetNumUninitialized(NumCandidates);
		SampleDensityBatch(NoiseOut.GetData(), NumCandidates,
			NoiseX.GetData(), NoiseY.GetData(), NoiseZ.GetData());

		// Phase 3: accept/reject + write
		for (int32 i = 0; i < NumCandidates; ++i)
		{
			if (TotalAccepted >= SlotCapacity) break;

			// Rejection against the CELL'S OWN envelope rather than a global
			// reference. This is the whole point of the local envelope: the ratio is
			// order-1 across the cell instead of order-1e-3, so most candidates land.
			//
			// SpawnDensityReference is deliberately not used here; it remains the
			// mapping for the mid and small tiers, which have no prepass to derive a
			// local bound from.
			const float RawDensity = FMath::Clamp(NoiseOut[i] / Cell.MaxDensity, 0.0f, 1.0f);

			// Density response curve gates spawning only.
			const float SpawnDensity = (dCurve && dCurve->GetNumKeys() > 0)
				? FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f)
				: RawDensity;
			if (Stream.FRand() > SpawnDensity) continue;

			const float ScaleSample = Stream.FRand();
			const double Scale = FPointData::SampleScaleFromDistribution(
				Params.LargeTier.MinScale, Params.LargeTier.MaxScale,
				ScaleSample, Params.LargeTier.ScaleDistribution);
			// Authored size is truth (see the tier note above). No FPointData
			// here - insert depth is derived at octree-insert time.
			const float FinalExtent = static_cast<float>(Scale / Params.UnitScale);

			const FVector CompVec = Stream.GetUnitVector();

			const int32 Idx = BufferStart + TotalAccepted;
			InBuffer.Positions[Idx] = CandidatePositions[i];
			InBuffer.Extents[Idx] = FinalExtent;
			InBuffer.Colors[Idx] = FLinearColor(
				FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

			++TotalAccepted;
		}
	}

	InBuffer.PadSlotDead(InSlotIndex, TotalAccepted);
	OutSlotCount = TotalAccepted;

	UE_LOG(LogTemp, Log,
		TEXT("GalaxyDataGenerator::GenerateLargeTierSlot - %d active cells, %d/%d particles accepted"),
		ActiveCells.Num(), TotalAccepted, SlotCapacity);
}

#pragma endregion
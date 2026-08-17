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
	// Subdivide [-Extent, +Extent]^3 into a uniform grid at LargeTierCullDepth.
	// For each cell, evaluate SampleDensity at all 8 corners (in normalized
	// [-1, 1] space). If every corner returns zero, the cell is entirely
	// outside every layer's support and is skipped. Any cell with at least one
	// corner > 0 is kept.
	//
	// We intentionally test corners rather than the cell center so that cells
	// straddling a boundary are never wrongly discarded. A cell whose center
	// happens to fall in the gap between two arms but whose corner clips an arm
	// edge will still be included - candidates generated inside it will simply
	// fail the per-candidate rejection gate as normal, at very low cost.
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

				// Test all 8 corners in normalized space. Early-out as soon as
				// any corner has non-zero density.
				bool bAnyActive = false;
				for (int32 c = 0; c < 8; ++c)
				{
					const FVector CornerWorld = Center + CornerOffsets[c] * HalfCell;
					const FVector CornerNorm = CornerWorld * InvExtent;

					// Hard bounds check: SampleDensity already returns 0 beyond
					// the unit sphere, but an explicit check lets us skip the
					// full evaluation for corners clearly outside the extents.
					if (FMath::Abs(CornerNorm.X) > 1.0 ||
						FMath::Abs(CornerNorm.Y) > 1.0 ||
						FMath::Abs(CornerNorm.Z) > 1.0)
					{
						continue; // This corner is outside - try next
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
	// Collect active cells, then distribute the slot's candidate budget
	// proportionally to each cell's mean corner density. Dense arm-core cells
	// receive more candidates than cells that merely clip an edge, so the final
	// particle distribution mirrors the density field rather than treating all
	// active cells equally.
	//
	// Budget allocation:
	//   CellWeight[i]     = mean SampleDensity of the cell's 8 corners
	//   CellCandidates[i] = round( SlotCapacity * CellWeight[i] / TotalWeight )
	//
	// with a minimum of 1 candidate per active cell so no cell is silently
	// skipped, and a final clamp so the sum never exceeds SlotCapacity.
	//
	// The weights are RAW density, deliberately: the allocation is proportional,
	// so an unbounded scale cancels in CellWeight/TotalWeight. Only the
	// per-candidate gate needs a probability.
	//
	// Writing is sequential into the slot region. Dead padding applied once at
	// the end.
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

	// Compute per-cell mean corner density and total weight
	const double InvExtent = 1.0 / static_cast<double>(Params.Extent);
	static const FVector CornerOffsets[8] =
	{
		FVector(-1,-1,-1), FVector(1,-1,-1),
		FVector(-1, 1,-1), FVector(1, 1,-1),
		FVector(-1,-1, 1), FVector(1,-1, 1),
		FVector(-1, 1, 1), FVector(1, 1, 1),
	};

	TArray<float> CellWeights;
	CellWeights.SetNumUninitialized(ActiveCells.Num());
	float TotalWeight = 0.0f;

	for (int32 ci = 0; ci < ActiveCells.Num(); ++ci)
	{
		const FActiveLargeTierCell& Cell = ActiveCells[ci];
		float CornerSum = 0.0f;
		for (int32 c = 0; c < 8; ++c)
		{
			const FVector CornerNorm = (Cell.Center + CornerOffsets[c] * Cell.HalfExt) * InvExtent;
			if (FMath::Abs(CornerNorm.X) <= 1.0 &&
				FMath::Abs(CornerNorm.Y) <= 1.0 &&
				FMath::Abs(CornerNorm.Z) <= 1.0)
			{
				CornerSum += SampleDensity(CornerNorm);
			}
		}
		CellWeights[ci] = FMath::Max(CornerSum / 8.0f, 1e-6f); // minimum weight so cell isn't starved
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

			// See GenerateTierNode: the field is an optical depth, so it must be
			// mapped to a probability before gating rather than clamped.
			const float RawDensity = Params.DensityParams.ToSpawnProbability(NoiseOut[i]);

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
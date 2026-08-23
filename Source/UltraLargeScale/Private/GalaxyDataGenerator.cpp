// GalaxyDataGenerator.cpp
// Density evaluation and tier generation for the galaxy layer.
//
// The density field lives in Shaders/GalaxyDensityCore.ush and is compiled by BOTH
// the shader and this module, so star placement and the rendered gas are one function
// rather than two implementations kept in agreement by hand.

#include "GalaxyDataGenerator.h"

#include "GalaxyEntityGen.h"
#include "GalaxyGenProbe.h"
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

float GalaxyDataGenerator::EnvelopePadding() const
{
	// TWO errors to cover, and only one of them is a fixed factor.
	//
	// The first is sampling: the estimate is a max over 32 points, so it under-shoots
	// the true peak inside the cell by an amount that depends on how sharp the field is
	// there. That is the fixed part.
	//
	// The second is SYSTEMATIC and was being ignored. The probes run SampleAnalytic on
	// the CPU while the dispatch rejects against GalaxySample, which multiplies by
	// max(1 + Amount * n, 0) with n roughly in [-1,1]. The textured field can therefore
	// reach (1 + Amount) times the analytic value at the same point, and the LARGEST
	// per-layer amount bounds it because a candidate can land in any layer.
	//
	// A fixed 1.5 covered this only while every amount stayed under 0.5.
	// BackgroundNoiseAmount defaults to 1.0 and BulgeNoiseAmount to 0.5, so the
	// envelope was already being exceeded at default tuning -- and an exceeded envelope
	// clips the ratio at 1, flattening exactly the peaks the prepass went looking for.
	//
	// Erring high costs acceptance rate; erring low costs fidelity, and only one of
	// those is recoverable.
	constexpr float SampleMiss = 1.15f;

	const FGalaxyDensityParams& D = Params.DensityParams;

	if (!D.bEnableNoise)
	{
		return SampleMiss;
	}

	const float MaxAmount = FMath::Max(
		FMath::Max(D.ArmNoiseAmount, D.DiscNoiseAmount),
		FMath::Max(D.BulgeNoiseAmount, D.BackgroundNoiseAmount));

	return SampleMiss * (1.0f + FMath::Max(MaxAmount, 0.0f));
}

float GalaxyDataGenerator::EstimateCellEnvelope(
	const FIntVector& InCoord,
	const FVector& InCentre,
	double InHalfExtent) const
{
	// 8 corner offsets in cell-local space (+/-HalfExtent on each axis).
	static const FVector CornerOffsets[8] =
	{
		FVector(-1, -1, -1), FVector(1, -1, -1),
		FVector(-1,  1, -1), FVector(1,  1, -1),
		FVector(-1, -1,  1), FVector(1, -1,  1),
		FVector(-1,  1,  1), FVector(1,  1,  1),
	};

	constexpr int32 InteriorProbes = 24;

	const double InvExtent = 1.0 / FMath::Max(static_cast<double>(Params.Extent), 1e-9);

	// The candidate span MakeCandidate uses is +/-HalfExtent about the centre, so the
	// probes cover exactly the region the envelope has to bound. Sampling a smaller
	// region would leave candidates outside it rejecting against a peak that does not
	// apply to them.
	float CellMax = 0.0f;

	auto Probe = [this, InvExtent, &CellMax](const FVector& InPoint)
		{
			const FVector Norm = InPoint * InvExtent;

			// SampleDensity already returns zero outside the field, but an explicit
			// check skips the full evaluation for points clearly beyond the extents.
			if (FMath::Abs(Norm.X) > 1.0 ||
				FMath::Abs(Norm.Y) > 1.0 ||
				FMath::Abs(Norm.Z) > 1.0)
			{
				return;
			}

			CellMax = FMath::Max(CellMax, SampleDensity(Norm));
		};

	for (int32 c = 0; c < 8; ++c)
	{
		Probe(InCentre + CornerOffsets[c] * InHalfExtent);
	}

	// Seeded from the CELL COORD, not from anything about the batch. The same cell must
	// yield the same envelope whichever neighbourhood it arrives in, or its candidate
	// budget shifts between visits and the region regenerates differently.
	const int32 CellHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z));

	FRandomStream ProbeStream(HashCombine(Params.Seed, CellHash));

	for (int32 p = 0; p < InteriorProbes; ++p)
	{
		Probe(InCentre + FVector(
			ProbeStream.FRandRange(-InHalfExtent, InHalfExtent),
			ProbeStream.FRandRange(-InHalfExtent, InHalfExtent),
			ProbeStream.FRandRange(-InHalfExtent, InHalfExtent)));
	}

	// Zero means empty, and stays exactly zero so the caller can test it. Padding is
	// applied only to a real peak.
	return (CellMax > 0.0f) ? CellMax * EnvelopePadding() : 0.0f;
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

bool GalaxyDataGenerator::BuildLargeTierCells(
	const TArray<TPair<FIntVector, int32>>& InSlots,
	TArray<FTierBatchCell>& OutCells) const
{
	OutCells.Reset();

	if (InSlots.Num() == 0)
	{
		return true;
	}

	const TArray<FActiveLargeTierCell> ActiveCells = CollectActiveLargeTierCells();
	if (ActiveCells.Num() == 0)
	{
		return false;
	}

	// The large tier has ONE slot, and its active cells all feed it. Every cell is
	// tagged with that slot so the scatter writes them into the same run, and the
	// dispatch's per-cell counters still keep the runs apart.
	//
	// Not the grid coord of the slot: the KEY has to be the cell's own coord or two
	// cells sharing a slot would draw identical candidates.
	const int32 SlotIndex = InSlots[0].Value;

	// Budget SHARED across the active set, weighted by each cell's peak density --
	// the same distribution the per-slot CPU path used, and for the same reason.
	//
	// Accepted count per cell is budget x mean(d)/envelope, so weighting the budget by
	// MAX and rejecting against that cell's own max gives accepted ~ mean, which is
	// proportional to cell mass. Weighting by mean instead would count the same factor
	// twice and over-concentrate in the arms quadratically.
	const int32 Budget = FMath::Max(Params.LargeTier.CandidateBudget, ActiveCells.Num());

	double TotalWeight = 0.0;
	for (const FActiveLargeTierCell& Cell : ActiveCells)
	{
		TotalWeight += FMath::Max(static_cast<double>(Cell.MaxDensity), 1e-6);
	}

	OutCells.Reserve(ActiveCells.Num());

	for (const FActiveLargeTierCell& Cell : ActiveCells)
	{
		const double Weight = FMath::Max(static_cast<double>(Cell.MaxDensity), 1e-6);

		FTierBatchCell Out;
		Out.Coord = Cell.GridCoord;
		Out.SlotIndex = SlotIndex;
		Out.Centre = Cell.Center;
		Out.HalfExtent = Cell.HalfExt;
		Out.DensityReference = Cell.MaxDensity;
		Out.Candidates = FMath::Max(1,
			FMath::RoundToInt(static_cast<double>(Budget) * Weight / TotalWeight));

		OutCells.Add(Out);
	}

	return true;
}

void GalaxyDataGenerator::ApplyCellEnvelopes(
	TArray<FTierBatchCell>& InOutCells,
	const FTierParams& InTierParams) const
{
	// The budget anchor. A cell whose peak reaches this draws the full tier budget;
	// everything fainter draws proportionally fewer candidates and therefore accepts
	// proportionally fewer entities, which is what carries the structure BETWEEN cells
	// now that each cell rejects against its own peak.
	const float GlobalReference =
		FMath::Max(Params.DensityParams.SpawnDensityReference, 1e-3f);

	const int32 Budget = FMath::Max(InTierParams.CandidateBudget, 1);

	int32 ClampedCells = 0;
	int32 EmptyCells = 0;

	for (FTierBatchCell& Cell : InOutCells)
	{
		const float Envelope = EstimateCellEnvelope(Cell.Coord, Cell.Centre, Cell.HalfExtent);

		if (Envelope <= 0.0f)
		{
			// Nothing here. The cell still has to appear in the batch so its slot gets
			// padded dead, but it should not spend threads.
			//
			// One candidate rather than zero because zero is already spoken for:
			// FTierBatchCell::Candidates uses it to mean "take the tier's budget", so a
			// literal zero would hand this cell the full budget instead of none.
			Cell.DensityReference = GlobalReference;
			Cell.Candidates = 1;
			++EmptyCells;
			continue;
		}

		Cell.DensityReference = Envelope;

		// Clamped at the tier budget so the dispatch stride -- which is the LARGEST
		// per-cell budget -- cannot be inflated by one unusually dense cell.
		const int32 Weighted = FMath::RoundToInt(
			static_cast<double>(Budget) * static_cast<double>(Envelope)
			/ static_cast<double>(GlobalReference));

		Cell.Candidates = FMath::Clamp(Weighted, 1, Budget);

		if (Weighted > Budget)
		{
			++ClampedCells;
		}
	}

	// Clamping means the cell wanted more candidates than the anchor allows, so its
	// accepted count no longer tracks its mass and inter-cell contrast flattens at the
	// top end. A few clamped cells are fine; a lot means SpawnDensityReference is sitting
	// well below what the field actually reaches and should be raised toward its peak.
	//
	// Note this is the OPPOSITE reason to the one the reference used to be raised for.
	// It is no longer the rejection envelope for these tiers -- the per-cell peak is --
	// so raising it no longer costs acceptance. It only re-anchors the budget.
	if (ClampedCells * 4 > InOutCells.Num())
	{
		static bool bWarnedClamp = false;
		if (!bWarnedClamp)
		{
			bWarnedClamp = true;
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: %d of %d cells clamped at the tier budget (%d empty). ")
				TEXT("SpawnDensityReference %.3f is below the field's working range, so ")
				TEXT("candidate budgets are pinned and inter-cell contrast is flattening. ")
				TEXT("Raise it toward the peak density the field actually reaches."),
				ClampedCells, InOutCells.Num(), EmptyCells,
				Params.DensityParams.SpawnDensityReference);
		}
	}
}

bool GalaxyDataGenerator::GenerateTierBatchGPU(
	const TArray<FTierBatchCell>& InCells,
	FNiagaraParticleBuffer& InBuffer,
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	TArray<int32>& OutSlotCounts) const
{
	// Once per run, not per batch. This confirms the compute plumbing -- module, shader
	// path, parameter struct, UAV binding, dispatch, readback -- without touching a real
	// shader, which is what separated setup from shader in the first place. It costs
	// microseconds, but this function runs on every boundary cross.
	static bool bProbed = false;
	if (!bProbed)
	{
		bProbed = true;
		GalaxyGenProbe::Run();
	}

	if (InCells.Num() == 0)
	{
		return true;
	}

	// FAIL CLOSED, VISIBLY. There is no CPU path behind this any more, so a failure
	// here means these slots get nothing -- and a slot is REUSED as the player crosses
	// boundaries, so "nothing written" is not an empty slot, it is the previous
	// occupant's entities still sitting there at a coord they no longer belong to.
	// That reads as a placement bug rather than a generation failure.
	//
	// Blanking costs an empty region, which is honest, and the caller logs.
	auto FailBatch = [&InCells, &InBuffer, &OutSlotCounts]() -> bool
		{
			OutSlotCounts.SetNumZeroed(InBuffer.SlotCoord.Num());

			for (const FTierBatchCell& Cell : InCells)
			{
				if (OutSlotCounts.IsValidIndex(Cell.SlotIndex))
				{
					InBuffer.PadSlotDead(Cell.SlotIndex, 0);
					OutSlotCounts[Cell.SlotIndex] = 0;
				}
			}

			return false;
		};

	// LOUD ONCE, then quiet. ensure fires on its first hit per call site per session,
	// which is exactly right for a setup error: it stops the developer and lands in the
	// log, and it does not then spam a warning on every boundary cross for the rest of
	// the run. A misconfiguration would otherwise surface only as a tier that never
	// populates, which reads as a streaming problem rather than a setup one.
	if (Params.NoiseTexture == nullptr)
	{
		ensureMsgf(false,
			TEXT("GalaxyEntityGen: NoiseTexture is unset, so the galaxy will generate ")
			TEXT("NOTHING -- placement is GPU-only and the dispatch samples it. Set it to ")
			TEXT("the same volume texture the material samples, with NEVER STREAM on the ")
			TEXT("asset."));
		return FailBatch();
	}

	// One cell per queued slot. The KEY is the grid coord, never the dispatch index:
	// identity is (coord, slot), which is what makes a region regenerate identically
	// after the player leaves and returns.
	TArray<FGalaxyGenCell> Cells;
	Cells.Reserve(InCells.Num());

	// The dispatch is sized on the LARGEST per-cell budget; cells wanting fewer simply
	// have their surplus threads return early. Sizing on the sum instead would need a
	// prefix sum to map a thread back to its cell, for no gain when the budgets are
	// near-uniform.
	// The run reserved per cell is the slot's capacity divided by HOW MANY CELLS SHARE
	// THAT SLOT, not the capacity itself.
	//
	// Mid and small have one cell per slot, so they get the whole capacity. The large
	// tier feeds its entire active set -- fourteen hundred cells -- into a single slot
	// whose capacity is three thousand, so a full run each reserved four million
	// entries to hold at most three thousand entities. Sizing by share keeps the
	// readback proportional to what can actually be kept.
	//
	// Four times the fair share, because acceptance is uneven: a dense cell should be
	// able to exceed its average without being clipped, and the total is still capped
	// by the slot capacity during the scatter.
	int32 MaxCellsPerSlot = 1;
	{
		TMap<int32, int32> CellsPerSlot;
		for (const FTierBatchCell& In : InCells)
		{
			MaxCellsPerSlot = FMath::Max(MaxCellsPerSlot, ++CellsPerSlot.FindOrAdd(In.SlotIndex));
		}
	}

	const int32 FairShare = FMath::DivideAndRoundUp(
		FMath::Max(InBuffer.SlotCapacity, 1), MaxCellsPerSlot);

	const int32 SlotStride = FMath::Clamp(FairShare * 4, 8,
		FMath::Max(InBuffer.SlotCapacity, 1));

	// The tier's nominal budget, used by any cell that does not state its own.
	const int32 CandidateBudget = FMath::Max(InTierParams.CandidateBudget, 1);

	int32 MaxCandidates = 0;

	for (const FTierBatchCell& In : InCells)
	{
		FGalaxyGenCell Cell;
		Cell.Centre = FVector3f(
			static_cast<float>(In.Centre.X),
			static_cast<float>(In.Centre.Y),
			static_cast<float>(In.Centre.Z));
		Cell.HalfExtent = static_cast<float>(In.HalfExtent);

		// The KEY is the grid coord, never the dispatch index: identity is
		// (coord, slot), which is what makes a region regenerate identically after
		// the player leaves and returns.
		Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);

		// Zero means "use the global reference". The large tier supplies its own
		// per-cell peak instead, so the ratio runs order-1 across the cell rather
		// than order-1e-3 and most candidates land -- that is the entire point of
		// its cull prepass, and it is expressible here because the reference lives
		// in GalaxyPlacement rather than in the field parameters.
		Cell.DensityReference = (In.DensityReference > 0.0f)
			? In.DensityReference
			: Params.DensityParams.SpawnDensityReference;

		const int32 CellCandidates = (In.Candidates > 0) ? In.Candidates : CandidateBudget;
		Cell.Candidates = static_cast<uint32>(CellCandidates);
		MaxCandidates = FMath::Max(MaxCandidates, CellCandidates);

		Cells.Add(Cell);
	}

	// The run reserved per cell is the BUFFER's capacity, not the candidate budget.
	// The dispatch compacts, so anything beyond what the slot can hold would be read
	// back only to be discarded -- at nine thousand candidates a cell that was six and
	// a half megabytes a batch to extract a few hundred entities, which is what the
	// readbacks were timing out on.

	// SIZE GUARD. Total is the readback, and the dispatch is Cells x CandidateBudget
	// threads; both scale with the cell count, and the large tier's active set is not
	// a fixed nine or twenty-seven cells the way a grid neighbourhood is. Left
	// unchecked, a large active set asks for a buffer of hundreds of megabytes and a
	// dispatch of hundreds of millions of threads, and the failure surfaces as a
	// crash inside memcpy with no indication that the SIZE was the problem.
	const int64 TotalEntries = static_cast<int64>(SlotStride) * InCells.Num();
	const int64 TotalThreads = static_cast<int64>(MaxCandidates) * InCells.Num();

	constexpr int64 kMaxEntries = 4 * 1024 * 1024;   // 192 MB at 48 bytes each
	constexpr int64 kMaxThreads = 64 * 1024 * 1024;

	if (TotalEntries > kMaxEntries || TotalThreads > kMaxThreads)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: batch too large -- %d cells x %d candidates = %lld threads, ")
			TEXT("%lld entries (%lld MB). Reduce CandidateBudget or split the batch."),
			InCells.Num(), MaxCandidates, TotalThreads, TotalEntries,
			(TotalEntries * static_cast<int64>(sizeof(FGalaxyEntityOut))) >> 20);
		return FailBatch();
	}

	TArray<FGalaxyEntityOut> Entities;
	TArray<uint32> Counts;

	const bool bOk = GalaxyEntityGen::GenerateBatchBlocking(
		Params, InTierParams, Cells, SlotStride,
		Params.Seed + InSeedOffset,
		Params.NoiseTexture,
		MaxCandidates,
		Entities, Counts);

	if (!bOk)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: dispatch or readback failed for %d cells; those slots ")
			TEXT("will be EMPTY. Check the log above for an RHI or shader complaint; if ")
			TEXT("it was the timeout, GalaxyEntityGen::ReadbackTimeoutSeconds says how long ")
			TEXT("it waited."), Cells.Num());
		return FailBatch();
	}

	// Confirms the compute path ran AND that it produced something. "The dispatch
	// succeeded" is not enough on its own: an uncleared counter, an empty batch or a
	// saturated bounds test all return success and place nothing, and the symptom --
	// one tier populated and the others empty -- reads as a streaming problem rather
	// than a generation one.
	//
	// Counts is FOUR per cell. Only [0] is load-bearing; [1..3] are CALIBRATION and go
	// once the numbers below have settled:
	//   [0] accepted            -- against SlotStride, says whether cells are saturating
	//   [1] threads that ran    -- against Cells x CandidateBudget, says the dispatch sized
	//   [2] max density seen    -- against SpawnDensityReference, says the reference is sane
	//   [3] Candidates as read  -- says the cell buffer bound
	uint32 TotalAccepted = 0;
	uint32 TotalThreadsRun = 0;
	float MaxGpuDensity = 0.0f;
	uint32 MaxSeenCandidates = 0;
	int32 SaturatedCells = 0;

	for (int32 i = 0; i < InCells.Num(); ++i)
	{
		const uint32 Accepted = Counts[i * 4];

		TotalAccepted += Accepted;
		TotalThreadsRun += Counts[i * 4 + 1];

		const uint32 Bits = Counts[i * 4 + 2];
		MaxGpuDensity = FMath::Max(MaxGpuDensity, *reinterpret_cast<const float*>(&Bits));

		MaxSeenCandidates = FMath::Max(MaxSeenCandidates, Counts[i * 4 + 3]);

		// Pinned exactly at the run width means the cell had more to give and was
		// clipped, not that the field is thin there. That is the signal to raise
		// SpawnDensityReference and bring CandidateBudget down, rather than the other
		// way round.
		if (Accepted >= static_cast<uint32>(SlotStride))
		{
			++SaturatedCells;
		}
	}

	// Once per run while things are healthy, and every time they are not. Saturation
	// counts as "not healthy" for the moment: it is the calibration signal, and it
	// should stop appearing once the reference is set correctly.
	static bool bAnnounced = false;
	if (!bAnnounced || TotalAccepted == 0 || SaturatedCells > 0)
	{
		bAnnounced = true;

		UE_LOG(LogTemp, Display,
			TEXT("GalaxyEntityGen: %d cells x %d candidates -> %u accepted ")
			TEXT("(run %d, %u threads ran, %d cells saturated, ")
			TEXT("GPU max density %.5f against reference %.3f, cell.Candidates read as %u)."),
			Cells.Num(), CandidateBudget, TotalAccepted, SlotStride,
			TotalThreadsRun, SaturatedCells,
			MaxGpuDensity, Params.DensityParams.SpawnDensityReference,
			MaxSeenCandidates);
	}

	// Scatter, with a WRITE CURSOR PER SLOT.
	//
	// Cells do not map one-to-one onto slots. The large tier feeds its whole active
	// set into a single slot, so without a cursor each cell would start writing at
	// offset zero and overwrite the one before it -- leaving only the last cell's
	// entities and looking like a generation failure rather than a scatter bug.
	//
	// The mid and small tiers have one cell per slot, where the cursor is simply
	// always zero, so there is one path rather than a special case.
	const int32 NumSlots = InBuffer.SlotCoord.Num();

	OutSlotCounts.SetNumZeroed(NumSlots);

	TArray<int32> Cursor;
	Cursor.SetNumZeroed(NumSlots);

	TArray<int32> TouchedSlots;
	TouchedSlots.Reserve(InCells.Num());

	for (int32 c = 0; c < InCells.Num(); ++c)
	{
		const int32 SlotIndex = InCells[c].SlotIndex;
		if (!InBuffer.Positions.IsValidIndex(SlotIndex * InBuffer.SlotCapacity))
		{
			continue;
		}

		TouchedSlots.AddUnique(SlotIndex);

		const int32 BufferStart = SlotIndex * InBuffer.SlotCapacity;
		const int32 Base = c * SlotStride;

		// Already compacted on the GPU, so this is a straight copy of the live run
		// rather than a filter. Counts is clamped to the run width by the readback.
		const int32 Accepted = FMath::Min(static_cast<int32>(Counts[c * 4]), SlotStride);

		for (int32 i = 0; i < Accepted; ++i)
		{
			if (Cursor[SlotIndex] >= InBuffer.SlotCapacity)
			{
				break;
			}

			const FGalaxyEntityOut& E = Entities[Base + i];
			const int32 Idx = BufferStart + Cursor[SlotIndex];

			InBuffer.Positions[Idx] = FVector(E.Pos.X, E.Pos.Y, E.Pos.Z);

			// AUTHORED SIZE IS TRUTH: Extent already carries the real-unit range
			// divided by UnitScale exactly once, on the GPU via MakeGalaxyPlacement.
			InBuffer.Extents[Idx] = E.Extent;
			InBuffer.Colors[Idx] = FLinearColor(E.Decor.X, E.Decor.Y, E.Decor.Z);

			++Cursor[SlotIndex];
		}
	}

	// Padded once per slot, AFTER every cell that feeds it has been written. Doing it
	// inside the loop would blank the entities the previous cell just placed.
	for (int32 SlotIndex : TouchedSlots)
	{
		InBuffer.PadSlotDead(SlotIndex, Cursor[SlotIndex]);
		OutSlotCounts[SlotIndex] = Cursor[SlotIndex];
	}

	return true;
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
	// The sampling and the padding both live in EstimateCellEnvelope, which the mid and
	// small tiers now share -- this prepass differs from them only in walking the whole
	// grid rather than a streamed neighbourhood.
	// -----------------------------------------------------------------------

	const int32 CullDepth = FMath::Max(Params.LargeTierCullDepth, 1);
	const int32 GridSide = 1 << CullDepth;          // cells per axis
	const double FullExtent = static_cast<double>(Params.Extent);
	const double CellFull = (2.0 * FullExtent) / static_cast<double>(GridSide);
	const double HalfCell = CellFull * 0.5;

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
				// Already padded, and zero exactly when the cell is empty.
				const FIntVector Coord(ix, iy, iz);
				const float Envelope = EstimateCellEnvelope(Coord, Center, HalfCell);

				if (Envelope > 0.0f)
				{
					FActiveLargeTierCell Cell;
					Cell.Center = Center;
					Cell.HalfExt = HalfCell;
					Cell.GridCoord = Coord;
					Cell.MaxDensity = Envelope;
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

#pragma endregion
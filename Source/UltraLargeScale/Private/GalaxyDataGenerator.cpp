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
using GalaxyHLSL::GalaxyEntity;
using GalaxyHLSL::GalaxyPlacement;

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

GalaxyHLSL::GalaxyPlacement GalaxyDataGenerator::MakePlacement(
	const FTierParams& InTierParams,
	float InDensityReference) const
{
	// Extent converts through UnitScale HERE, so the field stays unit-agnostic and
	// the conversion happens once in the place UnitScale already lives. AUTHORED SIZE
	// IS TRUTH still holds: the real-unit range passes through UnitScale exactly once
	// and nothing downstream reconstructs size from a quantized octree depth.
	const double InvUnit = 1.0 / FMath::Max(Params.UnitScale, UE_DOUBLE_SMALL_NUMBER);

	return GalaxyHLSL::MakeGalaxyPlacement(
		InDensityReference,
		Params.DensityParams.SpawnCompression,
		InTierParams.SpawnExponent,
		static_cast<float>(InTierParams.MinScale * InvUnit),
		static_cast<float>(InTierParams.MaxScale * InvUnit),
		InTierParams.ExtentExponent);
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

	if (!Params.bUseGPUGeneration)
	{
		return false;
	}

	// SAY WHY, ONCE. Every bail here falls back to the CPU path and produces a
	// perfectly ordinary-looking galaxy, so a misconfiguration is invisible: the only
	// symptom of GPU generation never running is that placement quietly keeps
	// ignoring the volume texture, which is the entire thing this was built to fix.
	if (Params.NoiseTexture == nullptr)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: bUseGPUGeneration is on but NoiseTexture is unset; ")
			TEXT("falling back to CPU placement. Set it to the same volume texture the ")
			TEXT("material samples."));
		return false;
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
		return false;
	}

	TArray<FGalaxyEntityOut> Entities;
	TArray<uint32> Counts;

	const bool bOk = GalaxyEntityGen::GenerateBatchBlocking(
		Params, InTierParams, Cells, SlotStride,
		Params.Seed + InSeedOffset,
		Params.NoiseTexture,
		MaxCandidates,
		Params.bGPUForceNoiseOff,
		Params.GPUReadbackTimeoutSeconds,
		Entities, Counts);

	if (!bOk)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: dispatch or readback failed for %d cells; ")
			TEXT("falling back to CPU placement."), Cells.Num());
		return false;
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
	// One slot at a time, decided entirely by its own key.
	//
	// FRandomStream is gone, and that is not merely a port. A stream made candidate
	// i depend on every draw before it, so no entity could be produced without
	// producing all of its predecessors -- fine in a loop, impossible for one thread
	// per slot. Keying on (coord, slot) instead makes any entity recomputable alone
	// and stops identity being "position in the append order", so a region
	// regenerates identically after the player leaves and comes back.
	//
	// The three-phase batch that used to live here is gone with it: a slot has to be
	// decidable in isolation, and batching would force every candidate to be held
	// before any of them could be judged.
	// -----------------------------------------------------------------------

	if (!Derived)
	{
		InBuffer.PadSlotDead(InSlotIndex, 0);
		OutSlotCount = 0;
		return;
	}

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;

	// Fixed budget, variable acceptance. The candidate count no longer tracks the
	// buffer's remaining room, so a cell's yield depends on the field there and
	// nothing else.
	const int32 NumCandidates = FMath::Max(InTierParams.CandidateBudget, 1);
	const int32 MaxAccepted = InBuffer.SlotCapacity;

	const float InvExtent = static_cast<float>(1.0 / static_cast<double>(Params.Extent));

	const GalaxyPlacement Place = MakePlacement(
		InTierParams, Params.DensityParams.SpawnDensityReference);

	// InSeedOffset separates tiers that share a coordinate; the galaxy seed separates
	// galaxies. Both fold into the key rather than into a stream, so nothing here is
	// order dependent.
	const int32 KeySeed = Params.Seed + InSeedOffset;

	const float3 Centre(
		static_cast<float>(InNodeCenter.X),
		static_cast<float>(InNodeCenter.Y),
		static_cast<float>(InNodeCenter.Z));

	int32 ActualCount = 0;

	for (int32 i = 0; i < NumCandidates && ActualCount < MaxAccepted; ++i)
	{
		const GalaxyEntity E = Derived->SampleEntity(
			Place, Centre,
			static_cast<float>(InCellExtent), InvExtent,
			InCoord.X, InCoord.Y, InCoord.Z,
			i * 977 + KeySeed);

		if (E.bValid < 0.5f)
		{
			continue;
		}

		const int32 Idx = BufferStart + ActualCount;

		InBuffer.Positions[Idx] = FVector(E.Pos.x, E.Pos.y, E.Pos.z);

		// AUTHORED SIZE IS TRUTH: Extent already carries the real-unit range divided
		// by UnitScale exactly once, in MakePlacement. Octree insert depth is derived
		// later at insert time; never reconstruct size from the quantized depth, which
		// would inflate every particle to its node extent and make sprite sizes
		// octave-step with UnitScale.
		InBuffer.Extents[Idx] = E.Extent;

		// Three decorrelated uniforms on a second key, so colour is stable for a given
		// entity without being tied to where it landed.
		InBuffer.Colors[Idx] = FLinearColor(E.Decor.x, E.Decor.y, E.Decor.z);

		++ActualCount;
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
	// arms quadratically.
	//
	// The per-cell envelope is why the acceptance mapping lives in GalaxyPlacement
	// rather than in the derived field: this tier substitutes its own reference,
	// which a field-level parameter could not express. SpawnDensityReference stays
	// the mapping for mid and small, which have no prepass to derive a local bound
	// from; substituting it here would collapse acceptance by three orders.
	// -----------------------------------------------------------------------

	if (!Derived)
	{
		InBuffer.PadSlotDead(InSlotIndex, 0);
		OutSlotCount = 0;
		return;
	}

	const TArray<FActiveLargeTierCell> ActiveCells = CollectActiveLargeTierCells();
	if (ActiveCells.Num() == 0)
	{
		InBuffer.PadSlotDead(InSlotIndex, 0);
		OutSlotCount = 0;
		return;
	}

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;
	const int32 MaxAccepted = InBuffer.SlotCapacity;
	const float InvExtent = static_cast<float>(1.0 / static_cast<double>(Params.Extent));

	// Never fewer candidates than cells: a cell that survived the prepass has
	// structure in it and should get at least one draw.
	const int32 Budget = FMath::Max(Params.LargeTier.CandidateBudget, ActiveCells.Num());

	double TotalWeight = 0.0;
	for (const FActiveLargeTierCell& Cell : ActiveCells)
	{
		TotalWeight += FMath::Max(static_cast<double>(Cell.MaxDensity), 1e-6);
	}

	int32 TotalAccepted = 0;

	for (int32 ci = 0; ci < ActiveCells.Num() && TotalAccepted < MaxAccepted; ++ci)
	{
		const FActiveLargeTierCell& Cell = ActiveCells[ci];

		const double Weight = FMath::Max(static_cast<double>(Cell.MaxDensity), 1e-6);
		const int32 NumCandidates = FMath::Max(1,
			FMath::RoundToInt(static_cast<double>(Budget) * Weight / TotalWeight));

		const GalaxyPlacement Place = MakePlacement(Params.LargeTier, Cell.MaxDensity);

		const float3 Centre(
			static_cast<float>(Cell.Center.X),
			static_cast<float>(Cell.Center.Y),
			static_cast<float>(Cell.Center.Z));

		for (int32 i = 0; i < NumCandidates && TotalAccepted < MaxAccepted; ++i)
		{
			const GalaxyEntity E = Derived->SampleEntity(
				Place, Centre,
				static_cast<float>(Cell.HalfExt), InvExtent,
				Cell.GridCoord.X, Cell.GridCoord.Y, Cell.GridCoord.Z,
				i * 977 + Params.Seed);

			if (E.bValid < 0.5f)
			{
				continue;
			}

			const int32 Idx = BufferStart + TotalAccepted;

			InBuffer.Positions[Idx] = FVector(E.Pos.x, E.Pos.y, E.Pos.z);
			InBuffer.Extents[Idx] = E.Extent;
			InBuffer.Colors[Idx] = FLinearColor(E.Decor.x, E.Decor.y, E.Decor.z);

			++TotalAccepted;
		}
	}

	InBuffer.PadSlotDead(InSlotIndex, TotalAccepted);
	OutSlotCount = TotalAccepted;
}

#pragma endregion
// GalaxyDataGenerator.cpp
// Density evaluation and tier generation for the galaxy layer.
//
// The density field lives in Shaders/GalaxyDensityCore.ush and is compiled by BOTH
// the shader and this module, so star placement and the rendered gas are one function
// rather than two implementations kept in agreement by hand.

#include "GalaxyDataGenerator.h"

#include "GalaxyEntityGen.h"
#include "GalaxyGenProbe.h"
#include "ProceduralSpaceActor.h"
#include "Misc/ScopeLock.h"
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

void GalaxyDataGenerator::SubdivideCells(
	const TArray<FTierBatchCell>& InCells,
	int32 InLevels,
	double InExtent,
	TArray<FTierBatchCell>& OutCells)
{
	OutCells.Reset();

	if (InLevels <= 0)
	{
		OutCells = InCells;
		return;
	}

	const double InvExtent = 1.0 / FMath::Max(InExtent, 1e-9);
	const int32 Side = 1 << InLevels;
	const int32 PerCell = Side * Side * Side;

	OutCells.Reserve(InCells.Num() * PerCell / 2);

	for (const FTierBatchCell& Parent : InCells)
	{
		const double SubHalf = Parent.HalfExtent / static_cast<double>(Side);
		const double SubFull = SubHalf * 2.0;

		// Children tile the parent exactly, which is what keeps the sum of their masses
		// equal to 8^Levels times the parent's -- the relation the tier's calibrated
		// constant is scaled by.
		const double Origin = -(static_cast<double>(Side) - 1.0) * 0.5;

		for (int32 iz = 0; iz < Side; ++iz)
		{
			for (int32 iy = 0; iy < Side; ++iy)
			{
				for (int32 ix = 0; ix < Side; ++ix)
				{
					const FVector Centre = Parent.Centre + FVector(
						(Origin + static_cast<double>(ix)) * SubFull,
						(Origin + static_cast<double>(iy)) * SubFull,
						(Origin + static_cast<double>(iz)) * SubFull);

					// BOUNDS CULL, on geometry alone. The field is zero outside the unit
					// sphere, so a child whose nearest point already lies past it can
					// hold nothing -- and a sphere fills only pi/6 of its bounding cube.
					// One dot product here against sixty-four field evaluations if the
					// child's thread group has to discover it instead.
					const FVector Nearest(
						FMath::Max(FMath::Abs(Centre.X) - SubHalf, 0.0) * InvExtent,
						FMath::Max(FMath::Abs(Centre.Y) - SubHalf, 0.0) * InvExtent,
						FMath::Max(FMath::Abs(Centre.Z) - SubHalf, 0.0) * InvExtent);

					if (Nearest.SizeSquared() >= 1.0)
					{
						continue;
					}

					FTierBatchCell Child;

					// The SLOT is the parent's. Children are a generation detail; the
					// buffer still holds one region per streamed cell.
					Child.SlotIndex = Parent.SlotIndex;

					Child.Coord = FIntVector(
						Parent.Coord.X * Side + ix - Side / 2,
						Parent.Coord.Y * Side + iy - Side / 2,
						Parent.Coord.Z * Side + iz - Side / 2);

					Child.Centre = Centre;
					Child.HalfExtent = SubHalf;

					OutCells.Add(Child);
				}
			}
		}
	}
}

float GalaxyDataGenerator::GetTierBudgetScale(
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	bool bInCellsShareSlot) const
{
	// Null only on a moved-from generator. Same contract as SampleDensity against a null
	// Derived: report nothing rather than dereference.
	if (!TierBudgetScaleLock.IsValid())
	{
		return 0.0f;
	}

	{
		FScopeLock Lock(TierBudgetScaleLock.Get());
		if (const float* Cached = TierBudgetScales.Find(InSeedOffset))
		{
			return *Cached;
		}
	}

	// -----------------------------------------------------------------------
	// ONCE PER TIER, OVER ITS WHOLE GRID. Never per batch.
	//
	// Accepted count per cell is BudgetScale x mass_i, so BudgetScale is the entirety of
	// a tier's placement tuning. Solving it from the cells in a BATCH made a cell's yield
	// depend on which neighbours streamed in beside it: a neighbourhood of empty sky was
	// forced to the same total as one full of arms, so voids came back populated and cost
	// several times the candidates to produce. It also meant the same region generated
	// differently depending on the direction of approach.
	//
	// WHICH REDUCTION divides the capacity depends on how the tier maps cells to slots:
	//
	//   cells SHARE a slot  -> capacity / TOTAL mass.
	//     The large tier feeds its entire grid into one slot, so the slot holds the sum
	//     and the sum is what has to fit.
	//
	//   one cell PER slot   -> capacity / LARGEST cell mass.
	//     The densest cell in the galaxy fills its slot exactly; every other cell is
	//     proportionally less and no slot can overflow. Dividing a total here would give
	//     every slot the same count regardless of what is in it.
	// -----------------------------------------------------------------------
	FScopeLock Lock(TierBudgetScaleLock.Get());

	if (const float* Cached = TierBudgetScales.Find(InSeedOffset))
	{
		return *Cached;
	}

	const int32 Subdivision = FMath::Clamp(InTierParams.GenerationSubdivision, 0, 6);

	TArray<FTierBatchCell> AllCells;
	BuildFullTierGrid(InTierParams.GridDepth, AllCells);

	// SUBDIVIDED ONLY WHEN THE ANSWER IS A SUM.
	//
	// A tier whose cells share a slot needs the TOTAL mass of its grid, and a total is
	// only as good as the resolution it was summed at -- one cell spanning the galaxy
	// cannot be characterised by five hundred samples of a mostly empty cube.
	//
	// A tier with one cell per slot needs the LARGEST PARENT's total instead, and that
	// cannot be read off the subdivided grid at all: the reduce pass produces a global
	// max, not a max of per-parent sums. It measures parents and scales, which is exact
	// because children tile their parent -- a parent's mass is the mean of theirs, so
	// their sum is 8^Levels times it.
	if (bInCellsShareSlot && Subdivision > 0)
	{
		TArray<FTierBatchCell> Parents = MoveTemp(AllCells);
		SubdivideCells(Parents, Subdivision,
			static_cast<double>(Params.Extent), AllCells);
	}

	float Scale = 0.0f;

	if (AllCells.Num() > 0 && Params.NoiseTexture != nullptr)
	{
		TArray<FGalaxyGenCell> Cells;
		Cells.Reserve(AllCells.Num());

		for (const FTierBatchCell& In : AllCells)
		{
			FGalaxyGenCell Cell;
			Cell.Centre = FVector3f(
				static_cast<float>(In.Centre.X),
				static_cast<float>(In.Centre.Y),
				static_cast<float>(In.Centre.Z));
			Cell.HalfExtent = static_cast<float>(In.HalfExtent);
			Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);
			Cells.Add(Cell);
		}

		float TotalMass = 0.0f;
		float MaxCellMass = 0.0f;

		if (GalaxyEntityGen::CalibrateBlocking(
			Params, InTierParams, Cells,
			Params.Seed + InSeedOffset, Params.NoiseTexture,
			TotalMass, MaxCellMass))
		{
			// Share-slot measured the subdivided grid, so its total is already the sum
			// the slot receives. One-cell-per-slot measured parents, so the largest is
			// scaled by the number of children each has.
			const double SubCellsPerCell = FMath::Pow(8.0, static_cast<double>(Subdivision));

			const double Divisor = bInCellsShareSlot
				? static_cast<double>(TotalMass)
				: static_cast<double>(MaxCellMass) * SubCellsPerCell;

			const int32 Capacity = FMath::Max(InTierParams.SlotCapacity, 1);

			if (Divisor > 0.0)
			{
				Scale = static_cast<float>(static_cast<double>(Capacity) / Divisor);
			}

			UE_LOG(LogTemp, Display,
				TEXT("GalaxyEntityGen: calibrated tier +%d over %d cells -- total mass ")
				TEXT("%.6f, largest cell %.6f, capacity %d, %s, subdivision %d ")
				TEXT("(%.0f subcells per cell) -> scale %.1f."),
				InSeedOffset, AllCells.Num(), TotalMass, MaxCellMass, Capacity,
				bInCellsShareSlot ? TEXT("cells share a slot") : TEXT("one cell per slot"),
				Subdivision, SubCellsPerCell, Scale);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: calibration failed for tier +%d over %d cells. ")
				TEXT("This tier will generate NOTHING until it succeeds -- generating with ")
				TEXT("an uncalibrated constant would place entities at an arbitrary density."),
				InSeedOffset, AllCells.Num());
		}
	}

	// Cached even at zero. A retry every batch would re-probe an entire grid on the
	// streaming path, and the cause of a failure here -- no texture, no device -- does
	// not resolve itself between boundary crosses.
	TierBudgetScales.Add(InSeedOffset, Scale);

	return Scale;
}

void GalaxyDataGenerator::BuildFullTierGrid(
	int32 InGridDepth,
	TArray<FTierBatchCell>& OutCells) const
{
	OutCells.Reset();

	// The streaming system's grid, enumerated exhaustively rather than around a viewer.
	// Same formula, because calibration has to measure the cells the tier will actually
	// generate with -- a different coord labelling would reseed the probe jitter and give
	// a slightly different answer.
	const double CellSize = (static_cast<double>(Params.Extent)
		* AProceduralSpaceActor::GridExtentMultiplier) / static_cast<double>(1 << InGridDepth);

	const double HalfCell = CellSize * 0.5;

	if (!(CellSize > 0.0))
	{
		return;
	}

	// Coords run outward from the origin, far enough that the last ring still touches the
	// field. The +1 covers the half cell of overhang.
	const int32 Ring = FMath::CeilToInt32(
		static_cast<double>(Params.Extent) / CellSize) + 1;

	const double InvExtent = 1.0 / FMath::Max(static_cast<double>(Params.Extent), 1e-9);

	OutCells.Reserve((2 * Ring + 1) * (2 * Ring + 1) * (2 * Ring + 1) / 2);

	for (int32 iz = -Ring; iz <= Ring; ++iz)
	{
		for (int32 iy = -Ring; iy <= Ring; ++iy)
		{
			for (int32 ix = -Ring; ix <= Ring; ++ix)
			{
				const FVector Centre(
					static_cast<double>(ix) * CellSize,
					static_cast<double>(iy) * CellSize,
					static_cast<double>(iz) * CellSize);

				// Nearest point of the cell to the origin. Beyond the unit sphere the
				// field is zero everywhere in it, so no probe would find anything.
				const FVector Nearest(
					FMath::Max(FMath::Abs(Centre.X) - HalfCell, 0.0) * InvExtent,
					FMath::Max(FMath::Abs(Centre.Y) - HalfCell, 0.0) * InvExtent,
					FMath::Max(FMath::Abs(Centre.Z) - HalfCell, 0.0) * InvExtent);

				if (Nearest.SizeSquared() >= 1.0)
				{
					continue;
				}

				FTierBatchCell Out;
				Out.Coord = FIntVector(ix, iy, iz);
				Out.SlotIndex = 0;
				Out.Centre = Centre;
				Out.HalfExtent = HalfCell;

				OutCells.Add(Out);
			}
		}
	}
}

bool GalaxyDataGenerator::GenerateTierBatchGPU(
	const TArray<FTierBatchCell>& InQueuedCells,
	FNiagaraParticleBuffer& InBuffer,
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	bool bInCellsShareSlot,
	TArray<int32>& OutSlotCounts) const
{
	// GENERATION GRANULARITY, SEPARATED FROM SLOT GRANULARITY.
	//
	// A streamed cell is sized so a neighbourhood of them stays resident; the field's
	// structure has no reason to match. With a disc a few percent of a cell thick, the
	// cell's mean density is a small fraction of its peak -- and rejection against a
	// per-cell envelope accepts at exactly that ratio, which is why acceptance sat
	// around one percent while voids and arms both ran the full candidate count.
	//
	// Descending inside each cell cuts both ends: children clear of the structure are
	// culled by the probe pass for sixty-four evaluations and draw nothing at all, and
	// the ones that survive have a peak much closer to their own mean.
	//
	// Every child keeps its PARENT'S SLOT. The buffer still holds one region per
	// streamed cell; only the generation grid got finer.
	TArray<FTierBatchCell> Subdivided;
	SubdivideCells(InQueuedCells,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0, 6),
		static_cast<double>(Params.Extent), Subdivided);

	const TArray<FTierBatchCell>& InCells = Subdivided;

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
	auto FailBatch = [&InQueuedCells, &InBuffer, &OutSlotCounts]() -> bool
		{
			OutSlotCounts.SetNumZeroed(InBuffer.SlotCoord.Num());

			// The QUEUED cells, not the subdivided ones: blanking is per slot, and the
			// children of a cell all share its slot.
			for (const FTierBatchCell& Cell : InQueuedCells)
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

	// COMPRESSION BREAKS THE CANCELLATION AND NOTHING RESTORES IT.
	//
	// Per-cell normalisation is artifact-free because the envelope cancels: budget
	// scales as E^g, acceptance as (d/E)^g, and the realised density comes out as a
	// function of d alone. That only works because a power law composes with itself.
	// lerp(min(r,1), 1-exp(-r), C) is not a power law, so no choice of budget removes
	// the cell from the result, and every cell boundary becomes a discontinuity.
	//
	// It also has no job left. Compression existed to soften the hard clip at r = 1
	// against a GLOBAL reference. Against a per-cell envelope r never exceeds 1, so
	// there is nothing to soften.
	if (Params.DensityParams.SpawnCompression > 0.0f)
	{
		static bool bWarnedCompression = false;
		if (!bWarnedCompression)
		{
			bWarnedCompression = true;
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: SpawnCompression is %.3f. Placement rejects against ")
				TEXT("a PER-CELL envelope, and compression makes the acceptance mapping ")
				TEXT("non-power-law, so the cell no longer cancels out of the result and ")
				TEXT("cell boundaries show as grid artifacts. Set it to 0."),
				Params.DensityParams.SpawnCompression);
		}
	}

	// One cell per queued slot. The KEY is the grid coord, never the dispatch index:
	// identity is (coord, slot), which is what makes a region regenerate identically
	// after the player leaves and returns.
	TArray<FGalaxyGenCell> Cells;
	Cells.Reserve(InCells.Num());

	// ONE SHARED BUFFER, sized on what the SLOTS can hold -- not on cells.
	//
	// It used to be a uniform run per cell, and a uniform run has to be as wide as the
	// densest cell needs while every cell pays for it. Measured on a real galaxy: 2728
	// cells with the densest wanting a few thousand entities is over half a gigabyte of
	// readback, so the run gets set to what fits instead -- 32 -- and a third of the live
	// cells pin against it exactly. A pinned cell emits the same count whatever it holds,
	// which is a cubic lattice in the sky, and the arms clip hardest because they are
	// what exceeds the run.
	//
	// No cell has a ceiling now, so the only question is how many entities can possibly
	// be KEPT. That is the slots' business, and it is known here.
	//
	// Twice capacity as headroom. Acceptance is stochastic and the anchor is authored,
	// so the total lands near capacity rather than on it. Overshoot within the headroom
	// is thinned deterministically below; overshoot BEYOND it is truncated by the GPU's
	// arrival order, which is both nondeterministic and spatially biased -- the failure
	// this change exists to remove, so the headroom is what protects it.
	TSet<int32> DistinctSlots;
	for (const FTierBatchCell& In : InCells)
	{
		DistinctSlots.Add(In.SlotIndex);
	}

	// The tier's calibrated constant. Measured once over its whole grid, never from this
	// batch -- otherwise a cell's yield would depend on which neighbours streamed in with
	// it, and the same region would generate differently depending on approach.
	//
	// Whether cells share slots is STATED by the caller, not counted here. Subdivision
	// also makes cells outnumber slots and it means the opposite thing: those cells all
	// belong to one parent rather than to the whole tier, so the constant still comes
	// from the largest single streamed cell.
	const float BudgetScale =
		GetTierBudgetScale(InTierParams, InSeedOffset, bInCellsShareSlot);

	if (!(BudgetScale > 0.0f))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: tier +%d has no calibrated budget scale, so these slots ")
			TEXT("get nothing. See the calibration failure above."), InSeedOffset);
		return FailBatch();
	}

	// Headroom over what the slots can hold. A quarter.
	//
	// It was doubled while the budget was authored and could miss by any factor, then cut
	// to a tenth once calibration made overshoot stochastic. A tenth turned out to be too
	// tight: calibration measures a whole streamed cell while generation measures its
	// subcells, and the coarser estimate is BIASED low rather than merely noisy, so the
	// realised count lands consistently above target rather than scattering around it.
	// More probe rounds during calibration shrink that bias but do not erase it.
	//
	// Headroom exists at all because the alternative when it runs out is the GPU
	// truncating by arrival order, which is nondeterministic and spatially biased -- so
	// it is sized against the residual bias, not against the square-root noise, which at
	// a slot of ten thousand is only a hundred either way.
	const int32 EntityCapacity = FMath::Max(
		FMath::DivideAndRoundUp(
			DistinctSlots.Num() * FMath::Max(InBuffer.SlotCapacity, 1) * 5, 4), 64);

	for (const FTierBatchCell& In : InCells)
	{
		// GEOMETRY ONLY. Whether this cell has anything in it, what it rejects against
		// and how many candidates it draws are all decided by its group in the dispatch.
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

		Cells.Add(Cell);
	}

	// SIZE GUARD. Both axes are bounded by construction now -- the dispatch is one group
	// per cell, the readback is slots x capacity -- so this fires only on an absurd
	// authored capacity, never on the cell count, which is what it used to catch.
	constexpr int64 kMaxEntries = 4 * 1024 * 1024;   // 192 MB at 48 bytes each

	if (static_cast<int64>(EntityCapacity) > kMaxEntries)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("GalaxyEntityGen: batch too large -- %d slots x %d capacity x2 = %d ")
			TEXT("entries (%lld MB) to read back. Reduce the tier's SlotCapacity."),
			DistinctSlots.Num(), InBuffer.SlotCapacity, EntityCapacity,
			(static_cast<int64>(EntityCapacity) * static_cast<int64>(sizeof(FGalaxyEntityOut))) >> 20);
		return FailBatch();
	}

	TArray<FGalaxyEntityOut> Entities;
	TArray<uint32> Counts;

	const bool bOk = GalaxyEntityGen::GenerateBatchBlocking(
		Params, InTierParams, Cells, EntityCapacity,
		Params.Seed + InSeedOffset,
		Params.NoiseTexture,
		BudgetScale,
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
	// Counts is FOUR per cell:
	//   [0] accepted
	//   [1] candidates evaluated
	//   [2] the cell's envelope, as asuint
	//   [3] the largest density any candidate saw, as asuint
	//   [4] the cell's mass, as asuint -- what calibration solved the tier constant from

	uint32 TotalAccepted = 0;
	int64 TotalEvaluated = 0;
	float MaxEnvelope = 0.0f;
	int32 LiveCells = 0;
	int32 ExceededCells = 0;

	auto AsFloat = [](uint32 InBits) { return *reinterpret_cast<const float*>(&InBits); };

	for (int32 i = 0; i < InCells.Num(); ++i)
	{
		const int32 Base = i * GalaxyEntityGen::CountersPerCell;

		const uint32 Accepted = Counts[Base];
		const float Envelope = AsFloat(Counts[Base + 2]);
		const float PeakSeen = AsFloat(Counts[Base + 3]);

		TotalAccepted += Accepted;
		TotalEvaluated += Counts[Base + 1];
		MaxEnvelope = FMath::Max(MaxEnvelope, Envelope);

		if (Envelope <= 0.0f)
		{
			// Culled by the shader's own probes. Expected and common -- most of a
			// galaxy's bounding grid is void.
			continue;
		}

		++LiveCells;

		// THE ENVELOPE CHECK. A candidate found more density than thirty-two probes
		// did, so the ratio clipped at 1 and this cell's peak was flattened. Not fatal,
		// but it is what EnvelopePad exists to prevent, and it has no other observer.
		if (PeakSeen > Envelope)
		{
			++ExceededCells;
		}
	}

	// Once per run while things are healthy, and every time they are not.
	// The GLOBAL cursor, one past the per-cell region. Deliberately un-clamped by the
	// shader, so it reports the true total accepted even when the buffer overflowed --
	// which is exactly the case where the CPU needs to know by how much.
	const uint32 GlobalAccepted =
		Counts[GalaxyEntityGen::GlobalCursorIndex(InCells.Num())];
	const int32 Landed = FMath::Min(static_cast<int32>(GlobalAccepted), EntityCapacity);
	const bool bOverflowed = static_cast<int32>(GlobalAccepted) > EntityCapacity;

	static bool bAnnounced = false;
	if (!bAnnounced || TotalAccepted == 0 || bOverflowed || ExceededCells > 0)
	{
		bAnnounced = true;

		UE_LOG(LogTemp, Display,
			TEXT("GalaxyEntityGen: tier +%d, %d queued -> %d cells (%d live) -> ")
			TEXT("%lld candidates -> %u accepted (%d landed, %d capacity, ")
			TEXT("%d envelope exceeded, scale %.1f, max envelope %.5f)."),
			InSeedOffset, InQueuedCells.Num(), InCells.Num(), LiveCells,
			TotalEvaluated, GlobalAccepted, Landed, EntityCapacity,
			ExceededCells, BudgetScale, MaxEnvelope);
	}

	// The one remaining path where entities are dropped by ARRIVAL ORDER rather than
	// deterministically. Everything within the headroom is thinned uniformly below; this
	// is the overshoot beyond it, biased toward whichever groups the scheduler ran first.
	//
	// It should not be reachable. The buffer holds twice the target and the shader solves
	// the budget so the total lands ON the target, so overflowing it means the solve was
	// off by more than 2x -- which is the biased-probe case above, not a tuning miss.
	if (bOverflowed)
	{
		static bool bWarnedOverflow = false;
		if (!bWarnedOverflow)
		{
			bWarnedOverflow = true;
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: %u accepted against a %d-record buffer -- the excess ")
				TEXT("was dropped by GPU arrival order, which is nondeterministic and ")
				TEXT("spatially biased. Calibration should make this unreachable, so it ")
				TEXT("means the probes under-weighed this region: %d cells reported a ")
				TEXT("candidate density above their own envelope."),
				GlobalAccepted, EntityCapacity, ExceededCells);
		}
	}

	// Scatter, with a WRITE CURSOR PER SLOT.
	//
	// Entities arrive in one shared, globally-appended buffer rather than in per-cell
	// runs, so an entity's POSITION says nothing about where it belongs. Each record
	// carries the index of the cell that produced it, and the cell carries the slot.
	//
	// Nothing downstream may key off buffer order. Identity is still (cell, slot), and
	// Slot travels in the record for exactly that reason -- the thinning below uses it,
	// never the loop index.
	const int32 NumSlots = InBuffer.SlotCoord.Num();

	OutSlotCounts.SetNumZeroed(NumSlots);

	TArray<int32> Cursor;
	Cursor.SetNumZeroed(NumSlots);

	// Every queued slot is touched whether or not anything landed in it. A slot that
	// generated nothing still has to be padded dead, or it keeps the entities of
	// whichever coord occupied it last.
	TArray<int32> TouchedSlots;
	TouchedSlots.Reserve(InCells.Num());

	for (const FTierBatchCell& In : InCells)
	{
		if (InBuffer.Positions.IsValidIndex(In.SlotIndex * InBuffer.SlotCapacity))
		{
			TouchedSlots.AddUnique(In.SlotIndex);
		}
	}

	// POOLED BUDGET, APPLIED AT SCATTER TIME.
	//
	// A slot can still receive more than it holds, and WHICH entities survive is the
	// whole question. Filling in arrival order answers it by scheduling; filling in cell
	// order answers it by grid position, and the cut then lies along grid planes.
	//
	// Thinning keeps a uniform fraction of every cell's entities instead, so the shape
	// survives and only the count comes down. This is what the CPU prepass used to do by
	// dividing a pooled budget before generating; doing it here costs candidates that get
	// discarded, and buys not needing a second dispatch and a global reduction to know the
	// total in advance.
	TArray<int32> SlotTotals;
	SlotTotals.SetNumZeroed(NumSlots);

	for (int32 i = 0; i < Landed; ++i)
	{
		const int32 CellIndex = static_cast<int32>(Entities[i].CellIndex);
		if (!InCells.IsValidIndex(CellIndex))
		{
			continue;
		}

		const int32 SlotIndex = InCells[CellIndex].SlotIndex;
		if (SlotTotals.IsValidIndex(SlotIndex))
		{
			++SlotTotals[SlotIndex];
		}
	}

	// Five percent of headroom, because the keep test is per entity and independent so
	// the realised count varies around the target by roughly its square root.
	TArray<float> KeepFraction;
	KeepFraction.Init(1.0f, NumSlots);

	int32 ThinnedSlots = 0;
	float MinKeep = 1.0f;

	for (int32 SlotIndex = 0; SlotIndex < NumSlots; ++SlotIndex)
	{
		if (SlotTotals[SlotIndex] > InBuffer.SlotCapacity && SlotTotals[SlotIndex] > 0)
		{
			KeepFraction[SlotIndex] = 0.95f * static_cast<float>(InBuffer.SlotCapacity)
				/ static_cast<float>(SlotTotals[SlotIndex]);
			MinKeep = FMath::Min(MinKeep, KeepFraction[SlotIndex]);
			++ThinnedSlots;
		}
	}

	if (ThinnedSlots > 0)
	{
		static bool bWarnedThin = false;
		if (!bWarnedThin || MinKeep < 0.25f)
		{
			bWarnedThin = true;
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: %d slot(s) over capacity and thinned, keeping as ")
				TEXT("little as %.1f%%. The shape is preserved but the candidates behind ")
				TEXT("the discard was wasted. Calibration sizes the densest cell in the ")
				TEXT("galaxy to fill its slot exactly, so this should not happen -- it means ")
				TEXT("the probes under-weighed this cell during calibration, the same cause ")
				TEXT("as an exceeded envelope."),
				ThinnedSlots, MinKeep * 100.0f);
		}
	}

	for (int32 i = 0; i < Landed; ++i)
	{
		const FGalaxyEntityOut& E = Entities[i];

		const int32 CellIndex = static_cast<int32>(E.CellIndex);
		if (!InCells.IsValidIndex(CellIndex))
		{
			continue;
		}

		const int32 SlotIndex = InCells[CellIndex].SlotIndex;
		if (!Cursor.IsValidIndex(SlotIndex)
			|| !InBuffer.Positions.IsValidIndex(SlotIndex * InBuffer.SlotCapacity))
		{
			continue;
		}

		// A BACKSTOP, not the mechanism. If this trips with thinning active the headroom
		// was too small, and the discard goes back to being order-dependent.
		if (Cursor[SlotIndex] >= InBuffer.SlotCapacity)
		{
			continue;
		}

		const float Keep = KeepFraction[SlotIndex];

		if (Keep < 1.0f)
		{
			// Keyed on (coord, slot), never on i. Append order is scheduling-dependent,
			// so thinning on the buffer index would keep a different subset every visit
			// -- the stars would flicker rather than the region merely being sparser.
			const FIntVector& Coord = InCells[CellIndex].Coord;

			const uint32 Key = HashCombine(
				HashCombine(GetTypeHash(Coord.X), GetTypeHash(Coord.Y)),
				HashCombine(GetTypeHash(Coord.Z),
					GetTypeHash(static_cast<int32>(E.Slot))));

			const uint32 Scrambled = Key * 2654435761u;
			const float U = static_cast<float>(Scrambled >> 8) * (1.0f / 16777216.0f);

			if (U >= Keep)
			{
				continue;
			}
		}

		const int32 Idx = SlotIndex * InBuffer.SlotCapacity + Cursor[SlotIndex];

		InBuffer.Positions[Idx] = FVector(E.Pos.X, E.Pos.Y, E.Pos.Z);

		// AUTHORED SIZE IS TRUTH: Extent already carries the real-unit range divided by
		// UnitScale exactly once, on the GPU via MakeGalaxyPlacement.
		InBuffer.Extents[Idx] = E.Extent;
		InBuffer.Colors[Idx] = FLinearColor(E.Decor.X, E.Decor.Y, E.Decor.Z);

		++Cursor[SlotIndex];
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
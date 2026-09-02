// GalaxyDataGenerator.cpp
// Tier generation for the galaxy layer.
//
// NO FIELD EVALUATION HERE. The density field lives in GalaxyDensityCore.ush and is
// evaluated only by the GPU -- the raymarch draws it and GalaxyEntityGen.usf places
// against it, both from the same file, which is what makes the stars and the gas one
// function rather than two kept in agreement by hand. This module marshals parameters
// and reads back results.

#include "GalaxyDataGenerator.h"

#include "GalaxyEntityGen.h"
#include "ProceduralSpaceActor.h"
#include "Misc/ScopeLock.h"
#include <atomic>

#pragma region Lifetime

// Out of line so the header need not be complete for the compiler-generated bodies.
GalaxyDataGenerator::GalaxyDataGenerator() = default;
GalaxyDataGenerator::GalaxyDataGenerator(FGalaxyParams InParams) : Params(MoveTemp(InParams)) {}
GalaxyDataGenerator::~GalaxyDataGenerator() = default;
GalaxyDataGenerator::GalaxyDataGenerator(GalaxyDataGenerator&&) noexcept = default;
GalaxyDataGenerator& GalaxyDataGenerator::operator=(GalaxyDataGenerator&&) noexcept = default;

#pragma endregion

#pragma region Initialization

void GalaxyDataGenerator::Initialize()
{
	// DROP THE CALIBRATION. It is solved against a SPECIFIC density field, and this generator
	// is a by-value member of AGalaxyActor, which is POOLED -- ReInit assigns new Params and
	// calls straight back through here on an object that has already run a different galaxy.
	//
	// A SURVIVING CACHE ENTRY IS A SILENT WRONG ANSWER. The recycled actor finds an entry for
	// tier offset 0, 7 or 13 and returns it without calibrating, so the new galaxy's entities
	// are placed at a density solved for the previous one's field. Nothing warns; the only
	// visible trace is the ABSENCE of the "calibrated tier" line a first-run galaxy logs, and
	// the counts then miss SlotCapacity by whatever ratio the two fields' masses differ by.
	if (TierBudgetScaleLock.IsValid())
	{
		FScopeLock Lock(TierBudgetScaleLock.Get());
		TierBudgetScales.Empty();
	}

}

#pragma endregion

#pragma region Tier Generation Callbacks

float GalaxyDataGenerator::GetTierBudgetScale(
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	bool bInCellsShareSlot) const
{
	// Null only on a moved-from generator: report nothing rather than dereference.
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
	// a tier's placement tuning. Solving it from the cells in a BATCH makes a cell's
	// yield depend on which neighbours streamed in beside it: a neighbourhood of empty
	// sky is then forced to the same total as one full of arms, so voids come back
	// populated, and the same region generates differently depending on the direction of
	// approach.
	//
	// Which reduction divides the capacity is decided below, once the masses are in.
	// -----------------------------------------------------------------------
	FScopeLock Lock(TierBudgetScaleLock.Get());

	if (const float* Cached = TierBudgetScales.Find(InSeedOffset))
	{
		return *Cached;
	}

	const int32 Subdivision = FMath::Clamp(InTierParams.GenerationSubdivision, 0,
		FTierParams::MaxGenerationSubdivision);

	// THE PARAMETERS THIS TIER IS ACTUALLY RUNNING WITH, logged before anything is measured
	// against them.
	//
	// NOT INSTRUMENTATION FOR ITS OWN SAKE. A tier parameter that fails to arrive looks
	// exactly like a tier parameter that has no effect, and a generator holds a COPY of the
	// params taken at InitializeData -- so an edit can land on a struct nothing reads while
	// every symptom points at the field.
	//
	// THE SUBDIVISION IS LOGGED CLAMPED, not authored, because the clamp is silent: a value
	// above MaxGenerationSubdivision comes back as the ceiling with nothing saying so, and a
	// tier that appears not to respond to the setting is the result.
	//
	// Read these before concluding anything about a parameter's effect.
	UE_LOG(LogTemp, Log,
		TEXT("GalaxyEntityGen: tier +%d effective params -- GridDepth %d, ")
		TEXT("NeighborhoodRadius %d, SlotCapacity %d, GenerationSubdivision %d, ")
		TEXT("SpawnExponent %.4f, ExtentExponent %.4f, MinScale %.4g, MaxScale %.4g."),
		InSeedOffset, InTierParams.GridDepth, InTierParams.NeighborhoodRadius,
		InTierParams.SlotCapacity, Subdivision,
		InTierParams.SpawnExponent, InTierParams.ExtentExponent,
		InTierParams.MinScale, InTierParams.MaxScale);

	// MEASURE THE CELLS GENERATION WILL ACTUALLY USE.
	//
	// Both tier kinds descend to the generation grid, because a cell's mass is an
	// estimate of the mean over its own volume and that estimate depends on the volume.
	// Probing an undivided parent whose structure occupies a few percent of it reports a
	// mean that is too low -- as BIAS, in one direction, growing with the subdivision
	// depth -- so the constant came out too high and generation over-delivered.
	//
	// The reduction is then whichever one matches how the tier maps cells to slots, and
	// both are taken on the CPU from the per-cell masses. See below.
	TArray<FTierBatchCell> Parents;
	BuildFullTierGrid(InTierParams.GridDepth, Parents);

	TArray<FTierBatchCell> AllCells;
	FTierStreamingSystem::SubdivideCells(Parents, Subdivision,
		ETierChildCoords::Centred, AllCells,
		FTierStreamingSystem::MakeSphereBoundsCull(static_cast<double>(Params.Extent)));

	float Scale = 0.0f;

	if (AllCells.Num() > 0 && Params.Procedural.GetFieldTextures().IsComplete())
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

		TArray<float> CellMass;

		const double CalibrationStart = FPlatformTime::Seconds();

		if (GalaxyEntityGen::CalibrateBlocking(
			Params, InTierParams, Cells,
			TierKeySeed(InSeedOffset), Params.Procedural.GetFieldTextures(), CellMass)
			&& CellMass.Num() == AllCells.Num())
		{
			// REDUCED HERE, NOT ON THE GPU, and in double.
			//
			// A slot receives the sum of what its cells produce, so the divisor is
			// whichever sum the slot actually sees:
			//
			//   cells SHARE a slot  -> the TOTAL over the grid. The large tier feeds its
			//     entire subdivided grid into one slot, so the sum is what must fit.
			//
			//   one cell PER slot   -> the LARGEST PER-PARENT SUM. A streamed cell owns a
			//     slot and its children all write there, so the densest streamed cell
			//     fills its slot exactly and every other is proportionally less. A global
			//     max over subcells would answer a different question entirely, and a
			//     total would force every neighbourhood to the same count -- which is how
			//     a void came back as full as an arm.
			//
			// Summed in a fixed array order, so the answer is reproducible run to run --
			// which matters because it divides every cell's budget. Double, because a
			// subdivided grid is hundreds of thousands of terms and a float accumulator
			// loses the small ones against the large.
			double Total = 0.0;
			for (const float Mass : CellMass)
			{
				Total += static_cast<double>(Mass);
			}

			// Taken even when the tier does not divide by it, because the two numbers
			// side by side in the log are what say whether a tier is calibrated against
			// the right one: a share-slot tier's total should dwarf its largest parent,
			// and a per-slot tier's should not be far off a multiple of it.
			TArray<double> ParentSums;
			ParentSums.SetNumZeroed(Parents.Num());

			for (int32 i = 0; i < AllCells.Num(); ++i)
			{
				const int32 ParentIndex = AllCells[i].ParentIndex;
				if (ParentSums.IsValidIndex(ParentIndex))
				{
					ParentSums[ParentIndex] += static_cast<double>(CellMass[i]);
				}
			}

			double LargestParent = 0.0;
			for (const double Sum : ParentSums)
			{
				LargestParent = FMath::Max(LargestParent, Sum);
			}

			const double Divisor = bInCellsShareSlot ? Total : LargestParent;
			const int32 Capacity = FMath::Max(InTierParams.SlotCapacity, 1);

			// THE PER-GALAXY DENSITY KNOB, APPLIED HERE AND NOWHERE ELSE.
			//
			// It scales the TARGET rather than the field, which is the only place it can go.
			// Scaling the density field does nothing: acceptance normalises against each
			// cell's own probed envelope, so multiplying the whole field by c scales every
			// mass by c^exponent and BudgetScale by its inverse, and the accepted counts come
			// out identical. Scaling SlotCapacity would resize the Niagara buffers, which are
			// allocated once per pooled actor -- reallocating on every spawn is the cost
			// pooling exists to avoid.
			//
			// Solving BudgetScale against a reduced capacity makes a sparse galaxy GENERATE
			// fewer entities rather than being thinned afterwards, so the shape it does place
			// is the shape the field describes.
			//
			// CLAMPED TO 1, AND THINNING ONLY. SlotCapacity both sizes the buffer and is the
			// calibration target, so above 1 the tier aims past what the buffer holds and the
			// excess is dropped by the cursor backstop -- silently, in arrival order, which is
			// spatial. Denser than capacity needs a larger SlotCapacity, which is the thing
			// that cannot roll per galaxy.
			const double DensityScale = FMath::Clamp(
				static_cast<double>(Params.Procedural.StarDensityScale), 0.0, 1.0);

			const double Target = static_cast<double>(Capacity) * DensityScale;

			if (Divisor > 0.0)
			{
				Scale = static_cast<float>(Target / Divisor);
			}

			UE_LOG(LogTemp, Display,
				TEXT("GalaxyEntityGen: calibrated tier +%d over %d parents -> %d cells ")
				TEXT("(subdivision %d) in %.0f ms -- total mass %.6f, largest parent ")
				TEXT("%.6f, capacity %d, %s -> scale %.1f."),
				InSeedOffset, Parents.Num(), AllCells.Num(), Subdivision,
				(FPlatformTime::Seconds() - CalibrationStart) * 1000.0,
				Total, LargestParent, Capacity,
				bInCellsShareSlot ? TEXT("cells share a slot") : TEXT("one cell per slot"),
				Scale);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: calibration failed for tier +%d over %d cells, or ")
				TEXT("returned a mass array of the wrong length. This tier will generate ")
				TEXT("NOTHING until it succeeds -- generating with an uncalibrated ")
				TEXT("constant would place entities at an arbitrary density."),
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

	// THE SAME CULL THE SUBDIVISION USES, and it has to be. This grid and the children it is
	// subdivided into are measured against one field, so two spellings of "outside the unit
	// sphere" would let the two disagree about a boundary cell -- calibration would then
	// weigh a set of cells generation never produces.
	const auto KeepCell = FTierStreamingSystem::MakeSphereBoundsCull(
		static_cast<double>(Params.Extent));

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

				// Beyond the unit sphere the field is zero everywhere in the cell, so no
				// probe would find anything.
				if (!KeepCell(Centre, HalfCell))
				{
					continue;
				}

				FTierBatchCell Out;
				Out.Coord = FIntVector(ix, iy, iz);
				Out.SlotIndex = 0;
				Out.ParentIndex = OutCells.Num();
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
	FTierStreamingSystem::SubdivideCells(InQueuedCells,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0,
			FTierParams::MaxGenerationSubdivision),
		ETierChildCoords::Centred, Subdivided,
		FTierStreamingSystem::MakeSphereBoundsCull(static_cast<double>(Params.Extent)));

	const TArray<FTierBatchCell>& InCells = Subdivided;

	if (InCells.Num() == 0)
	{
		return true;
	}

	// FAIL CLOSED, VISIBLY. There is no CPU path behind this, so a failure here means these
	// slots get nothing -- and a slot is REUSED as the player crosses boundaries, so
	// "nothing written" is not an empty slot. It is the previous occupant's entities sitting
	// there at a coord they do not belong to, which reads as a placement bug rather than a
	// generation failure.
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
	if (!Params.Procedural.GetFieldTextures().IsComplete())
	{
		// NAMES THE MISSING ONES. With four assets "the texture is unset" sends the reader to
		// check four bags and four defaults, and the common case is three fine and one not.
		ensureMsgf(false,
			TEXT("GalaxyEntityGen: field textures unset (%s), so the galaxy will generate ")
			TEXT("NOTHING -- placement is GPU-only and the dispatch samples all four. They ")
			TEXT("must be the same assets the material samples, with NEVER STREAM set on ")
			TEXT("each."),
			*Params.Procedural.GetFieldTextures().DescribeMissing());
		return FailBatch();
	}

	// One cell per queued slot. The KEY is the grid coord, never the dispatch index:
	// identity is (coord, slot), which is what makes a region regenerate identically
	// after the player leaves and returns.
	TArray<FGalaxyGenCell> Cells;
	Cells.Reserve(InCells.Num());

	// ONE SHARED BUFFER, sized on what the SLOTS can hold -- not on cells.
	//
	// No cell has a ceiling, so the only question is how many entities can possibly be
	// KEPT, and that is the slots' business. Sizing per cell instead means a uniform run
	// as wide as the densest cell needs with every cell paying for it: either hundreds of
	// megabytes of readback, or a run small enough that dense cells pin against it and
	// emit identical counts, which is a cubic lattice in the sky.
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

	// AN EIGHTH OVER TARGET, against stochastic overshoot alone.
	//
	// Calibration measures the cells generation will actually use, so the realised count
	// lands ON the slot rather than above it -- and what is left is the square root of
	// the count, about 0.7% at a slot of twenty thousand. An eighth is roughly eighteen
	// sigma of that.
	//
	// Envelope clipping pushes the realised count UNDER target, never over, so it costs
	// nothing here.
	//
	// Headroom exists at all because the alternative when it runs out is the GPU
	// truncating by arrival order, which is nondeterministic and spatially biased.
	const int32 EntityCapacity = FMath::Max(
		FMath::DivideAndRoundUp(
			DistinctSlots.Num() * FMath::Max(InBuffer.SlotCapacity, 1) * 9, 8), 64);

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

	// SIZE GUARD. Both axes are bounded by construction -- the dispatch is one group per
	// cell, the readback is slots x capacity -- so this can only fire on an absurd authored
	// capacity, never on the cell count.
	constexpr int64 kMaxEntries = 4 * 1024 * 1024;   // 128 MB at 32 bytes each

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
		TierKeySeed(InSeedOffset),
		Params.Procedural.GetFieldTextures(),
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
	// Counts is FIVE per cell, then the global append cursor:
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

	// THE TARGET THIS BATCH WAS SOLVED FOR, summed from the same counter calibration
	// reduced. accepted_i is BudgetScale x mass_i by construction, so BudgetScale times the
	// batch's own mass is what the batch is supposed to land on -- and it works for a
	// partial neighbourhood as well as a full grid, which a flat SlotCapacity would not.
	double BatchMass = 0.0;

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
		BatchMass += static_cast<double>(AsFloat(Counts[Base + 4]));

		if (Envelope <= 0.0f)
		{
			// Culled by the shader's own probes. Expected and common -- most of a
			// galaxy's bounding grid is void.
			continue;
		}

		++LiveCells;

		// THE ENVELOPE CHECK. A candidate found more density than the cell's whole probe
		// set did, so the ratio clipped at 1 and this cell's peak was flattened. Not fatal,
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

	// REDUCED IN uint32 BEFORE IT NARROWS. This counter is deliberately un-clamped by the
	// shader so it reports the true total even when the buffer overflowed, which is the one
	// case where a value past INT32_MAX is reachable -- and narrowing first makes it
	// NEGATIVE, so Landed goes negative and the overflow test reports no overflow. The
	// scatter would then write nothing and the warning would not fire, on the exact input
	// the counter exists to report. The universe path reduces in the wider type for the
	// same reason.
	const int32 Landed = static_cast<int32>(
		FMath::Min<uint32>(GlobalAccepted, static_cast<uint32>(EntityCapacity)));
	const bool bOverflowed = GlobalAccepted > static_cast<uint32>(EntityCapacity);

	const double CalibratedTarget = static_cast<double>(BudgetScale) * BatchMass;

	// Below one expected entity the ratio is shot noise rather than a measurement.
	const bool bTargetMeaningful = CalibratedTarget >= 1.0;

	const double DeliveryRatio = bTargetMeaningful
		? static_cast<double>(GlobalAccepted) / CalibratedTarget
		: 1.0;

	// ATOMIC, because this runs on background workers for several galaxies at once.
	// A plain bool here is a data race -- benign in effect, since the worst outcome is a
	// duplicate log line, but a race the sanitizers are right to flag.
	static std::atomic<bool> bAnnounced{ false };

	const bool bFirst = !bAnnounced.exchange(true);

	if (bFirst || TotalAccepted == 0 || bOverflowed || ExceededCells > 0)
	{

		// PROBES AND CANDIDATES ARE BOTH FIELD EVALUATIONS, so their ratio is what says
		// whether the tier's GenerationSubdivision sits on the right side of its
		// crossover. Deeper subdivision multiplies probes by the cell growth factor and
		// divides candidates by the acceptance gain; below about 1 the probes have
		// overtaken placement and the tier wants one level fewer, above about 9 it wants
		// one more.
		const int64 TotalProbes =
			static_cast<int64>(InCells.Num()) * FGalaxyEntityGenCS::ProbesPerCell;

		UE_LOG(LogTemp, Display,
			TEXT("GalaxyEntityGen: tier +%d, %d queued -> %d cells (%d live) -> ")
			TEXT("%lld probes + %lld candidates (C/P %.2f) -> %u accepted ")
			TEXT("(%d landed, %d capacity, %d envelope exceeded, scale %.1f, ")
			TEXT("max envelope %.5f, delivered %.0f%% of target)."),
			InSeedOffset, InQueuedCells.Num(), InCells.Num(), LiveCells,
			TotalProbes, TotalEvaluated,
			TotalProbes > 0 ? static_cast<double>(TotalEvaluated) / static_cast<double>(TotalProbes) : 0.0,
			GlobalAccepted, Landed, EntityCapacity,
			ExceededCells, BudgetScale, MaxEnvelope, DeliveryRatio * 100.0);
	}

	// THE IDENTITY THIS TIER RESTS ON, CHECKED RATHER THAN ASSUMED.
	//
	// A cell draws BudgetScale x (envelope/anchor)^g candidates and keeps each with
	// probability p, and the envelope divides out, so the batch is supposed to land on the
	// number calibration solved BudgetScale for. Nothing observed that. A batch delivering
	// half its target looked exactly like a batch delivering all of it, and the shortfall
	// was only visible as missing stars.
	//
	// THE RATIO IS NOT THE SAME QUESTION AS THE COUNTERS BESIDE IT. Envelope-exceeded says
	// the probes under-weighed a CELL; this says the tier as a whole is off, which is a
	// calibration fault rather than a sampling one. Both can be clean while the other is not.
	//
	// Wide band, because the realised count varies around the target by roughly its square
	// root and a partial neighbourhood legitimately delivers less than a full one. It is
	// looking for a factor, not a few percent.
	if (bTargetMeaningful && (DeliveryRatio < 0.6 || DeliveryRatio > 1.6))
	{
		static std::atomic<bool> bWarnedDelivery{ false };
		if (!bWarnedDelivery.exchange(true))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("GalaxyEntityGen: tier +%d delivered %u entities against a calibrated ")
				TEXT("target of %.0f (%.0f%%). BudgetScale is solved so this lands on the ")
				TEXT("target, so a factor here means calibration and generation are ")
				TEXT("measuring different things -- check the envelope-exceeded count first, ")
				TEXT("then whether the two dispatches were handed the same EnvelopePad and ")
				TEXT("the same GenerationSubdivision."),
				InSeedOffset, GlobalAccepted, CalibratedTarget, DeliveryRatio * 100.0);
		}
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
		static std::atomic<bool> bWarnedOverflow{ false };
		if (!bWarnedOverflow.exchange(true))
		{
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
	// survives and only the count comes down.
	//
	// AFTER GENERATION RATHER THAN BEFORE IT. Dividing a pooled budget up front needs the
	// batch total in advance, which needs a second dispatch and a global reduction to
	// obtain. Thinning here spends candidates that get discarded and buys that away.
	TArray<int32> SlotTotals;
	SlotTotals.SetNumZeroed(NumSlots);

	for (int32 i = 0; i < Landed; ++i)
	{
		const int32 CellIndex = static_cast<int32>(Entities[i].CellIndex);  // exact: uint32
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
		static std::atomic<bool> bWarnedThin{ false };
		if (!bWarnedThin.exchange(true) || MinKeep < 0.25f)
		{
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
					GetTypeHash(E.Slot)));

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
		const FVector3f Decor = E.DecodeDecor();
		InBuffer.Colors[Idx] = FLinearColor(Decor.X, Decor.Y, Decor.Z);

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
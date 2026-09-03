#include "UniverseDataGenerator.h"

namespace
{
	/** Splits a caller-space cell centre into the exact field cell containing it plus the
	 *  remainder, wrapped into the field period.
	 *
	 *  THE SPLIT RUNS IN DOUBLE AND THE NARROWING HAPPENS AFTER IT, which is the entire
	 *  point. Caller space tracks VirtualTraversal without bound, so an FVector3f built from
	 *  one of these centres directly has a unit of last place of several million caller units
	 *  at long traversals -- and MakeUniverseProbe builds its probes as centre +/- extent, so
	 *  once the extent falls below that ulp every probe in the cell collapses onto one point,
	 *  the envelope reads a single sample of a mostly-empty field, and the tier places
	 *  nothing. Generation stops while the raymarch carries on drawing the field correctly,
	 *  which is about the least helpful pair of symptoms available.
	 *
	 *  Reuses FUniverseFieldOffset::FromCellPosition rather than flooring here, so the cell
	 *  boundary is placed by the same code the material offset uses -- floor, not truncation.
	 *
	 *  WRAPPED WITH THE SAME PERIOD THE CORE DERIVES. The cell index crosses to the shader
	 *  through a float3, so it has to be reduced for the same reason the material offset does;
	 *  reducing by a DIFFERENT period would put placement and the render on wraps that
	 *  disagree, and the two would sample different fields a long way out with nothing
	 *  logging anything. See UniverseCellWrap. */
	void SplitCellCentre(const FVector& InCentreCaller, double InInvFieldCell, int32 InPeriod,
		FIntVector3& OutCell, FVector3f& OutFrac)
	{
		// Wrapped BEFORE the narrow to int32: past 2.1e9 the plain split's cast is undefined,
		// and reducing the result of that reduces a number that already means nothing.
		const FUniverseFieldOffset Split = FUniverseFieldOffset::FromCellPositionWrapped(
			InCentreCaller * InInvFieldCell, InPeriod);

		OutCell = FIntVector3(Split.Cell.X, Split.Cell.Y, Split.Cell.Z);

		OutFrac = FVector3f(
			static_cast<float>(Split.Frac.X),
			static_cast<float>(Split.Frac.Y),
			static_cast<float>(Split.Frac.Z));
	}
}


#pragma region GPU Entity Generation

void UniverseDataGenerator::BuildGenCells(
	const TArray<FTierBatchCell>& InCells,
	TArray<FUniverseGenCell>& OutCells) const
{
	OutCells.Reset();
	OutCells.Reserve(InCells.Num());

	// Hoisted not for the divide but because both are properties of the batch rather than of a
	// cell: the shader derives its own period from one parameter set for the whole draw.
	const double InvFieldCell = 1.0 / FMath::Max(FieldCellSize(), UE_DOUBLE_SMALL_NUMBER);
	const int32 CellPeriod = FieldCellPeriod();

	for (const FTierBatchCell& In : InCells)
	{
		FUniverseGenCell Cell;

		SplitCellCentre(In.Centre, InvFieldCell, CellPeriod, Cell.CentreCell, Cell.CentreFrac);

		Cell.HalfExtent = static_cast<float>(In.HalfExtent);
		Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);

		// THE SHADER CAPS PER SLOT, so it needs to know which; every child carries its
		// parent's.
		Cell.SlotIndex = static_cast<uint32>(FMath::Max(In.SlotIndex, 0));

		OutCells.Add(Cell);
	}
}


void UniverseDataGenerator::BuildCalibrationGrid(
	const FTierParams& InTierParams,
	double InCellHalfExtent,
	TArray<FTierBatchCell>& OutCells) const
{
	OutCells.Reset();

	const double CellFull = InCellHalfExtent * 2.0;

	if (CellFull <= 0.0 || FieldExtent <= 0.0)
	{
		return;
	}

	// THE SPAN IS MEASURED IN FIELD CELLS, NOT TIER CELLS. A field cell is the scale the web's
	// structure lives at, while a tier cell is unrelated and shrinks by four per grid depth, so
	// sizing in tier cells makes a fine tier sample a vanishing fraction of the field.
	const double FieldCell = FieldCellSize();

	const double Span = FieldCell * UniverseEntityGen::kCalibrationSpanFieldCells;

	// How many tier cells fit across that span: a fine tier draws from a much wider coord range
	// than a coarse one, since their cells differ in size but the field does not. CLAMPED IN
	// DOUBLE BEFORE THE NARROW -- the quotient grows by four per grid depth, and RoundToInt on
	// a double past 2.1e9 yields INT32_MIN on x86, inverting RandRange's interval.
	const int32 HalfRange = static_cast<int32>(FMath::Clamp(
		FMath::RoundToDouble(Span / (2.0 * CellFull)), 1.0, 1048576.0));

	// SCATTERED, AND DETERMINISTIC: a fixed stream, so the sample is a property of the tier
	// rather than of when it streamed, and drawn on the tier's own coord lattice, since the
	// coord is a placement key that would otherwise get jitter no real cell ever sees.
	FRandomStream Stream(ProcSeed::MixSeed(
		Params.Seed, UniverseSeed::Placement, InTierParams.GridDepth));

	TArray<FTierBatchCell> Parents;
	Parents.Reserve(UniverseEntityGen::kCalibrationParents);

	for (int32 i = 0; i < UniverseEntityGen::kCalibrationParents; ++i)
	{
		const FIntVector Coord(
			Stream.RandRange(-HalfRange, HalfRange),
			Stream.RandRange(-HalfRange, HalfRange),
			Stream.RandRange(-HalfRange, HalfRange));

		FTierBatchCell Cell;
		Cell.Coord = Coord;
		Cell.Centre = FVector(
			static_cast<double>(Coord.X) * CellFull,
			static_cast<double>(Coord.Y) * CellFull,
			static_cast<double>(Coord.Z) * CellFull);
		Cell.HalfExtent = InCellHalfExtent;
		Cell.SlotIndex = 0;

		Parents.Add(Cell);
	}

	// SUBDIVIDED EXACTLY AS GENERATION WILL SUBDIVIDE. A cell's mass is the mean over its own
	// volume, so probing an undivided parent reports a mean too low by a factor growing with
	// depth -- which comes back as a constant too high and a tier that over-delivers.
	FTierStreamingSystem::SubdivideCells(Parents,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0,
			FTierParams::MaxGenerationSubdivision),
		ETierChildCoords::Ascending, OutCells);
}

float UniverseDataGenerator::GetTierBudgetScale(
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	int32 InGridDepth,
	double InCellHalfExtent) const
{
	if (!TierBudgetScaleLock.IsValid())
	{
		// Moved-from: return zero rather than dereference, and the caller treats it as a
		// failure.
		return 0.0f;
	}

	FScopeLock Lock(TierBudgetScaleLock.Get());

	if (const float* Cached = TierBudgetScales.Find(InSeedOffset))
	{
		return *Cached;
	}

	if (!FieldTextures.IsComplete() || FieldExtent <= 0.0)
	{
		return 0.0f;
	}

	// THE PARAMETERS THIS TIER IS ACTUALLY RUNNING WITH. A tier parameter that fails to arrive
	// looks exactly like one that has no effect, and a generator holds a COPY taken at
	// InitializeData, so an edit can land on something nothing reads. Read these before
	// concluding anything about a parameter's effect.
	UE_LOG(LogTemp, Log,
		TEXT("UniverseEntityGen: tier offset %d effective params -- GridDepth %d, ")
		TEXT("NeighborhoodRadius %d, SlotCapacity %d, GenerationSubdivision %d, ")
		TEXT("SpawnExponent %.4f, ExtentExponent %.4f, MinScale %.4g, MaxScale %.4g."),
		InSeedOffset, InTierParams.GridDepth, InTierParams.NeighborhoodRadius,
		InTierParams.SlotCapacity, InTierParams.GenerationSubdivision,
		InTierParams.SpawnExponent, InTierParams.ExtentExponent,
		InTierParams.MinScale, InTierParams.MaxScale);

	TArray<FTierBatchCell> AllCells;
	BuildCalibrationGrid(InTierParams, InCellHalfExtent, AllCells);

	if (AllCells.Num() == 0)
	{
		return 0.0f;
	}

	// THE SAME BUILDER GENERATION USES: a calibration that decomposed its cells differently
	// would solve the constant against a field offset by a fraction of a cell from where
	// entities land.
	TArray<FUniverseGenCell> Cells;
	BuildGenCells(AllCells, Cells);

	TArray<float> CellMass;

	const double CalibrationStart = FPlatformTime::Seconds();

	float Scale = 0.0f;

	if (UniverseEntityGen::CalibrateBlocking(
		Params, InTierParams, Cells, TierKeySeed(InSeedOffset),
		InvFieldExtent(), FieldTextures, CellMass)
		&& CellMass.Num() == AllCells.Num())
	{
		// REDUCED HERE, NOT ON THE GPU, and in double. A slot receives the sum of what its
		// cells produce, and every streamed cell owns its own slot, so the divisor is a
		// per-parent SUM: a global max over children would be wrong by the child count, and a
		// mean would let dense cells overflow while the rest ran empty.
		TMap<int32, double> ParentSums;

		double TotalMass = 0.0;

		for (int32 i = 0; i < AllCells.Num(); ++i)
		{
			const double M = static_cast<double>(CellMass[i]);

			TotalMass += M;

			double& Sum = ParentSums.FindOrAdd(AllCells[i].ParentIndex);
			Sum += M;
		}

		TArray<double> Sums;
		Sums.Reserve(ParentSums.Num());
		for (const TPair<int32, double>& Pair : ParentSums)
		{
			Sums.Add(Pair.Value);
		}
		Sums.Sort();

		// A PERCENTILE, NOT THE MAXIMUM (see kCalibrationPercentile): this field's dynamic
		// range is wider than a slot can represent, so the choice is where to put the knee, not
		// whether to clip. Dividing by the max leaves all but the densest cells invisible,
		// while dividing just below the tail lets the top couple of percent pin at capacity.
		double Divisor = 0.0;
		double MedianSum = 0.0;
		double MaxSum = 0.0;

		if (Sums.Num() > 0)
		{
			const int32 PercentileIdx = FMath::Clamp(
				FMath::RoundToInt(UniverseEntityGen::kCalibrationPercentile
					* static_cast<float>(Sums.Num() - 1)),
				0, Sums.Num() - 1);

			Divisor = Sums[PercentileIdx];
			MedianSum = Sums[Sums.Num() / 2];
			MaxSum = Sums.Last();
		}

		if (Divisor > 0.0)
		{
			Scale = static_cast<float>(
				static_cast<double>(InTierParams.SlotCapacity) / Divisor);
		}

		// THE SHAPE OF THE DISTRIBUTION, which says whether one constant can serve the whole
		// field: max over median is the dynamic range this tier's cells see, a few times over
		// meaning a constant works and orders of magnitude meaning the percentile is doing real
		// work.
		const double Range = (MedianSum > 0.0) ? (MaxSum / MedianSum) : 0.0;

		UE_LOG(LogTemp, Log,
			TEXT("UniverseEntityGen: tier offset %d calibrated in %.3fs over %d cells ")
			TEXT("(%d scattered parents); BudgetScale %.6f from the %.0fth percentile ")
			TEXT("mass %.5f; median %.5f, max %.5f, range %.1fx."),
			InSeedOffset, FPlatformTime::Seconds() - CalibrationStart,
			AllCells.Num(), ParentSums.Num(), Scale,
			UniverseEntityGen::kCalibrationPercentile * 100.0f,
			Divisor, MedianSum, MaxSum, Range);
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("UniverseEntityGen: calibration FAILED for tier offset %d over %d cells. ")
			TEXT("This tier will generate nothing until it succeeds."),
			InSeedOffset, AllCells.Num());
	}

	// CACHED EVEN AT ZERO IS WRONG, so it is not: a failed calibration is usually transient,
	// and caching the failure would leave the tier empty for the session with nothing to retry
	// it.
	if (Scale > 0.0f)
	{
		TierBudgetScales.Add(InSeedOffset, Scale);
	}

	return Scale;
}

bool UniverseDataGenerator::GenerateTierBatchGPU(
	const TArray<FTierBatchCell>& InQueuedCells,
	FNiagaraParticleBuffer& InBuffer,
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	int32 InGridDepth,
	TArray<int32>& OutSlotCounts) const
{
	// FAIL CLOSED, VISIBLY: there is no CPU path behind this, and a slot is REUSED across
	// boundary crosses, so "nothing written" leaves the previous occupant at a stale coord.
	auto FailBatch = [&InQueuedCells, &InBuffer, &OutSlotCounts]() -> bool
		{
			OutSlotCounts.SetNumZeroed(InBuffer.SlotCoord.Num());

			// The QUEUED cells, not the subdivided ones: blanking is per slot and children
			// share.
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

	if (InQueuedCells.Num() == 0)
	{
		return true;
	}

	// LOUD ONCE, then quiet: ensure fires on its first hit per call site per session. A
	// misconfiguration would otherwise surface only as a tier that never populates, which reads
	// as a streaming problem rather than a setup one.
	if (!FieldTextures.IsComplete())
	{
		// NAMES THE MISSING ONES: with four assets, "the texture is unresolved" sends the
		// developer to check four paths when three are usually fine.
		ensureMsgf(false,
			TEXT("UniverseEntityGen: field textures unresolved (%s), so the sector will ")
			TEXT("place NOTHING -- placement is GPU-only and the dispatch samples all four. ")
			TEXT("They are loaded from the VarianceVolumeA/B and WarpVolumeLarge/Small paths ")
			TEXT("in MaterialParams; check those paths, that UniverseNoisePack is enabled, ")
			TEXT("and ")
			TEXT("that each asset has NEVER STREAM set."),
			*FieldTextures.DescribeMissing());
		return FailBatch();
	}

	if (FieldExtent <= 0.0)
	{
		ensureMsgf(false,
			TEXT("UniverseEntityGen: FieldExtent is unset. The actor must hand the ")
			TEXT("generator the ray march proxy's half extent before generation runs, or ")
			TEXT("placement and render sample different scalings of the field."));
		return FailBatch();
	}

	// Descend inside each queued cell. Children clear of the structure are culled by the probe
	// pass, and survivors have a peak much closer to their own mean, keeping the accepted
	// fraction high.
	TArray<FTierBatchCell> Subdivided;
	FTierStreamingSystem::SubdivideCells(InQueuedCells,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0,
			FTierParams::MaxGenerationSubdivision),
		ETierChildCoords::Ascending, Subdivided);

	if (Subdivided.Num() == 0)
	{
		return true;
	}

	const double CellHalfExtent = InQueuedCells[0].HalfExtent;

	const float BudgetScale =
		GetTierBudgetScale(InTierParams, InSeedOffset, InGridDepth, CellHalfExtent);

	if (!(BudgetScale > 0.0f))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("UniverseEntityGen: tier offset %d has no placement constant; blanking ")
			TEXT("%d slots."), InSeedOffset, InQueuedCells.Num());
		return FailBatch();
	}

	TArray<FUniverseGenCell> Cells;
	BuildGenCells(Subdivided, Cells);

	// ONE SHARED BUFFER, sized on what the SLOTS can hold rather than cells times a worst case:
	// every queued cell owns one slot, so this is exactly what the batch can keep.
	const int32 EntityCapacity = InQueuedCells.Num() * InBuffer.SlotCapacity;

	TArray<FUniverseEntityOut> Entities;
	TArray<uint32> Counts;

	if (!UniverseEntityGen::GenerateBatchBlocking(
		Params, InTierParams, Cells, EntityCapacity,
		InBuffer.SlotCoord.Num(), InBuffer.SlotCapacity,
		TierKeySeed(InSeedOffset), InvFieldExtent(), FieldTextures,
		BudgetScale, Entities, Counts))
	{
		return FailBatch();
	}

	// --- scatter --- STORAGE ORDER SAYS NOTHING. The dispatch compacts with a global atomic,
	// so an entity's position in the readback is scheduling dependent; its CellIndex says which
	// cell, and therefore which slot, it belongs to. Keying off position shuffles a region
	// between visits.
	OutSlotCounts.SetNumZeroed(InBuffer.SlotCoord.Num());

	// Per-slot write cursors: the buffer reserves SlotCapacity per slot, the dispatch does not
	// know about that partition, so the overflow is caught here.
	TArray<int32> SlotCursors;
	SlotCursors.SetNumZeroed(InBuffer.SlotCoord.Num());

	// RECORDS ACTUALLY WRITTEN. The shader caps each slot at SlotCapacity before claiming a
	// global index, so this is bounded by capacity; the demand figure is in the per-slot
	// counters below.
	const uint32 TotalAccepted =
		Counts.IsValidIndex(UniverseEntityGen::GlobalCursorIndex(Cells.Num()))
		? Counts[UniverseEntityGen::GlobalCursorIndex(Cells.Num())]
		: 0u;

	// THE CURSOR BOUNDS THE LOOP, NOT THE ARRAY LENGTH. The readback copies the WHOLE buffer,
	// capacity records, because the copy size must be known before the dispatch runs, while the
	// shader appends only up to the global cursor -- so everything past it is whatever RDG's
	// pool handed back. Walking the full array scatters garbage: NaN positions and arbitrary
	// CellIndex values, and one landing in range by chance passes every bounds test and writes
	// a NaN, which crashes the octree's child-index arithmetic out of bounds.
	const int32 Written = FMath::Min(
		static_cast<int32>(FMath::Min<uint32>(TotalAccepted, static_cast<uint32>(INT32_MAX))),
		EntityCapacity);

	int32 Dropped = 0;
	int32 Rejected = 0;

	// THE SAMPLED DENSITY OF WHAT WAS ACTUALLY PLACED, answering the alignment question
	// numerically rather than by eye. Every accepted entity passed a test against its cell's
	// envelope, so these should sit well up the field's range; a mean near the void floor means
	// the dispatch is sampling somewhere the render is not.
	double DensitySum = 0.0;
	float DensityMin = TNumericLimits<float>::Max();
	float DensityMax = 0.0f;
	int32 DensityCount = 0;

	for (int32 i = 0; i < Written; ++i)
	{
		const FUniverseEntityOut& E = Entities[i];

		if (!Cells.IsValidIndex(static_cast<int32>(E.CellIndex)))
		{
			continue;
		}

		const FTierBatchCell& SourceCell = Subdivided[static_cast<int32>(E.CellIndex)];
		const int32 SlotIndex = SourceCell.SlotIndex;

		if (!SlotCursors.IsValidIndex(SlotIndex))
		{
			continue;
		}

		int32& Cursor = SlotCursors[SlotIndex];

		if (Cursor >= InBuffer.SlotCapacity)
		{
			++Dropped;
			continue;
		}

		const int32 Idx = SlotIndex * InBuffer.SlotCapacity + Cursor;

		if (!InBuffer.Positions.IsValidIndex(Idx))
		{
			continue;
		}

		// THE CELL'S CENTRE IS ADDED BACK HERE, IN DOUBLE. E.Pos is the entity's offset from
		// its cell centre, so the sum is exact where an absolute float32 was not, and
		// SourceCell.Centre is the same double the split above consumed.
		const FVector Pos = SourceCell.Centre + FVector(E.Pos.X, E.Pos.Y, E.Pos.Z);

		// BELT AND BRACES, which should never fire now that the cursor bounds the loop. It
		// stays because the octree's child-index arithmetic on a non-finite coordinate
		// crashes with a stack that says nothing about where the value came from. If Rejected
		// is non-zero, the buffer is being read past what the shader wrote.
		if (Pos.ContainsNaN() || !FMath::IsFinite(E.Extent))
		{
			++Rejected;
			continue;
		}

		InBuffer.Positions[Idx] = Pos;
		InBuffer.Extents[Idx] = E.Extent;

		DensitySum += static_cast<double>(E.Density);
		DensityMin = FMath::Min(DensityMin, E.Density);
		DensityMax = FMath::Max(DensityMax, E.Density);
		++DensityCount;

		// Three decorrelated uniforms, in the colour convention the Niagara systems already
		// read.
		const FVector3f Decor = E.DecodeDecor();

		if (Params.MaterialParams.bDebugColorByDensity)
		{
			// THE DENSITY THE DISPATCH ACTUALLY READ at this entity's position, painted so it
			// can be compared against the raymarch; see bDebugColorByDensity. Normalised by
			// the analytic ceiling, so the scale means the same across tiers and retunes.
			const FUniverseDensityParams& DP = Params.DensityParams;

			const float Ceiling =
				FMath::Max(DP.Wall.Density.Min, DP.Wall.Density.Max)
				+ FMath::Max(DP.Filament.Density.Min, DP.Filament.Density.Max)
				+ FMath::Max(DP.Void.Floor.Min, DP.Void.Floor.Max);

			const float T = FMath::Clamp(E.Density / FMath::Max(Ceiling, 1e-6f), 0.0f, 1.0f);

			InBuffer.Colors[Idx] = FLinearColor(T, T * T, 1.0f - T, 1.0f);
		}
		else
		{
			InBuffer.Colors[Idx] = FLinearColor(Decor.X, Decor.Y, Decor.Z, 1.0f);
		}

		if (InBuffer.Rotations.IsValidIndex(Idx))
		{
			// A unit vector from the decoratives, already decorrelated, rather than a fourth
			// hash.
			const FVector Dir(
				static_cast<double>(Decor.X) * 2.0 - 1.0,
				static_cast<double>(Decor.Y) * 2.0 - 1.0,
				static_cast<double>(Decor.Z) * 2.0 - 1.0);

			InBuffer.Rotations[Idx] = Dir.GetSafeNormal(UE_DOUBLE_SMALL_NUMBER, FVector::UpVector);
		}

		++Cursor;
	}

	// PAD EVERY QUEUED SLOT, including ones that received nothing: untouched still holds the
	// last.
	for (const FTierBatchCell& Cell : InQueuedCells)
	{
		if (SlotCursors.IsValidIndex(Cell.SlotIndex))
		{
			const int32 Count = SlotCursors[Cell.SlotIndex];

			InBuffer.PadSlotDead(Cell.SlotIndex, Count);
			OutSlotCounts[Cell.SlotIndex] = Count;
		}
	}

	// HOW FAR THE CALIBRATED CONSTANT IS OUT, and the only observer of it: a full slot looks
	// the same whether it wanted one more entity or twenty times more, and the counters keep
	// counting past the ceiling. THIS IS THE NUMBER THAT SAYS WHETHER CALIBRATION
	// GENERALISES -- a ratio near 1 means the origin sample was representative, a ratio in the
	// tens means the constant describes a part of the field the player is not in.
	uint32 SlotDemand = 0;
	uint32 WorstSlotDemand = 0;

	for (int32 sIdx = 0; sIdx < InBuffer.SlotCoord.Num(); ++sIdx)
	{
		const int32 CountIdx = UniverseEntityGen::SlotCursorIndex(Cells.Num(), sIdx);

		if (Counts.IsValidIndex(CountIdx))
		{
			SlotDemand += Counts[CountIdx];
			WorstSlotDemand = FMath::Max(WorstSlotDemand, Counts[CountIdx]);
		}
	}

	const double DemandRatio = (InBuffer.SlotCapacity > 0)
		? (static_cast<double>(WorstSlotDemand) / static_cast<double>(InBuffer.SlotCapacity))
		: 0.0;

	// THE DIAGNOSTIC THAT DECIDES SUBDIVISION. Probes and candidates are both field evaluations
	// -- here a fifty-four candidate walk plus five fetches -- so their ratio is the whole cost
	// argument: below about 1 the tier wants one subdivision fewer, above 9 one more.
	uint32 CandidatesEvaluated = 0;

	// THE ENVELOPE CHECK, the first thing to read when placement has the right SHAPE but the
	// wrong GRADIENT. Acceptance is saturate(d / envelope)^g, so everything above the envelope
	// accepts with probability 1. WHAT THAT COSTS IS THE COUNT, NOT THE SHAPE -- the clipped
	// region is the cell's brightest part -- but accepted_i = BudgetScale x mass_i holds only
	// while nothing saturates. A clipping cell therefore OVER-DELIVERS against its budget and
	// the surplus is cut by slot capacity rather than by the field, so bright cells all pin at
	// capacity while faint ones under-fill; check `dropped over capacity` alongside this.
	//
	// THE SPAWN EXPONENT MAKES THIS SHARPER BOTH WAYS: at g = 8 a candidate at 90% of the
	// envelope accepts at 0.43 against 1 at 100%, so a high exponent is only meaningful once
	// the envelope is known to be tight.
	int32 CellsClipped = 0;
	double WorstOvershoot = 0.0;

	for (int32 c = 0; c < Cells.Num(); ++c)
	{
		const int32 Base = c * UniverseEntityGen::CountersPerCell;
		if (Counts.IsValidIndex(Base + 1))
		{
			CandidatesEvaluated += Counts[Base + 1];
		}

		if (!Counts.IsValidIndex(Base + 3))
		{
			continue;
		}

		// Both written as asuint by the shader.
		const uint32 EnvBits = Counts[Base + 2];
		const uint32 PeakBits = Counts[Base + 3];

		const float Envelope = *reinterpret_cast<const float*>(&EnvBits);
		const float Peak = *reinterpret_cast<const float*>(&PeakBits);

		if (Envelope > 0.0f && Peak > Envelope)
		{
			++CellsClipped;
			WorstOvershoot = FMath::Max(WorstOvershoot,
				static_cast<double>(Peak) / static_cast<double>(Envelope));
		}
	}

	const double Probes =
		static_cast<double>(Cells.Num()) * FUniverseEntityGenCS::ProbesPerCell;

	UE_LOG(LogTemp, Verbose,
		TEXT("UniverseEntityGen: tier offset %d placed %d/%u accepted into %d slots over ")
		TEXT("%d cells; C/P %.2f, dropped %d, rejected %d non-finite, ")
		TEXT("%d/%d cells clipped (worst %.2fx), worst slot demand %.1fx capacity; ")
		TEXT("placed density min %.4f mean %.4f max %.4f; slots %.1f%% full."),
		InSeedOffset, Written, TotalAccepted, InQueuedCells.Num(), Cells.Num(),
		(Probes > 0.0) ? (static_cast<double>(CandidatesEvaluated) / Probes) : 0.0,
		Dropped, Rejected, CellsClipped, Cells.Num(), WorstOvershoot, DemandRatio,
		(DensityCount > 0) ? DensityMin : 0.0f,
		(DensityCount > 0) ? (DensitySum / DensityCount) : 0.0,
		DensityMax,
		(EntityCapacity > 0)
		? (100.0 * static_cast<double>(Written) / static_cast<double>(EntityCapacity))
		: 0.0);

	// FOUR TIMES CAPACITY IS WHERE THE SLOT STOPS DESCRIBING THE FIELD: below it a full slot is
	// a genuinely dense region, far above it every dense region is equally full.
	if (DemandRatio > 4.0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("UniverseEntityGen: tier offset %d wants %.1fx its slot capacity in the ")
			TEXT("worst slot. The calibrated constant was measured at the origin and does ")
			TEXT("not describe the field here; dense slots are all pinning at capacity and ")
			TEXT("the population has stopped tracking density."),
			InSeedOffset, DemandRatio);
	}

	// A FEW PERCENT IS NORMAL -- a finite probe set cannot find every peak, which is what
	// EnvelopePad absorbs. A large fraction means acceptance is saturating and the gradient is
	// flattened cell by cell, a SAMPLING problem: raise EnvelopePad or lower the spawn
	// exponent. Raising the budget does not help, every candidate seeing the same envelope.
	if (Cells.Num() > 0 && CellsClipped * 4 > Cells.Num())
	{
		UE_LOG(LogTemp, Warning,
			TEXT("UniverseEntityGen: tier offset %d clipped in %d of %d cells, worst %.2fx ")
			TEXT("over envelope. Those cells over-deliver against their budget and get cut ")
			TEXT("by slot capacity instead, which flattens the population across bright ")
			TEXT("cells. Raise the probe rounds before EnvelopePad -- the pad costs ")
			TEXT("candidates exponentially in SpawnExponent."),
			InSeedOffset, CellsClipped, Cells.Num(), WorstOvershoot);
	}

	// A NON-FINITE RECORD IS NOT A TUNING PROBLEM: the scatter read past what the dispatch
	// wrote -- a plumbing fault whose crash surfaces inside the octree.
	if (Rejected > 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("UniverseEntityGen: tier offset %d rejected %d non-finite entity records ")
			TEXT("out of %d read against a cursor of %u. The scatter is reading beyond the ")
			TEXT("shader's global append cursor."),
			InSeedOffset, Rejected, Written, TotalAccepted);
	}

	return true;
}

#pragma endregion
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
		// Wrapped BEFORE the narrow to int32, not after: a cell position past 2.1e9 makes the
		// plain split's cast undefined, and reducing the result of that is reducing a number
		// that already means nothing. Same call the material offset uses.
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

void UniverseDataGenerator::SubdivideCells(
	const TArray<FTierBatchCell>& InCells,
	int32 InLevels,
	TArray<FTierBatchCell>& OutCells)
{
	OutCells.Reset();

	if (InLevels <= 0)
	{
		OutCells = InCells;

		// Every cell is its own parent, so a caller that groups by ParentIndex gets the
		// same answer whether or not the tier subdivides.
		for (int32 i = 0; i < OutCells.Num(); ++i)
		{
			OutCells[i].ParentIndex = i;
		}

		return;
	}

	const int32 Side = 1 << InLevels;
	const int32 PerCell = Side * Side * Side;

	OutCells.Reserve(InCells.Num() * PerCell);

	for (int32 ParentIndex = 0; ParentIndex < InCells.Num(); ++ParentIndex)
	{
		const FTierBatchCell& Parent = InCells[ParentIndex];

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
					FTierBatchCell Child;

					Child.Centre = Parent.Centre + FVector(
						(Origin + static_cast<double>(ix)) * SubFull,
						(Origin + static_cast<double>(iy)) * SubFull,
						(Origin + static_cast<double>(iz)) * SubFull);

					Child.HalfExtent = SubHalf;

					// UNIQUE ACROSS PARENTS and a pure function of the parent, so a child
					// regenerates identically however the batch was assembled. These do NOT
					// correspond to positions on the streaming grid at the deeper level, and
					// nothing requires them to -- they are placement keys, not grid coords.
					Child.Coord = FIntVector(
						Parent.Coord.X * Side + ix,
						Parent.Coord.Y * Side + iy,
						Parent.Coord.Z * Side + iz);

					// EVERY CHILD KEEPS ITS PARENT'S SLOT. The buffer still holds one region
					// per streamed cell; only the generation grid got finer.
					Child.SlotIndex = Parent.SlotIndex;
					Child.ParentIndex = ParentIndex;

					// NO BOUNDS CULL. See the header: this field has no outside, so a child
					// past any boundary is still a child that can hold structure.
					OutCells.Add(Child);
				}
			}
		}
	}
}

void UniverseDataGenerator::BuildGenCells(
	const TArray<FTierBatchCell>& InCells,
	TArray<FUniverseGenCell>& OutCells) const
{
	OutCells.Reset();
	OutCells.Reserve(InCells.Num());

	// Hoisted, not because the divide is expensive but because both are properties of the
	// batch rather than of a cell: reading them per cell would let a caller believe they
	// could vary within one dispatch, and they cannot -- the shader derives its own period
	// from one parameter set for the whole draw.
	const double InvFieldCell = 1.0 / FMath::Max(FieldCellSize(), UE_DOUBLE_SMALL_NUMBER);
	const int32 CellPeriod = FieldCellPeriod();

	for (const FTierBatchCell& In : InCells)
	{
		FUniverseGenCell Cell;

		SplitCellCentre(In.Centre, InvFieldCell, CellPeriod, Cell.CentreCell, Cell.CentreFrac);

		Cell.HalfExtent = static_cast<float>(In.HalfExtent);
		Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);

		// THE SHADER CAPS PER SLOT, so it needs to know which one. Every child of a
		// subdivided streamed cell carries its parent's.
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

	// THE SPAN IS MEASURED IN FIELD CELLS, NOT TIER CELLS, and that is the whole correction.
	//
	// A field cell is FieldExtent * CellSizeSmall in caller units, and it is the scale the
	// web's structure actually lives at. A tier cell is unrelated to it and shrinks by four
	// per grid depth, so sizing the sample in tier cells made the fine tiers sample a
	// vanishing fraction of the field -- the Small tier was measuring four tenths of one
	// field cell and calibrating the whole universe against it.
	const double FieldCell = FieldCellSize();

	const double Span = FieldCell * UniverseEntityGen::kCalibrationSpanFieldCells;

	// How many tier cells fit across that span. The scatter picks coords in this range, so
	// a fine tier draws its sample from a much wider coord range than a coarse one -- which
	// is exactly the point, since their cells differ in size but the field does not.
	const int32 HalfRange = FMath::Max(
		FMath::RoundToInt(Span / (2.0 * CellFull)), 1);

	// SCATTERED, AND DETERMINISTIC. A fixed stream, so the sample is a property of the tier
	// rather than of when it first streamed -- the result is cached for the session and two
	// runs of the same seed must calibrate identically.
	//
	// Drawn on the tier's own coord lattice rather than at arbitrary positions, because the
	// coord is a placement key: MakeUniverseProbe hashes it, so a cell at a coord the grid
	// could never produce would be probed with jitter no real cell ever sees.
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
	// volume, so probing an undivided parent reports a mean that is too low by a factor
	// growing with the subdivision depth -- which comes back as a constant that is too high
	// and a tier that over-delivers.
	SubdivideCells(Parents,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0,
			FTierParams::MaxGenerationSubdivision),
		OutCells);
}

float UniverseDataGenerator::GetTierBudgetScale(
	const FTierParams& InTierParams,
	int32 InSeedOffset,
	int32 InGridDepth,
	double InCellHalfExtent) const
{
	if (!TierBudgetScaleLock.IsValid())
	{
		// Moved-from. Return zero rather than dereference; the caller treats it as a failed
		// batch, which is the honest answer.
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

	// THE PARAMETERS THIS TIER IS ACTUALLY RUNNING WITH, logged before anything is measured
	// against them.
	//
	// NOT INSTRUMENTATION FOR ITS OWN SAKE. A tier parameter that fails to arrive looks
	// exactly like a tier parameter that has no effect, and this layer has already lost two
	// debugging rounds to that: GenerationSubdivision read 0 after being set to 2, and
	// SpawnExponent showed no difference between 0.01 and 16, both because the edit was
	// landing on a copy nothing read. The indirection responsible is gone -- see the note
	// where FUniverseParamBounds used to be -- but the class of failure is not, since a
	// generator holds a COPY of the params taken at InitializeData.
	//
	// Read these before concluding anything about a parameter's effect.
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

	// THE SAME BUILDER GENERATION USES. Calibration coords are drawn near the origin, so
	// this path was never the one that lost precision -- it shares the split anyway, because
	// a calibration that decomposed its cells differently from generation would solve the
	// constant against a field offset by a fraction of a cell from the one entities land in.
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
		// REDUCED HERE, NOT ON THE GPU, and in double.
		//
		// A slot receives the sum of what its cells produce, and for this layer every
		// streamed cell owns its own slot, so the divisor is a per-parent SUM rather than a
		// per-cell value. A global maximum over children would be wrong by the child count,
		// and a mean would let dense cells overflow while the rest ran empty.
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

		// A PERCENTILE, NOT THE MAXIMUM. See kCalibrationPercentile: this field's dynamic
		// range is wider than a slot can represent, so the choice is not whether to clip but
		// where to put the knee. Dividing by the max leaves everything but the densest cells
		// invisible; dividing just below the tail lets the top couple of percent pin at
		// capacity while every other cell tracks density.
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

		// THE SHAPE OF THE DISTRIBUTION, which is what says whether one constant can serve
		// the whole field. The max-over-median ratio is the field's dynamic range as this
		// tier's cells see it: a few times over means a constant works well, orders of
		// magnitude means the tail will always clip and the percentile is doing real work.
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

	// CACHED EVEN AT ZERO IS WRONG, so it is not. A failed calibration is usually transient
	// -- the render thread was busy, the texture had not streamed -- and caching the
	// failure would leave the tier permanently empty for the session with nothing to
	// retry it.
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
	// FAIL CLOSED, VISIBLY. There is no CPU path behind this, so a failure means these
	// slots get nothing -- and a slot is REUSED as the player crosses boundaries, so
	// "nothing written" is not an empty slot, it is the previous occupant's entities still
	// sitting there at a coord they no longer belong to.
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

	if (InQueuedCells.Num() == 0)
	{
		return true;
	}

	// LOUD ONCE, then quiet. ensure fires on its first hit per call site per session, which
	// is right for a setup error: it stops the developer and lands in the log without
	// spamming on every boundary cross. A misconfiguration would otherwise surface only as
	// a tier that never populates, which reads as a streaming problem rather than a setup
	// one.
	if (!FieldTextures.IsComplete())
	{
		// NAMES THE MISSING ONES. With four assets "the texture is unresolved" sends the
		// developer to check four paths, and the common case is that three are fine and one
		// path has a typo in it.
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

	// Descend inside each queued cell. Children clear of the structure are culled by the
	// probe pass and draw no candidates, and the ones that survive have a peak much closer
	// to their own mean -- which is what keeps the accepted fraction high enough that the
	// candidate budget is not mostly waste.
	TArray<FTierBatchCell> Subdivided;
	SubdivideCells(InQueuedCells,
		FMath::Clamp(InTierParams.GenerationSubdivision, 0,
			FTierParams::MaxGenerationSubdivision),
		Subdivided);

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

	// ONE SHARED BUFFER, sized on what the SLOTS can hold rather than on cells times a
	// worst case. Every queued cell owns one slot, so this is exactly what the batch can
	// possibly keep.
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

	// --- scatter ---
	//
	// STORAGE ORDER SAYS NOTHING. The dispatch compacts with a global atomic, so an
	// entity's position in the readback is scheduling dependent; its CellIndex is what
	// says which cell -- and therefore which slot -- it belongs to. Keying off position
	// would shuffle a region's entities between visits.
	OutSlotCounts.SetNumZeroed(InBuffer.SlotCoord.Num());

	// Per-slot write cursors. The buffer reserves SlotCapacity per slot and the dispatch
	// does not know about that partition, so the overflow is caught here.
	TArray<int32> SlotCursors;
	SlotCursors.SetNumZeroed(InBuffer.SlotCoord.Num());

	// RECORDS ACTUALLY WRITTEN. The shader now caps each slot at SlotCapacity before it
	// claims a global index, so this is bounded by capacity rather than being the raw
	// accepted count -- the demand figure moved to the per-slot counters below.
	const uint32 TotalAccepted =
		Counts.IsValidIndex(UniverseEntityGen::GlobalCursorIndex(Cells.Num()))
		? Counts[UniverseEntityGen::GlobalCursorIndex(Cells.Num())]
		: 0u;

	// THE CURSOR BOUNDS THE LOOP, NOT THE ARRAY LENGTH, and the difference is not cosmetic.
	//
	// The readback copies the WHOLE entity buffer -- capacity records -- because the copy
	// size has to be known before the dispatch runs. The shader only appends up to the
	// global cursor, so everything past it is untouched: whatever RDG's pool handed back,
	// which is a previous dispatch's entities or uninitialised device memory.
	//
	// Walking the full array therefore scatters garbage. Read as a record it gives NaN
	// positions and arbitrary CellIndex values, and a CellIndex that lands in range by
	// chance passes every bounds test here and writes a NaN position into the buffer. That
	// is two bugs at once: the octree's child-index arithmetic on a NaN goes out of bounds
	// and crashes releasing a garbage TSharedPtr, and the entities that do survive bear no
	// relation to the density field, because they were never placed against it.
	const int32 Written = FMath::Min(
		static_cast<int32>(FMath::Min<uint32>(TotalAccepted, static_cast<uint32>(INT32_MAX))),
		EntityCapacity);

	int32 Dropped = 0;
	int32 Rejected = 0;

	// THE SAMPLED DENSITY OF WHAT WAS ACTUALLY PLACED, which answers the alignment question
	// numerically rather than by eye. Every accepted entity passed a test against its cell's
	// envelope, so if the dispatch is reading the field it means to read, these values sit
	// well up the field's range. A mean hovering near the void floor means the dispatch is
	// sampling somewhere the render is not.
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
		// its cell centre in caller units -- bounded by the cell -- so the sum is exact where
		// the shader returning an absolute float32 was not. SourceCell.Centre is the same
		// double the split above consumed, so this is the inverse of that split and not a
		// reconstruction of it.
		const FVector Pos = SourceCell.Centre + FVector(E.Pos.X, E.Pos.Y, E.Pos.Z);

		// BELT AND BRACES, and it should never fire now that the cursor bounds the loop.
		// It stays because the consumer is the octree, whose child-index arithmetic on a
		// non-finite coordinate walks out of its Children array and crashes releasing a
		// garbage shared pointer -- a failure whose stack points at the octree and says
		// nothing about where the bad value came from. One comparison is cheap insurance
		// against ever debugging that again; if Rejected is non-zero, the entity buffer is
		// being read past what the shader wrote.
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

		// The decoratives are three decorrelated uniforms; the colour convention matches
		// what the CPU path wrote, so the Niagara systems need no change.
		const FVector3f Decor = E.DecodeDecor();

		if (Params.MaterialParams.bDebugColorByDensity)
		{
			// THE DENSITY THE DISPATCH ACTUALLY READ at this entity's own position, painted
			// so it can be compared against the raymarch drawing the same field. See
			// bDebugColorByDensity for how to read the three outcomes.
			//
			// Normalised by the analytic ceiling rather than by anything measured, so the
			// scale means the same thing between tiers and survives a retune of the ranges.
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
			// A unit vector from the decoratives rather than a fourth hash: the three are
			// already decorrelated from position and from each other, and a face normal only
			// has to be stable per entity.
			const FVector Dir(
				static_cast<double>(Decor.X) * 2.0 - 1.0,
				static_cast<double>(Decor.Y) * 2.0 - 1.0,
				static_cast<double>(Decor.Z) * 2.0 - 1.0);

			InBuffer.Rotations[Idx] = Dir.GetSafeNormal(UE_DOUBLE_SMALL_NUMBER, FVector::UpVector);
		}

		++Cursor;
	}

	// PAD EVERY QUEUED SLOT, including the ones that received nothing. A slot left
	// untouched still holds its previous occupant.
	for (const FTierBatchCell& Cell : InQueuedCells)
	{
		if (SlotCursors.IsValidIndex(Cell.SlotIndex))
		{
			const int32 Count = SlotCursors[Cell.SlotIndex];

			InBuffer.PadSlotDead(Cell.SlotIndex, Count);
			OutSlotCounts[Cell.SlotIndex] = Count;
		}
	}

	// HOW FAR THE CALIBRATED CONSTANT IS OUT, which is the only observer of that: a slot
	// that is merely full looks identical whether it wanted one more entity or twenty times
	// more, and the per-slot counters keep counting past the ceiling so the difference is
	// visible.
	//
	// THIS IS THE NUMBER THAT SAYS WHETHER CALIBRATION GENERALISES. It is measured once at
	// the origin against a field that is only statistically homogeneous, so a demand ratio
	// that stays near 1 means the sample was representative and a ratio in the tens means
	// it was not -- the constant is describing a part of the field the player is not in.
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

	// THE DIAGNOSTIC THAT DECIDES SUBDIVISION. Probes and candidates are both field
	// evaluations, and a universe field evaluation is a fifty-four candidate walk plus five
	// texture fetches -- so the ratio between them is the whole cost argument. Below about
	// 1 the probes have overtaken placement and the tier wants one subdivision level fewer;
	// above about 9 it wants one more.
	uint32 CandidatesEvaluated = 0;

	// THE ENVELOPE CHECK, and it is the first thing to read when placement has the right
	// SHAPE but the wrong GRADIENT.
	//
	// Acceptance is saturate(d / envelope)^g, so everything above the envelope accepts with
	// probability 1. WHAT THAT COSTS IS THE COUNT, NOT THE SHAPE, and the distinction is
	// worth keeping straight: the clipped region is the cell's brightest part, so entities
	// landing in it are still on the structure. What breaks is the relation the whole budget
	// rests on -- accepted_i = BudgetScale x mass_i holds only while nothing saturates.
	//
	// A clipping cell therefore OVER-DELIVERS against its budget, and the surplus is then
	// cut by slot capacity rather than by the field. That is what reads as a loose fit at
	// the large scale: bright cells all pin at capacity and become indistinguishable from
	// each other, while faint cells under-fill. Check `dropped over capacity` alongside
	// this; the two move together.
	//
	// THE SPAWN EXPONENT MAKES THIS SHARPER IN BOTH DIRECTIONS. At g = 8 a candidate at
	// 90% of the envelope accepts at 0.43 while one at 100% accepts at 1, so the fraction
	// sitting in the clipped region dominates the result. A high exponent is only
	// meaningful once the envelope is known to be tight.
	//
	// This field is worse for it than the galaxy's: its features are the SURFACES between
	// nodes, which sit at no particular place relative to the generation grid, so a thin
	// wall crossing a cell can fall between all fifty-six jittered probes.
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

	// FOUR TIMES CAPACITY IS THE POINT AT WHICH THE SLOT STOPS DESCRIBING THE FIELD. Below
	// that a full slot is a genuinely dense region; far above it, every dense region is
	// equally full and the population no longer varies with density at all.
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
	// EnvelopePad exists to absorb. A large fraction means acceptance is saturating across
	// the batch and the placement gradient is being flattened cell by cell, which is a
	// SAMPLING problem rather than a tuning one: raise EnvelopePad, or lower the spawn
	// exponent until the envelope is trustworthy. Raising the candidate budget does not
	// help, because every extra candidate is drawn against the same short envelope.
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

	// A NON-FINITE RECORD IS NOT A TUNING PROBLEM. It means the scatter read past what the
	// dispatch wrote, which is a plumbing fault rather than a field one, and it is worth
	// saying loudly because the crash it causes surfaces inside the octree.
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
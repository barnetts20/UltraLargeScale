#include "UniverseDataGenerator.h"

#pragma region Noise Composition

void UniverseDataGenerator::Initialize()
{
	DensityNoise = BuildNoise();
}

FastNoise::SmartNode<> UniverseDataGenerator::BuildNoise() const
{
	//auto Voronoi = FastNoise::New<FastNoise::CellularDistance>();
	//Voronoi->SetDistanceFunction(FastNoise::DistanceFunction::EuclideanSquared);
	//Voronoi->SetReturnType(FastNoise::CellularDistance::ReturnType::Index0);
	//auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	//SeedOffset->SetSource(Voronoi);
	//SeedOffset->SetOffset(Params.Seed);
	//auto DomainScale = FastNoise::New<FastNoise::DomainScale>();
	//DomainScale->SetSource(SeedOffset);
	//DomainScale->SetScale(Params.NoiseGraphParams.MasterScale);
	//auto Fbm0 = FastNoise::New<FastNoise::FractalFBm>();
	//Fbm0->SetSource(DomainScale);
	//Fbm0->SetOctaveCount(3);
	//auto Remap0 = FastNoise::New<FastNoise::Remap>();
	//Remap0->SetSource(Fbm0);
	//Remap0->SetRemap(0, 1, Params.NoiseGraphParams.ClusterRemapMax, Params.NoiseGraphParams.ClusterRemapMin);
	//auto Pow0 = FastNoise::New<FastNoise::PowInt>();
	//Pow0->SetValue(Remap0);
	//Pow0->SetPow(Params.NoiseGraphParams.ClusterFalloff);
	//auto Scale0 = FastNoise::New<FastNoise::DomainScale>();
	//Scale0->SetSource(Pow0);
	//Scale0->SetScale(Params.NoiseGraphParams.ClusterScale);
	//auto Pow1 = FastNoise::New<FastNoise::PowInt>();
	//Pow1->SetValue(Fbm0);
	//Pow1->SetPow(Params.NoiseGraphParams.WebFalloff);
	//auto Mul0 = FastNoise::New<FastNoise::Multiply>();
	//Mul0->SetLHS(Scale0);
	//Mul0->SetRHS(Pow1);
	//auto Mul1 = FastNoise::New<FastNoise::Multiply>();
	//Mul1->SetLHS(Mul0);
	//Mul1->SetRHS(Params.NoiseGraphParams.ClusterMulti);
	//auto Remap1 = FastNoise::New<FastNoise::Remap>();
	//Remap1->SetSource(Pow1);
	//Remap1->SetRemap(0, 1, Params.NoiseGraphParams.WebRemapMin, Params.NoiseGraphParams.WebRemapMax);
	//auto Add0 = FastNoise::New<FastNoise::Add>();
	//Add0->SetLHS(Remap1);
	//Add0->SetRHS(Mul1);
	//auto Warp0 = FastNoise::New<FastNoise::DomainWarpGradient>();
	//Warp0->SetSource(Add0);
	//Warp0->SetWarpAmplitude(Params.NoiseGraphParams.WarpAmp);
	//Warp0->SetWarpFrequency(Params.NoiseGraphParams.WarpFreq);
	//return Warp0;
	auto n = FastNoise::NewFromEncodedNodeTree("GQAbABsAEwAAAIBAIAAXAM3MTD4AAAA/AAAAAAAAgD8NAAYAAAAAAABABwAAAAAAPwAAAAAAAAAAAAAAzczMPQAAAMhCASQABgAAABIABgAAAAAAAEAQAAAAgD8eAAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAAAAAACAPwAAAAA/AAAAAAABGwAkAAIAAAD//wkAAAAAAD8=");
	return n;
}

TArray<uint8> UniverseDataGenerator::SampleNoiseVolume(int InNoiseResolution, const FIntVector& InCellCoord) const {
	const FVector NoiseOffset(
		static_cast<double>(InCellCoord.X) * 2.0,
		static_cast<double>(InCellCoord.Y) * 2.0,
		static_cast<double>(InCellCoord.Z) * 2.0);

	return FVolumeTextureUtils::SampleNoiseToVolume(
		DensityNoise,
		Params.Seed,
		InNoiseResolution,
		Params.Extent,
		nullptr,
		-1,
		1.0f,
		3,
		NoiseOffset
	);
}

#pragma endregion

#pragma region Tier Generation Callbacks

void UniverseDataGenerator::GenerateLargeTierNode(const FIntVector& InCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InClusterBuffer, const FVector& InNodeCenter, int32& OutSlotCount) const {
	// Batched noise sampling, three phases:
	//   1. Generate candidate positions + normalized noise coords.
	//   2. One GenPositionArray3D call covering all candidates.
	//   3. Walk results, rejection-gate, write accepted to slot buffers.
	//
	// ONE BUFFER NOW. This used to write a second, gas entry at the same slot index --
	// same position, a much larger extent lerped by the same density -- for a nebula
	// sprite layer. The universe raymarch supersedes it: the density field is both more
	// accurate and better looking, and the sprites cost a second full set of per-frame
	// and per-transition buffer writes plus a screenful of very large translucent quads.

	const int32 BufferStart = InSlotIndex * InClusterBuffer.SlotCapacity;

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = InClusterBuffer.SlotCapacity;
	const double InvExtent = 1.0 / (double)Params.Extent;

	// Every candidate in this cell shares the same coord-derived noise offset.
	const double NoiseOffsetX = (double)InCoord.X * 2.0;
	const double NoiseOffsetY = (double)InCoord.Y * 2.0;
	const double NoiseOffsetZ = (double)InCoord.Z * 2.0;

	// Phase 1: generate candidates + normalized noise coords
	TArray<FVector> CandidatePositions;
	TArray<float> NoiseX, NoiseY, NoiseZ;
	CandidatePositions.SetNumUninitialized(NumCandidates);
	NoiseX.SetNumUninitialized(NumCandidates);
	NoiseY.SetNumUninitialized(NumCandidates);
	NoiseZ.SetNumUninitialized(NumCandidates);

	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const FVector Candidate(
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent),
			Stream.FRandRange(-(double)Params.Extent, (double)Params.Extent));
		CandidatePositions[i] = Candidate;

		NoiseX[i] = (float)(Candidate.X * InvExtent + NoiseOffsetX);
		NoiseY[i] = (float)(Candidate.Y * InvExtent + NoiseOffsetY);
		NoiseZ[i] = (float)(Candidate.Z * InvExtent + NoiseOffsetZ);
	}

	// Phase 2: batch noise evaluation
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	DensityNoise->GenPositionArray3D(
		NoiseOut.GetData(),
		NumCandidates,
		NoiseX.GetData(),
		NoiseY.GetData(),
		NoiseZ.GetData(),
		0.0f, 0.0f, 0.0f,
		Params.Seed);

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// Phase 3: accept/reject + write to slot
	int32 ActualCount = 0;
	auto dCurve = Params.LargeTier.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		// Density response curve gates spawning only; raw density is used for
		// particle sizing below and never feeds the pseudovolume texture.
		const float SpawnDensity = (dCurve && dCurve->GetNumKeys() > 0)
			? FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f)
			: RawDensity;
		if (Stream.FRand() > SpawnDensity) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.LargeTier.MinScale,
			Params.LargeTier.MaxScale,
			ScaleSample, Params.LargeTier.ScaleDistribution);

		// AUTHORED SIZE IS TRUTH: convert the real-unit Scale through the
		// layer's constant UnitScale exactly once. Octree insert depth is
		// derived later, at insert time, by InsertParticleIntoOctree, so no
		// FPointData is needed here. Never reconstruct size from the quantized
		// depth: that would inflate every particle to its power-of-two node
		// extent and make sprite sizes octave-step with UnitScale.
		const float ClusterExtent = static_cast<float>(Scale / Params.UnitScale);

		const FVector CompVec = Stream.GetUnitVector();
		const FVector NodeRotation = Stream.GetUnitVector();
		const FVector LocalPos = CandidatePositions[i] + InNodeCenter;

		const int32 Idx = BufferStart + ActualCount;
		InClusterBuffer.Positions[Idx] = LocalPos;
		InClusterBuffer.Rotations[Idx] = NodeRotation;
		InClusterBuffer.Extents[Idx] = ClusterExtent;
		InClusterBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	InClusterBuffer.PadSlotDead(InSlotIndex, ActualCount);

	OutSlotCount = ActualCount;
}

void UniverseDataGenerator::GenerateMidTierNode(
	const FIntVector& InCoord,
	int32 InSlotIndex,
	FNiagaraParticleBuffer& InBuffer,
	const FVector& InNodeCenter,
	double InCellExtent,
	int32& OutSlotCount) const
{
	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;

	const int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z));
	const int32 NodeSeed = HashCombine(Params.Seed + 7, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = InBuffer.SlotCapacity;
	const double InvExtent = 1.0 / (double)Params.Extent;

	const double TwoExtent = 2.0 * (double)Params.Extent;
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

		const int32 CX = FMath::FloorToInt32(Candidate.X / TwoExtent + 0.5);
		const int32 CY = FMath::FloorToInt32(Candidate.Y / TwoExtent + 0.5);
		const int32 CZ = FMath::FloorToInt32(Candidate.Z / TwoExtent + 0.5);
		const double CenterX = (double)CX * TwoExtent;
		const double CenterY = (double)CY * TwoExtent;
		const double CenterZ = (double)CZ * TwoExtent;
		NoiseX[i] = (float)((Candidate.X - CenterX) * InvExtent + (double)CX * 2.0);
		NoiseY[i] = (float)((Candidate.Y - CenterY) * InvExtent + (double)CY * 2.0);
		NoiseZ[i] = (float)((Candidate.Z - CenterZ) * InvExtent + (double)CZ * 2.0);
	}

	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	DensityNoise->GenPositionArray3D(
		NoiseOut.GetData(), NumCandidates,
		NoiseX.GetData(), NoiseY.GetData(), NoiseZ.GetData(),
		0.0f, 0.0f, 0.0f, Params.Seed);

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	int32 ActualCount = 0;
	auto dCurve = Params.MidTier.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		// Density response curve gates spawning only; raw density is used for
		// particle sizing below and never feeds the pseudovolume texture.
		const float SpawnDensity = (dCurve && dCurve->GetNumKeys() > 0)
			? FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f)
			: RawDensity;
		if (Stream.FRand() > SpawnDensity) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.MidTier.MinScale, Params.MidTier.MaxScale,
			ScaleSample, Params.MidTier.ScaleDistribution);
		// Authored size is truth (see the Large-tier note above). No FPointData
		// here; insert depth is derived at octree-insert time.
		const float ClusterExtent = static_cast<float>(Scale / Params.UnitScale);

		const FVector CompVec = Stream.GetUnitVector();
		const FVector NodeRotation = Stream.GetUnitVector();

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Rotations[Idx] = NodeRotation;
		InBuffer.Extents[Idx] = ClusterExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));
		ActualCount++;
	}

	InBuffer.PadSlotDead(InSlotIndex, ActualCount);
	OutSlotCount = ActualCount;
}

void UniverseDataGenerator::GenerateSmallTierNode(
	const FIntVector& InCoord,
	int32 InSlotIndex,
	FNiagaraParticleBuffer& InBuffer,
	const FVector& InNodeCenter,
	double InCellExtent,
	int32& OutSlotCount) const
{
	// Batched noise sampling, three phases matching GenerateLargeTierNode.
	// Candidates are scan-node-local rather than cell-local; each candidate
	// may straddle coarse cell boundaries so noise offset is computed
	// per-candidate rather than shared across the node.

	const int32 BufferStart = InSlotIndex * InBuffer.SlotCapacity;

	int32 CoordHash = HashCombine(
		HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
		GetTypeHash(InCoord.Z)
	);
	int32 NodeSeed = HashCombine(Params.Seed + 23, CoordHash);
	FRandomStream Stream(NodeSeed);

	const int32 NumCandidates = InBuffer.SlotCapacity;
	const double InvExtent = 1.0 / (double)Params.Extent;
	const double TwoExtent = 2.0 * (double)Params.Extent;

	// Phase 1: generate candidates + normalized noise coords
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
			Stream.FRandRange(-InCellExtent, InCellExtent)
		);
		Candidate += InNodeCenter;
		CandidatePositions[i] = Candidate;

		// Determine which coarse cell this candidate falls into and compute
		// its cell-local normalized coord + coord-derived offset.
		const int32 CX = FMath::FloorToInt32(Candidate.X / TwoExtent + 0.5);
		const int32 CY = FMath::FloorToInt32(Candidate.Y / TwoExtent + 0.5);
		const int32 CZ = FMath::FloorToInt32(Candidate.Z / TwoExtent + 0.5);

		const double CenterX = (double)CX * TwoExtent;
		const double CenterY = (double)CY * TwoExtent;
		const double CenterZ = (double)CZ * TwoExtent;

		NoiseX[i] = (float)((Candidate.X - CenterX) * InvExtent + (double)CX * 2.0);
		NoiseY[i] = (float)((Candidate.Y - CenterY) * InvExtent + (double)CY * 2.0);
		NoiseZ[i] = (float)((Candidate.Z - CenterZ) * InvExtent + (double)CZ * 2.0);
	}

	// Phase 2: batch noise evaluation
	TArray<float> NoiseOut;
	NoiseOut.SetNumUninitialized(NumCandidates);
	DensityNoise->GenPositionArray3D(
		NoiseOut.GetData(),
		NumCandidates,
		NoiseX.GetData(),
		NoiseY.GetData(),
		NoiseZ.GetData(),
		0.0f, 0.0f, 0.0f,
		Params.Seed
	);

	NoiseX.Empty();
	NoiseY.Empty();
	NoiseZ.Empty();

	// Phase 3: accept/reject + write to slot
	int32 ActualCount = 0;
	auto dCurve = Params.SmallTier.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		// Density response curve gates spawning only; raw density is used for
		// particle sizing below and never feeds the pseudovolume texture.
		const float SpawnDensity = (dCurve && dCurve->GetNumKeys() > 0)
			? FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f)
			: RawDensity;
		if (Stream.FRand() > SpawnDensity) continue;

		FVector CompVec = Stream.GetUnitVector();

		// DRAWN HERE, matching the large and mid generators, so the stream advances
		// identically across all three. Small-tier particles were the only ones with no
		// rotation: the buffer allocates the array (bWantRotations is set on this tier)
		// and it stayed zeroed, so the sprites had no orientation and every galaxy born
		// from a small-tier particle inherited nothing and came out disc-up.
		const FVector NodeRotation = Stream.GetUnitVector();

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.SmallTier.MinScale,
			Params.SmallTier.MaxScale,
			ScaleSample, Params.SmallTier.ScaleDistribution);

		// Authored size is truth (see the Large-tier note above). No FPointData
		// here; insert depth is derived at octree-insert time.
		const float FinalExtent = static_cast<float>(Scale / Params.UnitScale);

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Extents[Idx] = FinalExtent;
		InBuffer.Rotations[Idx] = NodeRotation;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		ActualCount++;
	}

	InBuffer.PadSlotDead(InSlotIndex, ActualCount);

	OutSlotCount = ActualCount;
}
#pragma endregion
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

void UniverseDataGenerator::BuildCalibrationGrid(
	const FTierParams& InTierParams,
	int32 InGridDepth,
	double InCellHalfExtent,
	TArray<FTierBatchCell>& OutCells) const
{
	OutCells.Reset();

	// THE SAME SHAPE AS A STREAMED NEIGHBOURHOOD, so a calibration cell and a generation
	// cell are the same size and a mass measured here means what it means there. A cell's
	// mass is the mean over its own volume, so measuring a different volume answers a
	// different question -- and the error is BIAS in one direction, growing with the size
	// mismatch, not noise that averages out.
	const int32 Radius = FMath::Max(InTierParams.NeighborhoodRadius, 0);
	const int32 Side = 2 * Radius + 1;

	const double FullCell = InCellHalfExtent * 2.0;

	// ANCHORED AT THE ORIGIN, not at the player. The result is cached for the session, so
	// it has to be a property of the field rather than of where someone happened to be
	// standing the first time a tier streamed -- otherwise two runs of the same seed
	// calibrate differently and the population density changes between them.
	const double Origin = -(static_cast<double>(Side) - 1.0) * 0.5;

	TArray<FTierBatchCell> Parents;
	Parents.Reserve(Side * Side * Side);

	for (int32 iz = 0; iz < Side; ++iz)
	{
		for (int32 iy = 0; iy < Side; ++iy)
		{
			for (int32 ix = 0; ix < Side; ++ix)
			{
				FTierBatchCell Cell;

				Cell.Coord = FIntVector(ix - Radius, iy - Radius, iz - Radius);
				Cell.Centre = FVector(
					(Origin + static_cast<double>(ix)) * FullCell,
					(Origin + static_cast<double>(iy)) * FullCell,
					(Origin + static_cast<double>(iz)) * FullCell);
				Cell.HalfExtent = InCellHalfExtent;
				Cell.SlotIndex = 0;

				Parents.Add(Cell);
			}
		}
	}

	// SUBDIVIDED EXACTLY AS GENERATION WILL SUBDIVIDE, for the same reason the block is
	// sized like a neighbourhood: the mass is an estimate of a mean over a volume, and
	// probing an undivided parent reports a mean that is too low by a factor growing with
	// the subdivision depth. That comes back as a constant that is too high and a tier
	// that over-delivers.
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

	if (NoiseTexture == nullptr || FieldExtent <= 0.0)
	{
		return 0.0f;
	}

	TArray<FTierBatchCell> AllCells;
	BuildCalibrationGrid(InTierParams, InGridDepth, InCellHalfExtent, AllCells);

	if (AllCells.Num() == 0)
	{
		return 0.0f;
	}

	TArray<FUniverseGenCell> Cells;
	Cells.Reserve(AllCells.Num());

	for (const FTierBatchCell& In : AllCells)
	{
		FUniverseGenCell Cell;
		Cell.Centre = FVector3f(
			static_cast<float>(In.Centre.X),
			static_cast<float>(In.Centre.Y),
			static_cast<float>(In.Centre.Z));
		Cell.HalfExtent = static_cast<float>(In.HalfExtent);
		Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);
		Cells.Add(Cell);
	}

	const float InvFieldExtent =
		static_cast<float>(1.0 / FMath::Max(FieldExtent, 1e-9));

	TArray<float> CellMass;

	const double CalibrationStart = FPlatformTime::Seconds();

	float Scale = 0.0f;

	if (UniverseEntityGen::CalibrateBlocking(
		Params, InTierParams, Cells, TierKeySeed(InSeedOffset),
		InvFieldExtent, NoiseTexture, CellMass)
		&& CellMass.Num() == AllCells.Num())
	{
		// REDUCED HERE, NOT ON THE GPU, and in double.
		//
		// A slot receives the sum of what its cells produce, and for this layer every
		// streamed cell owns its own slot -- there is no tier that funnels a whole grid
		// into one. So the divisor is the LARGEST PER-PARENT SUM: the densest streamed
		// cell fills its slot exactly and every other is proportionally less.
		//
		// A global maximum over children would be wrong by the child count, and a mean
		// would let the densest cell overflow while the rest ran empty.
		TMap<int32, double> ParentSums;

		double TotalMass = 0.0;
		double MaxCellMass = 0.0;

		for (int32 i = 0; i < AllCells.Num(); ++i)
		{
			const double M = static_cast<double>(CellMass[i]);

			TotalMass += M;
			MaxCellMass = FMath::Max(MaxCellMass, M);

			double& Sum = ParentSums.FindOrAdd(AllCells[i].ParentIndex);
			Sum += M;
		}

		double Divisor = 0.0;
		for (const TPair<int32, double>& Pair : ParentSums)
		{
			Divisor = FMath::Max(Divisor, Pair.Value);
		}

		if (Divisor > 0.0)
		{
			Scale = static_cast<float>(
				static_cast<double>(InTierParams.SlotCapacity) / Divisor);
		}

		// THE HOMOGENEITY CHECK, and the reason it is logged rather than asserted.
		//
		// This layer calibrates against a SAMPLE of an unbounded field rather than an
		// exhaustive grid, which is only sound while the field is statistically the same
		// everywhere. The spread across parents is the direct measure of that: a ratio near
		// 1 means one block is as good as any other and the constant generalises, while a
		// large one means the block landed somewhere unrepresentative -- most likely inside
		// or outside the lattice crossover band, which runs about five times the mean
		// density of either end.
		//
		// If this reads much above 5, widen the calibration block or reconsider whether a
		// single constant can describe the whole field.
		const double MeanParent = (ParentSums.Num() > 0)
			? (TotalMass / static_cast<double>(ParentSums.Num()))
			: 0.0;

		const double Spread = (MeanParent > 0.0) ? (Divisor / MeanParent) : 0.0;

		UE_LOG(LogTemp, Log,
			TEXT("UniverseEntityGen: tier offset %d calibrated in %.3fs over %d cells ")
			TEXT("(%d parents); BudgetScale %.4f, max parent mass %.5f, mean %.5f, ")
			TEXT("spread %.2fx, largest single cell %.5f."),
			InSeedOffset, FPlatformTime::Seconds() - CalibrationStart,
			AllCells.Num(), ParentSums.Num(),
			Scale, Divisor, MeanParent, Spread, MaxCellMass);
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
	if (NoiseTexture == nullptr)
	{
		ensureMsgf(false,
			TEXT("UniverseEntityGen: NoiseTexture is unresolved, so the sector will place ")
			TEXT("NOTHING -- placement is GPU-only and the dispatch samples it. It is ")
			TEXT("loaded from MaterialParams.VolumeNoise; check that path and that the ")
			TEXT("asset has NEVER STREAM set."));
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
	Cells.Reserve(Subdivided.Num());

	for (const FTierBatchCell& In : Subdivided)
	{
		FUniverseGenCell Cell;
		Cell.Centre = FVector3f(
			static_cast<float>(In.Centre.X),
			static_cast<float>(In.Centre.Y),
			static_cast<float>(In.Centre.Z));
		Cell.HalfExtent = static_cast<float>(In.HalfExtent);
		Cell.Coord = FIntVector3(In.Coord.X, In.Coord.Y, In.Coord.Z);
		Cells.Add(Cell);
	}

	// ONE SHARED BUFFER, sized on what the SLOTS can hold rather than on cells times a
	// worst case. Every queued cell owns one slot, so this is exactly what the batch can
	// possibly keep.
	const int32 EntityCapacity = InQueuedCells.Num() * InBuffer.SlotCapacity;

	const float InvFieldExtent =
		static_cast<float>(1.0 / FMath::Max(FieldExtent, 1e-9));

	TArray<FUniverseEntityOut> Entities;
	TArray<uint32> Counts;

	if (!UniverseEntityGen::GenerateBatchBlocking(
		Params, InTierParams, Cells, EntityCapacity,
		TierKeySeed(InSeedOffset), InvFieldExtent, NoiseTexture,
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

	// THE TRUE TOTAL, which deliberately over-counts past capacity: the shader increments
	// the global cursor even when the buffer is full, so this is what the batch WANTED to
	// place rather than what fitted. Clamping it in the shader would make an overflowing
	// dispatch report exactly full and hide the thinning.
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

		const FVector Pos(E.Pos.X, E.Pos.Y, E.Pos.Z);

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

		// The decoratives are three decorrelated uniforms; the colour convention matches
		// what the CPU path wrote, so the Niagara systems need no change.
		const FVector3f Decor = E.DecodeDecor();
		InBuffer.Colors[Idx] = FLinearColor(Decor.X, Decor.Y, Decor.Z, 1.0f);

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

	// THE DIAGNOSTIC THAT DECIDES SUBDIVISION. Probes and candidates are both field
	// evaluations, and a universe field evaluation is a fifty-four candidate walk plus five
	// texture fetches -- so the ratio between them is the whole cost argument. Below about
	// 1 the probes have overtaken placement and the tier wants one subdivision level fewer;
	// above about 9 it wants one more.
	uint32 CandidatesEvaluated = 0;
	for (int32 c = 0; c < Cells.Num(); ++c)
	{
		const int32 Base = c * UniverseEntityGen::CountersPerCell;
		if (Counts.IsValidIndex(Base + 1))
		{
			CandidatesEvaluated += Counts[Base + 1];
		}
	}

	const double Probes =
		static_cast<double>(Cells.Num()) * FUniverseEntityGenCS::ProbesPerCell;

	UE_LOG(LogTemp, Verbose,
		TEXT("UniverseEntityGen: tier offset %d placed %d/%u accepted into %d slots over ")
		TEXT("%d cells; C/P %.2f, dropped %d over capacity, rejected %d non-finite."),
		InSeedOffset, Written, TotalAccepted, InQueuedCells.Num(), Cells.Num(),
		(Probes > 0.0) ? (static_cast<double>(CandidatesEvaluated) / Probes) : 0.0,
		Dropped, Rejected);

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
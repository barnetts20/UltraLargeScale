#include "UniverseDataGenerator.h"

#pragma region Noise Composition

void UniverseDataGenerator::Initialize()
{
	DensityNoise = BuildNoise();
}

FastNoise::SmartNode<> UniverseDataGenerator::BuildNoise() const
{
	auto Voronoi = FastNoise::New<FastNoise::CellularDistance>();
	Voronoi->SetDistanceFunction(FastNoise::DistanceFunction::EuclideanSquared);
	Voronoi->SetReturnType(FastNoise::CellularDistance::ReturnType::Index0);
	auto SeedOffset = FastNoise::New<FastNoise::SeedOffset>();
	SeedOffset->SetSource(Voronoi);
	SeedOffset->SetOffset(Params.Seed);
	auto DomainScale = FastNoise::New<FastNoise::DomainScale>();
	DomainScale->SetSource(SeedOffset);
	DomainScale->SetScale(Params.NoiseParams.MasterScale);
	auto Fbm0 = FastNoise::New<FastNoise::FractalFBm>();
	Fbm0->SetSource(DomainScale);
	Fbm0->SetOctaveCount(3);
	auto Remap0 = FastNoise::New<FastNoise::Remap>();
	Remap0->SetSource(Fbm0);
	Remap0->SetRemap(0, 1, Params.NoiseParams.ClusterRemapMax, Params.NoiseParams.ClusterRemapMin);
	auto Pow0 = FastNoise::New<FastNoise::PowInt>();
	Pow0->SetValue(Remap0);
	Pow0->SetPow(Params.NoiseParams.ClusterFalloff);
	auto Scale0 = FastNoise::New<FastNoise::DomainScale>();
	Scale0->SetSource(Pow0);
	Scale0->SetScale(Params.NoiseParams.ClusterScale);
	auto Pow1 = FastNoise::New<FastNoise::PowInt>();
	Pow1->SetValue(Fbm0);
	Pow1->SetPow(Params.NoiseParams.WebFalloff);
	auto Mul0 = FastNoise::New<FastNoise::Multiply>();
	Mul0->SetLHS(Scale0);
	Mul0->SetRHS(Pow1);
	auto Mul1 = FastNoise::New<FastNoise::Multiply>();
	Mul1->SetLHS(Mul0);
	Mul1->SetRHS(Params.NoiseParams.ClusterMulti);
	auto Remap1 = FastNoise::New<FastNoise::Remap>();
	Remap1->SetSource(Pow1);
	Remap1->SetRemap(0, 1, Params.NoiseParams.WebRemapMin, Params.NoiseParams.WebRemapMax);
	auto Add0 = FastNoise::New<FastNoise::Add>();
	Add0->SetLHS(Remap1);
	Add0->SetRHS(Mul1);
	auto Warp0 = FastNoise::New<FastNoise::DomainWarpGradient>();
	Warp0->SetSource(Add0);
	Warp0->SetWarpAmplitude(Params.NoiseParams.WarpAmp);
	Warp0->SetWarpFrequency(Params.NoiseParams.WarpFreq);
	return Warp0;
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

void UniverseDataGenerator::GenerateLargeTierNode(const FIntVector& InCoord, int32 InSlotIndex, FNiagaraParticleBuffer& InClusterBuffer, FNiagaraParticleBuffer& InGasBuffer, const FVector& InNodeCenter, int32& OutSlotCount) const {
	// Batched noise sampling — three phases:
	//   1. Generate candidate positions + normalized noise coords.
	//   2. One GenPositionArray3D call covering all candidates.
	//   3. Walk results, rejection-gate, write accepted to slot buffers.
	//
	// Cluster and gas share 1:1 positions and slot indexing; accepted points
	// write a cluster entry and a gas entry at the same slot index.
	// Gas extent is lerped by the same density value used for rejection.

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

	// --- Phase 1: generate candidates + normalized noise coords ---
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

	// --- Phase 2: batch noise evaluation ---
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

	// --- Phase 3: accept/reject + write to slot ---
	int32 ActualCount = 0;
	auto dCurve = Params.LargeTier.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		const float Density = FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.LargeTier.MinScale,
			Params.LargeTier.MaxScale,
			ScaleSample, Params.LargeTier.ScaleDistribution);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		const float ClusterExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		const FVector CompVec = Stream.GetUnitVector();
		const FVector NodeRotation = Stream.GetUnitVector();
		const float GasMultiplier = FMath::Lerp(Params.GasExtentMinMultiplier, Params.GasExtentMaxMultiplier, Density);
		const float GasExtent = ClusterExtent * GasMultiplier;
		const FVector LocalPos = CandidatePositions[i] + InNodeCenter;

		const int32 Idx = BufferStart + ActualCount;
		InClusterBuffer.Positions[Idx] = LocalPos;
		InClusterBuffer.Rotations[Idx] = NodeRotation;
		InClusterBuffer.Extents[Idx] = ClusterExtent;
		InClusterBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));

		InGasBuffer.Positions[Idx] = LocalPos;
		InGasBuffer.Extents[Idx] = GasExtent;
		InGasBuffer.Colors[Idx] = FLinearColor(1.0f, 1.0f, 1.0f, 1.0);

		ActualCount++;
	}

	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	InClusterBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);
	InGasBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);

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
		const float Density = FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.MidTier.MinScale, Params.MidTier.MaxScale,
			ScaleSample, Params.MidTier.ScaleDistribution);
		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		const float ClusterExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

		const FVector CompVec = Stream.GetUnitVector();
		const FVector NodeRotation = Stream.GetUnitVector();

		const int32 Idx = BufferStart + ActualCount;
		InBuffer.Positions[Idx] = CandidatePositions[i];
		InBuffer.Rotations[Idx] = NodeRotation;
		InBuffer.Extents[Idx] = ClusterExtent;
		InBuffer.Colors[Idx] = FLinearColor(FMath::Abs(CompVec.X), FMath::Abs(CompVec.Y), FMath::Abs(CompVec.Z));
		ActualCount++;
	}

	const FVector DeadPos(Params.Extent * 10.0, Params.Extent * 10.0, Params.Extent * 10.0);
	InBuffer.PadSlotDead(InSlotIndex, ActualCount, DeadPos);
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
	// Batched noise sampling — three phases matching GenerateLargeTierNode.
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

	// --- Phase 2: batch noise evaluation ---
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

	// --- Phase 3: accept/reject + write to slot ---
	int32 ActualCount = 0;
	auto dCurve = Params.SmallTier.DensityResponse.GetRichCurveConst();
	for (int32 i = 0; i < NumCandidates; ++i)
	{
		const float RawDensity = FMath::Clamp(NoiseOut[i], 0.0f, 1.0f);
		const float Density = FMath::Clamp(dCurve->Eval(RawDensity), 0.0f, 1.0f);
		if (Stream.FRand() > Density) continue;

		FVector CompVec = Stream.GetUnitVector();

		const float ScaleSample = Stream.FRand();
		const double Scale = FPointData::SampleScaleFromDistribution(
			Params.SmallTier.MinScale,
			Params.SmallTier.MaxScale,
			ScaleSample, Params.SmallTier.ScaleDistribution);

		FPointData PointData = FPointData::MakePointDataFromWorldScale(Scale, Params.UnitScale, Params.Extent);
		const double ExtentAtDepth = (double)Params.Extent / (double)(1 << PointData.InsertDepth);
		const float FinalExtent = static_cast<float>(ExtentAtDepth * (1.0 + PointData.Data.ScaleFactor));

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
#pragma endregion
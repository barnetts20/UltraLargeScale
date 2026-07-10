#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#pragma endregion

struct SVO_API FVoxelData {
public:
	FVoxelData() : ScaleFactor(0.0), Density(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1), ParticleIndex(-1), ParticlePosition(FVector::ZeroVector), ParticleExtent(0.0f) {};
	FVoxelData(float InDensity, float InGasDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : ScaleFactor(InDensity), Density(InGasDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId), ParticleIndex(-1), ParticlePosition(FVector::ZeroVector), ParticleExtent(0.0f) {};

	float ScaleFactor;
	float Density;
	FVector Composition;

	/** Deterministic hierarchical seed for this node. */
	//TODO: If this is truly the hashed seed and not a human discernable index may be worth changing to be named Seed
	int ObjectId;

	//TypeId allows insertion of different classes of objects in the same tree. Type mapping framework must be externally managed.
	int TypeId;

	/** Absolute index into the tier's flat particle buffer.  */
	int ParticleIndex;

	/** EXACT particle position (actor-local virtual space) captured at insert time*/
	FVector ParticlePosition;

	/** EXACT particle extent captured at insert time (actor-local units). */
	float ParticleExtent;

	// Collision overflow for nodes that receive multiple inserts at the same
	// quantized depth/position. 
	// TODO: NOT SURE THIS IS STILL RELEVANT
	TArray<int32> AdditionalObjectIds;

	/**
	 * Composes a deterministic, always-positive seed from a parent seed, a grid coordinate, and a particle index within that cell.
	 */
	static int32 ComposeSeed(int32 InParentSeed, const FIntVector& InCoord, int32 InParticleIndex)
	{
		const uint32 CoordHash = HashCombine(HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)), GetTypeHash(InCoord.Z));
		const uint32 Raw = HashCombine(static_cast<uint32>(InParentSeed), HashCombine(CoordHash, static_cast<uint32>(InParticleIndex)));
		return static_cast<int32>(Raw & 0x7FFFFFFF);
	}
};

struct SVO_API FPointData
{
private:
	FVector PositionInternal;
	FInt64Vector Int64PositionInternal;

public:
	int InsertDepth;
	FVoxelData Data;

	const FVector& GetPosition() const { return PositionInternal; }
	const FInt64Vector& GetInt64Position() const { return Int64PositionInternal; }

	void SetPosition(const FVector& InPosition)
	{
		PositionInternal = InPosition;
		Int64PositionInternal = FInt64Vector(FMath::RoundToInt64(InPosition.X), FMath::RoundToInt64(InPosition.Y), FMath::RoundToInt64(InPosition.Z));
	}

	void SetInt64Position(const FInt64Vector& InInt64)
	{
		Int64PositionInternal = InInt64;
		PositionInternal = FVector(static_cast<double>(InInt64.X), static_cast<double>(InInt64.Y), static_cast<double>(InInt64.Z));
	}

	//TODO: Should probably just do the scale calculation and manage the data struct creation a level up
	static FPointData MakePointDataFromWorldScale(const double InScaleWorldUnits, const double InUnitScale, const int64 InExtent)
	{
		double LocalSize = InScaleWorldUnits / InUnitScale;
		int MinDepth = 1, BestDepth = 1, MaxDepth = static_cast<int>(FMath::Log2(static_cast<double>(InExtent)));
		int64 BestNodeExtent = InExtent >> MinDepth;
		double BestRatio = FMath::Abs(1.0 - LocalSize / static_cast<double>(BestNodeExtent));
		for (int d = MinDepth; d <= MaxDepth; d++)
		{
			int64 ExtentAtDepth = InExtent >> d;
			if (LocalSize > ExtentAtDepth) break;
			double Ratio = FMath::Abs(1.0 - LocalSize / static_cast<double>(ExtentAtDepth));
			if (Ratio < BestRatio)
			{
				BestRatio = Ratio;
				BestDepth = d;
				BestNodeExtent = ExtentAtDepth;
			}
		}
		float ScaleFactor = FMath::Clamp(static_cast<float>((LocalSize / static_cast<double>(BestNodeExtent)) - 1.0), 0.0001f, 1.0f);
		FVoxelData Data;
		Data.ScaleFactor = ScaleFactor;
		return FPointData(FInt64Vector::ZeroValue, BestDepth, Data);
	}

	static double SampleScaleFromDistribution(double InMinScale, double InMaxScale, double InSample, const FRuntimeFloatCurve& InDistributionCurve) {
		const auto* Curve = InDistributionCurve.GetRichCurveConst();
		const double T = (Curve && Curve->GetNumKeys() > 0) ? static_cast<double>(FMath::Clamp(Curve->Eval(FMath::Clamp(static_cast<float>(InSample), 0.0f, 1.0f)), 0.0f, 1.0f)) : InSample;
		return FMath::Lerp(InMinScale, InMaxScale, T);
	}

	FPointData() : InsertDepth(0), Data() {}

	FPointData(const FVector& InPosition, int InDepth, const FVoxelData& InData) : InsertDepth(InDepth), Data(InData)
	{
		SetPosition(InPosition);
	}

	//TODO: Check if we are even using int64 or if we have swapped to double octree bounds... if its double we could strip the whole 64 bit stack
	FPointData(const FInt64Vector& InInt64, int InDepth, const FVoxelData& InData) : InsertDepth(InDepth), Data(InData)
	{
		SetInt64Position(InInt64);
	}
};

struct SVO_API FCachedCellData
{
	TArray<TArray<FVector>>        PerBufferPositions;
	TArray<TArray<float>>          PerBufferExtents;
	TArray<TArray<FLinearColor>>   PerBufferColors;
	TArray<TArray<FVector>>        PerBufferRotations;
	int32 ParticleCount = 0;
};

UENUM()
enum class ELifecycleState : uint8
{
	Uninitialized,
	Initializing,
	Ready,
	Pooling,
	Destroying
};
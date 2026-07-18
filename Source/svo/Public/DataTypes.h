#pragma once

#include "CoreMinimal.h"

/** Payload carried by every FOctreeNode: the procgen attributes of one
 *  inserted entity, its deterministic seed, and the exact particle it maps to.
 *  ScaleFactor/Density/Composition drive sizing, density accumulation, and
 *  color; Seed/TypeId identify and re-derive the entity; the Particle* fields
 *  pin the exact particle captured at insert. */
struct SVO_API FVoxelData {
public:
	FVoxelData() : ScaleFactor(0.0), Density(0.0), Composition(0, 0, 0), Seed(-1), TypeId(-1), ParticleIndex(-1), ParticlePosition(FVector::ZeroVector), ParticleExtent(0.0f) {};
	FVoxelData(float InDensity, float InGasDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : ScaleFactor(InDensity), Density(InGasDensity), Composition(InComposition), Seed(InObjectId), TypeId(InTypeId), ParticleIndex(-1), ParticlePosition(FVector::ZeroVector), ParticleExtent(0.0f) {};

	/** Fractional oversize of the true entity beyond its best-fit node extent,
	 *  in [0,1]. Consumed as Extent * (1 + ScaleFactor); set by MakePointData. */
	float ScaleFactor;

	/** Volumetric (gas) density at this node. Accumulated along traversal, gates
	 *  node filtering, and is baked into the density volume texture. */
	float Density;

	/** RGB material composition packed in a vector; handed to child layers as
	 *  their ParentColor / node LinearColor. */
	FVector Composition;

	/** Deterministic hierarchical seed for this node. */
	int Seed;

	/** Tags a node's object class so heterogeneous entities share one tree; the
	 *  type-to-class mapping is managed externally. */
	int TypeId;

	/** Absolute index into the tier's flat particle buffer. */
	int ParticleIndex;

	/** Exact particle position (actor-local virtual space) captured at insert. */
	FVector ParticlePosition;

	/** Exact particle extent (actor-local units) captured at insert. */
	float ParticleExtent;

	/** Composes a deterministic, always-positive seed from a parent seed, a grid
	 *  coord, and a particle index within that cell. */
	static int32 ComposeSeed(int32 InParentSeed, const FIntVector& InCoord, int32 InParticleIndex)
	{
		const uint32 CoordHash = HashCombine(HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)), GetTypeHash(InCoord.Z));
		const uint32 Raw = HashCombine(static_cast<uint32>(InParentSeed), HashCombine(CoordHash, static_cast<uint32>(InParticleIndex)));
		return static_cast<int32>(Raw & 0x7FFFFFFF);
	}
};

/** Transient insert record: a position plus the FVoxelData payload and octree
 *  depth chosen for one entity. Built by MakePointData, consumed by the octree
 *  insert pipeline. */
struct SVO_API FPointData
{
private:
	/** Backing store for the position; access via GetPosition/SetPosition. */
	FVector PositionInternal;

public:
	/** Octree depth this point inserts at, chosen by MakePointData. */
	int InsertDepth;

	/** Voxel payload carried into the octree node. */
	FVoxelData Data;

	const FVector& GetPosition() const { return PositionInternal; }

	void SetPosition(const FVector& InPosition)
	{
		PositionInternal = InPosition;
	}

	/** Selects the octree depth whose node extent best matches InScaleWorldUnits
	 *  (converted to tree-local units by InUnitScale), then stores the residual
	 *  oversize as ScaleFactor = clamp(LocalSize / BestNodeExtent - 1, 1e-4, 1).
	 *  Walks depths [1, log2(InExtent)], stops once the object exceeds a node's
	 *  extent, and minimizes |1 - LocalSize / ExtentAtDepth|. */
	static FPointData MakePointData(const FVector InPosition, const double InScaleWorldUnits, const double InUnitScale, const int64 InExtent, const int InTypeId, const FVector InComposition) {
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
		Data.Composition = InComposition;
		Data.TypeId = InTypeId;
		FPointData ReturnData = FPointData(FVector::ZeroVector, BestDepth, Data);
		ReturnData.SetPosition(InPosition);
		return ReturnData;
	}

	/** Maps a uniform [0,1] sample through InDistributionCurve (clamped to [0,1]),
	 *  then lerps InMinScale..InMaxScale by the result. Falls back to the raw
	 *  sample when the curve has no keys. */
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
};

/** Cached procgen output for one grid cell, replayed on a cache hit to skip
 *  noise and rejection sampling. Arrays are per-buffer, parallel to the tier's
 *  Niagara buffers. */
struct SVO_API FCachedCellData
{
	TArray<TArray<FVector>>        PerBufferPositions;
	TArray<TArray<float>>          PerBufferExtents;
	TArray<TArray<FLinearColor>>   PerBufferColors;
	TArray<TArray<FVector>>        PerBufferRotations;
	int32 ParticleCount = 0;
};

/** Actor lifecycle stage gating the tier pipeline: UpdateTier early-outs unless
 *  Ready; Pooling/Destroying signal teardown to in-flight tasks. */
UENUM()
enum class ELifecycleState : uint8
{
	Uninitialized,
	Initializing,
	Ready,
	Pooling,
	Destroying
};
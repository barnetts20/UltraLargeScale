// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#pragma endregion

struct SVO_API FVoxelData {
public:
	FVoxelData() : ScaleFactor(0.0), Density(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1), ParticleIndex(-1) {};
	FVoxelData(float InDensity, float InGasDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : ScaleFactor(InDensity), Density(InGasDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId), ParticleIndex(-1) {};

	float ScaleFactor;
	float Density;
	FVector Composition;

	/** Deterministic hierarchical seed for this node. Composed via ComposeSeed
	 *  from the parent actor's seed, the tier grid coordinate, and the particle's
	 *  generation index. Always >= 0; -1 is reserved as the "empty node" sentinel
	 *  in the octree collision logic. Passed down as Params.Seed when spawning
	 *  child actors (galaxies, star systems, planets). */
	int ObjectId;

	int TypeId;

	/** Absolute index into the tier's flat particle buffer. Set during octree
	 *  insertion: AbsoluteIndex = SlotIndex * SlotCapacity + ParticleOffset.
	 *  Spawn hooks read positions/extents directly at Buffer[ParticleIndex].
	 *  Slot can be recovered as ParticleIndex / SlotCapacity when needed.
	 *  -1 if not set (e.g. manually inserted nodes outside the tier system). */
	int ParticleIndex;

	// Collision overflow for nodes that receive multiple inserts at the same
	// quantized depth/position. ObjectId carries the first inserter's seed;
	// any subsequent inserts at the same node append to AdditionalObjectIds.
	// Empty for the non-collision case (vast majority of nodes), so the
	// per-node memory cost is just the empty TArray header.
	TArray<int32> AdditionalObjectIds;

	/**
	 * Composes a deterministic, always-positive seed from a parent seed,
	 * a grid coordinate, and a particle index within that cell. The result
	 * is globally unique across the hierarchy when each level passes its
	 * own Params.Seed (itself a ComposeSeed output) as InParentSeed.
	 *
	 * The sign bit is masked off so the result is always >= 0, preserving
	 * the ObjectId == -1 "empty node" sentinel in the octree.
	 *
	 * @param InParentSeed     The owning actor's Params.Seed.
	 * @param InCoord          Tier grid coordinate of the cell.
	 * @param InParticleIndex  Generation-order index within the cell.
	 * @return                 Positive int32 seed, never -1.
	 */
	static int32 ComposeSeed(int32 InParentSeed, const FIntVector& InCoord, int32 InParticleIndex)
	{
		const uint32 CoordHash = HashCombine(
			HashCombine(GetTypeHash(InCoord.X), GetTypeHash(InCoord.Y)),
			GetTypeHash(InCoord.Z));
		const uint32 Raw = HashCombine(
			static_cast<uint32>(InParentSeed),
			HashCombine(CoordHash, static_cast<uint32>(InParticleIndex)));
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

	// Accessors
	const FVector& GetPosition() const { return PositionInternal; }
	const FInt64Vector& GetInt64Position() const { return Int64PositionInternal; }

	// Setters (keep values in sync)
	void SetPosition(const FVector& InPosition)
	{
		PositionInternal = InPosition;
		Int64PositionInternal = FInt64Vector(
			FMath::RoundToInt64(InPosition.X),
			FMath::RoundToInt64(InPosition.Y),
			FMath::RoundToInt64(InPosition.Z));
	}

	void SetInt64Position(const FInt64Vector& InInt64)
	{
		Int64PositionInternal = InInt64;
		PositionInternal = FVector(
			static_cast<double>(InInt64.X),
			static_cast<double>(InInt64.Y),
			static_cast<double>(InInt64.Z));
	}

	// Unified depth calculation from real-world scale
	// InScaleWorldUnits: Size in centimeters (real world)
	// InUnitScale: The octree's unit scale (cm per octree unit)
	// InExtent: The octree's extent (must be power of 2)
	static FPointData MakePointDataFromWorldScale(
		const double InScaleWorldUnits,
		const double InUnitScale,
		const int64 InExtent)
	{
		// Convert world scale to octree local units
		double LocalSize = InScaleWorldUnits / InUnitScale;

		// Calculate max depth based on extent
		// Extent is power of 2, minimum node size is 2 units
		// At depth d: NodeExtent = InExtent >> d
		// We want: InExtent >> MaxDepth = 2
		// So: MaxDepth = log2(InExtent) - 1
		int MinDepth = 1;
		int MaxDepth = static_cast<int>(FMath::Log2(static_cast<double>(InExtent)));

		// Find the deepest depth where this object still fits
		int BestDepth = MinDepth;
		int64 BestNodeExtent = InExtent >> MinDepth;
		double BestRatio = FMath::Abs(1.0 - LocalSize / static_cast<double>(BestNodeExtent));

		for (int d = MinDepth; d <= MaxDepth; d++)
		{
			int64 ExtentAtDepth = InExtent >> d;

			// Object must fit within node (LocalSize <= ExtentAtDepth)
			if (LocalSize > ExtentAtDepth)
			{
				// Too deep, object doesn't fit anymore
				break;
			}

			double Ratio = FMath::Abs(1.0 - LocalSize / static_cast<double>(ExtentAtDepth));

			if (Ratio < BestRatio)
			{
				BestRatio = Ratio;
				BestDepth = d;
				BestNodeExtent = ExtentAtDepth;
			}
		}

		// Calculate density to encode sub-node precision
		// Actual object size = NodeExtent * (1 + ScaleFactor)
		// ScaleFactor = (LocalSize / BestNodeExtent) - 1
		float ScaleFactor = FMath::Clamp(
			static_cast<float>((LocalSize / static_cast<double>(BestNodeExtent)) - 1.0),
			0.0001f,
			1.0f
		);

		FVoxelData Data;
		Data.ScaleFactor = ScaleFactor;

		return FPointData(FInt64Vector::ZeroValue, BestDepth, Data);
	}

	static double SampleScaleFromDistribution(double InMinScale, double InMaxScale, double InSample, const FRuntimeFloatCurve& InDistributionCurve) {
		// Lerp between min and max using the distribution value
		return FMath::Lerp(InMinScale, InMaxScale, InSample);
		//return FMath::Lerp(InMinScale, InMaxScale, static_cast<double>(FMath::Clamp(InDistributionCurve.GetRichCurveConst()->Eval(FMath::Clamp(InSample, 0.0f, 1.0f)), 0.0f, 1.0f)));
	}

	// Constructors
	FPointData() : InsertDepth(0), Data() {}

	FPointData(const FVector& InPosition, int InDepth, const FVoxelData& InData)
		: InsertDepth(InDepth), Data(InData)
	{
		SetPosition(InPosition);
	}

	FPointData(const FInt64Vector& InInt64, int InDepth, const FVoxelData& InData)
		: InsertDepth(InDepth), Data(InData)
	{
		SetInt64Position(InInt64);
	}
};

// ---------------------------------------------------------------------------
// Cached cell data � stores all generated particle data for a single
// streaming cell so it can be restored on re-entry without re-running
// procgen. Used by the persistent-cache refactor; the streaming pipeline
// writes one of these on first generation (cache-miss) and reads it back
// on subsequent entries (cache-hit).
//
// Each tier may have multiple Niagara buffers (e.g. Large = cluster + gas).
// PerBufferPositions / PerBufferExtents / etc. are outer-indexed by buffer
// index (parallel to FParticleTierConfig::NiagaraAssets).
// ---------------------------------------------------------------------------
struct SVO_API FCachedCellData
{
	// Per-buffer particle arrays. Outer index = buffer index within the tier.
	// Inner arrays hold exactly ParticleCount elements (no dead padding).
	TArray<TArray<FVector>>        PerBufferPositions;
	TArray<TArray<float>>          PerBufferExtents;
	TArray<TArray<FLinearColor>>   PerBufferColors;
	TArray<TArray<FVector>>        PerBufferRotations; // Empty inner array if tier doesn't use rotations.

	// The procedural center offset used when this cell was generated.
	// Final position = NodeCenter + CenterOffset + local offsets.
	// Persisting this avoids re-quantization artifacts on cache restore.
	FVector CenterOffset = FVector::ZeroVector;

	// Number of accepted (live) particles this cell produced.
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
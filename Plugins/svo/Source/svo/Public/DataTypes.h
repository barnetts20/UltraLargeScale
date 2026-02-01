// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#pragma region Includes/ForwardDec
#include "CoreMinimal.h"
#pragma endregion

struct SVO_API FVoxelData {
public:
	FVoxelData() : Density(0.0), GasDensity(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1) {};
	FVoxelData(float InDensity, float InGasDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : Density(InDensity), GasDensity(InGasDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId) {};

	float Density;
	float GasDensity;
	FVector Composition;
	int ObjectId;
	int TypeId;
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
        // Actual object size = NodeExtent * (1 + Density)
        // Density = (LocalSize / BestNodeExtent) - 1
        float Density = FMath::Clamp(
            static_cast<float>((LocalSize / static_cast<double>(BestNodeExtent)) - 1.0),
            0.0001f,
            1.0f
        );

        FVoxelData Data;
        Data.Density = Density;

        return FPointData(FInt64Vector::ZeroValue, BestDepth, Data);
    }

    static double SampleScaleFromDistribution(double InMinScale, double InMaxScale, double InSample, const FRuntimeFloatCurve& InDistributionCurve) {
        // Lerp between min and max using the distribution value
        return FMath::Lerp(InMinScale, InMaxScale, static_cast<double>(FMath::Clamp(InDistributionCurve.GetRichCurveConst()->Eval(FMath::Clamp(InSample, 0.0f, 1.0f)), 0.0f, 1.0f)));
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

UENUM()
enum class ELifecycleState : uint8
{
	Uninitialized,
	Initializing,
	Ready,
	Pooling,
	Destroying
};

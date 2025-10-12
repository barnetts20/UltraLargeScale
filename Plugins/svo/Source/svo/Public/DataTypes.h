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

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

struct SVO_API FVoxelData {
public:
	FVoxelData() : Density(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1) {};
	FVoxelData(double InDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : Density(InDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId) {};

	double Density;
	FVector Composition; // Density could be accumulated in alpha
	int ObjectId;
	int TypeId;
};

struct SVO_API FPointData {
	FVector Position;
	int InsertDepth;
	FVoxelData Data;
	FInt64Vector GetInt64Position() {
		return FInt64Vector(FMath::RoundToInt64(Position.X), FMath::RoundToInt64(Position.Y), FMath::RoundToInt64(Position.Z));
	}
};

UENUM()
enum class ELifecycleState : uint8
{
	Initializing,
	Ready,
	Destroying
};
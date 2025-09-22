// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

struct SVO_API FVoxelData {
public:
	FVoxelData() : Density(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1) {};
	FVoxelData(double InDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : Density(InDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId) {};

	float Density;
	FVector Composition; //TODO:: We still are not making effective use of this extra data. Investigate ways we could encode extra utility
	int ObjectId;
	int TypeId;
};

struct SVO_API FPointData {
	FVector Position;
	int InsertDepth;
	FVoxelData Data;

	//TODO:: Use overloaded setter to directly set a position... then we wouldnt need to do these rounding operations every time we need to insert point data... saves a bit of overhead
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
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class SVO_API DataTypes
{
public:
	DataTypes();
	~DataTypes();
};

struct SVO_API FVoxelData {
public:
	FVoxelData() : Density(0.0), Composition(0, 0, 0), ObjectId(-1), TypeId(-1) {};
	FVoxelData(double InDensity, FVector InComposition, int InObjectId, int InTypeId = -1) : Density(InDensity), Composition(InComposition), ObjectId(InObjectId), TypeId(InTypeId) {};

	double Density;
	FVector Composition; // Density could be accumulated in alpha
	int ObjectId;
	int TypeId;
};

UENUM()
enum class EGalaxyState : uint8
{
	Initializing,
	Ready,
	Destroying
};
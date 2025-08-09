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
	FVoxelData() : Density(0.0), ObjectId(-1), TypeId(-1) {};
	FVoxelData(double InDensity, int InObjectId, int InTypeId = -1) : Density(InDensity), ObjectId(InObjectId), TypeId(InTypeId) {};

	double Density;
	int ObjectId;
	int TypeId;
};

struct SVO_API FInt64Coordinate {
public:
	FInt64Coordinate() : X(0), Y(0), Z(0) {};
	FInt64Coordinate(int64 InX, int64 InY, int64 InZ) : X(InX), Y(InY), Z(InZ) {};

	int64 X;
	int64 Y;
	int64 Z;

	// Addition
	FInt64Coordinate operator+(const FInt64Coordinate& Other) const
	{
		return FInt64Coordinate(X + Other.X, Y + Other.Y, Z + Other.Z);
	}

	// Subtraction
	FInt64Coordinate operator-(const FInt64Coordinate& Other) const
	{
		return FInt64Coordinate(X - Other.X, Y - Other.Y, Z - Other.Z);
	}

	// Negation
	FInt64Coordinate operator-() const
	{
		return FInt64Coordinate(-X, -Y, -Z);
	}

	// Compound assignment (optional but useful)
	FInt64Coordinate& operator+=(const FInt64Coordinate& Other)
	{
		X += Other.X;
		Y += Other.Y;
		Z += Other.Z;
		return *this;
	}

	FInt64Coordinate& operator-=(const FInt64Coordinate& Other)
	{
		X -= Other.X;
		Y -= Other.Y;
		Z -= Other.Z;
		return *this;
	}

	// Equality check (optional)
	bool operator==(const FInt64Coordinate& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	bool operator!=(const FInt64Coordinate& Other) const
	{
		return !(*this == Other);
	}

};
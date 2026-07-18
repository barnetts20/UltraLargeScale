// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "FastNoise/FastNoise.h"
#include "ProceduralSpaceActor.h"
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"
#include "UniverseParams.h"

/** Generates the data that populates a universe sector: owns all noise
 *  composition and particle generation logic. The sector actor wires tier
 *  callbacks that delegate here; this class has no knowledge of actors, Niagara,
 *  octrees, or the streaming pipeline. */
class SVO_API UniverseDataGenerator {
public:
	UniverseDataGenerator() {};
	UniverseDataGenerator(FUniverseParams InParams) {
		Params = InParams;
	};

	FUniverseParams Params;
	FastNoise::SmartNode<> DensityNoise;
	TArray<FPointData> GeneratedData;

#pragma region Noise Composition

	/** Builds and stores DensityNoise from the current Params. */
	void Initialize();

	/** Builds the sector-scale density noise graph from Params. Pure function of
	 *  FUniverseDensityParams; needs no actor state. */
	FastNoise::SmartNode<> BuildNoise() const;

	/** Samples the noise field into a CPU-side volume texture buffer, returning
	 *  raw BGRA8 data suitable for FDensityVolume or GPU upload. */
	TArray<uint8> SampleNoiseVolume(
		int InNoiseResolution,
		const FIntVector& InCellCoord) const;

#pragma endregion

#pragma region Tier Generation Callbacks
	/** Self-contained generation functions that write directly into particle
	 *  buffers; the sector actor's tier system calls them via
	 *  FParticleTierConfig::GenerateCallback lambdas. Grid geometry (NodeCenter,
	 *  CellExtent) is passed in rather than computed internally so the generator
	 *  stays decoupled from the actor's tree extent multiplier and grid-depth
	 *  conventions. */

	 /** Large tier: generates cluster + gas particles using batched noise.
	  *  OutSlotCount receives the number of accepted particles. */
	void GenerateLargeTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InClusterBuffer,
		FNiagaraParticleBuffer& InGasBuffer,
		const FVector& InNodeCenter,
		int32& OutSlotCount) const;

	/** Mid tier: generates cluster particles at mid-grid scale. OutSlotCount
	 *  receives the number of accepted particles. */
	void GenerateMidTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;

	/** Small tier: generates galaxy-scale particles. OutSlotCount receives the
	 *  number of accepted particles. */
	void GenerateSmallTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;

#pragma endregion
};
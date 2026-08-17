// GalaxyDataGenerator.h
// Galaxy density field and tier generation.
//
// The density field itself lives in Shaders/GalaxyDensityCore.ush and is compiled by
// BOTH the shader and this module, so star placement and the rendered gas are one
// function rather than two implementations kept in agreement by hand. Only
// GalaxyDataGenerator.cpp compiles it, inside namespace GalaxyHLSL -- see
// GalaxyHLSLShim.h for why that namespace must not be opened.

#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "FastNoise/FastNoise.h"
#include "ProceduralSpaceActor.h"
#include "GalaxyParams.h"
#include "FTierStreamingSystem.h"
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"

/** Declared in GalaxyDensityCore.ush, compiled inside namespace GalaxyHLSL by
 *  GalaxyDataGenerator.cpp. Held by pointer so this header needs neither the shim nor
 *  the field itself. */
namespace GalaxyHLSL { struct GalaxyDensityParams; }

/** Owns density evaluation and tier generation for the galaxy layer; mirrors
 *  UniverseDataGenerator. The galaxy actor wires tier callbacks that delegate here;
 *  this class has no knowledge of actors, Niagara, octrees, or streaming. */
class ULTRALARGESCALE_API GalaxyDataGenerator
{
public:
	GalaxyDataGenerator();
	explicit GalaxyDataGenerator(FGalaxyParams InParams);

	// Out of line, and move-only: TUniquePtr needs GalaxyDensityParams complete at
	// the point of destruction, which is only true inside the .cpp.
	~GalaxyDataGenerator();
	GalaxyDataGenerator(GalaxyDataGenerator&&) noexcept;
	GalaxyDataGenerator& operator=(GalaxyDataGenerator&&) noexcept;
	GalaxyDataGenerator(const GalaxyDataGenerator&) = delete;
	GalaxyDataGenerator& operator=(const GalaxyDataGenerator&) = delete;

	FGalaxyParams Params;
	FastNoise::SmartNode<> DensityNoise;

	/** Clamped, pre-inverted fields plus the 16 per-arm records, derived once in
	 *  Initialize(). Rebuilding it per sample would repeat 16 hashes, a tan and every
	 *  reciprocal on every candidate. */
	TUniquePtr<GalaxyHLSL::GalaxyDensityParams> Derived;

#pragma region Density Sampling

	/** Raw field value at a normalized position, InNormPos in [-1,1] (position /
	 *  Extent).
	 *
	 *  UNBOUNDED. This is an OPTICAL DEPTH, not a probability: it peaks near 260 at
	 *  the default tuning while most of the volume sits below 0.01. Feeding it
	 *  straight to a rejection test accepts every candidate above 1.0 -- the arms,
	 *  the inner disc and the whole bulge -- erasing the structure it describes.
	 *  Always route it through FGalaxyDensityParams::ToSpawnProbability first. */
	float SampleDensity(const FVector& InNormPos) const;

	/** Batch form. Same contract: raw optical depth out. */
	void SampleDensityBatch(float* OutDensity, int32 InCount,
		const float* InX, const float* InY, const float* InZ) const;

#pragma endregion

#pragma region Initialization

	/** Derive the density parameters and build the encoded noise graph. MUST run
	 *  before any sampling; SampleDensity returns zero until it does. */
	void Initialize();

	/** Build noise from encoded tree. Kept for a future FastNoise swap-in. */
	FastNoise::SmartNode<> BuildNoise() const;

	/** Sample the field into a CPU-side BGRA8 volume buffer.
	 *
	 *  TRANSITIONAL. Once the raymarch material evaluates the field directly there is
	 *  nothing to bake, and removing this path also removes the async upload, the
	 *  upscale, and roughly 320 MB of resident volume texture. It is kept for now as
	 *  an independent cross-check: the CPU evaluates and bakes, the old material
	 *  renders the texture, and that should match the new material evaluating the
	 *  same parameters live. */
	TArray<uint8> SampleNoiseVolume(int InNoiseResolution) const;

#pragma endregion

#pragma region Tier Generation Callbacks
	/** Self-contained generation functions that write directly into particle buffers;
	 *  the galaxy actor's tier system calls them via
	 *  FParticleTierConfig::GenerateCallback lambdas.
	 *
	 *  A single function serves all tiers; Large/Mid/Small differ only in the
	 *  candidate volume (full extent vs cell-local), the tier params (scale range,
	 *  density curve), and the seed offset for stream isolation. */
	void GenerateTierNode(const FIntVector& InCoord, int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer, const FVector& InNodeCenter,
		double InCellExtent, const FTierParams& InTierParams,
		int32 InSeedOffset, int32& OutSlotCount) const;

	void GenerateLargeTierSlot(int32 InSlotIndex, FNiagaraParticleBuffer& InBuffer,
		int32& OutSlotCount) const;

#pragma endregion

#pragma region Large Tier Culling

	struct FActiveLargeTierCell
	{
		FVector Center = FVector::ZeroVector;
		double HalfExt = 0.0;
		FIntVector GridCoord = FIntVector::ZeroValue;
	};

	/** Cells whose eight corners are all zero-density are skipped entirely,
	 *  concentrating candidates on arms, disc and bulge. Corners rather than centres,
	 *  so a cell straddling an envelope boundary is never wrongly discarded. */
	TArray<FActiveLargeTierCell> CollectActiveLargeTierCells() const;

#pragma endregion
};
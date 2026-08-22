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
namespace GalaxyHLSL
{
	struct GalaxyDensityParams;
}

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
	 *  Always route it through GalaxyDensityParams::SpawnProbability first. */
	float SampleDensity(const FVector& InNormPos) const;

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

#pragma region Tier Generation
	/** Generation is GPU-ONLY for this layer. The per-slot CPU functions that used to
	 *  sit here -- GenerateTierNode, GenerateLargeTierSlot and the MakePlacement helper
	 *  they shared -- are gone, along with the FParticleTierConfig::GenerateCallback
	 *  bindings that reached them.
	 *
	 *  They existed because the field had to stay resource-free to be reachable from
	 *  the CPU, which is exactly the constraint the compute path was built to lift. A
	 *  second implementation kept in agreement by hand would put it straight back.
	 *
	 *  The other two layers still bind GenerateCallback, so it stays on the config. */

public:
	/** GPU generation for a whole batch of tier slots, in one dispatch.
	 *
	 *  This is the reason for the migration: the compute path can sample the volume
	 *  texture, so entity placement sees the warp and modulation the material draws
	 *  instead of a texture-free approximation of it. It also lifts the constraint
	 *  that shaped the field in the first place -- features no longer have to be
	 *  affordable analytically to be reachable from placement.
	 *
	 *  BACKGROUND THREAD ONLY; it blocks on a GPU readback. That is safe because tier
	 *  generation already runs on AnyBackgroundHiPriTask, so the wait costs a worker
	 *  rather than a frame.
	 *
	 *  Returns false if the dispatch could not run or the readback timed out. There is
	 *  no CPU path behind it any more, so it FAILS CLOSED: the affected slots are
	 *  blanked and their counts zeroed before returning, because a slot is reused as
	 *  the player crosses boundaries and leaving it untouched would show the previous
	 *  occupant's entities at a coord they no longer belong to. Every such path logs. */
	 /** One cell of a GPU generation batch.
	  *
	  *  The CENTRE IS SUPPLIED, not derived. Grid-coord-to-centre lives on the actor,
	  *  which owns the grid; the generator inferring it from a buffer's slot centres
	  *  put every candidate somewhere else entirely and every batch came back with
	  *  nothing accepted. It also lets the large tier pass the centres its cull prepass
	  *  already computed, so one path serves every tier. */
	struct FTierBatchCell
	{
		FIntVector Coord = FIntVector::ZeroValue;
		int32 SlotIndex = 0;
		FVector Centre = FVector::ZeroVector;
		double HalfExtent = 0.0;

		/** Density this cell rejects against. Zero means use the global reference;
		 *  the large tier passes its own per-cell peak, which is the whole reason its
		 *  acceptance rate is workable. */
		float DensityReference = 0.0f;

		/** Candidates to draw for this cell. Zero means the tier's CandidateBudget.
		 *
		 *  The large tier SHARES one budget across its whole active set rather than
		 *  giving every cell the full amount: its cell count is whatever survives the
		 *  cull, not a fixed neighbourhood, so per-cell budgeting turns a few thousand
		 *  cells into hundreds of millions of threads and a readback measured in
		 *  hundreds of megabytes. */
		int32 Candidates = 0;
	};

	/** Turns the large tier's active cell set into batch cells.
	 *
	 *  Its cells do not come from a grid neighbourhood: the cull prepass has already
	 *  discarded everything with no structure in it, and each survivor carries its own
	 *  peak density. That per-cell envelope is why the tier's acceptance rate is
	 *  workable at all -- against the global reference the ratio would run three orders
	 *  smaller and almost nothing would land.
	 *
	 *  All of that is DATA fed to the same dispatch, not a second code path. */
	bool BuildLargeTierCells(
		const TArray<TPair<FIntVector, int32>>& InSlots,
		TArray<FTierBatchCell>& OutCells) const;

	bool GenerateTierBatchGPU(
		const TArray<FTierBatchCell>& InCells,
		FNiagaraParticleBuffer& InBuffer,
		const FTierParams& InTierParams,
		int32 InSeedOffset,
		TArray<int32>& OutSlotCounts) const;

private:

#pragma endregion

#pragma region Large Tier Culling

	struct FActiveLargeTierCell
	{
		FVector Center = FVector::ZeroVector;
		double HalfExt = 0.0;
		FIntVector GridCoord = FIntVector::ZeroValue;

		/** Highest density found in this cell -- the REJECTION ENVELOPE for
		 *  candidates generated inside it. A local envelope is what takes acceptance
		 *  from a fraction of a percent to roughly a quarter: the field spans four
		 *  decades globally but only a narrow band within one cell.
		 *
		 *  It is an estimate from a finite sample, so it can under-shoot the true
		 *  maximum and clip a peak. Erring high costs only acceptance rate, which is
		 *  why the estimate is padded. */
		float MaxDensity = 0.0f;
	};

	/** Cells with no density anywhere are skipped entirely, concentrating candidates
	 *  on arms, disc and bulge. Also records each surviving cell's peak density for
	 *  use as a local rejection envelope.
	 *
	 *  Samples corners AND interior points: corners alone miss an arm that passes
	 *  through the middle of a cell without reaching any vertex, which both discards
	 *  live cells and under-estimates the envelope of the ones it keeps. */
	TArray<FActiveLargeTierCell> CollectActiveLargeTierCells() const;

#pragma endregion
};
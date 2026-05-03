// GalaxyDataGenerator.h
// Refactored to mirror UniverseDataGenerator architecture.
// Legacy galaxy generation code (FLegacyGalaxyParams, GalaxyParamFactory,
// LegacyGalaxyDataGenerator) is preserved at the bottom of this file
// for future reference during Phase E spiral density field work.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"  // FNoiseParams, FTierParams
#include "FVolumeTextureUtils.h"
#include "FNiagaraParticleBuffer.h"
#include "GalaxyDataGenerator.generated.h"

// ============================================================================
// FGalaxyParams — extends FBaseParams (mirrors FUniverseParams structure)
// ============================================================================

USTRUCT(BlueprintType)
struct SVO_API FGalaxyParams : public FBaseParams
{
	GENERATED_BODY()

	// --- Noise ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
	FNoiseParams NoiseParams;

	// --- Density volume ---

	/// Voxel resolution per axis for the density pseudo-volume texture.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density Volume")
	int32 DensityVolumeResolution = 64;

	// --- Tier scale derivation ---

	/// The absolute largest entity scale (world units) this galaxy supports.
	/// All tier scale ranges cascade downward from this single value,
	/// identical to FUniverseParams::MaxEntityScale logic.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
	double MaxEntityScale = 1e16;

	// --- Per-tier streaming configs ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Large")
	FTierParams LargeTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Mid")
	FTierParams MidTier;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tier|Small")
	FTierParams SmallTier;

	// --- Large tier particle count scaling ---

	/// Min particle count for the large tier (smallest galaxy scale).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Large Tier")
	int32 MinLargeParticles = 2000;

	/// Max particle count for the large tier (largest galaxy scale).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Large Tier")
	int32 MaxLargeParticles = 8000;

	/// Per-instance count variance fraction [0, 1]. Applied as a +/- offset
	/// on the lerped particle count for visual variety across galaxies.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Large Tier")
	float ParticleCountVariance = 0.15f;

	// --- Volume material params (carried over from legacy for volumetric setup) ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeAmbientColor = FLinearColor(1, 1, 1, 1);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeCoolShift = FLinearColor(.2, .5, .8);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FLinearColor VolumeHotShift = FLinearColor(.5, 1.5, 3);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeHueVariance = .1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeHueVarianceScale = .5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeSaturationVariance = .1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeTemperatureInfluence = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeTemperatureScale = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeDensity = .5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeWarpAmount = .05;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	double VolumeWarpScale = .33;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Volume Material")
	FString VolumeNoise = "/svo/VolumeTextures/VT_PerlinWorley_Balanced";

	// --- Encoded noise graph ---

	/// Pre-baked FastNoise2 encoded graph string for prototype density field.
	/// Build your graph in the FastNoise2 node editor, export the encoded
	/// string, and paste it here. This drives both the volumetric ray-march
	/// texture and the particle rejection sampling.
	/// Will be replaced by a programmatic spiral density field in Phase E.
	static constexpr const char* EncodedTree = "DQAFAAAAAAAAQAgAAAAAAD8AAAAAAA==";

	// --- Noise power for volume sampling ---

	/// Exponent applied to noise values during volume texture sampling.
	/// Higher = sharper contrast between dense and empty regions.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Density Volume")
	float NoisePower = 2.0f;

	/// Derive MinScale/MaxScale for each tier from MaxEntityScale and the
	/// depth sequence. Mirrors FUniverseParams::DeriveScaleRanges() exactly.
	void DeriveScaleRanges()
	{
		FTierParams* Tiers[] = { &LargeTier, &MidTier, &SmallTier };
		constexpr int32 NumTiers = UE_ARRAY_COUNT(Tiers);

		Tiers[0]->MaxScale = MaxEntityScale;

		for (int32 i = 0; i < NumTiers; ++i)
		{
			int32 DepthDelta;
			if (i + 1 < NumTiers)
			{
				DepthDelta = Tiers[i + 1]->GridDepth - Tiers[i]->GridDepth;
			}
			else
			{
				DepthDelta = Tiers[i]->GridDepth - Tiers[i - 1]->GridDepth;
			}

			const double Ratio = static_cast<double>(1 << FMath::Clamp(DepthDelta, 1, 20));
			Tiers[i]->MinScale = Tiers[i]->MaxScale / Ratio;

			if (i + 1 < NumTiers)
			{
				Tiers[i + 1]->MaxScale = Tiers[i]->MinScale;
			}
		}
	}

	FGalaxyParams()
	{
		Seed = 666;
		Extent = 2147483648;
		UnitScale = 1e11;
		Rotation = FRotator::ZeroRotator;
		ParentColor = FLinearColor(1, 1, 1, 0);

		// Large tier: single cell covering the full galaxy extent.
		// NeighborhoodRadius = 0 -> 1x1x1 = 1 slot, exhaustive single-pass.
		LargeTier.GridDepth = 1;
		LargeTier.NeighborhoodRadius = 0;
		LargeTier.MaxParticlesPerSlot = 8000;

		MidTier.GridDepth = 4;
		MidTier.NeighborhoodRadius = 1;
		MidTier.MaxParticlesPerSlot = 4000;

		SmallTier.GridDepth = 7;
		SmallTier.NeighborhoodRadius = 1;
		SmallTier.MaxParticlesPerSlot = 2000;

		DeriveScaleRanges();
	}
};


// ============================================================================
// GalaxyDataGenerator — owns noise composition and tier generation callbacks
// ============================================================================
//
// Mirrors UniverseDataGenerator structure. The galaxy actor wires tier
// callbacks that delegate here; this class has no knowledge of actors,
// Niagara, octrees, or the streaming pipeline.

class SVO_API GalaxyDataGenerator
{
public:
	GalaxyDataGenerator() {};
	GalaxyDataGenerator(FGalaxyParams InParams) : Params(InParams) {};

	FGalaxyParams Params;
	FastNoise::SmartNode<> DensityNoise;

	// -----------------------------------------------------------------------
	// Density Sampling — C++ prototype
	// -----------------------------------------------------------------------
	// Pure C++ density field for rapid iteration. This is the authoritative
	// density function — both volume texture baking and particle rejection
	// sampling call through here. When the math is finalized, port to a
	// FastNoise encoded graph and swap back to the GenPositionArray3D path.

	/// Sample density at a single normalized position.
	/// InNormPos is in [-1, 1] noise space (position / Extent).
	/// Returns density in [0, 1].
	static float SampleDensity(const FVector& InNormPos);

	/// Batch-evaluate the density field for an array of positions.
	/// Drop-in replacement for GenPositionArray3D — same signature pattern
	/// so swapping back to FastNoise is a one-line change.
	static void SampleDensityBatch(
		float* OutDensity,
		int32 InCount,
		const float* InX,
		const float* InY,
		const float* InZ);

	// -----------------------------------------------------------------------
	// Initialization
	// -----------------------------------------------------------------------

	/// Build the encoded noise graph (kept for future use) and mark ready.
	void Initialize();

	/// Build noise from encoded tree. Kept for future FastNoise swap-in.
	FastNoise::SmartNode<> BuildNoise() const;

	/// Sample the density field into a CPU-side BGRA8 volume texture buffer.
	/// Uses the C++ SampleDensity path directly rather than going through
	/// FVolumeTextureUtils::SampleNoiseToVolume.
	TArray<uint8> SampleNoiseVolume(int InNoiseResolution) const;

	// -----------------------------------------------------------------------
	// Tier Generation Callbacks
	// -----------------------------------------------------------------------
	// Self-contained generation functions that write directly into particle
	// buffers. The galaxy actor's tier system calls them via
	// FParticleTierConfig::GenerateCallback lambdas.

	/// Large tier: generates particles across the full galaxy extent using
	/// batched noise rejection sampling. Particle count is derived from
	/// ScaleFactor lerp + variance at spawn time.
	/// OutSlotCount receives the number of accepted particles.
	void GenerateLargeTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		int32& OutSlotCount) const;

	/// Mid tier: generates particles at mid-grid scale with neighborhood
	/// streaming. Same three-phase batched pattern.
	void GenerateMidTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;

	/// Small tier: generates particles at small-grid scale.
	void GenerateSmallTierNode(
		const FIntVector& InCoord,
		int32 InSlotIndex,
		FNiagaraParticleBuffer& InBuffer,
		const FVector& InNodeCenter,
		double InCellExtent,
		int32& OutSlotCount) const;

	// -----------------------------------------------------------------------
	// Particle Count Derivation
	// -----------------------------------------------------------------------

	/// Compute the large tier particle count for a galaxy with the given
	/// ScaleFactor [0,1]. Lerps between MinLargeParticles and
	/// MaxLargeParticles with optional seeded variance.
	int32 DeriveLargeParticleCount(float ScaleFactor, int32 Seed) const;
};


// ============================================================================
// LEGACY CODE — preserved for Phase E spiral density field reference
// ============================================================================
// The structures and classes below are the original galaxy generation system.
// They use a fundamentally different approach (explicit point generation via
// cluster/arm/bulge/disc/background passes + twist) rather than the noise
// field rejection sampling used by the new architecture.
//
// DO NOT USE for new code. These exist solely as reference material for
// implementing the spiral density field generator in Phase E.
// ============================================================================

#if 0  // ---- BEGIN LEGACY CODE (disabled, preserved for reference) ----

#include "PointCloudGenerator.h"

/// LEGACY GALAXY GENERATION PARAM STRUCT — original implementation
USTRUCT(BlueprintType)
struct SVO_API FLegacyGalaxyParams {
	GENERATED_BODY()
	// Generation parameters (affect WHAT is generated)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	int Seed = 666;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double Extent = 2147483648;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Generation")
	double UnitScale = 1e11;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MinStarSystemScale = 1.496e12;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	double MaxStarSystemScale = 1.496e16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FRotator Rotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FRuntimeFloatCurve ScaleDistributionCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
	FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);

	//Base Params - control overall sizes
	double GalaxyRatio = .30;
	double BulgeRatio = .75;
	double VoidRatio = .033;

	//Twist Params - Alters the behavior of the twist pass
	double TwistStrength = 4;
	double TwistCoreRadius = .01;
	double TwistCoreTwistExponent = 1;
	double TwistCoreStrength = 4;

	//Arm Params - changes arm appearance
	int ArmNumPoints = 75000;
	int ArmNumArms = 4;
	int ArmClusters = 124;
	double ArmDepthBias = .5;
	double ArmBaseDensity = 3;
	double ArmClusterRadiusMin = .05;
	double ArmClusterRadiusMax = .3;
	double ArmSpreadMin = .05;
	double ArmSpreadMax = .3;
	double ArmStartRatio = .5;
	double ArmHeightRatio = .5;
	double ArmProgressExponent = 1;
	double ArmRadialDensityExponent = 2;
	double ArmRadialDensityMax = 8;
	double ArmRadialDensityMin = .5;

	//Bulge Params - Controls the galactic bulge
	int BulgeNumPoints = 75000;
	double BulgeBaseDensity = 2;
	double BulgeDepthBias = .75;
	double BulgeRadiusScale = .33;
	double BulgeTruncationScale = 1;
	double BulgeAcceptanceExponent = 2;
	double BulgeJitter = .2;
	FVector BulgeAxisScale = FVector(1, 1, .6);

	//Cluster Params
	int ClusterNumPoints = 50000;
	int ClusterNumClusters = 128;
	FVector ClusterAxisScale = FVector(1, 1, 1);
	double ClusterSpreadFactor = .35;
	double ClusterMinScale = .3;
	double ClusterMaxScale = .7;
	double ClusterIncoherence = 3;
	double ClusterBaseDensity = 1;
	double ClusterDepthBias = 1;

	//Disc Params
	int DiscNumPoints = 50000;
	double DiscBaseDensity = 1;
	double DiscDepthBias = 1;
	double DiscHeightRatio = .1;

	//Background Params
	int BackgroundNumPoints = 50000;
	double BackgroundBaseDensity = 10;
	double BackgroundDepthBias = 1;
	double BackgroundHeightRatio = .8;

	//Volume Material Params
	FLinearColor VolumeAmbientColor = FLinearColor(1, 1, 1, 1);
	FLinearColor VolumeCoolShift = FLinearColor(.2, .5, .8);
	FLinearColor VolumeHotShift = FLinearColor(.5, 1.5, 3);
	double VolumeHueVariance = .1;
	double VolumeHueVarianceScale = .5;
	double VolumeSaturationVariance = .1;
	double VolumeTemperatureInfluence = 32;
	double VolumeTemperatureScale = 1;
	double VolumeDensity = .5;
	double VolumeWarpAmount = .05;
	double VolumeWarpScale = .33;
	FString VolumeNoise = "/svo/VolumeTextures/VT_PerlinWorley_Balanced";

	double GalaxyRadius;
	double BulgeRadius;
	double VoidRadius;

	FLegacyGalaxyParams() {
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.0f, 0.0f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.25f, 0.001f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.75f, 0.005f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.9375f, 0.01f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.984375f, 0.05f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.996f, 0.1f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(0.999f, 0.4f);
		ScaleDistributionCurve.GetRichCurve()->AddKey(1.0f, 1.0f);
	}
};

/// LEGACY GALAXY PARAM FACTORY — defines randomization ranges and archetypes
/// (E0, E3, E5, E7, S0, Sa, Sb, Sc, SBa, SBb, SBc, Irr)
class SVO_API LegacyGalaxyParamFactory {
public:
	int Seed = 666;
	FLegacyGalaxyParams E0, E3, E5, E7, S0, Sa, Sb, Sc, SBa, SBb, SBc, Irr;
	FLegacyGalaxyParams E0_Min, E0_Max, E3_Min, E3_Max, E5_Min, E5_Max, E7_Min, E7_Max;
	FLegacyGalaxyParams S0_Min, S0_Max, Sa_Min, Sa_Max, Sb_Min, Sb_Max, Sc_Min, Sc_Max;
	FLegacyGalaxyParams SBa_Min, SBa_Max, SBb_Min, SBb_Max, SBc_Min, SBc_Max;
	FLegacyGalaxyParams Irr_Min, Irr_Max;
	FLegacyGalaxyParams Volume_Min, Volume_Max;

	TArray<TPair<float, FLegacyGalaxyParams*>> GalaxyWeights;
	TArray<const char*> NoisePaths;

	LegacyGalaxyParamFactory();
	FLegacyGalaxyParams GenerateParams();
	FLegacyGalaxyParams BoundedRandomizeParams(FLegacyGalaxyParams MinParams, FLegacyGalaxyParams MaxParams);
	int SelectGalaxyTypeIndex();
};

/// LEGACY GALAXY GENERATOR — explicit point generation with arms/bulge/twist
class SVO_API LegacyGalaxyDataGenerator : public PointCloudGenerator {
public:
	LegacyGalaxyDataGenerator() : PointCloudGenerator(69) {};
	LegacyGalaxyDataGenerator(int InSeed) : PointCloudGenerator(InSeed) {};

	FLegacyGalaxyParams Params;
	TArray<FPointData> GeneratedData;

	virtual void GenerateData(TSharedPtr<FOctree> InOctree) override;

	void GenerateArms();
	void ApplyTwist();
	void GenerateBulge();
	void GenerateClusters();
	void GenerateDisc();
	void GenerateBackground();
	void ApplyRotation();
	void GenerateCluster(int InSeed, FVector InClusterCenter, FVector InClusterRadius, int InCount, double InBaseDensity = 1, double InDepthBias = 1);
};

#endif // ---- END LEGACY CODE ----
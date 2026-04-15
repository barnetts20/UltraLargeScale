#pragma once
#include "CoreMinimal.h"
#include "ProceduralSpaceActor.h"
#include "UniverseDataGenerator.h"
#include "SectorActor.generated.h"

class AGalaxyActor;

UCLASS()
class SVO_API ASectorActor : public AProceduralSpaceActor
{
	GENERATED_BODY()

public:
	ASectorActor();

#pragma region Editor Exposed Parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Universe Properties")
	FUniverseParams Params;
#pragma endregion

#pragma region Pooled Spawn/Despawn Hooks
	TMap<TSharedPtr<FOctreeNode>, TWeakObjectPtr<AGalaxyActor>> SpawnedGalaxies;
	void SpawnGalaxyFromPool(TSharedPtr<FOctreeNode> InNode);
	void ReturnGalaxyToPool(TSharedPtr<FOctreeNode> InNode);
#pragma endregion

protected:
#pragma region Params Accessors
	virtual double GetUnitScale() const override { return Params.UnitScale; }
	virtual double GetExtent() const override { return Params.Extent; }
	virtual double GetParentSpeedScale() const override { return SpeedScale; }
#pragma endregion

#pragma region Initialization
	virtual void InitializeData() override;
	virtual void InitializeVolumetric() override;
	virtual void InitializeNiagara() override;
	virtual void InitializeChildPool() override;
#pragma endregion

#pragma region Data Generation
	UniverseDataGenerator UniverseGenerator;
#pragma endregion

#pragma region Niagara
	TArray<FVector> Rotations;
#pragma endregion

#pragma region Volumetric
	FString VolumetricMaterialPath = FString("/svo/Materials/RayMarchers/MT_UniverseRaymarchPseudoVolume_Inst.MT_UniverseRaymarchPseudoVolume_Inst");
#pragma endregion

#pragma region Galaxy Pool
	TSubclassOf<AGalaxyActor> GalaxyActorClass;
	int GalaxyPoolSize = 5;
	TArray<AGalaxyActor*> GalaxyPool;
#pragma endregion

#pragma region Overrides
	virtual void BeginPlay() override;
	virtual void ApplyParallaxOffset() override;
	virtual void Tick(float DeltaTime) override;
#pragma endregion

#pragma region Player-Centered Parallax
	double ParallaxRatio = 0.0;
#pragma endregion

#pragma region Proximity Galaxy Streaming
	// --- Configuration ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 ScanDepth = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 MaxParticlesPerNode = 2000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Proximity")
	int32 RejectionOversampleFactor = 4;

	// --- Slot State (only touched by async task, guarded by bProximityUpdateInProgress) ---
	FIntVector CurrentScanCoord = FIntVector(INT32_MIN);
	TMap<FIntVector, int32> ActiveNodeSlots;
	TArray<int32> FreeSlots;
	TArray<int32> SlotParticleCounts;

	// --- Double-Buffered Particle Data ---
	struct FProximityBuffer
	{
		TArray<FVector> Positions;
		TArray<float> Extents;
		TArray<FLinearColor> Colors;

		void Allocate(int32 TotalParticles)
		{
			Positions.SetNumZeroed(TotalParticles);
			Extents.SetNumZeroed(TotalParticles);
			Colors.SetNumZeroed(TotalParticles);
		}
	};

	FProximityBuffer ProximityBuffers[2];
	std::atomic<int32> FrontBufferIndex{ 0 };
	std::atomic<bool> bProximityUpdateInProgress{ false };
	std::atomic<bool> bProximityNeedsPush{ false };

	// --- Proximity Niagara ---
	UPROPERTY()
	UNiagaraComponent* ProximityNiagaraComponent;

	// --- Methods ---
	void InitializeProximitySystem();
	void UpdateProximityNodes();
	void GenerateNodeGalaxies(const FIntVector& InNodeCoord, int32 InSlotIndex, FProximityBuffer& InBuffer);
	void PushProximityToNiagara();

	FIntVector PositionToScanCoord(const FVector& InLocalPos) const;
	FVector ScanCoordToCenter(const FIntVector& InCoord) const;
	double GetScanNodeExtent() const;
#pragma endregion
};
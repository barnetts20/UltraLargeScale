// ProceduralSpaceActor.h
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FOctree.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "ProceduralSpaceActor.generated.h"

/// <summary>
/// BASE GENERATION PARAMS - Shared fields across Universe/Galaxy/StarSystem
/// </summary>
USTRUCT(BlueprintType)
struct SVO_API FBaseParams {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    int Seed = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    double Extent = 2147483648;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Generation")
    double UnitScale = 1.0;  // Derived - set by spawning actor

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Generation")
    FLinearColor ParentColor = FLinearColor(1, 1, 1, 0);
};

/// <summary>
/// BASE PROCEDURAL SPACE ACTOR - Abstract base for Universe/Galaxy/StarSystem
/// </summary>
UCLASS(Abstract)
class SVO_API AProceduralSpaceActor : public AActor
{
    GENERATED_BODY()

public:
    AProceduralSpaceActor();
    virtual ~AProceduralSpaceActor() = default;

#pragma region Shared State
    TSharedPtr<FOctree> Octree;
    ELifecycleState InitializationState = ELifecycleState::Uninitialized;
    bool IsDebug = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parallax Properties")
    double SpeedScale = 1.0;  // Runtime parameter (not in Params)
#pragma endregion

#pragma region Lifecycle (overrideable)
    virtual void Initialize();      // Kicks off async init
    virtual void ResetForSpawn();   // Called before Initialize
    virtual void ResetForPool();    // Called when returning to pool
#pragma endregion

public:
#pragma region Initialization (pure virtual - must implement)
    virtual void InitializeData();
    virtual void InitializeVolumetric();
    virtual void InitializeNiagara();
    virtual void InitializeChildPool();  // Optional - Universe/Galaxy override this
#pragma endregion

#pragma region Params Accessors (now with default implementations!)
    // Children can override these if they have extended params, otherwise use base
    virtual double GetUnitScale() const { return 1; }
    virtual double GetExtent() const { return 2147483648; }
    virtual int GetSeed() const { return 1; }
    virtual FRotator GetRotation() const { return FRotator::ZeroRotator; }
    virtual FLinearColor GetParentColor() const { return FLinearColor(1, 1, 1); }
    // Only this one truly needs override (for parent chain)
    virtual double GetParentSpeedScale() const { return 1; }
#pragma endregion

#pragma region Shared Niagara
    TArray<FVector> Positions;
    TArray<float> Extents;
    TArray<FLinearColor> Colors;

    UPROPERTY()
    UNiagaraSystem* ProximityCloud;

    UPROPERTY()
    UNiagaraComponent* NiagaraComponent;
#pragma endregion

#pragma region Shared Volumetric
    UPROPERTY()
    UTexture2D* PseudoVolumeTexture;

    UPROPERTY()
    UStaticMeshComponent* VolumetricComponent;

    UPROPERTY()
    UMaterialInstanceDynamic* VolumeMaterial;
#pragma endregion

#pragma region Parallax
    FVector LastFrameOfReferenceLocation = FVector::ZeroVector;
    FVector CurrentFrameOfReferenceLocation = FVector::ZeroVector;

#pragma region Parallax Spawn Calculation
    // Computes correct spawn position for a child actor based on parallax ratios
    FVector ComputeChildSpawnLocation(const FVector& NodeCenter, double ChildUnitScale) const;
#pragma endregion
    virtual void ApplyParallaxOffset();
    virtual void DrawDebugBounds();
    virtual void Tick(float DeltaTime) override;

    // Called by the parent actor (Universe→Galaxy, Galaxy→StarSystem) instead of
    // UE's per-actor tick dispatch. InPlayerPos is the already-resolved player
    // world position for this frame — no child needs to query the controller.
    // Base implementation applies the standard parallax offset (covers StarSystem).
    // Galaxy overrides this to also handle VirtualTraversal, Niagara pushes,
    // tier streaming, and cascading down to its own star systems.
    virtual void TickFromParent(float DeltaTime, const FVector& InPlayerPos);
#pragma endregion
};
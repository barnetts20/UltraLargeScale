#pragma region Includes/ForwardDec
#include "ProximityTrackerComponent.h"
#include <Engine.h>
#pragma endregion

#pragma region Constructor/Destructor
UProximityTrackerComponent::UProximityTrackerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}
#pragma endregion

#pragma region Initialization
void UProximityTrackerComponent::BeginPlay()
{
    Super::BeginPlay();
    UniverseActor = Cast<AUniverseActor>(UGameplayStatics::GetActorOfClass(GetWorld(), AUniverseActor::StaticClass()));
    if (UniverseActor)
    {
        GetWorld()->GetTimerManager().SetTimer(
            UpdateTimerHandle,
            this,
            &UProximityTrackerComponent::OnProximityUpdate,
            UpdateInterval,
            true);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("ProximityTrackerComponent: AUniverseActor not found in the world!"));
    }
}
#pragma endregion

#pragma region Proximity Polling
void UProximityTrackerComponent::OnProximityUpdate()
{
    if (!UniverseActor) return;

    AActor* Owner = GetOwner();
    if (!Owner) return;
    if (!UniverseActor || UniverseActor->InitializationState != ELifecycleState::Ready) return;

    FVector WorldLocation = Owner->GetActorLocation();

    // ============================================
    // HIERARCHICAL SHORT-CIRCUIT SEARCH
    // Check from deepest to shallowest level
    // ============================================

    // LEVEL 3: Star System Entities (deepest)
    // Check if we're inside any spawned star system
    bool bInsideStarSystem = false;
    for (TSharedPtr<FOctreeNode> StarSystemNode : SpawnedStarSystemNodes) {
        // Find the galaxy this star system belongs to
        AGalaxyActor* ParentGalaxy = nullptr;
        for (TSharedPtr<FOctreeNode> GalaxyNode : SpawnedGalaxyNodes) {
            auto ga = UniverseActor->SpawnedGalaxies.Find(GalaxyNode);
            if (ga && ga->IsValid()) {
                auto GalaxyActor = ga->Get();
                if (GalaxyActor && GalaxyActor->SpawnedStarSystems.Contains(StarSystemNode)) {
                    ParentGalaxy = GalaxyActor;
                    break;
                }
            }
        }

        if (!ParentGalaxy) continue;

        auto ssa = ParentGalaxy->SpawnedStarSystems.Find(StarSystemNode);
        if (!ssa || !ssa->IsValid()) continue;

        auto StarSystemActor = ssa->Get();
        if (!StarSystemActor || StarSystemActor->InitializationState != ELifecycleState::Ready) continue;

        FVector StarSystemSampleLocation = WorldLocation - StarSystemActor->GetActorLocation();
        FInt64Vector StarSystemSampleCoordinate = FInt64Vector(
            FMath::RoundToInt64(StarSystemSampleLocation.X),
            FMath::RoundToInt64(StarSystemSampleLocation.Y),
            FMath::RoundToInt64(StarSystemSampleLocation.Z)
        );

        // Check if we're inside this star system's bounds
        int64 StarSystemExtent = StarSystemActor->Octree->Extent;
        if (FMath::Abs(StarSystemSampleCoordinate.X) <= StarSystemExtent &&
            FMath::Abs(StarSystemSampleCoordinate.Y) <= StarSystemExtent &&
            FMath::Abs(StarSystemSampleCoordinate.Z) <= StarSystemExtent)
        {
            bInsideStarSystem = true;

            // We're inside this star system - only scan entities here
            auto NearbyEntityNodes = StarSystemActor->Octree->GetNodesByScreenSpace(
                StarSystemSampleCoordinate,
                ScanExtent,
                .0001,
                -1,
                -1,
                -1
            );

            if (DebugMode) {
                for (const TSharedPtr<FOctreeNode>& Node : NearbyEntityNodes) {
                    if (Node.IsValid()) {
                        DebugDrawSystemEntityNode(
                            StarSystemActor->GetActorLocation() + FVector(Node->Center),
                            Node
                        );
                    }
                }
            }

            // Update tracked entities
            TSet<TSharedPtr<FOctreeNode>> StaleEntityNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedEntityNodes);
            for (const TSharedPtr<FOctreeNode>& Node : NearbyEntityNodes) {
                StaleEntityNodes.Remove(Node);
                if (!SpawnedEntityNodes.Contains(Node)) {
                    SpawnedEntityNodes.Add(Node);
                    StarSystemActor->SpawnEntityFromPool(Node);
                }
            }

            for (const TSharedPtr<FOctreeNode>& Node : StaleEntityNodes) {
                StarSystemActor->ReturnEntityToPool(Node);
                SpawnedEntityNodes.Remove(Node);
            }

            // SHORT CIRCUIT - we're in a star system, don't check galaxies or universe
            return;
        }
    }

    // If we're here, we're not inside any star system
    // Clean up all entity nodes since we've left all star systems
    if (!bInsideStarSystem && SpawnedEntityNodes.Num() > 0) {
        for (TSharedPtr<FOctreeNode> StarSystemNode : SpawnedStarSystemNodes) {
            for (TSharedPtr<FOctreeNode> GalaxyNode : SpawnedGalaxyNodes) {
                auto ga = UniverseActor->SpawnedGalaxies.Find(GalaxyNode);
                if (ga && ga->IsValid()) {
                    auto GalaxyActor = ga->Get();
                    if (GalaxyActor && GalaxyActor->SpawnedStarSystems.Contains(StarSystemNode)) {
                        auto ssa = GalaxyActor->SpawnedStarSystems.Find(StarSystemNode);
                        if (ssa && ssa->IsValid()) {
                            auto StarSystemActor = ssa->Get();
                            if (StarSystemActor) {
                                for (const TSharedPtr<FOctreeNode>& Node : SpawnedEntityNodes) {
                                    StarSystemActor->ReturnEntityToPool(Node);
                                }
                            }
                        }
                    }
                }
            }
        }
        SpawnedEntityNodes.Empty();
    }

    // LEVEL 2: Star Systems (middle)
    // Check if we're inside any spawned galaxy
    bool bInsideGalaxy = false;
    for (TSharedPtr<FOctreeNode> GalaxyNode : SpawnedGalaxyNodes) {
        auto ga = UniverseActor->SpawnedGalaxies.Find(GalaxyNode);
        if (!ga || !ga->IsValid()) continue;

        auto GalaxyActor = ga->Get();
        if (!GalaxyActor || GalaxyActor->InitializationState != ELifecycleState::Ready) continue;

        FVector GalaxySampleLocation = WorldLocation - GalaxyActor->GetActorLocation();
        FInt64Vector GalaxySampleCoordinate = FInt64Vector(
            FMath::RoundToInt64(GalaxySampleLocation.X),
            FMath::RoundToInt64(GalaxySampleLocation.Y),
            FMath::RoundToInt64(GalaxySampleLocation.Z)
        );

        // Check if we're inside this galaxy's bounds
        int64 GalaxyExtent = GalaxyActor->Octree->Extent;
        if (FMath::Abs(GalaxySampleCoordinate.X) <= GalaxyExtent &&
            FMath::Abs(GalaxySampleCoordinate.Y) <= GalaxyExtent &&
            FMath::Abs(GalaxySampleCoordinate.Z) <= GalaxyExtent)
        {
            bInsideGalaxy = true;

            // We're inside this galaxy - only scan star systems here
            auto NearbyStarSystemNodes = GalaxyActor->Octree->GetNodesByScreenSpace(
                GalaxySampleCoordinate,
                ScanExtent,
                .0001,
                -1,
                -1,
                1
            );

            if (DebugMode) {
                for (const TSharedPtr<FOctreeNode>& Node : NearbyStarSystemNodes) {
                    if (Node.IsValid()) {
                        DebugDrawStarSystemNode(GalaxyActor->GetActorLocation() + FVector(Node->Center), Node);
                    }
                }
            }

            // Update tracked star systems
            TSet<TSharedPtr<FOctreeNode>> StaleStarSystemNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedStarSystemNodes);
            for (const TSharedPtr<FOctreeNode>& Node : NearbyStarSystemNodes) {
                StaleStarSystemNodes.Remove(Node);
                if (!SpawnedStarSystemNodes.Contains(Node)) {
                    SpawnedStarSystemNodes.Add(Node);
                    GalaxyActor->SpawnStarSystemFromPool(Node);
                }
            }

            for (const TSharedPtr<FOctreeNode>& Node : StaleStarSystemNodes) {
                GalaxyActor->ReturnStarSystemToPool(Node);
                SpawnedStarSystemNodes.Remove(Node);
            }

            // SHORT CIRCUIT - we're in a galaxy, don't check universe
            return;
        }
    }

    // If we're here, we're not inside any galaxy
    // Clean up all star system nodes since we've left all galaxies
    if (!bInsideGalaxy && SpawnedStarSystemNodes.Num() > 0) {
        for (TSharedPtr<FOctreeNode> GalaxyNode : SpawnedGalaxyNodes) {
            auto ga = UniverseActor->SpawnedGalaxies.Find(GalaxyNode);
            if (ga && ga->IsValid()) {
                auto GalaxyActor = ga->Get();
                if (GalaxyActor) {
                    for (const TSharedPtr<FOctreeNode>& Node : SpawnedStarSystemNodes) {
                        GalaxyActor->ReturnStarSystemToPool(Node);
                    }
                }
            }
        }
        SpawnedStarSystemNodes.Empty();
    }

    // LEVEL 1: Galaxies (shallowest)
    // If we're not in a star system or galaxy, scan the universe
    FVector SampleLocation = WorldLocation - UniverseActor->GetActorLocation();
    FInt64Vector SampleCoordinate = FInt64Vector(
        FMath::RoundToInt64(SampleLocation.X),
        FMath::RoundToInt64(SampleLocation.Y),
        FMath::RoundToInt64(SampleLocation.Z)
    );

    TArray<TSharedPtr<FOctreeNode>> NearbyGalaxyNodes = UniverseActor->Octree->GetNodesByScreenSpace(
        SampleCoordinate,
        ScanExtent,
        .0001,
        -1,
        -1,
        1
    );

    if (DebugMode) {
        for (const TSharedPtr<FOctreeNode>& Node : NearbyGalaxyNodes) {
            if (Node.IsValid()) {
                DebugDrawNode(Node);
            }
        }
    }

    // Update tracked galaxies
    TSet<TSharedPtr<FOctreeNode>> StaleGalaxyNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedGalaxyNodes);
    for (const TSharedPtr<FOctreeNode>& Node : NearbyGalaxyNodes) {
        StaleGalaxyNodes.Remove(Node);
        if (!SpawnedGalaxyNodes.Contains(Node)) {
            SpawnedGalaxyNodes.Add(Node);
            UniverseActor->SpawnGalaxyFromPool(Node);
        }
    }

    for (const TSharedPtr<FOctreeNode>& Node : StaleGalaxyNodes) {
        UniverseActor->ReturnGalaxyToPool(Node);
        SpawnedGalaxyNodes.Remove(Node);
    }
}
#pragma endregion

#pragma region Debug
void UProximityTrackerComponent::DebugDrawNode(TSharedPtr<FOctreeNode> InNode)
{
    if (!InNode.IsValid() || !UniverseActor) return;

    // Convert FInt64Coordinate to FVector in world space
    const FVector UniverseOrigin = UniverseActor->GetActorLocation(); // world-space offset
    const FVector NodeCenter = FVector(
        static_cast<float>(InNode->Center.X),
        static_cast<float>(InNode->Center.Y),
        static_cast<float>(InNode->Center.Z)
    ) + UniverseOrigin;

    const float HalfExtent = static_cast<float>(InNode->Extent);
    const FVector BoxExtent = FVector(HalfExtent);

    const FColor BoxColor = FColor::Green;
    const float Lifetime = UpdateInterval;
    const bool bPersistent = false;
    const uint8 DepthPriority = 0;
    const float Thickness = 10.0f;

    DrawDebugBox(
        GetWorld(),
        NodeCenter,
        BoxExtent,
        BoxColor,
        bPersistent,
        Lifetime,
        DepthPriority,
        Thickness
    );
}

void UProximityTrackerComponent::DebugDrawStarSystemNode(FVector NodeCenter, TSharedPtr<FOctreeNode> InNode)
{
    if (!InNode.IsValid() || !UniverseActor) return;

    const float HalfExtent = static_cast<float>(InNode->Extent);
    const FVector BoxExtent = FVector(HalfExtent);

    const FColor BoxColor = FColor::Green;
    const float Lifetime = UpdateInterval;
    const bool bPersistent = false;
    const uint8 DepthPriority = 0;
    const float Thickness = 10.0f;

    DrawDebugBox(
        GetWorld(),
        NodeCenter,
        BoxExtent,
        BoxColor,
        bPersistent,
        Lifetime,
        DepthPriority,
        Thickness
    );
}

void UProximityTrackerComponent::DebugDrawSystemEntityNode(FVector NodeCenter, TSharedPtr<FOctreeNode> InNode)
{
    if (!InNode.IsValid()) return;

    const float HalfExtent = static_cast<float>(InNode->Extent);
    const FVector BoxExtent = FVector(HalfExtent);

    // Use different color for entities to distinguish them
    const FColor BoxColor = FColor::Cyan;
    const float Lifetime = UpdateInterval;
    const bool bPersistent = false;
    const uint8 DepthPriority = 0;
    const float Thickness = 10.0f;

    DrawDebugBox(
        GetWorld(),
        NodeCenter,
        BoxExtent,
        BoxColor,
        bPersistent,
        Lifetime,
        DepthPriority,
        Thickness
    );
}
#pragma endregion

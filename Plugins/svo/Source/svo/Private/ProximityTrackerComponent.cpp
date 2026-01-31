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

    //Get parent position
    AActor* Owner = GetOwner();
    if (!Owner) return;
    if (!UniverseActor || UniverseActor->InitializationState != ELifecycleState::Ready) return;
    FVector WorldLocation = Owner->GetActorLocation();
    FVector SampleLocation = WorldLocation - UniverseActor->GetActorLocation();
    FInt64Vector SampleCoordinate = FInt64Vector(FMath::RoundToInt64(SampleLocation.X), FMath::RoundToInt64(SampleLocation.Y), FMath::RoundToInt64(SampleLocation.Z));

    //Proximity Query Octree
    TArray<TSharedPtr<FOctreeNode>> NearbyGalaxyNodes = UniverseActor->Octree->GetNodesByScreenSpace(SampleCoordinate, ScanExtent, .0001, -1, -1, 1);

    if (DebugMode) {
        for (const TSharedPtr<FOctreeNode>& Node : NearbyGalaxyNodes)
        {
            if (Node.IsValid())
            {
                DebugDrawNode(Node);
            }
        }
    }

    //Update the currently tracked galaxies and trigger lifecycle changes
    TSet<TSharedPtr<FOctreeNode>> StaleGalaxyNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedGalaxyNodes);
    for (const TSharedPtr<FOctreeNode>& Node : NearbyGalaxyNodes)
    {
        StaleGalaxyNodes.Remove(Node);
        if (UniverseActor)
        {
            if (!SpawnedGalaxyNodes.Contains(Node))
            {
                SpawnedGalaxyNodes.Add(Node);
                UniverseActor->SpawnGalaxyFromPool(Node);
            }
        }
    }

    for (const TSharedPtr<FOctreeNode>& Node : StaleGalaxyNodes) {
        if (UniverseActor) {
            UniverseActor->ReturnGalaxyToPool(Node);
            SpawnedGalaxyNodes.Remove(Node);
        }
    }

    // Process each spawned galaxy
    for (TSharedPtr<FOctreeNode> GalaxyNode : SpawnedGalaxyNodes) {
        auto ga = UniverseActor->SpawnedGalaxies.Find(GalaxyNode);
        if (ga) {
            auto GalaxyActor = ga->Get();
            if (!GalaxyActor || GalaxyActor->InitializationState != ELifecycleState::Ready) continue;

            FVector GalaxySampleLocation = WorldLocation - GalaxyActor->GetActorLocation();
            FInt64Vector GalaxySampleCoordinate = FInt64Vector(FMath::RoundToInt64(GalaxySampleLocation.X), FMath::RoundToInt64(GalaxySampleLocation.Y), FMath::RoundToInt64(GalaxySampleLocation.Z));

            auto NearbyStarSystemNodes = GalaxyActor->Octree->GetNodesByScreenSpace(GalaxySampleCoordinate, ScanExtent, .0001, -1, -1, 1);

            if (DebugMode) {
                for (const TSharedPtr<FOctreeNode>& Node : NearbyStarSystemNodes)
                {
                    if (Node.IsValid())
                    {
                        DebugDrawStarSystemNode(GalaxyActor->GetActorLocation() + FVector(Node->Center), Node);
                    }
                }
            }

            //Update the currently tracked Star Systems and trigger lifecycle changes
            TSet<TSharedPtr<FOctreeNode>> StaleStarSystemNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedStarSystemNodes);
            for (const TSharedPtr<FOctreeNode>& Node : NearbyStarSystemNodes)
            {
                StaleStarSystemNodes.Remove(Node);
                if (GalaxyActor)
                {
                    if (!SpawnedStarSystemNodes.Contains(Node))
                    {
                        SpawnedStarSystemNodes.Add(Node);
                        GalaxyActor->SpawnStarSystemFromPool(Node);
                    }
                }
            }

            for (const TSharedPtr<FOctreeNode>& Node : StaleStarSystemNodes) {
                if (GalaxyActor) {
                    GalaxyActor->ReturnStarSystemToPool(Node);
                    SpawnedStarSystemNodes.Remove(Node);
                }
            }

            // NEW: Process each spawned star system for nearby entities
            for (TSharedPtr<FOctreeNode> StarSystemNode : SpawnedStarSystemNodes) {
                auto ssa = GalaxyActor->SpawnedStarSystems.Find(StarSystemNode);
                if (ssa) {
                    auto StarSystemActor = ssa->Get();
                    if (!StarSystemActor || StarSystemActor->InitializationState != ELifecycleState::Ready) continue;

                    FVector StarSystemSampleLocation = WorldLocation - StarSystemActor->GetActorLocation();
                    FInt64Vector StarSystemSampleCoordinate = FInt64Vector(
                        FMath::RoundToInt64(StarSystemSampleLocation.X),
                        FMath::RoundToInt64(StarSystemSampleLocation.Y),
                        FMath::RoundToInt64(StarSystemSampleLocation.Z)
                    );

                    // Query star system octree for nearby entities (planets, moons, debris, etc.)
                    auto NearbyEntityNodes = StarSystemActor->Octree->GetNodesByScreenSpace(
                        StarSystemSampleCoordinate,
                        ScanExtent,
                        .0001,
                        -1,
                        -1,
                        -1
                    );

                    if (DebugMode) {
                        for (const TSharedPtr<FOctreeNode>& Node : NearbyEntityNodes)
                        {
                            if (Node.IsValid())
                            {
                                DebugDrawSystemEntityNode(
                                    StarSystemActor->GetActorLocation() + FVector(Node->Center),
                                    Node
                                );
                            }
                        }
                    }

                    // Update the currently tracked entities and trigger lifecycle changes
                    TSet<TSharedPtr<FOctreeNode>> StaleEntityNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedEntityNodes);
                    for (const TSharedPtr<FOctreeNode>& Node : NearbyEntityNodes)
                    {
                        StaleEntityNodes.Remove(Node);
                        if (StarSystemActor)
                        {
                            if (!SpawnedEntityNodes.Contains(Node))
                            {
                                SpawnedEntityNodes.Add(Node);
                                // This will spawn actual interactable world objects
                                StarSystemActor->SpawnEntityFromPool(Node);
                            }
                        }
                    }

                    for (const TSharedPtr<FOctreeNode>& Node : StaleEntityNodes) {
                        if (StarSystemActor) {
                            StarSystemActor->ReturnEntityToPool(Node);
                            SpawnedEntityNodes.Remove(Node);
                        }
                    }
                }
            }
        }
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

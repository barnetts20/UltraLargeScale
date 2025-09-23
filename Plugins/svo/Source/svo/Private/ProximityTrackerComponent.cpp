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
	TArray<TSharedPtr<FOctreeNode>> NearbyNodes = UniverseActor->Octree->GetNodesInRange(SampleCoordinate, ScanExtent, -1, -1, 1);

    //Draw the node bounding boxes in debug mode
    if (DebugMode) {
        for (const TSharedPtr<FOctreeNode>& Node : NearbyNodes)
        {
            if (Node.IsValid())
            {
                DebugDrawNode(Node);
            }
        }
    }
    
    //Update the currently tracked galaxies and trigger lifecycle changes
    TSet<TSharedPtr<FOctreeNode>> StaleGalaxyNodes = TSet<TSharedPtr<FOctreeNode>>(SpawnedGalaxyNodes);
    for (const TSharedPtr<FOctreeNode>& Node : NearbyNodes)
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
#pragma endregion

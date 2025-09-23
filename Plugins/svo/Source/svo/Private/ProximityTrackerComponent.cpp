
#include "ProximityTrackerComponent.h"
#include <Engine.h>

// Sets default values for this component's properties
UProximityTrackerComponent::UProximityTrackerComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}

void UProximityTrackerComponent::OnProximityUpdate()
{
	if (!UniverseActor) return;

	// Get current position in universe space
	AActor* Owner = GetOwner();
	if (!Owner) return;
    if (!UniverseActor || UniverseActor->InitializationState != ELifecycleState::Ready) return;
	FVector WorldLocation = Owner->GetActorLocation();
	FVector SampleLocation = WorldLocation - UniverseActor->GetActorLocation();
    FInt64Vector SampleCoordinate = FInt64Vector(FMath::RoundToInt64(SampleLocation.X), FMath::RoundToInt64(SampleLocation.Y), FMath::RoundToInt64(SampleLocation.Z));

	// Perform query
	TArray<TSharedPtr<FOctreeNode>> NearbyNodes = UniverseActor->Octree->GetNodesInRange(SampleCoordinate, ScanExtent, -1, -1, 1);

	// Example processing
    bool bDebugDraw = false;
    if (bDebugDraw) {
        for (const TSharedPtr<FOctreeNode>& Node : NearbyNodes)
        {
            if (Node.IsValid())
            {
                DebugDrawNode(Node);
            }
        }
    }
    
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

void UProximityTrackerComponent::BeginPlay()
{
    Super::BeginPlay();

    // Find the single instance of AUniverseActor in the world.
    UniverseActor = Cast<AUniverseActor>(UGameplayStatics::GetActorOfClass(GetWorld(), AUniverseActor::StaticClass()));

    // It's good practice to check if the actor was actually found before using it.
    if (UniverseActor)
    {
        // Start the timer only if the UniverseActor is valid.
        GetWorld()->GetTimerManager().SetTimer(
            UpdateTimerHandle,
            this,
            &UProximityTrackerComponent::OnProximityUpdate,
            UpdateInterval,
            true);
    }
    else
    {
        // Log an error so you know if something is wrong with your level setup.
        UE_LOG(LogTemp, Error, TEXT("ProximityTrackerComponent: AUniverseActor not found in the world!"));
    }
}

void UProximityTrackerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}


// ParallaxNiagaraSystem.cpp
#include "ParallaxNiagaraSystem.h"

#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "Async/Async.h"
#include "SectorActor.h"
#include "DataTypes.h"

// -----------------------------------------------------------------------------
// UParallaxNiagaraSystem base
// -----------------------------------------------------------------------------

void UParallaxNiagaraSystem::ApplyParallax(const FVector& InParallaxOffset, const FVector& InCurrentPlayerPos)
{
	if (!NiagaraComponent)
	{
		return;
	}

	NiagaraComponent->SetVectorParameter(FName("User.ParallaxOffset"), InParallaxOffset);
	NiagaraComponent->SetWorldLocation(InCurrentPlayerPos);
}

void UParallaxNiagaraSystem::Shutdown()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Deactivate();
		NiagaraComponent->DestroyComponent();
		NiagaraComponent = nullptr;
	}
}

// -----------------------------------------------------------------------------
// UClusterNiagaraSystem
// -----------------------------------------------------------------------------

void UClusterNiagaraSystem::ConfigureFromPointNodes(
	const TArray<TSharedPtr<FOctreeNode>>& InPointNodes,
	double InSectorExtent)
{
	SectorExtent = InSectorExtent;

	const int32 Num = InPointNodes.Num();
	Positions.SetNumUninitialized(Num);
	Rotations.SetNumUninitialized(Num);
	Extents.SetNumUninitialized(Num);
	Colors.SetNumUninitialized(Num);

	// Mirror of the per-node array build that used to live inline in
	// ASectorActor::InitializeData. Per-point rotation is derived from a
	// random stream seeded on ObjectId so the rotation is stable across runs.
	ParallelFor(Num, [&](int32 Index)
		{
			const TSharedPtr<FOctreeNode>& Node = InPointNodes[Index];
			FRandomStream RandStream(Node->Data.ObjectId);

			Positions[Index] = Node->Center;
			Rotations[Index] = RandStream.GetUnitVector();
			Extents[Index] = static_cast<float>(Node->Extent * (1.0 + Node->Data.ScaleFactor));
			Colors[Index] = FLinearColor(Node->Data.Composition);
		}, EParallelForFlags::BackgroundPriority);
}

TFuture<void> UClusterNiagaraSystem::Initialize(
	ASectorActor* InOwner,
	USceneComponent* InOwnerRoot,
	UNiagaraSystem* InTemplate,
	const FVector& InInitialPlayerPos)
{
	OwnerSector = InOwner;

	TPromise<void> CompletionPromise;
	TFuture<void> CompletionFuture = CompletionPromise.GetFuture();

	// Async captures use TWeakObjectPtr instead of raw `this`. If PIE ends
	// mid-init (or the actor is pooled/destroyed), the weak ptr goes null
	// and the continuation bails cleanly instead of dereferencing a dying
	// UObject. Raw-this captures in nested AsyncTasks are the canonical
	// cause of FReferenceChainSearch stale-reference warnings at PIE end.
	TWeakObjectPtr<UClusterNiagaraSystem> WeakThis(this);
	TWeakObjectPtr<USceneComponent> WeakRoot(InOwnerRoot);
	TWeakObjectPtr<UNiagaraSystem> WeakTemplate(InTemplate);

	// Phase 1 (game thread): spawn the component attached to the sector root.
	AsyncTask(ENamedThreads::GameThread,
		[WeakThis, WeakRoot, WeakTemplate, InInitialPlayerPos,
		CompletionPromise = MoveTemp(CompletionPromise)]() mutable
		{
			UClusterNiagaraSystem* Self = WeakThis.Get();
			USceneComponent* Root = WeakRoot.Get();
			UNiagaraSystem* Template = WeakTemplate.Get();
			if (!Self || !Root || !Template)
			{
				CompletionPromise.SetValue();
				return;
			}

			Self->NiagaraComponent = UNiagaraFunctionLibrary::SpawnSystemAttached(
				Template,
				Root,
				NAME_None,
				FVector::ZeroVector,
				FRotator::ZeroRotator,
				EAttachLocation::SnapToTarget,
				/*bAutoDestroy=*/ true,
				/*bAutoActivate=*/ false
			);

			if (!Self->NiagaraComponent)
			{
				CompletionPromise.SetValue();
				return;
			}

			Self->NiagaraComponent->SetSystemFixedBounds(
				FBox(FVector(-Self->SectorExtent), FVector(Self->SectorExtent)));
			Self->NiagaraComponent->TranslucencySortPriority = 0;
			Self->NiagaraComponent->SetWorldLocation(InInitialPlayerPos);

			// Phase 2 (background): prepare relative positions + push arrays.
			// Particles live in sector-relative space, so positions get offset by
			// the initial player position before being pushed to Niagara.
			AsyncTask(ENamedThreads::AnyBackgroundHiPriTask,
				[WeakThis, InInitialPlayerPos,
				CompletionPromise = MoveTemp(CompletionPromise)]() mutable
				{
					UClusterNiagaraSystem* Self = WeakThis.Get();
					if (!Self || !Self->NiagaraComponent)
					{
						CompletionPromise.SetValue();
						return;
					}

					TArray<FVector> RelativePositions;
					const int32 Num = Self->Positions.Num();
					RelativePositions.SetNumUninitialized(Num);
					ParallelFor(Num, [&](int32 i)
						{
							RelativePositions[i] = Self->Positions[i] - InInitialPlayerPos;
						}, EParallelForFlags::BackgroundPriority);

					UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
						Self->NiagaraComponent, FName("User.Positions"), RelativePositions);
					UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
						Self->NiagaraComponent, FName("User.Rotations"), Self->Rotations);
					UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
						Self->NiagaraComponent, FName("User.Colors"), Self->Colors);
					UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
						Self->NiagaraComponent, FName("User.Extents"), Self->Extents);

					// Phase 3 (game thread): activate.
					AsyncTask(ENamedThreads::GameThread,
						[WeakThis, CompletionPromise = MoveTemp(CompletionPromise)]() mutable
						{
							UClusterNiagaraSystem* Self = WeakThis.Get();
							if (Self && Self->NiagaraComponent)
							{
								Self->NiagaraComponent->Activate(true);
							}
							CompletionPromise.SetValue();
						});
				});
		});

	return CompletionFuture;
}
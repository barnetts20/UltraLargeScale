// NiagaraParticleBuffer.h
#pragma once
#include "CoreMinimal.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

// Names of the Niagara user-exposed arrays this buffer knows how to push to.
// Centralized here so a rename in the material is one edit, not a grep.
namespace NiagaraBufferParams
{
    static const FName Positions = TEXT("User.Positions");
    static const FName Extents = TEXT("User.Extents");
    static const FName Colors = TEXT("User.Colors");
    static const FName Rotations = TEXT("User.Rotations");
}

// A slot-packed, double-buffer-friendly array of particle data for a single
// Niagara component. Arrays are optional � only allocate what the tier needs.
// Dead particles are written with Extent == 0 and an off-screen DeadPos so
// Niagara culls them regardless of which arrays are active.
//
// Slot packing convention: slot S owns indices [S * SlotCapacity, (S+1) * SlotCapacity).
// The SlotCapacity is fixed at Allocate time and shared across all arrays.
struct FNiagaraParticleBuffer
{
    // --- Optional arrays. Each is empty if not allocated. ---
    TArray<FVector>         Positions;
    TArray<float>           Extents;
    TArray<FLinearColor>    Colors;
    TArray<FVector>         Rotations;  // Face normals for non-billboard rendering

    // Persistent scratch buffer for MakeRelativePositions. Avoids allocating
    // and discarding a large TArray on every PushToNiagara call.
    mutable TArray<FVector> RelativePositionsScratch;

    // Slot geometry � set once in Allocate, read-only after.
    int32 TotalSlots = 0;
    int32 SlotCapacity = 0;

    // --- Lifecycle ---

    // Allocate (or reallocate) all active arrays. Pass bWantRotations=false for
    // tiers that don't need face-normal data � saves the alloc and push cost.
    void Allocate(int32 InTotalSlots, int32 InSlotCapacity, bool bWantRotations = false)
    {
        TotalSlots = InTotalSlots;
        SlotCapacity = InSlotCapacity;
        const int32 Total = TotalSlots * SlotCapacity;

        Positions.SetNumZeroed(Total);
        Extents.SetNumZeroed(Total);
        Colors.SetNumZeroed(Total);
        RelativePositionsScratch.SetNumUninitialized(Total);

        if (bWantRotations)
            Rotations.SetNumZeroed(Total);
        else
            Rotations.Empty();
    }

    // Deep copy from another buffer. Only copies arrays that are allocated in
    // this buffer. Used for initial front-to-back mirroring at init time.
    void CopyFrom(const FNiagaraParticleBuffer& Other)
    {
        Positions = Other.Positions;
        Extents = Other.Extents;
        Colors = Other.Colors;
        if (Rotations.Num() > 0 && Other.Rotations.Num() > 0)
            Rotations = Other.Rotations;
    }

    // Swap array storage with another buffer. O(1) pointer swap instead of
    // O(N) memcpy. Use for the front/back double-buffer exchange in
    // UpdateTier where the old front becomes the new back baseline.
    // Both buffers must have the same allocation shape (same TotalSlots,
    // SlotCapacity, and Rotations presence).
    void SwapWith(FNiagaraParticleBuffer& Other)
    {
        Swap(Positions, Other.Positions);
        Swap(Extents, Other.Extents);
        Swap(Colors, Other.Colors);
        if (Rotations.Num() > 0 && Other.Rotations.Num() > 0)
            Swap(Rotations, Other.Rotations);
    }

    // --- Slot helpers ---

    int32 SlotStart(int32 SlotIndex) const { return SlotIndex * SlotCapacity; }

    // Write one dead particle entry at absolute index Idx.
    void WriteDeadParticle(int32 Idx, const FVector& DeadPos)
    {
        if (Positions.IsValidIndex(Idx)) Positions[Idx] = DeadPos;
        if (Extents.IsValidIndex(Idx))   Extents[Idx] = 0.0f;
        if (Colors.IsValidIndex(Idx))    Colors[Idx] = FLinearColor::Black;
        if (Rotations.IsValidIndex(Idx)) Rotations[Idx] = FVector::ZeroVector;
    }

    // Zero out an entire slot (used when a cell exits the streaming window).
    void ClearSlot(int32 SlotIndex, const FVector& DeadPos)
    {
        const int32 Start = SlotStart(SlotIndex);
        for (int32 i = 0; i < SlotCapacity; ++i)
            WriteDeadParticle(Start + i, DeadPos);
    }

    // Fill trailing dead particles after ActualCount accepted particles.
    void PadSlotDead(int32 SlotIndex, int32 ActualCount, const FVector& DeadPos)
    {
        const int32 Start = SlotStart(SlotIndex);
        for (int32 i = ActualCount; i < SlotCapacity; ++i)
            WriteDeadParticle(Start + i, DeadPos);
    }

    // --- Push helpers ---

    // Build a camera-relative position array into the persistent scratch
    // buffer. Dead particles (Extent == 0) get ZeroVector so they collapse
    // to the camera origin and are culled by the Niagara material, rather
    // than appearing at the parked DeadPos far off-screen.
    // Returns a const reference to the internal scratch buffer.
    const TArray<FVector>& MakeRelativePositions(const FVector& VirtualTraversal) const
    {
        const int32 Num = Positions.Num();
        if (RelativePositionsScratch.Num() != Num)
            RelativePositionsScratch.SetNumUninitialized(Num);
        for (int32 i = 0; i < Num; ++i)
        {
            RelativePositionsScratch[i] = (Extents[i] > 0.0f)
                ? (Positions[i] - VirtualTraversal)
                : FVector::ZeroVector;
        }
        return RelativePositionsScratch;
    }

    // Push all allocated arrays to a Niagara component. Relative positions are
    // computed here so the caller doesn't need to manage the intermediate array.
    // Does NOT call Activate — particle IDs are stable for the system lifetime;
    // the Niagara scratch-pad reads the updated arrays each tick automatically.
    void PushToNiagara(UNiagaraComponent* Component, const FVector& VirtualTraversal) const
    {
        if (!Component) return;

        const TArray<FVector>& RelPos = MakeRelativePositions(VirtualTraversal);

        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
            Component, NiagaraBufferParams::Positions, RelPos);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(
            Component, NiagaraBufferParams::Extents, Extents);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(
            Component, NiagaraBufferParams::Colors, Colors);

        if (Rotations.Num() > 0)
        {
            UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
                Component, NiagaraBufferParams::Rotations, Rotations);
        }
    }

    // Called exactly once at tier init. Pushes the full dead-particle buffer
    // (all slots zeroed) so Niagara spawns TotalSlots * SlotCapacity particles
    // with stable IDs, then activates the system. Never called again — all
    // subsequent updates go through PushToNiagara which only writes data.
    void ActivateOnce(UNiagaraComponent* Component, const FVector& VirtualTraversal) const
    {
        if (!Component) return;
        PushToNiagara(Component, VirtualTraversal);
        Component->Activate(true);
    }
};
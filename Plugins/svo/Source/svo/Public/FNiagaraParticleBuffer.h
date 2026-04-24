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
// Niagara component. Arrays are optional — only allocate what the tier needs.
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

    // Slot geometry — set once in Allocate, read-only after.
    int32 TotalSlots = 0;
    int32 SlotCapacity = 0;

    // --- Lifecycle ---

    // Allocate (or reallocate) all active arrays. Pass bWantRotations=false for
    // tiers that don't need face-normal data — saves the alloc and push cost.
    void Allocate(int32 InTotalSlots, int32 InSlotCapacity, bool bWantRotations = false)
    {
        TotalSlots = InTotalSlots;
        SlotCapacity = InSlotCapacity;
        const int32 Total = TotalSlots * SlotCapacity;

        Positions.SetNumZeroed(Total);
        Extents.SetNumZeroed(Total);
        Colors.SetNumZeroed(Total);

        if (bWantRotations)
            Rotations.SetNumZeroed(Total);
        else
            Rotations.Empty();
    }

    // Deep copy from another buffer. Only copies arrays that are allocated in
    // this buffer — if we didn't allocate Rotations, we don't copy them.
    void CopyFrom(const FNiagaraParticleBuffer& Other)
    {
        Positions = Other.Positions;
        Extents = Other.Extents;
        Colors = Other.Colors;
        if (Rotations.Num() > 0 && Other.Rotations.Num() > 0)
            Rotations = Other.Rotations;
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

    // Build a camera-relative position array. Dead particles (Extent == 0)
    // get ZeroVector rather than their parked DeadPos — on ReinitializeSystem
    // they'd otherwise drift in from off-screen.
    TArray<FVector> MakeRelativePositions(const FVector& VirtualTraversal) const
    {
        TArray<FVector> Out;
        Out.SetNumUninitialized(Positions.Num());
        for (int32 i = 0; i < Positions.Num(); ++i)
        {
            Out[i] = (Extents[i] > 0.0f)
                ? (Positions[i] - VirtualTraversal)
                : FVector::ZeroVector;
        }
        return Out;
    }

    // Push all allocated arrays to a Niagara component. Relative positions are
    // computed here so the caller doesn't need to manage the intermediate array.
    // Call after a buffer swap and before ReinitializeSystem.
    void PushToNiagara(UNiagaraComponent* Component, const FVector& VirtualTraversal) const
    {
        if (!Component) return;

        TArray<FVector> RelPos = MakeRelativePositions(VirtualTraversal);

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

        Component->Activate(true);
    }
};
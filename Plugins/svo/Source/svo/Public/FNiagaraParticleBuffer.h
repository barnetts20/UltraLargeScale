// NiagaraParticleBuffer.h
#pragma once
#include "CoreMinimal.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

// Names of the Niagara user-exposed arrays this buffer knows how to push to.
// Centralized here so a rename in the material is one edit, not a grep.
namespace NiagaraBufferParams
{
    inline const FName Positions = TEXT("User.Positions");
    inline const FName Extents = TEXT("User.Extents");
    inline const FName Colors = TEXT("User.Colors");
    inline const FName Rotations = TEXT("User.Rotations");

    // Cell-anchored VT path: per-slot (VirtualTraversal - cell center), plus the
    // scalar slot capacity the graph divides particle index by to find its cell.
    inline const FName CellRelativeVT = TEXT("User.CellRelativeVT");
    inline const FName SlotCapacity = TEXT("User.SlotCapacity");
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

    // --- Cell-anchored VT path (opt-in per tier via bCellAnchored) ---
    // Per-SLOT cell center in the actor's absolute virtual space (one entry per
    // slot, not per particle). Written when a cell is assigned to a slot and
    // carried across the double buffer by CopyFrom/SwapWith so it always matches
    // the live positions. Used to fold positions to cell-local at push time and
    // to compute the per-frame (VT - center) array.
    TArray<FVector> SlotCenters;

    // Persistent scratch for the per-slot (VT - center) push. Size == TotalSlots.
    mutable TArray<FVector> CellRelVTScratch;

    // When true this buffer uses the cell-anchored GPU compositing path:
    // cell-local positions pushed once per generation, and only a small per-slot
    // (VT - center) array pushed per frame. Set at Allocate time from
    // FParticleTierConfig::bUseCellAnchoredVT. Legacy path used when false.
    bool bCellAnchored = false;

    // Slot geometry � set once in Allocate, read-only after.
    int32 TotalSlots = 0;
    int32 SlotCapacity = 0;

    // Largest particle extent in this buffer. Updated by RecomputeMaxExtent
    // on the async thread after generation/cache-restore completes. Read by
    // PushTierToNiagara on the game thread to expand Niagara fixed bounds.
    float MaxExtent = 0.f;

    // Scans the Extents array and caches the result in MaxExtent.
    // Called on the async thread at the end of UpdateTier, after all
    // generation and cache restores are done.
    void RecomputeMaxExtent()
    {
        float Max = 0.f;
        for (int32 i = 0; i < Extents.Num(); ++i)
            Max = FMath::Max(Max, Extents[i]);
        MaxExtent = Max;
    }

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
        SlotCenters.SetNumZeroed(TotalSlots);
        CellRelVTScratch.SetNumUninitialized(TotalSlots);

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
        MaxExtent = Other.MaxExtent;
        SlotCenters = Other.SlotCenters;
        bCellAnchored = Other.bCellAnchored;
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
        Swap(SlotCenters, Other.SlotCenters);
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

    // Cell-anchored: fold absolute positions to cell-local (Position - SlotCenter)
    // into the scratch buffer. Camera-INDEPENDENT, so this is pushed once per
    // generation, not per frame. Dead particles collapse to ZeroVector (culled by
    // Extent==0). Double subtraction happens here before the array is narrowed to
    // float on upload; the per-cell anchor keeps operands small so there is no
    // catastrophic cancellation regardless of how large the virtual coords are.
    const TArray<FVector>& MakeCellLocalPositions() const
    {
        const int32 Num = Positions.Num();
        if (RelativePositionsScratch.Num() != Num)
            RelativePositionsScratch.SetNumUninitialized(Num);
        const int32 Cap = FMath::Max(SlotCapacity, 1);
        for (int32 i = 0; i < Num; ++i)
        {
            if (Extents[i] > 0.0f)
            {
                const int32 Slot = i / Cap;
                const FVector Center = SlotCenters.IsValidIndex(Slot)
                    ? SlotCenters[Slot] : FVector::ZeroVector;
                RelativePositionsScratch[i] = Positions[i] - Center;
            }
            else
            {
                RelativePositionsScratch[i] = FVector::ZeroVector;
            }
        }
        return RelativePositionsScratch;
    }

    // Cell-anchored: build the per-slot (VirtualTraversal - SlotCenter) array,
    // one entry per slot. Double subtraction, narrowed to float on upload. This
    // is the ENTIRE per-frame push - TotalSlots entries, not N particles.
    const TArray<FVector>& MakeCellRelativeVT(const FVector& VirtualTraversal) const
    {
        if (CellRelVTScratch.Num() != TotalSlots)
            CellRelVTScratch.SetNumUninitialized(TotalSlots);
        for (int32 s = 0; s < TotalSlots; ++s)
            CellRelVTScratch[s] = SlotCenters[s] - VirtualTraversal;
        return CellRelVTScratch;
    }

    // Cell-anchored per-frame push: upload only the per-slot (VT - center) array.
    // Reads front-buffer data only, so it is safe to call from a worker thread.
    void PushCellRelativeVT(UNiagaraComponent* Component, const FVector& VirtualTraversal) const
    {
        if (!Component) return;
        const TArray<FVector>& RelVT = MakeCellRelativeVT(VirtualTraversal);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(
            Component, NiagaraBufferParams::CellRelativeVT, RelVT);
    }

    // Push all allocated arrays to a Niagara component. Relative positions are
    // computed here so the caller doesn't need to manage the intermediate array.
    // Does NOT call Activate — particle IDs are stable for the system lifetime;
    // the Niagara scratch-pad reads the updated arrays each tick automatically.
    void PushToNiagara(UNiagaraComponent* Component, const FVector& VirtualTraversal) const
    {
        if (!Component) return;

        // Positions: cell-anchored pushes STATIC cell-local positions (once per
        // generation); legacy pushes camera-relative positions.
        if (bCellAnchored)
        {
            const TArray<FVector>& CellLocal = MakeCellLocalPositions();
            UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
                Component, NiagaraBufferParams::Positions, CellLocal);

            // Scalar the graph divides particle index by to recover slot -> cell.
            Component->SetVariableInt(NiagaraBufferParams::SlotCapacity, SlotCapacity);

            // Seed the per-slot (VT - center) array so the first frame is correct
            // before the per-frame push runs.
            PushCellRelativeVT(Component, VirtualTraversal);
        }
        else
        {
            const TArray<FVector>& RelPos = MakeRelativePositions(VirtualTraversal);
            UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
                Component, NiagaraBufferParams::Positions, RelPos);
        }

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
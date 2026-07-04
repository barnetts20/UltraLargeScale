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

    // Cell-anchored toroidal path. The graph reconstructs
    //   worldRel = Positions[i] + CellRelativeVT[slot] + NCenterMinusVT
    //   slot     = i / SlotCapacity
    //
    // CellRelativeVT: NAME KEPT from the previous (VT-inclusive) scheme to
    // minimize the Niagara/Blueprint diff — it now carries the VT-FREE lattice
    // offset (SlotCenter - NeighborhoodCenter) per slot, updated only on
    // boundary crosses. NCenterMinusVT is the single per-frame FVector uniform
    // (NeighborhoodCenter - VirtualTraversal).
    inline const FName CellRelativeVT = TEXT("User.CellRelativeVT");
    inline const FName NCenterMinusVT = TEXT("User.NCenterMinusVT");
    inline const FName SlotCapacity = TEXT("User.SlotCapacity");
}

// A slot-packed array of particle data for a single
// Niagara component. Arrays are optional — only allocate what the tier needs.
// Dead particles are written with Extent == 0 (zeroed position); the
// Niagara graph culls on Extent, and every CPU consumer skips Extent <= 0.
//
// Slot packing convention: slot S owns indices [S * SlotCapacity, (S+1) * SlotCapacity).
// The SlotCapacity is fixed at Allocate time and shared across all arrays.
//
// Toroidal (modular) slot addressing: slot indices are derived from the cell
// coordinate residues (see FTierStreamingSystem::SlotOf), so a resident cell
// keeps its slot — and its particle data — across boundary crosses. SlotCoord
// records which cell currently occupies each slot and is the per-slot source
// of truth; SlotCenters is its derived FVector cache (kept because this struct
// has no grid parameters — the streaming system writes both together).
struct FNiagaraParticleBuffer
{
    // --- Optional arrays. Each is empty if not allocated. ---
    TArray<FVector>         Positions;
    TArray<float>           Extents;
    TArray<FLinearColor>    Colors;
    TArray<FVector>         Rotations;  // Face normals for non-billboard rendering

    // Persistent scratch buffer for MakeCellLocalPositions. Avoids allocating
    // and discarding a large TArray on every transition push.
    mutable TArray<FVector> CellLocalScratch;

    // --- Cell-anchored VT path ---

    // Per-SLOT grid coordinate of the cell currently resident in that slot
    // (one entry per slot, not per particle). EmptySlotCoord() = unoccupied.
    // Written by the streaming system when a cell (re)generates into a slot,
    // always together with SlotCenters and the slot's particle data, so the
    // coord identity always matches the live positions.
    TArray<FIntVector> SlotCoord;

    // Per-SLOT cell center in the actor's absolute virtual space. DERIVED
    // cache of GridCoordToCenter(SlotCoord[s]) — this struct has no grid
    // params, so the streaming system computes it and writes it alongside
    // SlotCoord. Used to fold positions to cell-local at push time and to
    // derive the per-slot lattice.
    TArray<FVector> SlotCenters;

    // Persistent scratch for the per-slot lattice push. Size == TotalSlots.
    mutable TArray<FVector> LatticeScratch;


    // Slot geometry — set once in Allocate, read-only after.
    int32 TotalSlots = 0;
    int32 SlotCapacity = 0;

    // Largest particle extent in this buffer. Updated by RecomputeMaxExtent
    // on the async thread after generation/cache-restore completes. Read by
    // PushTierToNiagara on the game thread to expand Niagara fixed bounds.
    float MaxExtent = 0.f;

    // Sentinel for an unoccupied slot.
    static FIntVector EmptySlotCoord() { return FIntVector(INT32_MIN, INT32_MIN, INT32_MIN); }

    bool IsSlotOccupied(int32 SlotIndex) const
    {
        return SlotCoord.IsValidIndex(SlotIndex) && SlotCoord[SlotIndex].X != INT32_MIN;
    }

    // Full scan of the Extents array, cached in MaxExtent. Init-time only:
    // transitions grow MaxExtent incrementally from the entering plane via
    // SlotMaxExtent, since bounds are grow-only and never need it to shrink.
    void RecomputeMaxExtent()
    {
        float Max = 0.f;
        for (int32 i = 0; i < Extents.Num(); ++i)
            Max = FMath::Max(Max, Extents[i]);
        MaxExtent = Max;
    }

    // Largest extent within one slot's range. Lets the transition path grow
    // MaxExtent from just the entering plane instead of rescanning the
    // whole buffer.
    float SlotMaxExtent(int32 SlotIndex) const
    {
        const int32 Start = SlotStart(SlotIndex);
        float Max = 0.f;
        for (int32 i = 0; i < SlotCapacity; ++i)
            Max = FMath::Max(Max, Extents[Start + i]);
        return Max;
    }

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
        CellLocalScratch.SetNumUninitialized(Total);
        SlotCoord.Init(EmptySlotCoord(), TotalSlots);
        SlotCenters.SetNumZeroed(TotalSlots);
        LatticeScratch.SetNumUninitialized(TotalSlots);

        if (bWantRotations)
            Rotations.SetNumZeroed(Total);
        else
            Rotations.Empty();
    }

    // --- Slot helpers ---

    int32 SlotStart(int32 SlotIndex) const { return SlotIndex * SlotCapacity; }

    // Write one dead particle entry at absolute index Idx. Extent == 0 is
    // the liveness flag every consumer keys on (graph cull, octree skip,
    // cache skip); the position value is irrelevant for dead entries and is
    // simply zeroed.
    void WriteDeadParticle(int32 Idx)
    {
        if (Positions.IsValidIndex(Idx)) Positions[Idx] = FVector::ZeroVector;
        if (Extents.IsValidIndex(Idx))   Extents[Idx] = 0.0f;
        if (Colors.IsValidIndex(Idx))    Colors[Idx] = FLinearColor::Black;
        if (Rotations.IsValidIndex(Idx)) Rotations[Idx] = FVector::ZeroVector;
    }

    // Fill trailing dead particles after ActualCount accepted particles.
    void PadSlotDead(int32 SlotIndex, int32 ActualCount)
    {
        const int32 Start = SlotStart(SlotIndex);
        for (int32 i = ActualCount; i < SlotCapacity; ++i)
            WriteDeadParticle(Start + i);
    }

    // --- Push helpers ---

    // Cell-anchored: fold absolute positions to cell-local (Position - SlotCenter)
    // into the scratch buffer. Camera-INDEPENDENT and center-INDEPENDENT, so this
    // is pushed only when a slot's data changes (boundary cross), never per frame.
    // Dead particles collapse to ZeroVector (culled by Extent==0). Double
    // subtraction happens here before the array is narrowed to float on upload;
    // the per-cell anchor keeps operands small so there is no catastrophic
    // cancellation regardless of how large the virtual coords are.
    const TArray<FVector>& MakeCellLocalPositions() const
    {
        const int32 Num = Positions.Num();
        if (CellLocalScratch.Num() != Num)
            CellLocalScratch.SetNumUninitialized(Num);
        // Slot-major: hoists the center load per slot and drops the per-
        // particle divide + bounds check.
        for (int32 Slot = 0; Slot < TotalSlots; ++Slot)
        {
            const FVector Center = SlotCenters[Slot];
            const int32 Start = SlotStart(Slot);
            for (int32 i = Start; i < Start + SlotCapacity; ++i)
            {
                CellLocalScratch[i] = (Extents[i] > 0.0f)
                    ? (Positions[i] - Center)
                    : FVector::ZeroVector;
            }
        }
        return CellLocalScratch;
    }

    // Cell-anchored: build the per-slot VT-FREE lattice array,
    //   lattice[s] = SlotCenters[s] - NCenter,
    // one entry per slot, where NCenter is the neighborhood-center cell's
    // center (GridCoordToCenter of the stamped center coord). Both operands
    // are exact multiples of CellSize, so the subtraction is exact in double
    // and small in magnitude (at most Radius * CellSize per axis) before the
    // float narrowing on upload. The center cell's lattice is exactly zero —
    // this is what maximizes float precision nearest the camera. Unoccupied
    // slots emit ZeroVector (their particles are dead and culled).
    // Changes ONLY on boundary crosses, never per frame.
    const TArray<FVector>& MakeLattice(const FVector& NCenter) const
    {
        if (LatticeScratch.Num() != TotalSlots)
            LatticeScratch.SetNumUninitialized(TotalSlots);
        for (int32 s = 0; s < TotalSlots; ++s)
        {
            LatticeScratch[s] = IsSlotOccupied(s)
                ? (SlotCenters[s] - NCenter)
                : FVector::ZeroVector;
        }
        return LatticeScratch;
    }

    // Cell-anchored boundary-cross push: upload the per-slot lattice into the
    // (name-preserved) User.CellRelativeVT array. TotalSlots entries — tiny.
    void PushSlotOffsets(UNiagaraComponent* Component, const FVector& NCenter) const
    {
        if (!Component) return;
        const TArray<FVector>& Lattice = MakeLattice(NCenter);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(
            Component, NiagaraBufferParams::CellRelativeVT, Lattice);
    }

    // Cell-anchored per-frame push: the ENTIRE per-frame cost — one FVector
    // uniform, (NCenter - VirtualTraversal). NCenter MUST derive from the same
    // stamped center coord the live lattice was built against (see
    // FParticleTierState::StampedNCenter) so uniform and lattice can never
    // disagree, even on a transition frame.
    void PushNCenterMinusVT(UNiagaraComponent* Component,
        const FVector& NCenter, const FVector& VirtualTraversal) const
    {
        if (!Component) return;
        Component->SetVariableVec3(NiagaraBufferParams::NCenterMinusVT,
            NCenter - VirtualTraversal);
    }

    // Push all allocated arrays to a Niagara component. NCenter is the
    // neighborhood-center position the lattice/uniform are built against.
    // Does NOT call Activate — particle IDs are stable for the system
    // lifetime; the Niagara scratch-pad reads the updated arrays each tick
    // automatically.
    void PushToNiagara(UNiagaraComponent* Component, const FVector& VirtualTraversal,
        const FVector& NCenter = FVector::ZeroVector) const
    {
        if (!Component) return;

        // Positions: cell-anchored pushes STATIC cell-local positions (changed
        // Positions are STATIC cell-local values — changed slots only ever
        // change on boundary crosses. The whole-array re-upload at transition
        // frequency is accepted for v1 (partial upload is the custom-DI
        // capstone, tracked separately).
        const TArray<FVector>& CellLocal = MakeCellLocalPositions();
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(
            Component, NiagaraBufferParams::Positions, CellLocal);

        // Scalar the graph divides particle index by to recover slot -> cell.
        Component->SetVariableInt(NiagaraBufferParams::SlotCapacity, SlotCapacity);

        // Lattice + uniform, both derived from the SAME NCenter, so the
        // first frame after this push reconstructs consistently before the
        // per-frame push runs.
        PushSlotOffsets(Component, NCenter);
        PushNCenterMinusVT(Component, NCenter, VirtualTraversal);

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
    void ActivateOnce(UNiagaraComponent* Component, const FVector& VirtualTraversal,
        const FVector& NCenter = FVector::ZeroVector) const
    {
        if (!Component) return;
        PushToNiagara(Component, VirtualTraversal, NCenter);
        Component->Activate(true);
    }
};
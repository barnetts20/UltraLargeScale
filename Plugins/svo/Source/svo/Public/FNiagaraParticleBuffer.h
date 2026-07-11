#pragma once
#include "CoreMinimal.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

// Names of the Niagara user-exposed arrays this buffer knows how to push to.
namespace NiagaraBufferParams
{
    inline const FName Positions = TEXT("User.Positions");
    inline const FName Extents = TEXT("User.Extents");
    inline const FName Colors = TEXT("User.Colors");
    inline const FName Rotations = TEXT("User.Rotations");

    // Cell-anchored toroidal path. The graph reconstructs
    //   worldRel = Positions[i] + CellOffsets[slot] + NCenterMinusVT
    //   slot     = i / SlotCapacity
    inline const FName CellOffsets = TEXT("User.CellOffsets");
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
    TArray<FVector>         Positions;
    TArray<float>           Extents;
    TArray<FLinearColor>    Colors;
    // Face normals for non-billboard rendering. Need to enable within the particle system as well if desired.
    TArray<FVector>         Rotations;

    // Per-SLOT grid coordinate of the cell currently resident in that slot
    TArray<FIntVector> SlotCoord;

    // Per-SLOT cell center in the actor's absolute virtual space.
    TArray<FVector> SlotCenters;

    // Persistent scratch buffer for MakeCellLocalPositions. Avoids allocating and discarding a large TArray on every transition push.
    mutable TArray<FVector> CellLocalScratch;
    // Persistent scratch for the per-slot lattice push. Size == TotalSlots.
    mutable TArray<FVector> LatticeScratch;

    // Slot geometry - set once in Allocate, read-only after.
    int32 TotalSlots = 0;
    int32 SlotCapacity = 0;

    static FIntVector EmptySlotCoord() { return FIntVector(INT32_MIN, INT32_MIN, INT32_MIN); }

    bool IsSlotOccupied(int32 SlotIndex) const
    {
        return SlotCoord.IsValidIndex(SlotIndex) && SlotCoord[SlotIndex].X != INT32_MIN;
    }

    // Allocate (or reallocate) all active arrays. Pass bWantRotations=false for tiers that don't need face-normal data — saves the alloc and push cost.
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

        if (bWantRotations) Rotations.SetNumZeroed(Total);
        else Rotations.Empty();
    }

    // --- Slot helpers ---
    int32 SlotStart(int32 SlotIndex) const { return SlotIndex * SlotCapacity; }

    // Write one dead particle entry at absolute index Idx.
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

    // Cell-anchored: fold absolute positions to cell-local (Position - SlotCenter)
    // into the scratch buffer. Camera-INDEPENDENT and center-INDEPENDENT, so this
    // is pushed only when a slot's data changes (boundary cross), never per frame.
    const TArray<FVector>& MakeCellLocalPositions() const
    {
        const int32 Num = Positions.Num();
        if (CellLocalScratch.Num() != Num) CellLocalScratch.SetNumUninitialized(Num);
        for (int32 Slot = 0; Slot < TotalSlots; ++Slot)
        {
            const FVector Center = SlotCenters[Slot];
            const int32 Start = SlotStart(Slot);
            for (int32 i = Start; i < Start + SlotCapacity; ++i)
            {
                CellLocalScratch[i] = (Extents[i] > 0.0f) ? (Positions[i] - Center) : FVector::ZeroVector;
            }
        }
        return CellLocalScratch;
    }

    // Cell-anchored: build the per-slot lattice array,
    //   lattice[s] = SlotCenters[s] - NCenter
    const TArray<FVector>& MakeLattice(const FVector& NCenter) const
    {
        if (LatticeScratch.Num() != TotalSlots) LatticeScratch.SetNumUninitialized(TotalSlots);
        for (int32 s = 0; s < TotalSlots; ++s)
        {
            LatticeScratch[s] = IsSlotOccupied(s) ? (SlotCenters[s] - NCenter) : FVector::ZeroVector;
        }
        return LatticeScratch;
    }

    // Cell-anchored boundary-cross push: upload the per-slot lattice into the User.CellOffsets array.
    void PushSlotOffsets(UNiagaraComponent* Component, const FVector& NCenter) const
    {
        if (!Component) return;
        const TArray<FVector>& Lattice = MakeLattice(NCenter);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(Component, NiagaraBufferParams::CellOffsets, Lattice);
    }

    // Cell-anchored per-frame push: the ENTIRE per-frame cost — one FVector
    // uniform, (NCenter - VirtualTraversal). NCenter MUST derive from the same
    // stamped center coord the live lattice was built against (see
    // FParticleTierState::StampedNCenter) so uniform and lattice can never
    // disagree, even on a transition frame.
    void PushNCenterMinusVT(UNiagaraComponent* Component, const FVector& NCenter, const FVector& VirtualTraversal) const
    {
        if (!Component) return;
        Component->SetVariableVec3(NiagaraBufferParams::NCenterMinusVT, NCenter - VirtualTraversal);
    }

    // Push all allocated arrays to a Niagara component. NCenter is the
    // neighborhood-center position the lattice/uniform are built against.
    void PushToNiagara(UNiagaraComponent* Component, const FVector& VirtualTraversal, const FVector& NCenter = FVector::ZeroVector) const
    {
        if (!Component) return;
        const TArray<FVector>& CellLocal = MakeCellLocalPositions();
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(Component, NiagaraBufferParams::Positions, CellLocal);
        Component->SetVariableInt(NiagaraBufferParams::SlotCapacity, SlotCapacity);
        PushSlotOffsets(Component, NCenter);
        PushNCenterMinusVT(Component, NCenter, VirtualTraversal);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(Component, NiagaraBufferParams::Extents, Extents);
        UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(Component, NiagaraBufferParams::Colors, Colors);
        if (Rotations.Num() > 0) UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(Component, NiagaraBufferParams::Rotations, Rotations);
    }

    // Called exactly once at tier init. Pushes the full dead-particle buffer.
    void ActivateOnce(UNiagaraComponent* Component, const FVector& VirtualTraversal, const FVector& NCenter = FVector::ZeroVector) const
    {
        if (!Component) return;
        PushToNiagara(Component, VirtualTraversal, NCenter);
        Component->Activate(true);
    }
};
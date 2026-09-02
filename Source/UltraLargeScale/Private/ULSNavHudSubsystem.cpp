// ULSNavHudSubsystem.cpp
#include "ULSNavHudSubsystem.h"

#include "ULSScaleLadder.h"
#include "UniverseActor.h"

#include "CanvasItem.h"
#include "CoreGlobals.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

namespace
{
	static TAutoConsoleVariable<int32> CVarNavHud(
		TEXT("uls.NavHud"),
		1,
		TEXT("Draw the UltraLargeScale navigation readout. 0 = off, 1 = on."),
		ECVF_Cheat);

	static TAutoConsoleVariable<float> CVarNavHudScale(
		TEXT("uls.NavHud.Scale"),
		1.0f,
		TEXT("Extra text scale for the navigation readout, on top of resolution scaling."),
		ECVF_Cheat);

	static TAutoConsoleVariable<int32> CVarNavHudRaw(
		TEXT("uls.NavHud.Raw"),
		0,
		TEXT("Append the full-precision VirtualTraversal triple. This is the form to write "
			"down if you want to come back to a position later; the ladder line above it "
			"is rounded and will not round-trip."),
		ECVF_Cheat);

	/** SMOOTHING TIME CONSTANT for the two speed readouts, seconds.
	 *
	 *  Per-frame position differencing is genuinely noisy -- a 120Hz frame is 8ms of
	 *  movement and any hitch redistributes it -- so an unsmoothed speed flickers through
	 *  a whole ladder rung and is unreadable. A quarter second is long enough to settle
	 *  and short enough that a mousewheel change to SpeedScale looks immediate.
	 *
	 *  BOTH SPEEDS USE THE SAME TAU, and that is the part that matters: the implied ratio
	 *  is one smoothed value divided by the other, so any lag is common to both and
	 *  cancels. Smoothing them differently would make the ratio swing during every
	 *  acceleration and the mismatch warning would fire on nothing. */
	constexpr double SpeedSmoothingTau = 0.25;

	/** A single-frame world-space jump larger than this is treated as a discontinuity
	 *  (respawn, teleport, repossession) rather than as motion. 50 km at any plausible
	 *  frame rate is not something a flying pawn does; if a future ship makes it one,
	 *  this number moves, it does not get deleted -- without it a single teleport reads
	 *  as a speed spike that then decays over a second and looks like a physics bug. */
	constexpr double TeleportWorldCmThreshold = 5.0e6;

	/** Below this real speed the implied-ratio check is suppressed. Dividing two smoothed
	 *  values that are both approaching zero produces an arbitrary ratio, and a warning
	 *  that fires every time the player stops is a warning nobody reads. */
	constexpr double RatioCheckMinRealCmPerSec = 1.0;

	/** Fractional disagreement between the implied ratio and SpeedScale that gets
	 *  highlighted. One percent is well outside smoothing error and well inside anything
	 *  an actual desync would produce. */
	constexpr double RatioMismatchTolerance = 0.01;

	/** Local, not StaticEnum<ELifecycleState>(). DataTypes.h declares the UENUM without a
	 *  matching .generated.h, so the reflected metadata is not something to rely on from
	 *  here; a switch also fails at COMPILE time when a state is added, where a reflection
	 *  lookup would just print the new name and let a HUD that has no handling for it look
	 *  finished. */
	const TCHAR* LifecycleName(ELifecycleState InState)
	{
		switch (InState)
		{
		case ELifecycleState::Uninitialized: return TEXT("uninitialized");
		case ELifecycleState::Initializing:  return TEXT("initializing");
		case ELifecycleState::Ready:         return TEXT("ready");
		case ELifecycleState::Pooling:       return TEXT("pooling");
		case ELifecycleState::Destroying:    return TEXT("destroying");
		}
		return TEXT("?");
	}

	struct FRow
	{
		FString Label;
		FString Value;
		FString Note;
		FColor  ValueColor = FColor(230, 230, 230);
	};

	const FColor LabelColor(140, 150, 160);
	const FColor NoteColor(130, 140, 150);
	const FColor HeaderColor(255, 255, 255);
	const FColor WarnColor(255, 190, 70);
	const FColor GoodColor(150, 220, 150);
}

bool UULSNavHudSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	// Game and PIE worlds only. Editor preview / asset-thumbnail worlds also run
	// subsystems, and one of them drawing a nav readout over a material preview is the
	// kind of thing that gets a debug tool disabled permanently.
	const UWorld* World = Cast<UWorld>(Outer);
	return World && World->IsGameWorld();
}

void UULSNavHudSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	DebugDrawHandle = UDebugDrawService::Register(
		TEXT("Game"),
		FDebugDrawDelegate::CreateUObject(this, &UULSNavHudSubsystem::OnDebugDraw));
}

void UULSNavHudSubsystem::Deinitialize()
{
	// UNREGISTER UNCONDITIONALLY on teardown. The service holds a raw delegate against
	// this object; a surviving registration fires into freed memory on the next frame of
	// the next world, and the crash lands in the renderer with no trace of this file.
	if (DebugDrawHandle.IsValid())
	{
		UDebugDrawService::Unregister(DebugDrawHandle);
		DebugDrawHandle.Reset();
	}

	CachedUniverse.Reset();
	Super::Deinitialize();
}

AUniverseActor* UULSNavHudSubsystem::ResolveUniverse()
{
	if (AUniverseActor* Cached = CachedUniverse.Get())
	{
		return Cached;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return nullptr;
	}

	// The permanent universe is the root of the parent chain and there is one per world,
	// so the iteration runs once and then never again unless it is destroyed.
	for (TActorIterator<AUniverseActor> It(World); It; ++It)
	{
		CachedUniverse = *It;

		// A NEW universe means a new coordinate history. Whatever the previous one had
		// accumulated describes a different traversal.
		ResetTelemetry();
		return *It;
	}

	return nullptr;
}

void UULSNavHudSubsystem::ResetTelemetry()
{
	bHasHistory = false;
	PrevVirtualTraversal = FVector::ZeroVector;
	PrevFrameOfReferenceWorld = FVector::ZeroVector;
	SmoothedRealCmPerSec = 0.0;
	SmoothedCompressedCmPerSec = 0.0;
}

void UULSNavHudSubsystem::UpdateTelemetry(const FULSNavSample& InSample, double InDeltaSeconds)
{
	if (!InSample.bValid || InDeltaSeconds <= 0.0)
	{
		return;
	}

	if (!bHasHistory)
	{
		PrevVirtualTraversal = InSample.VirtualTraversal;
		PrevFrameOfReferenceWorld = InSample.FrameOfReferenceWorld;
		bHasHistory = true;
		return;
	}

	const FVector WorldDelta = InSample.FrameOfReferenceWorld - PrevFrameOfReferenceWorld;
	const FVector TraversalDelta = InSample.VirtualTraversal - PrevVirtualTraversal;

	PrevVirtualTraversal = InSample.VirtualTraversal;
	PrevFrameOfReferenceWorld = InSample.FrameOfReferenceWorld;

	if (WorldDelta.Length() > TeleportWorldCmThreshold)
	{
		ResetTelemetry();
		return;
	}

	// TWO DERIVATIVES OF THE SAME INPUT, deliberately.
	//
	// The real speed could have come from Pawn->GetVelocity() and the compressed speed
	// could have been real * SpeedScale, and both would have been cheaper. Neither would
	// have been a measurement. GetVelocity() is the movement component's own idea of
	// velocity, which is not exactly the frame-to-frame actor delta the parallax
	// accumulation consumes; and multiplying by SpeedScale to get the compressed figure
	// makes the ratio between the two lines an identity, so the pair could never disagree
	// and the readout could never catch a broken accumulation.
	//
	// Instead: real speed is the world-space delta the universe used this frame, and
	// compressed speed is the delta VirtualTraversal actually moved, converted back to
	// real cm through the same UnitScale. Their ratio SHOULD reproduce SpeedScale. When
	// it does not, something between the two has stopped agreeing -- a layer resolving a
	// stale scale, an accumulation running on a different position, a clamp firing
	// silently -- and the Implied line goes amber. That failure has no other symptom: the
	// universe still renders, the player still moves, and the field just scrolls at a
	// rate that does not match the speed on screen.
	const double RealCmPerSec = WorldDelta.Length() / InDeltaSeconds;
	const double CompressedCmPerSec =
		(TraversalDelta.Length() * InSample.Coord.UnitScaleCm) / InDeltaSeconds;

	const double Alpha = 1.0 - FMath::Exp(-InDeltaSeconds / SpeedSmoothingTau);
	SmoothedRealCmPerSec += (RealCmPerSec - SmoothedRealCmPerSec) * Alpha;
	SmoothedCompressedCmPerSec += (CompressedCmPerSec - SmoothedCompressedCmPerSec) * Alpha;
}

void UULSNavHudSubsystem::OnDebugDraw(UCanvas* InCanvas, APlayerController* InPC)
{
	if (CVarNavHud.GetValueOnGameThread() == 0 || !InCanvas || !GEngine)
	{
		return;
	}

	// UDebugDrawService is global, not per-world. Without this an editor with two PIE
	// worlds draws each world's readout over both.
	UWorld* World = GetWorld();
	if (!World || !InPC || InPC->GetWorld() != World)
	{
		return;
	}

	const UFont* Font = GEngine->GetSmallFont();
	if (!Font)
	{
		return;
	}

	AUniverseActor* Universe = ResolveUniverse();

	FULSNavSample Sample;
	if (Universe)
	{
		Sample = ULSSampleUniverseNav(*Universe);
	}

	// One sample per frame regardless of viewport count; see LastSampledFrame. The reset
	// path is inside the guard too -- a universe that is mid-initialization must clear
	// history exactly as often as a live one accumulates it, or the first frame after
	// Ready differences against a traversal from before the actor was reused.
	if (GFrameCounter != LastSampledFrame)
	{
		LastSampledFrame = GFrameCounter;

		if (Sample.bValid)
		{
			UpdateTelemetry(Sample, World->GetDeltaSeconds());
		}
		else
		{
			ResetTelemetry();
		}
	}

	// ---------------------------------------------------------------------------------
	// Compose
	// ---------------------------------------------------------------------------------
	TArray<FRow> Rows;

	FString HeaderText = TEXT("ULS NAV");
	if (!Universe)
	{
		HeaderText += TEXT("   [no universe actor]");
	}
	else if (!Sample.bValid)
	{
		HeaderText += FString::Printf(TEXT("   [universe %s]"), LifecycleName(Sample.State));
	}
	else
	{
		HeaderText += FString::Printf(TEXT("   [ready  galaxies %d  systems %d]"),
			Sample.SpawnedGalaxyCount, Sample.SpawnedSystemCount);
	}

	if (Sample.bValid)
	{
		Rows.Add({ TEXT("Position"), ULSScale::FormatVectorCm(Sample.Coord.RealCm), FString() });

		// THE LAST UNBOUNDED INTEGER IN THE CHAIN. The field cell index wraps and the period
		// index above counts the wraps; this one does neither, because wrapping it would make
		// generation repeat and the whole point of keying seeds on it is that they do not.
		//
		// SHOWN AS A COORDINATE, NOT A PERCENTAGE, because the coordinate is useful every
		// session -- it is the streaming cell you are in, so watching it tick is how you see
		// a boundary cross happen -- while the headroom is 0.00% for any distance anyone will
		// ever fly. The note appears only once the headroom is worth reading, which keeps a
		// permanently-zero number off the panel.
		{
			const FIntVector GC = Sample.GridCoord;
			FRow GridRow;
			GridRow.Label = TEXT("Grid cell");

			if (GC.X == INT32_MIN && GC.Y == INT32_MIN && GC.Z == INT32_MIN)
			{
				// The sentinel FParticleTierState uses for a tier that has never streamed.
				// Printing it raw would put -2147483648 on screen and read as an overflow.
				GridRow.Value = TEXT("--");
				GridRow.Note = TEXT("(finest tier not streamed yet)");
				GridRow.ValueColor = LabelColor;
			}
			else
			{
				GridRow.Value = FString::Printf(TEXT("%d, %d, %d"), GC.X, GC.Y, GC.Z);

				const double Used = FMath::Max3(
					FMath::Abs(static_cast<double>(GC.X)),
					FMath::Abs(static_cast<double>(GC.Y)),
					FMath::Abs(static_cast<double>(GC.Z))) / 2147483647.0;

				if (Used > 0.01)
				{
					GridRow.Note = FString::Printf(TEXT("%.1f%% of int32"), Used * 100.0);
					GridRow.ValueColor = (Used > 0.5) ? WarnColor : FColor(230, 230, 230);
				}
			}
			Rows.Add(GridRow);
		}

		// THE HIGHER-ORDER TERM, and the reason the row below is readable at all.
		//
		// Without it the cell index flips from -max to max at the wrap with nothing saying
		// why, which reads as an overflow. This says why: it moves by exactly one, in the
		// direction of travel, on the frame the flip happens.
		//
		// NOT THE SECTOR LATTICE. AUniverseActor::CellCoord is spaced at 2 * Extent; this is
		// spaced at the field period, millions of times further apart. See the note on
		// FUniverseFieldOffset::Period.
		Rows.Add({ TEXT("Field period"), FString::Printf(TEXT("%d, %d, %d"),
			Sample.FieldOffset.Period.X, Sample.FieldOffset.Period.Y, Sample.FieldOffset.Period.Z),
			(Sample.FieldCellPeriod > 0)
				? FString::Printf(TEXT("(%d cells per repeat)"), Sample.FieldCellPeriod)
				: FString() });

		// SIGNED FOR DISPLAY, unsigned everywhere else. UniverseCellWrap::ToSigned carries the
		// reasoning; the short version is that cell -1 legitimately arrives as period-1 and
		// nobody reads that as minus one.
		{
			const int32 Period = Sample.FieldCellPeriod;

			Rows.Add({ TEXT("Field cell"), FString::Printf(TEXT("%d, %d, %d"),
				UniverseCellWrap::ToSigned(Sample.FieldOffset.Cell.X, Period),
				UniverseCellWrap::ToSigned(Sample.FieldOffset.Cell.Y, Period),
				UniverseCellWrap::ToSigned(Sample.FieldOffset.Cell.Z, Period)),
				FString::Printf(TEXT("+ %.3f, %.3f, %.3f"),
					Sample.FieldOffset.Frac.X, Sample.FieldOffset.Frac.Y, Sample.FieldOffset.Frac.Z) });
		}

		if (CVarNavHudRaw.GetValueOnGameThread() != 0)
		{
			Rows.Add({ TEXT("Raw VT"), FString::Printf(TEXT("%.17g  %.17g  %.17g"),
				Sample.Coord.Local.X, Sample.Coord.Local.Y, Sample.Coord.Local.Z), FString() });
		}

		// --- Nested layers ---------------------------------------------------------
		//
		// TWO ROWS PER LAYER, identity then position, and the identity comes first
		// deliberately. The position tells you where you are; the SEED tells you which
		// galaxy you are in, and the seed is the half that survives a retune of the field
		// while the position does not. Indented so the nesting is visible without a tree
		// widget -- a flat list of three "Position" rows at three different scales is the
		// fastest way to misread this readout.
		for (const FULSLayerCoord& Layer : Sample.Layers)
		{
			FRow IdRow;
			IdRow.Label = Layer.LayerName;
			IdRow.Value = FString::Printf(TEXT("seed %d"), Layer.Seed);
			IdRow.ValueColor = Layer.bInside ? GoodColor : LabelColor;
			IdRow.Note = Layer.bInside
				? FString::Printf(TEXT("inside, %.2f of bounds"), Layer.BoundsFraction)
				: FString::Printf(TEXT("nearest, %.2f of bounds"), Layer.BoundsFraction);
			Rows.Add(IdRow);

			FRow PosRow;
			PosRow.Label = TEXT("  local");
			PosRow.Value = ULSScale::FormatVectorCm(Layer.RealCm);
			PosRow.ValueColor = Layer.bInside ? FColor(230, 230, 230) : LabelColor;
			Rows.Add(PosRow);
		}

		Rows.Add({ FString(), FString(), FString() });   // separator

		// The compression factor as a multiplier is exact but abstract; the second form
		// is the same number as a statement about the world, which is the one that
		// actually tells you whether the current setting is usable.
		Rows.Add({ TEXT("Compression"), ULSScale::FormatMultiplier(Sample.SpeedScale),
			FString::Printf(TEXT("1 m -> %s"), *ULSScale::FormatCm(ULSScale::CmPerMetre * Sample.SpeedScale)) });

		Rows.Add({ TEXT("Real speed"), ULSScale::FormatSpeed(SmoothedRealCmPerSec), TEXT("(pawn, world space)") });

		Rows.Add({ TEXT("Compressed"), ULSScale::FormatSpeed(SmoothedCompressedCmPerSec),
			ULSScale::FormatLightSpeedMultiple(SmoothedCompressedCmPerSec) });

		// Implied ratio: the cross-check described in UpdateTelemetry.
		FRow ImpliedRow;
		ImpliedRow.Label = TEXT("Implied");
		if (SmoothedRealCmPerSec < RatioCheckMinRealCmPerSec)
		{
			ImpliedRow.Value = TEXT("--");
			ImpliedRow.Note = TEXT("(stationary)");
			ImpliedRow.ValueColor = LabelColor;
		}
		else
		{
			const double Implied = SmoothedCompressedCmPerSec / SmoothedRealCmPerSec;
			const double Error = (Sample.SpeedScale > 0.0)
				? FMath::Abs(Implied / Sample.SpeedScale - 1.0)
				: 1.0;

			ImpliedRow.Value = ULSScale::FormatMultiplier(Implied);
			ImpliedRow.ValueColor = (Error > RatioMismatchTolerance) ? WarnColor : GoodColor;
			ImpliedRow.Note = (Error > RatioMismatchTolerance)
				? FString::Printf(TEXT("MISMATCH  %.1f%%"), Error * 100.0)
				: FString();
		}
		Rows.Add(ImpliedRow);
	}

	Rows.Add({ FString(), FString(), FString() });

	const APawn* Pawn = InPC->GetPawn();
	Rows.Add({ TEXT("Pawn"), Pawn ? Pawn->GetClass()->GetName() : TEXT("<none>"), FString() });

	// ---------------------------------------------------------------------------------
	// Draw
	// ---------------------------------------------------------------------------------

	// Resolution scaling so the readout stays the same physical size on a 4K monitor as
	// on a 1080p one. Clamped rather than uncapped: the small font is a bitmap and grows
	// ugly fast past 2x.
	const float ResScale = FMath::Clamp(static_cast<float>(InCanvas->SizeY) / 1080.0f, 0.75f, 2.0f);
	const float Scale = FMath::Max(0.25f, ResScale * CVarNavHudScale.GetValueOnGameThread());

	float LineH = 0.0f, Probe = 0.0f;
	InCanvas->TextSize(Font, TEXT("Wg"), Probe, LineH, Scale, Scale);
	LineH = FMath::Max(LineH, 1.0f) + 2.0f * Scale;

	const float Margin = 16.0f * Scale;
	const float PadX = 8.0f * Scale;
	const float PadY = 6.0f * Scale;
	const float LabelX = Margin + PadX;

	// Column width from the widest LABEL, measured rather than guessed: a guessed column
	// silently overlaps the moment a label is renamed.
	float LabelW = 0.0f;
	for (const FRow& Row : Rows)
	{
		float W = 0.0f, H = 0.0f;
		InCanvas->TextSize(Font, Row.Label, W, H, Scale, Scale);
		LabelW = FMath::Max(LabelW, W);
	}

	float HeaderW = 0.0f, HeaderH = 0.0f;
	InCanvas->TextSize(Font, HeaderText, HeaderW, HeaderH, Scale, Scale);

	const float ValueX = LabelX + LabelW + 14.0f * Scale;

	float ContentW = HeaderW + PadX;
	for (const FRow& Row : Rows)
	{
		float VW = 0.0f, NW = 0.0f, H = 0.0f;
		InCanvas->TextSize(Font, Row.Value, VW, H, Scale, Scale);
		InCanvas->TextSize(Font, Row.Note, NW, H, Scale, Scale);
		const float RowRight = (ValueX - LabelX) + VW + (NW > 0.0f ? NW + 12.0f * Scale : 0.0f);
		ContentW = FMath::Max(ContentW, RowRight);
	}

	const float PanelW = ContentW + PadX * 2.0f;
	const float PanelH = (Rows.Num() + 1) * LineH + PadY * 2.0f;

	FCanvasTileItem Panel(
		FVector2D(Margin, Margin),
		FVector2D(PanelW, PanelH),
		FLinearColor(0.0f, 0.0f, 0.0f, 0.55f));
	Panel.BlendMode = SE_BLEND_Translucent;
	InCanvas->DrawItem(Panel);

	float Y = Margin + PadY;

	InCanvas->SetDrawColor(HeaderColor);
	InCanvas->DrawText(Font, HeaderText, LabelX, Y, Scale, Scale);
	Y += LineH;

	for (const FRow& Row : Rows)
	{
		if (Row.Label.IsEmpty() && Row.Value.IsEmpty())
		{
			Y += LineH * 0.5f;   // separator: half a line, no glyphs
			continue;
		}

		InCanvas->SetDrawColor(LabelColor);
		InCanvas->DrawText(Font, Row.Label, LabelX, Y, Scale, Scale);

		InCanvas->SetDrawColor(Row.ValueColor);
		InCanvas->DrawText(Font, Row.Value, ValueX, Y, Scale, Scale);

		if (!Row.Note.IsEmpty())
		{
			float VW = 0.0f, VH = 0.0f;
			InCanvas->TextSize(Font, Row.Value, VW, VH, Scale, Scale);

			InCanvas->SetDrawColor(NoteColor);
			InCanvas->DrawText(Font, Row.Note, ValueX + VW + 12.0f * Scale, Y, Scale, Scale);
		}

		Y += LineH;
	}
}
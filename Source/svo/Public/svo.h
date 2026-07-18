// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

/** SVO module entry point; wires up module startup and shutdown. */
class FsvoModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
//TODO: Robust instrumentation could be updated... we have done several refactors since we needed this, but instrumentation of game thread/non game thread actions would be very useful from an optimization and tuning standpoint
/** Inline game-thread profiler (temporary; remove when optimization is done).
 *
 *  Drop SVO_GT_SCOPE("Label") at the top of any game-thread method. It:
 *    1. Emits an Unreal Insights CPU trace event (named scope on the timeline).
 *    2. Accumulates inclusive wall-time into a per-label bucket.
 *  Call SVO_GT_FLUSH() once per frame from the root game-thread tick
 *  (AUniverseActor::Tick). Every second it logs a summary sorted worst-first:
 *    Total(ms) | Calls | Avg(ms) | Max(ms)  (Max = single-frame outlier).
 *
 *  Game-thread only: the accumulator is an unlocked TMap. Do not place a scope
 *  inside an AsyncTask/ParallelFor body. Timing is inclusive (a parent's total
 *  includes any instrumented children). Set SVO_GT_PROFILING 0 to compile out.
 *
 *  To remove later: delete this block, the DEFINE_LOG_CATEGORY line in svo.cpp,
 *  the #include "svo.h" added to instrumented .cpp files, and every
 *  SVO_GT_SCOPE/SVO_GT_FLUSH call. */

#include "CoreMinimal.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSVOPerf, Log, All);

#ifndef SVO_GT_PROFILING
#define SVO_GT_PROFILING 0
#endif

#if SVO_GT_PROFILING

/** Per-label accumulation bucket: total and max wall-time plus call count. */
struct FSVOGTStat
{
	double TotalSeconds = 0.0;
	double MaxSeconds = 0.0;
	int32  Calls = 0;
};

/** Singleton game-thread time accumulator; buckets scoped timings by label and
 *  logs a periodic worst-first summary. */
class FSVOGameThreadProfiler
{
public:
	static FSVOGameThreadProfiler& Get()
	{
		static FSVOGameThreadProfiler Instance;
		return Instance;
	}

	/** Adds Seconds to Name's bucket and bumps its call count and max. */
	FORCEINLINE void Accumulate(const TCHAR* Name, double Seconds)
	{
		FSVOGTStat& S = Stats.FindOrAdd(Name);
		S.TotalSeconds += Seconds;
		S.MaxSeconds = FMath::Max(S.MaxSeconds, Seconds);
		++S.Calls;
	}

	/** Logs and resets the accumulated window every IntervalSeconds; no-op otherwise. */
	void FlushIfDue(double IntervalSeconds = 1.0)
	{
		const double Now = FPlatformTime::Seconds();
		if (LastFlush == 0.0) { LastFlush = Now; return; }   // prime on first call
		if (Now - LastFlush < IntervalSeconds) return;

		const double Window = Now - LastFlush;

		if (Stats.Num() > 0)
		{
			Stats.ValueSort([](const FSVOGTStat& A, const FSVOGTStat& B)
				{ return A.TotalSeconds > B.TotalSeconds; });

			UE_LOG(LogSVOPerf, Log,
				TEXT("===== SVO Game Thread Profile - %.2fs window ====="), Window);
			UE_LOG(LogSVOPerf, Log, TEXT("%-40s %11s %8s %11s %11s"),
				TEXT("Method"), TEXT("Total(ms)"), TEXT("Calls"), TEXT("Avg(ms)"), TEXT("Max(ms)"));

			for (const TPair<FString, FSVOGTStat>& Pair : Stats)
			{
				const FSVOGTStat& S = Pair.Value;
				const double TotalMs = S.TotalSeconds * 1000.0;
				const double AvgMs = (S.Calls > 0) ? TotalMs / S.Calls : 0.0;
				const double MaxMs = S.MaxSeconds * 1000.0;
				UE_LOG(LogSVOPerf, Log, TEXT("%-40s %11.3f %8d %11.4f %11.4f"),
					*Pair.Key, TotalMs, S.Calls, AvgMs, MaxMs);
			}
		}

		Stats.Reset();
		LastFlush = Now;
	}

private:
	TMap<FString, FSVOGTStat> Stats;
	double LastFlush = 0.0;
};

/** RAII scope timer: accumulates its lifetime into the named bucket on destruction. */
struct FSVOScopedGTTimer
{
	const TCHAR* Name;
	double Start;
	explicit FSVOScopedGTTimer(const TCHAR* InName)
		: Name(InName), Start(FPlatformTime::Seconds()) {}
	~FSVOScopedGTTimer()
	{
		FSVOGameThreadProfiler::Get().Accumulate(Name, FPlatformTime::Seconds() - Start);
	}
};

/** NameLiteral must be a narrow string literal, e.g. SVO_GT_SCOPE("Universe::Tick"). */
#define SVO_GT_SCOPE(NameLiteral) \
	TRACE_CPUPROFILER_EVENT_SCOPE_STR(NameLiteral); \
	FSVOScopedGTTimer ANONYMOUS_VARIABLE(SVOGTTimer_)(TEXT(NameLiteral))

#define SVO_GT_FLUSH() FSVOGameThreadProfiler::Get().FlushIfDue()

#else // SVO_GT_PROFILING

#define SVO_GT_SCOPE(NameLiteral)
#define SVO_GT_FLUSH()

#endif // SVO_GT_PROFILING
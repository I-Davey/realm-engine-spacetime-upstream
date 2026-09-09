#include "pch-il2cpp.h"
#include "MovementRuntime.h"
#include "MovementSpeed.h"
#include "MovementFrameBudget.h"

#include "Il2CppResolver.h"
#include "Il2CppHook.h"
#include "DbgFileLog.h"
#include "features/control/FeatureState.h"
#include "RuntimeOffsets.h"
#include "GameState.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <windows.h>

namespace {

using MoveToFn = bool(__fastcall*)(void* __this, float x, float y, void* methodInfo);
using CalcMoveSpeedFn = float(__fastcall*)(void* __this, void* methodInfo);
using GetDeltaTimeFn = float(__cdecl*)(void* method);

MoveToFn s_fnMoveTo = nullptr;
const MethodInfo* s_miMoveTo = nullptr;
CalcMoveSpeedFn s_fnCalcMoveSpeed = nullptr;
GetDeltaTimeFn s_fnGetDeltaTime = nullptr;
LARGE_INTEGER g_moveWallFreq = []{ LARGE_INTEGER f{}; QueryPerformanceFrequency(&f); return f; }();
bool s_moveResolved = false;
bool s_cmsResolved = false;
bool s_dtResolved = false;
float s_lastDeltaTime = 0.016f;
void* s_filterTarget=nullptr;
std::atomic<bool> s_filterInstalled{false};
MoveToFn s_filterOriginal=nullptr;
std::atomic<DodgeRuntime::MovementFilter> s_movementFilter{nullptr};
std::atomic<DodgeRuntime::MovementFeedback> s_movementFeedback{nullptr};
thread_local bool s_ownMove=false, s_inFilter=false;
thread_local DodgeRuntime::MovementFrameBudget s_frameBudget;
double MovementNowMs() {
    LARGE_INTEGER now{}; QueryPerformanceCounter(&now);
    return double(now.QuadPart)*1000./double(g_moveWallFreq.QuadPart);
}

bool FilterGuarded(void* player,float requestedX,float requestedY,float& x,float& y) {
    __try {
        const auto filter=s_movementFilter.load(std::memory_order_acquire);
        return filter && filter(player,requestedX,requestedY,x,y);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        DbgFileLogWrite("[DodgeRuntime] native movement filter fault; original step retained");
        return false;
    }
}
bool __fastcall FilteredMoveTo(void* player,float x,float y,void* method) {
    const bool wasInFilter=s_inFilter;
    const bool eligible=!s_ownMove && !s_inFilter && player && player==GameState::GetLocalPtr();
    bool changed=false;
    if(eligible) {
        // Account all natural local movement, including calls outside this
        // update hook. Never add an automatic step on top of a native one.
        __try {
            const auto base=static_cast<unsigned char*>(player);
            if(RuntimeOffsets::PosX && RuntimeOffsets::PosY &&
               (std::fabs(x-*reinterpret_cast<float*>(base+RuntimeOffsets::PosX))>1e-6f ||
                std::fabs(y-*reinterpret_cast<float*>(base+RuntimeOffsets::PosY))>1e-6f))
                s_frameBudget.Native(MovementNowMs());
        } __except(EXCEPTION_EXECUTE_HANDLER) { s_frameBudget.Native(MovementNowMs()); }
        float selectedX=x,selectedY=y;
        s_inFilter=true;
        changed=FilterGuarded(player,x,y,selectedX,selectedY);
        if(changed) { x=selectedX; y=selectedY; }
    }
    // Exactly one original invocation, including clear/bypass/fault cases.
    bool accepted=false;
    __try { accepted=s_filterOriginal(player,x,y,method); }
    __finally { s_inFilter=wasInFilter; }
    if(changed) {
        const auto feedback=s_movementFeedback.load(std::memory_order_acquire);
        if(feedback) feedback(accepted,x,y);
    }
    return accepted;
}

void ResolveMoveTo()
{
    if (s_moveResolved) return;
    const MethodInfo* mi = Il2CppHook::ResolveMethodCached("FKALGHJIADI", "DGLCONCOIBO", 2);
    if (!mi) return;
    s_fnMoveTo = reinterpret_cast<MoveToFn>(mi->methodPointer);
    s_miMoveTo = mi;
    s_moveResolved = true;
}

// DGLCONCOIBO (MoveTo) is virtual — the live player can be a FKALGHJIADI
// subclass whose override differs from the impl we bound at resolve time.
// Re-dispatch through the object's own vtable, caching per live class so the
// il2cpp lookup runs once per class, not per frame. Falls back to the bound
// FKALGHJIADI impl if anything about the lookup is off.
MoveToFn ResolveMoveToForObject(void* player)
{
    static void*    s_cachedKlass = nullptr;
    static MoveToFn s_cachedFn    = nullptr;

    if (!s_miMoveTo) return s_fnMoveTo;
    void* klass = nullptr;
    __try { klass = *reinterpret_cast<void**>(player); }
    __except (EXCEPTION_EXECUTE_HANDLER) { return s_fnMoveTo; }
    if (!klass) return s_fnMoveTo;
    if (klass == s_cachedKlass && s_cachedFn) return s_cachedFn;

    MoveToFn fn = s_fnMoveTo;
    __try {
        const MethodInfo* mi = il2cpp_object_get_virtual_method(
            reinterpret_cast<Il2CppObject*>(player), s_miMoveTo);
        if (mi && mi->methodPointer)
            fn = reinterpret_cast<MoveToFn>(mi->methodPointer);
    } __except (EXCEPTION_EXECUTE_HANDLER) { fn = s_fnMoveTo; }

    s_cachedKlass = klass;
    s_cachedFn    = fn;
    return fn;
}

// Raw CalcMoveSpeed (FKALGHJIADI::GCFKGLKAPND) call, SEH-guarded. Returns
// a negative sentinel on failure; zero remains a valid measured value.
float CallCalcMoveSpeedRaw(void* player)
{
    if (!s_fnCalcMoveSpeed || !player) return DodgeRuntime::kUnknownSpeed;
    float v = 0.f;
    __try { v = s_fnCalcMoveSpeed(player, nullptr); }
    __except (EXCEPTION_EXECUTE_HANDLER) { return DodgeRuntime::kUnknownSpeed; }
    if (!std::isfinite(v) || v < 0.f) return DodgeRuntime::kUnknownSpeed;
    return v;
}

void ResolveCalcMoveSpeed()
{
    if (s_cmsResolved) return;
    const MethodInfo* mi = Il2CppHook::ResolveMethodCached("FKALGHJIADI", "GCFKGLKAPND", 0);
    if (!mi) return;
    s_fnCalcMoveSpeed = reinterpret_cast<CalcMoveSpeedFn>(mi->methodPointer);
    s_cmsResolved = true;
}

void ResolveDeltaTime()
{
    if (s_dtResolved) return;
    const MethodInfo* mi = Il2CppHook::ResolveMethodCached("Time", "get_deltaTime", 0,
                                                            false, "UnityEngine");
    if (!mi) return;
    s_fnGetDeltaTime = reinterpret_cast<GetDeltaTimeFn>(mi->methodPointer);
    s_dtResolved = true;
}

} // namespace

namespace DodgeRuntime {

bool EnsureMovementFilter(void* player,MovementFilter filter,MovementFeedback feedback) {
    if(!player || !EnsureResolved()) return false;
    const auto target=reinterpret_cast<void*>(ResolveMoveToForObject(player));
    if(!target) return false;
    if(s_filterTarget) return s_filterTarget==target;
    static ULONGLONG lastAttempt=0;
    const auto now=GetTickCount64();
    if(lastAttempt && now-lastAttempt<250) return false;
    lastAttempt=now;
    if(!Il2CppHook::EnsureRuntime("SpacetimeMovement")) return false;
    s_movementFeedback.store(feedback,std::memory_order_release);
    s_movementFilter.store(filter,std::memory_order_release);
    if(!Il2CppHook::InstallMinHook(target,reinterpret_cast<void*>(&FilteredMoveTo),
       reinterpret_cast<void**>(&s_filterOriginal),"SpacetimeMovement")) return false;
    s_filterTarget=target; s_filterInstalled.store(true,std::memory_order_release); return true;
}
bool MovementFilterInstalled() { return s_filterInstalled.load(std::memory_order_acquire); }
void UninstallMovementFilter() {
    s_movementFilter.store(nullptr,std::memory_order_release);
    s_movementFeedback.store(nullptr,std::memory_order_release);
    s_filterInstalled.store(false,std::memory_order_release);
    Il2CppHook::UninstallMinHook(s_filterTarget,"SpacetimeMovement");
    s_filterOriginal=nullptr;
}

bool EnsureResolved()
{
    ResolveMoveTo();
    ResolveCalcMoveSpeed();
    ResolveDeltaTime();
    return s_fnMoveTo != nullptr;
}

float GetDeltaTime()
{
    if (!s_fnGetDeltaTime) return s_lastDeltaTime;
    float dt = s_lastDeltaTime;
    __try {
        dt = s_fnGetDeltaTime(nullptr);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        dt = s_lastDeltaTime;
    }
    if (dt <= 0.f || dt > 0.5f) dt = s_lastDeltaTime;
    s_lastDeltaTime = dt;
    return dt;
}

float GetMoveSpeedMul(void* player)
{
    ResolveCalcMoveSpeed();
    return SpeedOrFallback(CallCalcMoveSpeedRaw(player), 1.f);
}

// Set whenever WE issue a move, so the speed observer can ignore that frame:
// measuring while the dodge is driving would just measure the dodge.
static std::atomic<bool>  s_selfDriven{ false };
static std::atomic<float> s_obsTps{ 0.f };

void ObserveSpeed(float px, float py, float dtMs)
{
    static float lx = 0.f, ly = 0.f;
    static bool  have = false;
    const bool self = s_selfDriven.exchange(false, std::memory_order_relaxed);
    if (!have || dtMs <= 1.f || dtMs > 200.f) { lx = px; ly = py; have = true; return; }
    const float dist = std::hypot(px - lx, py - ly);
    lx = px; ly = py;
    if (self) return;                       // our own command, not the game's speed
    const float tps = dist / dtMs * 1000.f;
    if (!std::isfinite(tps) || tps <= 0.1f || tps > 60.f) return;

    // DIAGNOSTIC ONLY - this no longer feeds vmax. Planning uses the server
    // authorised speed; see GetTilesPerSec. Left in place for the diag panel.
    // A high-water mark latches onto outliers, and the worst outlier here is a
    // SERVER CORRECTION: a rubber-band moves the player a long way in one
    // update, which reads as enormous speed and would inflate the very estimate
    // that exists to stop us over-issuing - a feedback loop that makes the
    // rubber-band worse. Take a high percentile of a recent window instead:
    // sustained movement (including a speed multiplier, which is legitimate and
    // must be learned) dominates the window, while a one-frame jump does not.
    static float ring[64] = {};
    static int   head = 0, filled = 0;
    ring[head] = tps;
    head = (head + 1) % 64;
    if (filled < 64) ++filled;
    if (filled < 8) return;                    // not enough evidence yet
    float sorted[64];
    std::copy(ring, ring + filled, sorted);
    std::sort(sorted, sorted + filled);
    const float p90 = sorted[static_cast<int>(filled * 0.9f)];
    s_obsTps.store(p90, std::memory_order_relaxed);
}

float GetObservedTilesPerSec() { return s_obsTps.load(std::memory_order_relaxed); }

// Raw field read in its own function: SEH cannot live in a frame that needs
// C++ object unwinding.
static bool ReadWorldPos(void* obj, uint32_t ox, uint32_t oy, float& x, float& y)
{
    __try {
        x = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(obj) + ox);
        y = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(obj) + oy);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void AccountNativeMovement() {
    s_frameBudget.Native(MovementNowMs());
}

void BeginGameUpdate() { s_frameBudget.Begin(MovementNowMs(),GetDeltaTime()*1000.f); }
void EndGameUpdate() { s_frameBudget.End(); }
float GetFrameMoveMs() { return s_frameBudget.Available(MovementNowMs()); }
double GetMovementTimeMs() { return MovementNowMs(); }

MoveResult CallMoveToFrame(void* player,float x,float y,float durationMs) {
    if(!player || !std::isfinite(x) || !std::isfinite(y)) return MoveResult::Rejected;
    const float speed=GetVerifiedTilesPerSec(player)/1000.f;
    float px=0.f,py=0.f;
    if(!RuntimeOffsets::PosX || !RuntimeOffsets::PosY || !(speed>0.f) || !std::isfinite(speed) ||
       !ReadWorldPos(player,RuntimeOffsets::PosX,RuntimeOffsets::PosY,px,py) ||
       !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(durationMs) || durationMs<=0.f ||
       std::hypot(x-px,y-py)>speed*durationMs+1e-4f) return MoveResult::Rejected;
    MoveToFn fn=ResolveMoveToForObject(player);
    if(!fn) return MoveResult::Rejected;
    if(!s_frameBudget.Claim(MovementNowMs(),durationMs)) return MoveResult::Deferred;
    s_selfDriven.store(true,std::memory_order_relaxed);
    const bool wasOwnMove=s_ownMove; s_ownMove=true;
    bool ok=false;
    __try { ok=fn(player,x,y,nullptr); }
    __except(EXCEPTION_EXECUTE_HANDLER) { ok=false; }
    s_ownMove=wasOwnMove;
    return ok?MoveResult::Applied:MoveResult::Rejected;
}

float GetVerifiedTilesPerSec(void* player)
{
    ResolveCalcMoveSpeed();
    return ResolveTilesPerSec(FeatureState::GetClientSpeed(), CallCalcMoveSpeedRaw(player));
}

bool CallMoveTo(void* player, float x, float y)
{
    if (!s_fnMoveTo || !player) return false;
    MoveToFn fn = ResolveMoveToForObject(player);
    if (!fn) return false;
    s_selfDriven.store(true);
    const bool wasOwnMove=s_ownMove; s_ownMove=true;
    bool ok = false;
    __try {
        ok = fn(player, x, y, nullptr);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        ok = false;
    }
    s_ownMove=wasOwnMove;
    return ok;
}

float GetTilesPerSec(void* player)
{
    ResolveCalcMoveSpeed();
    const int32_t spd = FeatureState::GetClientSpeed();
    const float mul = CallCalcMoveSpeedRaw(player);
    // A readable multiplier still applies before the first SPD packet. Retain
    // the existing SPD-50 fallback only for the missing base stat, not the slow.
    const float speed = ResolveTilesPerSec(spd >= 0 ? spd : 50, SpeedOrFallback(mul, 1.f));
    return speed;
}

void Reset()
{
    s_moveResolved = false;
    s_cmsResolved = false;
    s_dtResolved = false;
    s_fnMoveTo = nullptr;
    s_fnCalcMoveSpeed = nullptr;
    s_fnGetDeltaTime = nullptr;
    s_lastDeltaTime = 0.016f;
}

} // namespace DodgeRuntime

#include "pch-il2cpp.h"

#include "features/combat/autoaim/core/WeaponProfile.h"
#include "features/combat/autoaim/core/WeaponProfileMath.h"
#include "features/combat/autoaim/core/AimMath.h"
#include "RuntimeOffsets.h"
#include "core/runtime/MemRead.h"
#include "game/objects/GameObjects.h"
#include "ProjectileTracking.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <windows.h>

namespace {

static std::atomic<void*> s_projProps{ nullptr };
static WeaponProfile      s_profile;
static uint64_t           s_lastSpawnMs = 0;
static void*              s_shortestProjProps = nullptr;

static float ReadLifetimeMsElement(void* props) {
    __try {
        auto* object=reinterpret_cast<Il2CppObject*>(props);
        static Il2CppClass* cachedClass=nullptr;
        static FieldInfo* field=nullptr;
        if(object->klass!=cachedClass) {
            cachedClass=object->klass;
            field=il2cpp_class_get_field_from_name(cachedClass,"LifetimeMsElement");
        }
        if(!field) return 0.f;
        Il2CppString* value=nullptr;
        il2cpp_field_get_value(object,field,&value);
        if(!value) return 0.f;
        const int length=il2cpp_string_length(value);
        const auto* chars=il2cpp_string_chars(value);
        if(length<=0 || length>=32 || !chars) return 0.f;
        char number[32]{};
        for(int i=0;i<length;i++) { if(chars[i]>127) return 0.f; number[i]=static_cast<char>(chars[i]); }
        char* end=nullptr;
        const float ms=std::strtof(number,&end);
        return end==number+length && std::isfinite(ms) && ms>0.f?ms:0.f;
    } __except(EXCEPTION_EXECUTE_HANDLER) { return 0.f; }
}

static bool ReadPlayerTuners(void* local, float& outSpeedMul, float& outLifetimeMul, float& outRangeMul)
{
    if (!Mem::AddrOk(local)) return false;
    __try {
        uint8_t* p = reinterpret_cast<uint8_t*>(local);
        outSpeedMul    = *reinterpret_cast<float*>(p + RuntimeOffsets::Char_ProjSpeedMul);  // raw-access-ok: hot-loop __try field sweep, per-field fallback would defeat the shared-SEH abort (plan 16)
        outLifetimeMul = *reinterpret_cast<float*>(p + RuntimeOffsets::Char_ProjLifetimeMul);  // raw-access-ok: hot-loop __try field sweep, per-field fallback would defeat the shared-SEH abort (plan 16)
    } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }

    // See the note above: no known offset for the range multiplier on this build.
    outRangeMul = 1.f;
    __try {
        outRangeMul = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(local) + RuntimeOffsets::Char_RangeMul);  // raw-access-ok: hot-loop __try field sweep, per-field fallback would defeat the shared-SEH abort (plan 16)
    } __except (EXCEPTION_EXECUTE_HANDLER) {}

    auto clamp1 = [](float v) { return (std::isfinite(v) && v > 0.f && v < 100.f) ? v : 1.f; };
    outSpeedMul    = clamp1(outSpeedMul);
    outLifetimeMul = clamp1(outLifetimeMul);
    outRangeMul    = clamp1(outRangeMul);
    return true;
}

static void Recalculate(void* local)
{
    void* pp = s_projProps.load(std::memory_order_relaxed);
    if (!Mem::AddrOk(pp) || !Mem::AddrOk(local))
        return;

    float speedMul = 1.f, lifetimeMul = 1.f, rangeMul = 1.f;
    if (!ReadPlayerTuners(local, speedMul, lifetimeMul, rangeMul))
        return;

    __try {
        Game::ProjProps props(pp);

        const bool isParam      = props.IsParametric();
        const int32_t rawSpeedI = props.Speed();
        const float rawLife     = props.Lifetime();
        const float mag         = props.Magnitude();

        // Check parametric FIRST — swords/daggers/other fixed-arc weapons store
        // PP_Speed = 0 (unused), which would fail the speed validation below.
        if (isParam) {
            if (!(std::isfinite(mag) && mag > 0.f)) return;
            float rangeTiles = mag * speedMul;
            if (rangeMul >= 0.5f && rangeMul <= 10.f)
                rangeTiles *= rangeMul;
            s_profile.speedRaw    = 0.f;
            s_profile.lifetimeMs  = 0.f;
            s_profile.rangeTiles  = rangeTiles;
            s_profile.avgSpeedTps = 200.f;
            s_profile.isResolved  = true;
            s_profile.isParametric = true;
            return;
        }

        // Standard speed+lifetime projectile path.
        // Lower bound is 0 (not 100) — melee weapons like swords store speed == 100
        // (0.01 tiles/ms), which a <=100 guard would wrongly reject.
        if (rawSpeedI <= 0 || rawSpeedI >= 500000) return;

        const float rawSpeed   = static_cast<float>(rawSpeedI);
        const float lifetimeMs = WeaponProfileMath::LifetimeMs(rawLife,ReadLifetimeMsElement(pp)) * lifetimeMul;
        if (!(lifetimeMs > 1.f) || !std::isfinite(lifetimeMs)) return;

        float rangeTiles = AimMath::IntegratedProjectileDistance(
            reinterpret_cast<uint8_t*>(pp), lifetimeMs, speedMul, rawSpeed);
        if (!(rangeTiles > 0.f) || !std::isfinite(rangeTiles)) return;
        if (rangeMul >= 0.5f && rangeMul <= 10.f)
            rangeTiles *= rangeMul;

        float avgSpeedTps = (rangeTiles / lifetimeMs) * 1000.f;
        if (!(avgSpeedTps > 0.01f) || !std::isfinite(avgSpeedTps))
            avgSpeedTps = (rawSpeed / 10000.f) * speedMul * 1000.f;

        s_profile.speedRaw    = rawSpeed;
        s_profile.lifetimeMs  = lifetimeMs;
        s_profile.rangeTiles  = rangeTiles;
        s_profile.avgSpeedTps = avgSpeedTps;
        s_profile.isResolved  = true;
        s_profile.isParametric = false;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

} // namespace

namespace WeaponCalibrator {

void OnProjectileSpawn(void* projProps, void* localPlayer)
{
    if (!projProps) return;
    const uint64_t now = GetTickCount64();
    const bool newVolley = s_lastSpawnMs == 0 || now - s_lastSpawnMs > 120ULL;
    s_lastSpawnMs = now;

    const WeaponProfile previous = s_profile;
    void* previousProps = s_shortestProjProps;
    s_projProps.store(projProps, std::memory_order_relaxed);

    // A new local projectile may be from a newly equipped weapon. Invalidate
    // the old calibration before reading it so a failed/changed projectile
    // cannot leave the previous weapon's range displayed indefinitely.
    s_profile = WeaponProfile{};

    // Read projId immediately while the pointer is hot.
    __try {
        s_profile.projId = *reinterpret_cast<int32_t*>(
            reinterpret_cast<uint8_t*>(projProps) + RuntimeOffsets::PP_ProjId);  // raw-access-ok: hot-loop __try field sweep, per-field fallback would defeat the shared-SEH abort (plan 16)
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        s_profile.projId = 0;
    }

    // Calibrate immediately — projProps is a managed IL2CPP object that may be
    // collected or reused before the next render tick, so we must read it now.
    Recalculate(localPlayer);

    // A multi-projectile weapon may spawn different projectile definitions in
    // the same volley. Preserve the shortest resolved reach so Target Assist
    // never follows to a distance where only some shots connect.
    if (newVolley || !previous.isResolved ||
        (s_profile.isResolved && s_profile.rangeTiles < previous.rangeTiles)) {
        s_shortestProjProps = projProps;
    } else {
        s_profile = previous;
        s_shortestProjProps = previousProps;
        s_projProps.store(previousProps, std::memory_order_relaxed);
    }
}

void Tick(void* localPlayer)
{
    // Re-read player multipliers each frame (speed/lifetime buffs can change).
    // projProps is already cached; only re-runs Recalculate, which is fast.
    Recalculate(localPlayer);
}

const WeaponProfile& GetProfile()
{
    return s_profile;
}

void Reset()
{
    s_projProps.store(nullptr, std::memory_order_relaxed);
    s_profile = WeaponProfile{};
    s_lastSpawnMs = 0;
    s_shortestProjProps = nullptr;
}

} // namespace WeaponCalibrator

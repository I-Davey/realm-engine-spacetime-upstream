#pragma once
#include "WeaponProfile.h"
#include <cmath>

namespace WeaponProfileMath {
// LifetimeMsElement carries explicit units. The runtime Lifetime field has
// differed between builds; only use its seconds/ms compatibility fallback
// when the XML-derived element is unavailable. Never use the old <250 rule.
inline float LifetimeMs(float runtimeValue,float explicitMs) {
    if(std::isfinite(explicitMs) && explicitMs>0.f) return explicitMs;
    if(!std::isfinite(runtimeValue) || runtimeValue<=0.f) return 0.f;
    return runtimeValue<10.f?runtimeValue*1000.f:runtimeValue;
}
inline bool HasFlightModel(const WeaponProfile& profile) {
    return profile.isParametric || (std::isfinite(profile.avgSpeedTps) && profile.avgSpeedTps>.1f &&
        std::isfinite(profile.lifetimeMs) && profile.lifetimeMs>0.f);
}
}

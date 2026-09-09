#pragma once
#include <algorithm>
#include <cmath>

namespace DodgeRuntime {
// One local movement allowance per game update. Idle time is never banked;
// native walking consumes the same allowance as automatic movement.
struct MovementFrameBudget {
    double previousMs=-1., lastNativeMs=-1.;
    float frameMs=0.f;
    bool active=false, spent=false;
    void Begin(double nowMs,float initialDeltaMs) {
        const double elapsed=previousMs>=0.?nowMs-previousMs:initialDeltaMs;
        previousMs=nowMs;
        frameMs=std::isfinite(elapsed)?static_cast<float>(std::clamp(elapsed,0.,50.)):0.f;
        active=true; spent=false;
    }
    void End() { active=false; }
    void Native(double nowMs) { lastNativeMs=nowMs; if(active) spent=true; }
    float Available(double nowMs) const {
        if(!active || spent) return 0.f;
        return lastNativeMs>=0.?std::min(frameMs,static_cast<float>(std::max(0.,nowMs-lastNativeMs))):frameMs;
    }
    bool Claim(double nowMs,float durationMs) {
        if(!std::isfinite(durationMs) || durationMs<=0.f || durationMs>Available(nowMs)+.001f) return false;
        spent=true; return true;
    }
};
}

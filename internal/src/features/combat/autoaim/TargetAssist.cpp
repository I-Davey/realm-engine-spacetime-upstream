#include "pch-il2cpp.h"
#include "TargetAssist.h"
#include "shoot/AimHooks.h"
#include "core/AimMath.h"
#include "core/WeaponProfileMath.h"
#include "modes/AutoAim.h"
#include "features/combat/enemytracker/EnemyTracker.h"
#include "features/movement/spacetime/SpacetimeDodge.h"
#include "features/movement/pjdodge/PJDodgeSensors.h"
#include "GameState.h"
#include "RuntimeOffsets.h"
#include "W2S.h"
#include <imgui/imgui.h>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>

namespace TargetAssist {
namespace {
constexpr float kPickRadiusPx=80.f, kMaxSaneWeaponRange=30.f;
constexpr uint64_t kTargetTrackerGraceMs=750;
std::atomic<bool> g_enabled{false},g_debug{false};
std::atomic<float> g_rangeFactor{.92f};
std::atomic<int32_t> g_targetId{0},g_scriptTargetId{0};
std::mutex g_firingMutex;
FiringZone g_firingZone{};
EnemyTracker::Entry g_lastTarget{};
uint64_t g_lastTargetSeenMs=0;
void* g_world=nullptr;
bool g_publishedAim=false;
bool ScriptTargetActive() { return SpacetimeDodge::IsEnabled() && g_scriptTargetId.load()>0; }
bool GuidanceEnabled() { return SpacetimeDodge::IsEnabled() && (g_enabled.load() || ScriptTargetActive()); }
struct View {
    bool valid=false,rangeResolved=false,inRange=false,los=false;
    float targetX=0.f,targetY=0.f,range=0.f;
    const char* state="idle";
};
View g_view;
bool KeyDown(int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }

bool ReadPlayerPos(void* player, float& x, float& y)
{
    if (!player) return false;
    __try {
        x = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(player) + RuntimeOffsets::PosX);
        y = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(player) + RuntimeOffsets::PosY);
        return std::isfinite(x) && std::isfinite(y);
    } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

const EnemyTracker::Entry* FindTarget(int32_t id)
{
    for (const auto& e : EnemyTracker::GetSnapshot())
        if (e.id == id) return &e;
    return nullptr;
}

bool HasLineOfSight(float px, float py, float tx, float ty)
{
    const float dx = tx - px, dy = ty - py;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.2f) return true;
    const int samples = std::max(1, static_cast<int>(std::ceil(len / 0.18f)));
    // Do not test the target's own final footprint as a wall.
    for (int i = 1; i < samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples);
        if (!PJDodge::Sensors::CanOccupy(px + dx * t, py + dy * t, false)) return false;
    }
    return true;
}

float ProjectRadius(float wx, float wy, float radius, float camX, float camY,
                    float angle, float zoom, float cx, float cy)
{
    float centerX = 0.f, centerY = 0.f;
    if (!W2S(wx, wy, centerX, centerY, camX, camY, angle, zoom, cx, cy)) return 0.f;
    float sum = 0.f;
    constexpr float dirs[4][2] = {{1.f,0.f},{0.f,1.f},{-1.f,0.f},{0.f,-1.f}};
    for (const auto& d : dirs) {
        float sx = 0.f, sy = 0.f;
        if (!W2S(wx + d[0] * radius, wy + d[1] * radius,
                 sx, sy, camX, camY, angle, zoom, cx, cy)) return 0.f;
        sum += std::hypot(sx - centerX, sy - centerY);
    }
    return sum * 0.25f;
}

void ClearRuntimeTarget() {
    { std::lock_guard<std::mutex> lock(g_firingMutex); g_firingZone={}; }
    g_targetId.store(0); g_lastTarget={}; g_lastTargetSeenMs=0; g_view={};
    if(g_publishedAim) AimHooks::SetTarget(false,0.f,0.f);
    g_publishedAim=false;
}
} // namespace

void SetEnabled(bool enabled) { g_enabled.store(enabled); if(!enabled && !ScriptTargetActive()) ClearRuntimeTarget(); }
bool IsEnabled() { return g_enabled.load(); }
void SetDebugOverlay(bool enabled) { g_debug.store(enabled); }
bool GetDebugOverlay() { return g_debug.load(); }
void SetRangeSafetyFactor(float factor) { if(std::isfinite(factor)) g_rangeFactor.store(std::clamp(factor,.5f,1.f)); }
float GetRangeSafetyFactor() { return g_rangeFactor.load(); }
void ClearTarget() { g_scriptTargetId.store(0); ClearRuntimeTarget(); }
void SetScriptTarget(int32_t id) { g_scriptTargetId.store(std::max(0,id)); if(SpacetimeDodge::IsEnabled()) g_targetId.store(0); }
int32_t GetTargetId() { return ScriptTargetActive()?g_scriptTargetId.load():g_targetId.load(); }
bool OwnsAim() { return GuidanceEnabled() && GetTargetId()>0; }
FiringZone GetFiringZone(void* player) {
    std::lock_guard<std::mutex> lock(g_firingMutex);
    FiringZone zone=g_firingZone;
    const uint64_t now=GetTickCount64();
    if(!GuidanceEnabled() || zone.targetId!=GetTargetId() || zone.player!=player ||
       zone.world!=GameState::GetWorldMgr() || now<zone.capturedMs || now-zone.capturedMs>150)
        zone.active=false;
    if(zone.active) {
        const float ageSeconds=static_cast<float>(now-zone.capturedMs)/1000.f;
        zone.targetX+=zone.velocityX*ageSeconds;
        zone.targetY+=zone.velocityY*ageSeconds;
        if(!std::isfinite(zone.targetX) || !std::isfinite(zone.targetY)) zone.active=false;
    }
    return zone;
}

bool CanHitFrom(const FiringZone& zone,float x,float y,float inset) {
    if(!zone.active || !std::isfinite(x) || !std::isfinite(y)) return false;
    const float range=std::max(0.f,zone.range-std::max(0.f,inset));
    // The drawn circle is also an actual constraint. Intercepting an enemy
    // coming toward us must not label a position outside that circle as arrived.
    if(std::hypot(zone.targetX-x,zone.targetY-y)>range) return false;
    if(zone.parametric) return HasLineOfSight(x,y,zone.targetX,zone.targetY);
    float aimX=zone.targetX,aimY=zone.targetY;
    const float time=AimMath::QuadraticIntercept(x,y,zone.targetX,zone.targetY,
        zone.velocityX,zone.velocityY,zone.projectileSpeed,aimX,aimY,zone.lifetimeSeconds);
    return time>=0.f && std::hypot(aimX-x,aimY-y)<=range && HasLineOfSight(x,y,aimX,aimY);
}

void ProcessMiddleClick(float sx, float sy, float camX, float camY,
                        float angle, float zoom, float cx, float cy)
{
    if (!IsEnabled() || !SpacetimeDodge::IsEnabled()) return;
    EnemyTracker::Tick();
    int32_t bestId = 0;
    float bestSq = kPickRadiusPx * kPickRadiusPx;
    for (const auto& e : EnemyTracker::GetSnapshot()) {
        if (e.isScenery || !e.hasHealthBar) continue;
        float ex = 0.f, ey = 0.f;
        if (!W2S(e.x, e.y, ex, ey, camX, camY, angle, zoom, cx, cy)) continue;
        const float d2 = (ex - sx) * (ex - sx) + (ey - sy) * (ey - sy);
        if (d2 < bestSq) { bestSq = d2; bestId = e.id; }
    }
    if (bestId == 0) return; // Empty click deliberately preserves selection.
    if (bestId == GetTargetId()) ClearTarget();
    else {
        g_targetId.store(bestId, std::memory_order_relaxed);
        g_scriptTargetId.store(0);
    }
}

void Tick(bool menuVisible) {
    void* world=GameState::GetWorldMgr();
    if(world!=g_world) { ClearTarget(); g_world=world; }
    if(!GuidanceEnabled()) {
        if(g_lastTargetSeenMs || g_publishedAim) ClearRuntimeTarget();
        return;
    }
    static bool previousEscape=false;
    const bool escape=KeyDown(VK_ESCAPE);
    if(escape && !previousEscape && !menuVisible) ClearTarget();
    previousEscape=escape;
    const int32_t id=GetTargetId();
    if(id<=0) { ClearRuntimeTarget(); return; }
    void* player=GameState::GetLocalPtr();
    float px=0.f,py=0.f;
    if(!ReadPlayerPos(player,px,py)) { ClearTarget(); return; }
    EnemyTracker::Tick();
    const uint64_t nowMs=GetTickCount64();
    const EnemyTracker::Entry* target=FindTarget(id);
    bool trackerStale=false;
    if(target && target->hp>0) { g_lastTarget=*target; g_lastTargetSeenMs=nowMs; }
    else if(g_lastTarget.id==id && g_lastTargetSeenMs && nowMs-g_lastTargetSeenMs<=kTargetTrackerGraceMs) {
        target=&g_lastTarget; trackerStale=true;
    } else { ClearTarget(); return; }
    WeaponCalibrator::Tick(player);
    const WeaponProfile& wp=AutoAim::GetWeaponProfile();
    const bool rangeResolved=wp.isResolved && std::isfinite(wp.rangeTiles) && wp.rangeTiles>0.f && wp.rangeTiles<=kMaxSaneWeaponRange;
    const float range=rangeResolved?wp.rangeTiles*GetRangeSafetyFactor():0.f;
    float aimX=target->x,aimY=target->y;
    if(!wp.isParametric && wp.avgSpeedTps>.1f)
        AimMath::QuadraticIntercept(px,py,target->x,target->y,target->vx*1000.f,target->vy*1000.f,
            wp.avgSpeedTps,aimX,aimY,wp.lifetimeMs/1000.f);
    AimHooks::SetTarget(!trackerStale && !menuVisible,aimX,aimY);
    g_publishedAim=true;
    FiringZone zone{};
    zone.active=rangeResolved && !trackerStale && !menuVisible && !target->isInvulnerable &&
        std::isfinite(target->x) && std::isfinite(target->y) && WeaponProfileMath::HasFlightModel(wp) &&
        std::isfinite(target->vx) && std::isfinite(target->vy);
    zone.targetId=id; zone.player=player; zone.world=world; zone.capturedMs=nowMs;
    zone.targetX=target->x; zone.targetY=target->y; zone.velocityX=target->vx*1000.f; zone.velocityY=target->vy*1000.f;
    zone.range=range; zone.projectileSpeed=wp.avgSpeedTps; zone.lifetimeSeconds=wp.lifetimeMs/1000.f; zone.parametric=wp.isParametric;
    { std::lock_guard<std::mutex> lock(g_firingMutex); g_firingZone=zone; }
    g_view.valid=true; g_view.rangeResolved=rangeResolved; g_view.targetX=target->x; g_view.targetY=target->y;
    g_view.range=range; g_view.inRange=CanHitFrom(zone,px,py);
    g_view.los=rangeResolved && HasLineOfSight(px,py,target->x,target->y);
    g_view.state=trackerStale?"tracker stale":!wp.isResolved?"fire once to calibrate range":
        !rangeResolved?"invalid weapon range":!zone.active?"firing zone unavailable":
        g_view.inRange?"in firing zone":"Spacetime approaching firing zone";
    // This provider never commands movement or shoots. Spacetime consumes the
    // coherent zone; existing game-update firing remains responsible for shots.
}

void RenderOverlay(ImDrawList* draw,float camX,float camY,float angle,float zoom,float cx,float cy) {
    if(!draw || !GuidanceEnabled() || !g_view.valid) return;
    float x=0.f,y=0.f;
    if(!W2S(g_view.targetX,g_view.targetY,x,y,camX,camY,angle,zoom,cx,cy)) return;
    draw->AddCircle(ImVec2(x,y),16.f,IM_COL32(70,225,255,230),24,2.f);
    if(g_view.rangeResolved) {
        const float r=ProjectRadius(g_view.targetX,g_view.targetY,g_view.range,camX,camY,angle,zoom,cx,cy);
        if(r>1.f) draw->AddCircle(ImVec2(x,y),r,IM_COL32(70,225,255,150),96,1.5f);
    }
    if(GetDebugOverlay()) draw->AddText(ImVec2(x+20.f,y-25.f),IM_COL32(255,255,255,235),g_view.state);
}
void RenderSettings() {
    bool enabled=IsEnabled(),debug=GetDebugOverlay(); float range=GetRangeSafetyFactor();
    if(ImGui::Checkbox("Target Assist##spacetimeTarget",&enabled)) SetEnabled(enabled);
    if(ImGui::SliderFloat("Range safety##spacetimeTarget",&range,.5f,1.f,"%.2f")) SetRangeSafetyFactor(range);
    if(ImGui::Checkbox("Target details##spacetimeTarget",&debug)) SetDebugOverlay(debug);
    ImGui::TextWrapped("Select Spacetime to approach a clear firing position. Middle-click selects a target; Escape clears it. Hold Shift to bypass movement.");
}
} // namespace TargetAssist

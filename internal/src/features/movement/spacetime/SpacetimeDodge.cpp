#include "pch-il2cpp.h"
#include "SpacetimeDodge.h"
#include "SpacetimeCore.h"
#include "SpacetimeMovement.h"
#include "SpacetimeReplay.h"
#include "features/combat/autoaim/TargetAssist.h"
#include "features/control/FeatureState.h"
#include "SpacetimeDebugGeometry.h"
#include "SpacetimeDebugStyle.h"
#include "../udodge/UDodgeSensors.h"
#include "../dodge/MovementRuntime.h"
#include "../dodge/SteerInput.h"
#include "../dodge/ProjectileTracking.h"
#include "../dodge/AoeTracking.h"
#include "core/runtime/RuntimeOffsets.h"
#include "GameState.h"
#include "BootGate.h"
#include "gui/tabs/CameraTAB.h"
#include "gui/tabs/WorldTAB.h"
#include "W2S.h"
#include "DbgFileLog.h"
#include "platform/hooks/DirectX.h"
#include <imgui/imgui.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>
#include <cstring>
#include <thread>
#include <memory>
#include <condition_variable>
#include <windows.h>

namespace SpacetimeDodge {
namespace {
using namespace UDodge;
std::atomic<bool> enabled{false}, overlay{true}, shadow{false}, reset{true};
std::atomic<float> lookRange{3.5f},contactScale{1.f},horizonMs{1475.f},maxDistance{4.5f};
std::atomic<float> enemyScale{.2f},searchBudgetMs{8.f};
std::atomic<bool> avoidBlocks{false};
std::atomic<bool> recordReplays{false};
std::atomic<float> stationaryLookRange{10.f};
std::atomic<float> stationaryContactScale{0.95f};
std::atomic<float> stationaryHorizonMs{4000.0f};
std::atomic<float> stationaryMaxDistance{6.5f};
std::atomic<float> stationaryEnemyScale{1.05f};
std::atomic<float> stationarySearchBudgetMs{8.0f};
std::atomic<int> bypassKey{VK_SHIFT},overlayKey{0};
bool lastMovingProfile=false;
std::atomic<ULONGLONG> rangePreviewUntil{0},distancePreviewUntil{0};
State state;
NavigationState navigation;
DangerMap map;
void* lastPlayer=nullptr;
double lastTick=0.;
Vec2 lastPosition{};
thread_local bool nativeMode=false, nativeHandled=false;
thread_local Vec2 nativeRequested{},nativeTarget{};
thread_local bool nativeChanged=false;
thread_local ULONGLONG lastNativeTick=0;
std::atomic<unsigned> nativeCalls{0},nativeOverrides{0},nativeFailures{0},nativeSkipped{0};
struct Debug {
    Output out{};
    double nowMs=0.;
    Vec2 player{}, nominal{}, command{};
    Vec2 goal{}, waypoint{};
    bool hasGoal=false, approaching=false, goalWaypoint=false, detouring=false;
    float speed=0.f, observedSpeed=0.f, solveMs=0.f, captureMs=0.f,
        frameMs=0.f, stepMs=0.f, leadMs=0.f, availableMs=0.f;
    float lookRange=16.f,contactScale=1.f,horizonMs=800.f,maxDistance=3.f;
    float enemyScale=1.f,searchBudgetMs=4.f;
    bool avoidBlocks=false, movingProfile=false;
    float requestedHorizonMs=800.f,requestedMaxDistance=3.f;
    bool evaluated=false;
    Replay::Audit prediction{};
    bool immediateGuard=false, overrideActive=false;
    int bullets=0, considered=0, enemies=0, zones=0, provisional=0, beams=0;
    int aoeHooks=0, capturedAoes=0;
    std::array<int,4> aoeSources{};
    float nextBlastMs=-1.f, blastRadius=0.f;
    bool zoneLimits=false, terrainAtPlayer=false;
    bool native=false, walkingHook=false;
    unsigned nativeCalls=0,nativeOverrides=0,nativeFailures=0,nativeSkipped=0;
    bool bypass=false, deferred=false, rejected=false, applied=false, limited=false, missing=false;
    const char* action="no override needed";
    const char* lastFailure="none";
    double failureMs=0.;
    unsigned moves=0, deferrals=0, rejections=0, noRouteFrames=0, replans=0;
    struct Lane : UDodge::LaneThreat { float radius=0.f,speed=0.f; bool threat=false; };
    // Heap storage keeps all captured threats without growing game-thread
    // stack frames. A changing 64-shot subset made dense patterns flicker.
    std::vector<Lane> lanes;
    int drawnLanes=0;
    std::array<TimedZone,16> zoneShapes{};
    int drawnZones=0;
    std::array<EnemyBlocker,kMaxEnemies> enemyShapes{};
    int drawnEnemies=0;
};
Debug debug;
std::mutex debugMutex;
void Publish(Debug d) {
    d.nativeCalls=nativeCalls.load(); d.nativeOverrides=nativeOverrides.load(); d.nativeFailures=nativeFailures.load();
    d.nativeSkipped=nativeSkipped.load();
    d.walkingHook=DodgeRuntime::MovementFilterInstalled();
    std::lock_guard<std::mutex> lock(debugMutex);
    d.moves=debug.moves+(d.applied?1:0); d.deferrals=debug.deferrals+(d.deferred?1:0);
    d.rejections=debug.rejections+(d.rejected?1:0);
    d.noRouteFrames=debug.noRouteFrames+((d.out.status==Status::NoPlan || d.out.status==Status::Incomplete)?1:0);
    d.replans=debug.replans+(d.out.replanReason!=Reason::None?1:0);
    d.lastFailure=debug.lastFailure; d.failureMs=debug.failureMs;
    if(d.rejected || d.out.reason!=Reason::None) {
        d.lastFailure=d.rejected?d.action:ReasonName(d.out.reason); d.failureMs=d.nowMs;
    }
    debug=d;
}
Debug ReadDebug() { std::lock_guard<std::mutex> lock(debugMutex); return debug; }

// A single pending capture, eight rotating files. The game thread only copies
// fixed-size plain data under try_lock; the worker owns all file I/O.
struct ReplayRecorder {
    std::mutex mutex;
    std::condition_variable wake;
    std::thread worker;
    Replay::Capture pending{};
    Replay::Frame previous{};
    bool ready=false,stopping=false,hasPrevious=false;
    double lastCapture=-1e9;
    ~ReplayRecorder() { Stop(); }
    void Stop() {
        { std::lock_guard<std::mutex> lock(mutex); stopping=true; }
        wake.notify_one(); if(worker.joinable()) worker.join();
    }
    void Start() {
        if(worker.joinable()) return;
        stopping=false;
        worker=std::thread([this] {
          try {
            auto capture=std::make_unique<Replay::Capture>();
            unsigned slot=0;
            for(;;) {
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    wake.wait(lock,[&]{return ready || stopping;});
                    if(stopping && !ready) return;
                    *capture=pending; ready=false;
                }
                wchar_t local[MAX_PATH]{},path[MAX_PATH]{},temporary[MAX_PATH]{};
                if(!GetEnvironmentVariableW(L"LOCALAPPDATA",local,MAX_PATH)) continue;
                swprintf_s(path,L"%s\\RealmEngine-Spacetime-replay%02u.bin",local,slot++%8);
                swprintf_s(temporary,L"%s.tmp",path);
                FILE* file=nullptr;
                if(_wfopen_s(&file,temporary,L"wb") || !file) continue;
                const bool written=std::fwrite(capture.get(),sizeof(*capture),1,file)==1;
                const bool closed=std::fclose(file)==0;
                if(written && closed) MoveFileExW(temporary,path,MOVEFILE_REPLACE_EXISTING);
            }
          } catch(...) { /* diagnostics must never terminate the game */ }
        });
        SetThreadPriority(worker.native_handle(),THREAD_PRIORITY_BELOW_NORMAL);
    }
    void Observe(const Input& in,Debug& d) {
        if(!recordReplays.load()) { hasPrevious=false; return; }
        if(hasPrevious) d.prediction=Replay::Compare(previous.map,*in.world.map,in.world.player,
            static_cast<float>(in.nowMs-previous.input.nowMs));
        const bool failure=d.out.status==Status::Recovery || d.out.status==Status::NoPlan ||
            d.out.status==Status::Incomplete || d.rejected || d.prediction.diverged || d.nextBlastMs>=0.f;
        if(failure && in.nowMs-lastCapture>=1000.) {
            try {
                Start();
                std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
                if(lock.owns_lock()) {
                    pending.hasPrevious=hasPrevious; pending.audit=d.prediction;
                    if(hasPrevious) pending.previous=previous;
                    pending.current.Capture(in,d.out);
                    pending.current.native=d.native; pending.current.requested=nativeRequested;
                    pending.current.command=d.command; pending.current.selected=d.native?nativeChanged:d.applied;
                    pending.current.applied=d.applied; pending.current.rejected=d.rejected;
                    pending.current.feedbackKnown=!d.native || d.rejected;
                    ready=true; lastCapture=in.nowMs; wake.notify_one();
                }
            } catch(...) { /* capture failure cannot interrupt movement */ }
        }
        previous.Capture(in,d.out); previous.native=d.native; previous.requested=nativeRequested;
        previous.command=d.command; previous.selected=d.native?nativeChanged:d.applied;
        previous.applied=d.applied; previous.rejected=d.rejected; hasPrevious=true;
        previous.feedbackKnown=!d.native || d.rejected;
    }
} replayRecorder;

void CaptureDebugShapes(Debug& d,const Input& in) {
    d.bullets=map.laneCount; d.enemies=map.enemyCount; d.zones=in.zoneCount;
    d.missing=map.projectileSourceUnavailable; d.limited=map.limited;
    for(int i=0;i<map.laneCount;i++) {
        const auto& lane=map.lanes[i]; d.provisional+=lane.provisional?1:0; d.beams+=lane.beam?1:0;
        if(LaneInLookRange(in,lane)) ++d.considered;
    }
    if(!GetDebugOverlay()) return;
    d.lanes.reserve(map.laneCount);
    d.drawnEnemies=std::min(map.enemyCount,static_cast<int>(d.enemyShapes.size()));
    for(int i=0;i<d.drawnEnemies;i++) d.enemyShapes[i]=map.enemies[i];
    // Retain blocker-first ordering while including every in-range lane.
    for(int pass=-1;pass<map.laneCount;pass++) {
        const int i=pass<0?d.out.diagnostics.threatLane:pass;
        if(i<0 || i>=map.laneCount || (pass>=0 && i==d.out.diagnostics.threatLane)) continue;
        const auto& lane=map.lanes[i];
        if(!LaneInLookRange(in,lane)) continue;
        d.lanes.emplace_back(); ++d.drawnLanes;
        auto& shape=d.lanes.back();
        static_cast<UDodge::LaneThreat&>(shape)=lane;
        shape.threat=i==d.out.diagnostics.threatLane;
        shape.beam=lane.beam; shape.radius=ProjectileRadius(lane,in.world.settings);
        float speed=0.f;
        for(int j=1;j<lane.pointCount;j++) {
            const float dt=lane.pointTimesMs[j]-lane.pointTimesMs[j-1];
            if(dt>0.f) speed=std::max(speed,Len(Sub(lane.points[j],lane.points[j-1]))/dt);
        }
        shape.speed=speed*1000.f;
    }
    d.drawnZones=std::min(static_cast<int>(d.zoneShapes.size()),in.zoneCount);
    std::array<int,kMaxTimedZones> order{};
    for(int i=0;i<in.zoneCount;i++) order[i]=i;
    std::sort(order.begin(),order.begin()+in.zoneCount,[&](int a,int b) {
        return Len(Sub(in.zones[a].center,in.world.player))-in.zones[a].radius <
               Len(Sub(in.zones[b].center,in.world.player))-in.zones[b].radius;
    });
    for(int i=0;i<d.drawnZones;i++) d.zoneShapes[i]=in.zones[order[i]];
}

bool Locked(void* player) {
    uint32_t a=0,b=0;
    if(!RuntimeOffsets::TryReadMapObjectConditions(player,&a,&b)) return false;
    using CE=RuntimeOffsets::ConditionEffects;
    const auto conditions=RuntimeOffsets::GetFullConditions(a,b);
    return RuntimeOffsets::HasCondition(conditions,CE::Paralyzed) ||
        RuntimeOffsets::HasCondition(conditions,CE::Petrified) ||
        RuntimeOffsets::HasCondition(conditions,CE::Stasis);
}
bool CanOccupyWorld(float x,float y,bool safeWalk) {
    if(!std::isfinite(x) || !std::isfinite(y)) return false;
    // The general live wall probe treats missing map tiles as open. Autonomous
    // movement must also reject unstreamed map edges/void, as UDodge navigation
    // already does. This check does not inherit the manual noclip override.
    unsigned char flags=0;
    WorldTAB::CopyBoxBlocked(x,y,1,1.f,kUOccPlayerHalfEdge,false,&flags);
    return (flags&0x9)==0 && Sensors::CanOccupy(x,y,safeWalk);
}
void CaptureZones(Input& in,Debug& d) {
    static std::vector<WorldAoe> aoes;
    aoes.clear(); AoeTracking::CopyActiveForDraw(aoes);
    d.aoeHooks=AoeTracking::CountHooks(); d.capturedAoes=static_cast<int>(aoes.size());
    for(const auto& a:aoes) {
        if(!a.valid || !a.isDamaging || (a.isEnemyChecked && !a.isEnemy)) continue;
        if(!std::isfinite(a.destX) || !std::isfinite(a.destY)) continue;
        // AoE spawn stamps use the coarse Windows uptime clock. Planner route
        // deadlines use QPC; never subtract timestamps from different clocks.
        const float elapsed=static_cast<float>(std::max(0.,static_cast<double>(GetTickCount64())-static_cast<double>(a.spawnTick)));
        const float end=AoeTracking::LifetimeMs(a)-elapsed+25.f;
        if(end<0.f) continue;
        if(in.zoneCount>=kMaxTimedZones) { map.limited=true; break; }
        const float radius=(std::isfinite(a.radius)&&a.radius>0.f)?std::clamp(a.radius,.2f,12.f):1.5f;
        in.zones[in.zoneCount++]={{a.destX,a.destY},radius,
            std::max(0.f,AoeTracking::LandDelayMs(a)-elapsed-35.f),end};
        if(a.source<d.aoeSources.size()) ++d.aoeSources[a.source];
        const auto& zone=in.zones[in.zoneCount-1];
        if(Len(Sub(zone.center,in.world.player))<radius && (d.nextBlastMs<0.f || zone.startsMs<d.nextBlastMs)) {
            d.nextBlastMs=zone.startsMs; d.blastRadius=radius;
        }
    }
}
}

void SetEnabled(bool value) { if(enabled.exchange(value)!=value) reset.store(true); }
bool IsEnabled() { return enabled.load(); }
void OnEnter() { reset.store(true); ProjectileTracking::Install(); AoeTracking::EnsureInstalled(); }
void SetDebugOverlay(bool value) { overlay.store(value); }
bool GetDebugOverlay() { return overlay.load(); }
void SetShadowMode(bool value) { if(shadow.exchange(value)!=value) reset.store(true); }
bool GetShadowMode() { return shadow.load(); }

static void SetValue(std::atomic<float>& target,float value,float lo,float hi) {
    if(!std::isfinite(value)) return;
    value=std::clamp(value,lo,hi);
    if(target.exchange(value)!=value) reset.store(true);
}
void SetLookRange(float value) {
    const float previous=GetLookRange(); SetValue(lookRange,value,2.f,32.f);
    if(previous!=GetLookRange()) rangePreviewUntil.store(GetTickCount64()+2000);
}
float GetLookRange() { return lookRange.load(); }
void SetContactScale(float value) { SetValue(contactScale,value,.25f,3.f); }
float GetContactScale() { return contactScale.load(); }
void SetHorizonMs(float value) { SetValue(horizonMs,value,400.f,4000.f); }
float GetHorizonMs() { return horizonMs.load(); }
void SetMaxDistance(float value) {
    const float previous=GetMaxDistance(); SetValue(maxDistance,value,.25f,12.f);
    if(previous!=GetMaxDistance()) distancePreviewUntil.store(GetTickCount64()+2000);
}
float GetMaxDistance() { return maxDistance.load(); }
void SetEnemyScale(float value) { SetValue(enemyScale,value,.1f,3.f); }
float GetEnemyScale() { return enemyScale.load(); }
void SetAvoidBlocks(bool value) { if(avoidBlocks.exchange(value)!=value) reset.store(true); }
bool GetAvoidBlocks() { return avoidBlocks.load(); }
void SetSearchBudgetMs(float value) { SetValue(searchBudgetMs,value,.5f,12.f); }
float GetSearchBudgetMs() { return searchBudgetMs.load(); }

void SetStationaryLookRange(float value) { SetValue(stationaryLookRange,value,2.0f,32.0f); }
float GetStationaryLookRange() { return stationaryLookRange.load(); }
void SetStationaryContactScale(float value) { SetValue(stationaryContactScale,value,0.25f,3.0f); }
float GetStationaryContactScale() { return stationaryContactScale.load(); }
void SetStationaryHorizonMs(float value) { SetValue(stationaryHorizonMs,value,400.0f,4000.0f); }
float GetStationaryHorizonMs() { return stationaryHorizonMs.load(); }
void SetStationaryMaxDistance(float value) { SetValue(stationaryMaxDistance,value,0.25f,12.0f); }
float GetStationaryMaxDistance() { return stationaryMaxDistance.load(); }
void SetStationaryEnemyScale(float value) { SetValue(stationaryEnemyScale,value,0.1f,3.0f); }
float GetStationaryEnemyScale() { return stationaryEnemyScale.load(); }
void SetStationarySearchBudgetMs(float value) { SetValue(stationarySearchBudgetMs,value,0.5f,12.0f); }
float GetStationarySearchBudgetMs() { return stationarySearchBudgetMs.load(); }
void SetBypassKey(int key) { bypassKey.store(std::clamp(key,0,255)); }
bool BypassHeld() { const int key=bypassKey.load(); return key>0 && (GetAsyncKeyState(key)&0x8000)!=0; }
void SetOverlayKey(int key) { overlayKey.store(std::clamp(key,0,255)); }

static bool RenderControls() {
    static bool stationary=false;
    ImGui::Checkbox("Edit stationary profile##st",&stationary);
    bool changed=false;
    float range=(stationary?GetStationaryLookRange():GetLookRange());
    if(ImGui::SliderFloat("Look range (tiles)##st",&range,2.f,32.f,"%.1f")) { (stationary?SetStationaryLookRange:SetLookRange)(range); changed=true; }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Includes projectile paths entering this radius during lookahead.\nShots whose whole path stays outside are excluded.");
    float scale=(stationary?GetStationaryContactScale():GetContactScale());
    if(ImGui::SliderFloat("Contact size##st",&scale,.25f,3.f,"%.2fx")) { (stationary?SetStationaryContactScale:SetContactScale)(scale); changed=true; }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Multiplies each projectile's collision radius in BOTH the solver and drawings.\nThe player remains a point. No cushion is added.");
    float horizon=(stationary?GetStationaryHorizonMs():GetHorizonMs());
    if(ImGui::SliderFloat("Lookahead (ms)##st",&horizon,400.f,4000.f,"%.0f")) { (stationary?SetStationaryHorizonMs:SetHorizonMs)(horizon); changed=true; }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Normal prediction horizon, not a departure timer.\nMovement still waits until required. Known bombs can extend this horizon.");
    float distance=(stationary?GetStationaryMaxDistance():GetMaxDistance());
    if(ImGui::SliderFloat("Normal dodge budget (tiles)##st",&distance,.25f,12.f,"%.2f")) { (stationary?SetStationaryMaxDistance:SetMaxDistance)(distance); changed=true; }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Maximum normal path budget, not a requested movement distance.\nIncreasing it need not change an already small dodge.\nKnown wide blasts can extend path/time budgets to permit escape.");
    float enemies=(stationary?GetStationaryEnemyScale():GetEnemyScale());
    if(ImGui::SliderFloat("Enemy avoidance size##st",&enemies,.1f,3.f,"%.2fx")) { (stationary?SetStationaryEnemyScale:SetEnemyScale)(enemies); changed=true; }
    bool blocks=GetAvoidBlocks();
    if(ImGui::Checkbox("Steer around harmless blocks while walking##st",&blocks)) { SetAvoidBlocks(blocks); changed=true; }
    float search=(stationary?GetStationarySearchBudgetMs():GetSearchBudgetMs());
    if(ImGui::SliderFloat("Route search budget (ms)##st",&search,.5f,12.f,"%.1f")) { (stationary?SetStationarySearchBudgetMs:SetSearchBudgetMs)(search); changed=true; }
    return changed;
}

namespace {
bool ReadPlayerPosition(void* player,float& x,float& y) {
    __try {
        x=*reinterpret_cast<float*>(static_cast<unsigned char*>(player)+RuntimeOffsets::PosX);
        y=*reinterpret_cast<float*>(static_cast<unsigned char*>(player)+RuntimeOffsets::PosY);
        return std::isfinite(x) && std::isfinite(y);
    } __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
}
void NativeTickGuarded(void* player,float x,float y,float dt) {
    nativeMode=true;
    __try { Tick(player,x,y,dt); }
    __finally { nativeMode=false; }
}
bool FilterWalkingMove(void* player,float requestedX,float requestedY,float& x,float& y) {
    if(!IsEnabled() || BootGate::Degraded()) return false;
    SteerInput::Tick();
    if(!SteerInput::Get().active) return false;
    ++nativeCalls;
    float px=0.f,py=0.f;
    if(!ReadPlayerPosition(player,px,py)) return false;
    const float dt=DodgeRuntime::GetDeltaTime();
    Input probe{};
    probe.world.player={px,py}; probe.world.speed=DodgeRuntime::GetVerifiedTilesPerSec(player)/1000.f;
    probe.frameMs=std::clamp(dt*1000.f,1.f,50.f);
    if(!PrepareNativeMovement(probe,{requestedX,requestedY})) {
        // A rejected native call must not erase the active route or publish an
        // empty world snapshot. The last evaluated drawing expires normally.
        ++nativeSkipped;
        return false;
    }
    if(UDodge::LenSq(UDodge::Sub({requestedX,requestedY},{px,py}))>1e-10f)
        DodgeRuntime::AccountNativeMovement();
    nativeRequested={requestedX,requestedY}; nativeChanged=false; nativeHandled=true;
    lastNativeTick=GetTickCount64();
    NativeTickGuarded(player,px,py,probe.frameMs/1000.f);
    if(!nativeChanged) return false;
    x=nativeTarget.x; y=nativeTarget.y;
    return true;
}
void NativeMoveFeedback(bool accepted,float,float) {
    if(accepted) ++nativeOverrides;
    else { ++nativeFailures; state.Reset(); }
    if(replayRecorder.hasPrevious && replayRecorder.previous.native) {
        replayRecorder.previous.applied=accepted; replayRecorder.previous.rejected=!accepted;
        replayRecorder.previous.feedbackKnown=true;
    }
    std::lock_guard<std::mutex> lock(debugMutex);
    debug.applied=accepted; debug.rejected=!accepted;
    debug.nativeOverrides=nativeOverrides.load(); debug.nativeFailures=nativeFailures.load();
    if(accepted) ++debug.moves; else ++debug.rejections;
    debug.action=accepted?"native walking step replaced":"native walking step rejected";
}
}
void BeginGameUpdate() { nativeHandled=false; }
void Tick(void* player,float x,float y,float dt) {
    if(!IsEnabled()) return;
    if(!nativeMode && nativeHandled) return;
    if(!nativeMode) DodgeRuntime::EnsureMovementFilter(player,&FilterWalkingMove,&NativeMoveFeedback);
    Debug d{};
    d.native=nativeMode;
    const auto firing=player?TargetAssist::GetFiringZone(player):TargetAssist::FiringZone{};
    const bool scriptMoving=FeatureState::GetWalkTargetActive() &&
        Len(Sub({FeatureState::GetWalkTargetX(),FeatureState::GetWalkTargetY()},{x,y}))>.2f;
    const bool assistMoving=firing.active && (!TargetAssist::CanHitFrom(firing,x,y) ||
        (navigation.entering && !TargetAssist::CanHitFrom(firing,x,y,std::min(.12f,firing.range*.05f))));
    d.movingProfile=SteerInput::Get().active || scriptMoving || assistMoving;
    if(d.movingProfile!=lastMovingProfile) { state.Reset(); map=DangerMap{}; lastMovingProfile=d.movingProfile; }
    static bool overlayWasDown=false;
    const int overlayVk=overlayKey.load();
    const bool overlayDown=overlayVk>0 && (GetAsyncKeyState(overlayVk)&0x8000)!=0;
    if(overlayDown && !overlayWasDown) SetDebugOverlay(!GetDebugOverlay());
    overlayWasDown=overlayDown;
    d.lookRange=(d.movingProfile?GetLookRange():GetStationaryLookRange()); d.contactScale=(d.movingProfile?GetContactScale():GetStationaryContactScale());
    d.horizonMs=(d.movingProfile?GetHorizonMs():GetStationaryHorizonMs()); d.maxDistance=(d.movingProfile?GetMaxDistance():GetStationaryMaxDistance());
    d.enemyScale=(d.movingProfile?GetEnemyScale():GetStationaryEnemyScale()); d.avoidBlocks=GetAvoidBlocks(); d.searchBudgetMs=(d.movingProfile?GetSearchBudgetMs():GetStationarySearchBudgetMs());
    d.requestedHorizonMs=d.horizonMs; d.requestedMaxDistance=d.maxDistance;
    d.nowMs=DodgeRuntime::GetMovementTimeMs(); d.player={x,y};
    if(!player || !std::isfinite(x) || !std::isfinite(y) || !DodgeRuntime::EnsureResolved()) {
        d.out.reason=Reason::MissingMap; d.action="player/runtime unavailable";
        state.Reset(); navigation.Reset(); Publish(d); return;
    }
    const double now=d.nowMs;
    const float frame=std::clamp(dt*1000.f,1.f,50.f);
    if(reset.exchange(false) || player!=lastPlayer || now-lastTick>250. ||
       Len(Sub({x,y},lastPosition))>3.f) {
        state.Reset(); navigation.Reset(); map=DangerMap{}; replayRecorder.hasPrevious=false;
    }
    lastPlayer=player; lastTick=now; lastPosition={x,y};
    d.bypass=BypassHeld();
    if(d.bypass) { state.Reset(); navigation.Reset(); d.out.status=Status::Clear; d.action="Movement bypass"; Publish(d); return; }
    const float speed=DodgeRuntime::GetVerifiedTilesPerSec(player);
    if(Locked(player) || speed==0.f) {
        state.Reset(); navigation.Reset(); d.out.status=Status::Locked; d.action="movement condition / zero speed"; Publish(d); return;
    }
    if(!(speed>0.f) || !std::isfinite(speed)) {
        state.Reset(); navigation.Reset(); d.out.reason=Reason::InvalidInput; d.action="live speed unreadable"; Publish(d); return;
    }
    d.speed=speed;
    d.observedSpeed=DodgeRuntime::GetObservedTilesPerSec();
    LARGE_INTEGER captureStart{},captureEnd{},captureFreq{};
    QueryPerformanceFrequency(&captureFreq); QueryPerformanceCounter(&captureStart);
    ProjectileTracking::Install(); AoeTracking::EnsureInstalled();
    UDodge::Settings sensorSettings{};
    sensorSettings.safeWalk=true;
    sensorSettings.hitScale=d.contactScale;
    sensorSettings.projectileCollisionThreshold=true;
    sensorSettings.captureRangeTiles=std::max(16.f,d.lookRange);
    sensorSettings.captureHorizonMs=std::max(kLaneCoverMs,d.horizonMs);
    sensorSettings.enemyAvoidanceScale=d.enemyScale;
    uint32_t tick=0; const bool tickValid=Sensors::ReadWorldTick(tick);
    if(!map.tickValid || !tickValid || tick!=map.tickId ||
       !Sensors::ReanchorMap(map,x,y,sensorSettings)) {
        Sensors::BuildMap(map,x,y,sensorSettings);
        map.tickId=tick; map.tickValid=tickValid;
    }
    Input in{};
    in.settings.lookRange=d.lookRange;
    in.settings.horizonMs=d.horizonMs;
    in.settings.maxDistance=d.maxDistance;
    in.settings.avoidHarmlessBlocks=d.avoidBlocks;
    in.collectDiagnostics=GetDebugOverlay();
    in.nowMs=now; in.frameMs=frame;
    in.world.player={x,y}; in.world.speed=speed/1000.f;
    in.world.map=&map; in.world.settings=sensorSettings;
    in.world.env.canOccupy=&CanOccupyWorld;
    in.world.env.isHazard=&Sensors::IsHazardAt;
    in.world.playerOnHazard=Sensors::IsHazardAt(x,y);
    const auto steer=SteerInput::Get();
    if(steer.active && !nativeMode) {
        // Some game builds schedule player movement outside AppEngine.Update.
        // That native call still owns walking; do not reset its retained route
        // or overwrite its diagnostics just because this hook runs separately.
        if(lastNativeTick && GetTickCount64()-lastNativeTick<250) return;
        d.out.reason=Reason::CaptureUnavailable;
        d.action=DodgeRuntime::MovementFilterInstalled()?"waiting for native walking step":"walking hook unavailable";
        Publish(d); return;
    }
    in.settings.maxSearchMs=std::min(d.searchBudgetMs,frame*.7f);
    d.searchBudgetMs=in.settings.maxSearchMs;
    in.settings.recoveryBudgetMs=std::min(2.f,frame*.25f);
    if(nativeMode) {
        if(!PrepareNativeMovement(in,nativeRequested)) { ++nativeSkipped; return; }
    } else {
        if(!PrepareFrameMovement(in,DodgeRuntime::GetFrameMoveMs())) {
            d.deferred=true; d.action="movement already owned / awaiting game update";
            Publish(d); return;
        }
    }
    in.settings.dwellMs=std::min(160.f,std::max(0.f,in.settings.horizonMs-in.settings.leadMs-in.settings.stepMs));
    CaptureZones(in,d);
    ApplyZonePlanningLimits(in);
    d.zoneLimits=in.expandedForZones; d.horizonMs=in.settings.horizonMs; d.maxDistance=in.settings.maxDistance;
    d.terrainAtPlayer=!CanOccupyWorld(x,y,false);
    QueryPerformanceCounter(&captureEnd);
    d.captureMs=static_cast<float>(double(captureEnd.QuadPart-captureStart.QuadPart)*1000./double(captureFreq.QuadPart));
    d.nominal=in.nominal; d.frameMs=frame; d.stepMs=in.settings.stepMs;
    d.leadMs=in.settings.leadMs; d.availableMs=nativeMode?in.frameMs:DodgeRuntime::GetFrameMoveMs();
    LARGE_INTEGER begin{},end{},freq{};
    QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&begin);
    MovementGoal goal{};
    goal.active=firing.active; goal.identity=static_cast<uint64_t>(firing.targetId);
    goal.center={firing.targetX,firing.targetY}; goal.range=firing.range; goal.context=&firing;
    goal.contains=[](const void* context,Vec2 position) {
        return TargetAssist::CanHitFrom(*static_cast<const TargetAssist::FiringZone*>(context),position.x,position.y);
    };
    goal.arrivalContains=[](const void* context,Vec2 position) {
        const auto& zone=*static_cast<const TargetAssist::FiringZone*>(context);
        return TargetAssist::CanHitFrom(zone,position.x,position.y,std::min(.12f,zone.range*.05f));
    };
    const WaypointGoal waypoint{FeatureState::GetWalkTargetActive(),
        {FeatureState::GetWalkTargetX(),FeatureState::GetWalkTargetY()},.2f};
    goal=SelectMovementGoal(waypoint,goal);
    MovementDecision decision{};
    EvaluateMovement(in,steer.active,goal,state,navigation,decision);
    d.overrideActive=decision.overrideActive;
    d.evaluated=true;
    d.out=decision.dodge; d.hasGoal=goal.active; d.goal=goal.center; d.approaching=decision.approaching;
    d.goalWaypoint=goal.waypoint; d.detouring=decision.detouring;
    if(navigation.next<navigation.count) d.waypoint=navigation.route[navigation.next];
    QueryPerformanceCounter(&end);
    d.solveMs=static_cast<float>(double(end.QuadPart-begin.QuadPart)*1000./double(freq.QuadPart));
    CaptureDebugShapes(d,in);
    d.action=GetShadowMode()?"preview only":d.out.status==Status::Waiting?"waiting for departure":
        d.out.status==Status::Recovery && Len(Sub(d.out.velocity,in.nominal))<=1e-6f?"waiting for recovery departure":
        d.out.status==Status::NoPlan || d.out.status==Status::Incomplete?"no executable route":decision.action;
    if(nativeMode) {
        Vec2 target{};
        if(!GetShadowMode()) {
            const float liveSpeed=DodgeRuntime::GetVerifiedTilesPerSec(player)/1000.f;
            if(!std::isfinite(liveSpeed) || liveSpeed<=0.f) {
                // A restriction appearing during search cannot authorize the
                // old-speed endpoint. Retain ownership while movement is unavailable.
                nativeTarget=in.world.player; nativeChanged=Len(Sub(nativeRequested,nativeTarget))>1e-6f;
                state.Reset();
                navigation.overrideActive=true; d.overrideActive=true; d.out.velocity={};
                d.out.status=liveSpeed==0.f?Status::Locked:Status::Incomplete;
                d.action="movement restricted / native step held";
                replayRecorder.Observe(in,d); Publish(d); return;
            }
            const bool speedChanged=std::fabs(liveSpeed-in.world.speed)>1e-6f;
            const bool usable=!speedChanged && d.captureMs+d.solveMs<=frame;
            if(!usable) { state.Reset(); }
            Input current=in;
            if(speedChanged && std::isfinite(liveSpeed) && liveSpeed>0.f) {
                current.world.speed=liveSpeed;
                current.nominal=Mul(Normalize(in.nominal),liveSpeed);
                current.maxCorrectionSpeed=liveSpeed*2.f;
            }
            if(ResolveNativeMovementTarget(current,d.out,usable,
                navigation.overrideActive || speedChanged,nativeRequested,target)) {
                nativeTarget=target; nativeChanged=true; d.command=Sub(target,in.world.player);
                if(decision.overrideActive || d.out.status==Status::Recovery) navigation.overrideActive=true;
                d.overrideActive=navigation.overrideActive;
                d.action="native walking replacement selected";
            }
            if(d.out.reason==Reason::CommandBlocked && d.out.status==Status::Recovery) {
                d.immediateGuard=true; state.Reset();
                d.action="dodge owns movement / emergency step";
            }
        }
        replayRecorder.Observe(in,d); Publish(d); return;
    }
    if((d.out.status==Status::Moving || d.out.status==Status::Recovery) &&
        Len(Sub(d.out.velocity,in.nominal))>1e-6f && !GetShadowMode()) {
        if(DodgeRuntime::GetFrameMoveMs()<in.frameMs) {
            d.deferred=true; d.action="movement already owned this update";
        }
        else {
            // Only automatic idle/approach movement reaches this actuator.
            // Walking is handled exclusively by native endpoint replacement.
            const float commandMs=in.frameMs;
            const Vec2 correction=MovementDisplacement(d.out.velocity,{},commandMs);
            const Vec2 target=Add(in.world.player,correction);
            d.command=correction;
            // Revalidate after search and never silently turn the route into a
            // wall slide. The existing game update has already applied keys.
            const float liveSpeed=DodgeRuntime::GetVerifiedTilesPerSec(player)/1000.f;
            const char* rejection=std::fabs(liveSpeed-in.world.speed)>1e-6f?"speed changed after search":
                Len(correction)>in.world.speed*in.frameMs+1e-4f?"command exceeds frame speed budget":
                !OccupancyPathClear(in.world,in.world.player,target)?"command terrain blocked":
                !EnemyPathClear(in.world,in.world.player,target)?"command enemy body blocked":
                d.solveMs>frame?"search exceeded render frame":nullptr;
            if(rejection) {
                d.action=rejection;
                d.rejected=true; state.Reset();
            } else {
                const auto result=DodgeRuntime::CallMoveToFrame(player,target.x,target.y,in.frameMs);
                d.deferred=result==DodgeRuntime::MoveResult::Deferred;
                d.rejected=result==DodgeRuntime::MoveResult::Rejected;
                d.applied=result==DodgeRuntime::MoveResult::Applied;
                d.action=d.applied?(d.approaching?decision.action:"dodge frame applied"):d.deferred?"movement already owned this update":"native frame move rejected";
                if(d.rejected) state.Reset();
            }
        }
    }
    static Status previous=Status::Incomplete;
    if(previous!=d.out.status) {
        DBG_FILE_LOG("[SpacetimeDodge] " << StatusName(d.out.status) << " waitMs=" << d.out.waitMs
            << " distance=" << d.out.plan.intervention << " searchMs=" << d.solveMs
            << " expansions=" << d.out.expansions);
        previous=d.out.status;
    }
    replayRecorder.Observe(in,d); Publish(d);
}

namespace {
bool showGrid=true,showOutlines=false,showPaths=false,showLookRange=false,showPlan=false;

ImU32 TimeColor(float time,int alpha,float horizon) {
    const auto c=DebugGeometry::TimeColor(time,horizon);
    return IM_COL32(c.r,c.g,c.b,alpha);
}
}

void RenderSettings() {
    ImGui::TextWrapped("Spacetime: latest safe departure, minimal correction. The configured bypass key returns manual control.");
    bool show=GetDebugOverlay();
    if(ImGui::Checkbox("Movement & threat overlay##spacetime",&show)) SetDebugOverlay(show);
    bool observe=GetShadowMode();
    if(ImGui::Checkbox("Preview only (do not move)##spacetime",&observe)) SetShadowMode(observe);
    RenderControls();
    const auto applied=ReadDebug();
    const bool fresh=applied.evaluated && DodgeRuntime::GetMovementTimeMs()-applied.nowMs<250.;
    const bool matching=fresh && applied.lookRange==(applied.movingProfile?GetLookRange():GetStationaryLookRange()) && applied.contactScale==(applied.movingProfile?GetContactScale():GetStationaryContactScale()) &&
        applied.requestedHorizonMs==(applied.movingProfile?GetHorizonMs():GetStationaryHorizonMs()) && applied.requestedMaxDistance==(applied.movingProfile?GetMaxDistance():GetStationaryMaxDistance());
    ImGui::Text("Active profile: %s",applied.movingProfile?"Moving":"Stationary");
    if(matching) ImGui::Text("Planner uses: %.1f tiles | %.2fx contact | %.0f ms | %.2f tiles",
        applied.lookRange,applied.contactScale,applied.horizonMs,applied.maxDistance);
    else ImGui::TextWrapped("Settings awaiting planner evaluation: %s",applied.action);
    if(matching && applied.zoneLimits)
        ImGui::TextWrapped("Bomb escape extends the selected %.0f ms / %.2f tiles to %.0f ms / %.2f tiles.",
            applied.requestedHorizonMs,applied.requestedMaxDistance,applied.horizonMs,applied.maxDistance);
    ImGui::TextDisabled("In-game edits last until dashboard reactivation or DLL reconnect.");
    if(ImGui::CollapsingHeader("Debug appearance and details##spacetime")) {
        bool recording=recordReplays.load();
        if(ImGui::Checkbox("Record diagnostic replays",&recording)) recordReplays.store(recording);
        if(recording) ImGui::TextWrapped("Captures up to eight rotating files in %%LOCALAPPDATA%%/RealmEngine-Spacetime-replayNN.bin. Recording lasts for this DLL session and works with the overlay hidden.");
        ImGui::Checkbox("Prediction grid",&showGrid);
        ImGui::Checkbox("Current collision outlines",&showOutlines);
        ImGui::Checkbox("Projectile paths",&showPaths);
        ImGui::Checkbox("Full planned route (inspection)",&showPlan);
        ImGui::Checkbox("Look range ring",&showLookRange);
        ImGui::TextWrapped("Pixel-scale world cells show earliest predicted projectile contact. Red = now, amber = halfway through lookahead, blue = the end. Purple = enemy body. Resolution follows zoom and projectile size, up to 1/128 tile, with a bounded texture size.");
        ImGui::TextWrapped("Ordinary bullets use the game's square contact threshold; beams use capsules. The player is a point for projectiles. Grid resolution affects drawing only; dodge collision checks remain continuous.");
        ImGui::TextWrapped("Mint arrow = selected steering. Faint dots = requested direction. These direction markers do not represent travel distance. Full routes are optional; they show exact segments without smoothing through obstacles.");
        const auto d=ReadDebug(); const auto& o=d.out; const auto& s=o.diagnostics;
        ImGui::TextWrapped("%s | %s",StatusName(o.status),d.action);
        if(d.hasGoal) ImGui::TextWrapped("Scripts supply travel goals; Target Assist supplies firing zones. Spacetime alone selects and executes movement.");
        ImGui::Text("Speed %.2f t/s | capture %.2f ms | solve %.2f ms",d.speed,d.captureMs,d.solveMs);
        ImGui::Text("Profile: %s",d.movingProfile?"Moving":"Stationary");
        ImGui::Text("Lookahead %.0f ms | search cap %.1f ms | enemy size %.2fx",d.horizonMs,d.searchBudgetMs,d.enemyScale);
        ImGui::Text("Harmless block steering: %s",d.avoidBlocks?"on":"off");
        ImGui::Text("Shots %d shown / %d considered / %d captured",d.drawnLanes,d.considered,d.bullets);
        ImGui::Text("Bomb capture: %d/4 hooks | %d captured / %d modeled",d.aoeHooks,d.capturedAoes,d.zones);
        ImGui::Text("Sources: thrown %d / visual %d / explosion %d / packet %d",
            d.aoeSources[0],d.aoeSources[1],d.aoeSources[2],d.aoeSources[3]);
        if(d.nextBlastMs>=0.f) ImGui::Text("Containing blast: %.2f tiles | lands in %.0f ms",d.blastRadius,d.nextBlastMs);
        if(d.zoneLimits) ImGui::Text("Blast escape budget: %.0f ms / %.2f tiles",d.horizonMs,d.maxDistance);
        if(d.terrainAtPlayer) ImGui::TextWrapped("Current position fails terrain occupancy; inspect the wall/map boundary.");
        ImGui::TextWrapped("Reason: %s | replan: %s",ReasonName(o.reason),ReasonName(o.replanReason));
        ImGui::Text("Departure %.0f ms | correction %.2f tiles",o.waitMs,o.plan.intervention);
        if(o.status==Status::Recovery) {
            if(o.expectedHits==0) ImGui::TextWrapped("Emergency step: no predicted hits in the checked interval; full route unavailable.");
            else ImGui::Text("Predicted contacts: %d | damage estimate %.0f%s",o.expectedHits,o.estimatedDamage,
                o.unknownDamage?" (includes unknown)":" (raw damage)");
        }
        ImGui::Text("Applied %u | deferred %u | rejected %u",d.moves,d.deferrals,d.rejections);
        ImGui::Text("Walking hook: %s | calls %u / replacements %u / rejected %u",
            d.walkingHook?"installed":"unavailable",d.nativeCalls,d.nativeOverrides,d.nativeFailures);
        ImGui::Text("Movement owner: %s",d.overrideActive?"dodge (requested path unsafe)":"player / navigation");
        ImGui::Text("Non-walking/invalid native calls skipped: %u",nativeSkipped.load());
        ImGui::Text("Rejects: shots %d / terrain %d / enemies %d / zones %d",
            s.projectileRejects,s.terrainRejects,s.enemyRejects,s.zoneRejects);
        ImGui::Text("Frame %.1f ms | command gate %.1f ms | budget %.1f ms",d.frameMs,d.leadMs,d.availableMs);
        ImGui::TextWrapped("Last failure: %s",d.lastFailure);
    }
}

namespace {
struct RasterFrame {
    std::vector<uint32_t> pixels;
    int width=0,height=0,originX=0,originY=0;
    float step=0.f;
    double capture=-1.;
};
struct RasterWorker {
    struct Request { Debug snapshot; Vec2 viewMin{},viewMax{}; float step=.5f; } pending;
    std::mutex mutex;
    std::condition_variable wake;
    std::thread thread;
    bool stopping=false,hasPending=false,hasReady=false;
    RasterFrame ready;
    double submitted=-1.;
    std::array<float,8> lastCamera{};

    ~RasterWorker() { Stop(); }
    void Stop() {
        { std::lock_guard<std::mutex> lock(mutex); stopping=true; hasPending=false; }
        wake.notify_one();
        if(thread.joinable()) thread.join();
        hasReady=false; submitted=-1.;
    }
    void Submit(const Debug& d,Vec2 viewMin,Vec2 viewMax,float step,const std::array<float,8>& camera) {
        if(submitted==d.nowMs && lastCamera==camera) return;
        if(!thread.joinable()) {
            stopping=false;
            try {
                thread=std::thread([this] { Run(); });
                SetThreadPriority(thread.native_handle(),THREAD_PRIORITY_BELOW_NORMAL);
            }
            catch(...) { return; } // drawing failure cannot stop gameplay
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            // One pending snapshot, replaced by newer input. Never accumulate
            // old frames when the game / desktop is under load.
            pending.snapshot=d; pending.viewMin=viewMin; pending.viewMax=viewMax; pending.step=step;
            hasPending=true;
        }
        submitted=d.nowMs; lastCamera=camera;
        wake.notify_one();
    }
    bool Take(RasterFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        if(!hasReady) return false;
        std::swap(frame,ready); hasReady=false; // recycle previous pixel storage
        return true;
    }
    void Run() {
        DebugGeometry::Grid grid;
        RasterFrame output;
        Request request;
        for(;;) {
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock,[this] { return stopping || hasPending; });
                if(stopping) return;
                request=pending; hasPending=false;
            }
            try {
                const auto& d=request.snapshot;
                grid.Reset(d.player,d.lookRange,request.step,request.viewMin,request.viewMax);
                for(int i=0;i<d.drawnLanes;i++) grid.Lane(d.lanes[i],d.lanes[i].radius,d.horizonMs);
                for(int i=0;i<d.drawnEnemies;i++) {
                    const auto& e=d.enemyShapes[i];
                    const float radius=(e.radius+kUPlayerHalf)*(e.passiveScenery?1.f:d.enemyScale);
                    grid.Segment(e.pos,e.pos,radius,0.f,0.f,true);
                }
                output.width=grid.width; output.height=grid.height;
                output.originX=grid.originX; output.originY=grid.originY; output.step=grid.step;
                output.pixels.resize(static_cast<size_t>(grid.width)*grid.height);
                grid.WritePixels(output.pixels.data(),grid.width*sizeof(uint32_t),d.horizonMs);
                output.capture=d.nowMs;
                std::lock_guard<std::mutex> lock(mutex);
                std::swap(output,ready); hasReady=true;
            } catch(...) {
                // Keep the worker alive after allocation failure. Stale-frame
                // rejection in the renderer prevents displaying obsolete data.
            }
        }
    }
} rasterWorker;

// One texture quad instead of millions of per-cell ImGui polygons. These
// resources are owned exclusively by the render thread / DirectX shutdown.
struct GridTexture {
    RasterFrame grid;
    ID3D11Texture2D* texture=nullptr;
    ID3D11ShaderResourceView* view=nullptr;
    ID3D11Device* device=nullptr;
    int width=0,height=0;
    bool valid=false;
    void Release() {
        if(view) { view->Release(); view=nullptr; }
        if(texture) { texture->Release(); texture=nullptr; }
        device=nullptr; width=height=0; valid=false;
    }
    bool Upload() {
        auto* current=DirectX::pDevice;
        auto* context=DirectX::pContext;
        if(!current || !context || grid.width<=0 || grid.height<=0) return false;
        if(device!=current || width<grid.width+1 || height<grid.height+1) {
            Release();
            D3D11_TEXTURE2D_DESC desc{};
            // Grow in chunks: subpixel camera movement must not recreate GPU
            // resources merely because the clipped grid gained one row.
            desc.Width=((grid.width+64)/64)*64; desc.Height=((grid.height+64)/64)*64;
            desc.MipLevels=desc.ArraySize=1;
            desc.Format=DXGI_FORMAT_R8G8B8A8_UNORM; desc.SampleDesc.Count=1;
            desc.Usage=D3D11_USAGE_DYNAMIC;
            desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;
            desc.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
            if(FAILED(current->CreateTexture2D(&desc,nullptr,&texture))) return false;
            if(FAILED(current->CreateShaderResourceView(texture,nullptr,&view))) { Release(); return false; }
            device=current; width=desc.Width; height=desc.Height;
        }
        D3D11_MAPPED_SUBRESOURCE mapped{};
        if(FAILED(context->Map(texture,0,D3D11_MAP_WRITE_DISCARD,0,&mapped))) return false;
        for(int y=0;y<grid.height;y++)
            std::memcpy(static_cast<unsigned char*>(mapped.pData)+mapped.RowPitch*y,
                grid.pixels.data()+static_cast<size_t>(grid.width)*y,grid.width*sizeof(uint32_t));
        // Linear sampling at the cropped right/bottom edge sees transparency,
        // never uninitialised or previous-frame texture contents.
        for(int y=0;y<grid.height;y++)
            reinterpret_cast<uint32_t*>(static_cast<unsigned char*>(mapped.pData)+mapped.RowPitch*y)[grid.width]=0;
        std::memset(static_cast<unsigned char*>(mapped.pData)+mapped.RowPitch*grid.height,0,(grid.width+1)*sizeof(uint32_t));
        context->Unmap(texture,0);
        return true;
    }
} gridTexture;
}

void ReleaseDebugResources() { rasterWorker.Stop(); replayRecorder.Stop(); gridTexture.Release(); }

void RenderDebugOverlay(float camX,float camY,float angle,float zoom,float cx,float cy) {
    if(!IsEnabled() || !GetDebugOverlay()) return;
    const auto d=ReadDebug();
    auto* draw=ImGui::GetBackgroundDrawList(); // world debug stays below settings/menu
    const auto display=ImGui::GetIO().DisplaySize;
    if(!draw || !std::isfinite(zoom) || zoom<=0.f || !std::isfinite(angle) ||
       !std::isfinite(camX) || !std::isfinite(camY) || !std::isfinite(cx) || !std::isfinite(cy)) return;
    const auto project=[&](Vec2 p) {
        ImVec2 screen;
        W2S(p.x,p.y,screen.x,screen.y,camX,camY,angle,zoom,cx,cy);
        return screen;
    };
    const auto line=[&](Vec2 a,Vec2 b,ImU32 color,float width=1.f) {
        const auto sa=project(a),sb=project(b);
        if(std::isfinite(sa.x) && std::isfinite(sa.y) && std::isfinite(sb.x) && std::isfinite(sb.y))
            draw->AddLine(sa,sb,color,width);
    };
    const auto circle=[&](Vec2 p,float radius,ImU32 color,int fillAlpha=0,float width=1.f) {
        DebugGeometry::Disk disk;
        if(!DebugGeometry::ProjectDisk(p,radius,camX,camY,angle,zoom,cx,cy,disk)) return;
        if(disk.center.x+disk.radius<0.f || disk.center.y+disk.radius<0.f ||
           disk.center.x-disk.radius>display.x || disk.center.y-disk.radius>display.y) return;
        const ImVec2 center(disk.center.x,disk.center.y);
        if(fillAlpha>0) draw->AddCircleFilled(center,disk.radius,(color&~IM_COL32_A_MASK)|(fillAlpha<<IM_COL32_A_SHIFT),32);
        draw->AddCircle(center,disk.radius,color,32,width);
    };
    draw->PushClipRect(ImVec2(0,0),display,true);
    const bool fresh=d.nowMs>0. && DodgeRuntime::GetMovementTimeMs()-d.nowMs<250.;
    if(fresh && showGrid) {
        auto& grid=gridTexture.grid;
        const std::array<float,8> camera={camX,camY,angle,zoom,cx,cy,display.x,display.y};
        if(rasterWorker.submitted!=d.nowMs || rasterWorker.lastCamera!=camera) {
            Vec2 viewMin{1e6f,1e6f},viewMax{-1e6f,-1e6f};
            const ImVec2 screenCorners[4]={{-2.f,-2.f},{display.x+2.f,-2.f},
                {display.x+2.f,display.y+2.f},{-2.f,display.y+2.f}};
            for(const auto& corner:screenCorners) {
                Vec2 p;
                S2W(corner.x,corner.y,p.x,p.y,camX,camY,angle,zoom,cx,cy);
                viewMin.x=std::min(viewMin.x,p.x); viewMin.y=std::min(viewMin.y,p.y);
                viewMax.x=std::max(viewMax.x,p.x); viewMax.y=std::max(viewMax.y,p.y);
            }
            float smallestRadius=1.f;
            for(int i=0;i<d.drawnLanes;i++) if(d.lanes[i].radius>0.f)
                smallestRadius=std::min(smallestRadius,d.lanes[i].radius);
            rasterWorker.Submit(d,viewMin,viewMax,DebugGeometry::Grid::Resolution(zoom,smallestRadius),camera);
        }
        if(rasterWorker.Take(grid)) gridTexture.valid=gridTexture.Upload();
        const float opacity=DebugGeometry::GridOpacity(DodgeRuntime::GetMovementTimeMs()-grid.capture);
        if(gridTexture.valid && gridTexture.view && opacity>0.f) {
            const Vec2 lo{grid.originX*grid.step,grid.originY*grid.step};
            const Vec2 hi{lo.x+grid.width*grid.step,lo.y+grid.height*grid.step};
            draw->AddImageQuad(static_cast<ImTextureID>(reinterpret_cast<uintptr_t>(gridTexture.view)),
                project(lo),project({hi.x,lo.y}),project(hi),project({lo.x,hi.y}),
                {0.f,0.f},{static_cast<float>(grid.width)/gridTexture.width,0.f},
                {static_cast<float>(grid.width)/gridTexture.width,static_cast<float>(grid.height)/gridTexture.height},
                {0.f,static_cast<float>(grid.height)/gridTexture.height},IM_COL32(255,255,255,static_cast<int>(255.f*opacity)));
        }
    }
    if(fresh) {
        if(showOutlines) for(int i=0;i<d.drawnEnemies;i++) {
            const auto& e=d.enemyShapes[i];
            circle(e.pos,(e.radius+kUPlayerHalf)*(e.passiveScenery?1.f:d.enemyScale),IM_COL32(195,135,235,150));
        }
        for(int i=0;i<d.drawnLanes;i++) {
            const auto& lane=d.lanes[i];
            if(showPaths) DebugGeometry::ForEachSegment(lane,0.f,d.horizonMs,[&](Vec2 a,Vec2 b,float ta,float tb) {
                // Split only for the color gradient. Source polyline corners
                // and lifetime endpoints are preserved exactly.
                const int pieces=std::max(1,static_cast<int>(std::ceil((tb-ta)/50.f)));
                for(int k=0;k<pieces;k++) {
                    const float lo=static_cast<float>(k)/pieces,hi=static_cast<float>(k+1)/pieces;
                    line(Add(a,Mul(Sub(b,a),lo)),Add(a,Mul(Sub(b,a),hi)),TimeColor(ta+(tb-ta)*(lo+hi)*.5f,130,d.horizonMs));
                }
            });
            Vec2 head{};
            if(!showOutlines || SampleProjectile(lane,0.f,head)!=SampleStatus::Known) continue;
            const ImU32 outline=lane.threat?IM_COL32(255,255,255,230):IM_COL32(230,220,190,135);
            if(lane.beam) circle(head,lane.radius,outline,12,lane.threat?1.8f:1.f);
            else {
                const float h=lane.radius;
                const Vec2 a=Add(head,{-h,-h}),b=Add(head,{h,-h}),c=Add(head,{h,h}),e=Add(head,{-h,h});
                line(a,b,outline); line(b,c,outline); line(c,e,outline); line(e,a,outline);
            }
            if(lane.beam && lane.pointCount>1) {
                const Vec2 end=lane.points[lane.pointCount-1];
                const Vec2 axis=Normalize(Sub(end,head)),normal=Mul({-axis.y,axis.x},lane.radius);
                circle(end,lane.radius,outline);
                line(Add(head,normal),Add(end,normal),outline);
                line(Sub(head,normal),Sub(end,normal),outline);
            }
        }
        for(int i=0;i<d.drawnZones;i++) {
            const auto& zone=d.zoneShapes[i];
            if(zone.endsMs<0.f) continue;
            circle(zone.center,zone.radius,TimeColor(zone.startsMs,135,d.horizonMs));
        }
        if(showLookRange || GetTickCount64()<rangePreviewUntil.load())
            circle(d.player,d.lookRange,IM_COL32(150,170,195,150));
        if(GetTickCount64()<distancePreviewUntil.load()) {
            circle(d.player,d.requestedMaxDistance,IM_COL32(100,230,255,150));
            draw->AddText(project(Add(d.player,{0.f,-d.requestedMaxDistance})),IM_COL32(100,230,255,230),"Normal dodge budget");
        }
        const auto& plan=d.out.plan;
        const float age=static_cast<float>(d.nowMs-plan.epochMs);
        if(showPlan) for(int i=1;i<plan.count;i++) {
            if(plan.points[i].timeMs<age) continue;
            line(plan.points[i-1].timeMs<age?PositionAt(plan,age):plan.points[i-1].pos,
                plan.points[i].pos,IM_COL32(94,234,190,95),1.5f);
        }
        const ImVec2 origin=project(d.player);
        const Vec2 intent=Len(d.nominal)>1e-6f?Normalize(d.nominal):
            d.approaching?Normalize(Sub(d.goal,d.player)):Vec2{};
        const ImVec2 wanted=project(Add(d.player,intent)),selected=project(Add(d.player,Normalize(d.out.velocity)));
        const bool executable=!d.rejected && !d.deferred && d.evaluated &&
            d.out.status!=Status::Incomplete && d.out.status!=Status::NoPlan && d.out.status!=Status::Locked;
        DebugStyle::Guidance(draw,origin,{wanted.x-origin.x,wanted.y-origin.y},
            {selected.x-origin.x,selected.y-origin.y},executable,GetShadowMode());
    }
    const bool moving=Len(d.out.velocity)>1e-6f;
    const bool blocked=d.rejected || d.deferred || d.out.status==Status::NoPlan || d.out.status==Status::Incomplete;
    const char* status=!fresh?"Awaiting game capture":d.bypass?"Manual control":
        d.out.status==Status::Locked?"Movement locked":blocked?"No validated passage":
        d.out.status==Status::Recovery?(d.out.expectedHits>0?"Reducing predicted contact":"Emergency steering"):
        !moving && d.out.waitMs>0.f?"Waiting for an opening":d.detouring?"Taking a safe detour":
        d.approaching?"Travelling to position":d.native && moving?"Following your direction":
        moving?"Avoiding contact":d.native && Len(d.nominal)>1e-6f?"Waiting for a safe passage":"Holding safe ground";
    ImU32 accent=blocked || d.out.status==Status::Recovery?DebugStyle::coral:
        !fresh || d.out.status==Status::Locked || (!moving && d.out.waitMs>0.f)?DebugStyle::amber:DebugStyle::mint;
    char detail[100];
    if(d.missing || d.limited) std::snprintf(detail,sizeof(detail),"Threat capture incomplete");
    else if(blocked) std::snprintf(detail,sizeof(detail),"%s",d.rejected || d.deferred?d.action:ReasonName(d.out.reason));
    else if(showGrid && gridTexture.valid && DodgeRuntime::GetMovementTimeMs()-gridTexture.grid.capture>100.)
        std::snprintf(detail,sizeof(detail),"Prediction refreshing  /  %.0f ms old",DodgeRuntime::GetMovementTimeMs()-gridTexture.grid.capture);
    else if(!moving && d.out.waitMs>0.f) std::snprintf(detail,sizeof(detail),"Resume in %.0f ms",d.out.waitMs);
    else std::snprintf(detail,sizeof(detail),"%s",d.bypass?"Movement bypass":d.native?"Keyboard intent  /  live speed":
        d.approaching?d.goalWaypoint?"Script destination  /  live speed":"Firing position  /  live speed":"Move only when needed");
    DebugStyle::Hud(draw,{16.f,std::max(16.f,display.y-104.f)},std::min(338.f,display.x-32.f),
        status,detail,accent,d.speed,d.considered,GetShadowMode());
    draw->PopClipRect();
}
}

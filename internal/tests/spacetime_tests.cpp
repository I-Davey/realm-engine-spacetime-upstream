#include "../src/features/movement/spacetime/SpacetimeCore.h"
#include "../src/features/movement/spacetime/SpacetimeMovement.h"
#include "../src/features/movement/spacetime/SpacetimeDebugGeometry.h"
#include "../src/features/movement/spacetime/SpacetimeReplay.h"
#include "../src/features/movement/udodge/TrajectoryPhase.h"
#include "../src/features/movement/dodge/AoeCapturePolicy.h"
#include "../src/features/movement/dodge/MovementFrameBudget.h"
#include "../src/features/combat/autoaim/core/WeaponProfileMath.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <cstring>
using namespace SpacetimeDodge;
using namespace UDodge;
static int failures = 0, checks = 0;
static void Check(bool ok, const char* name) {
    ++checks; if (!ok) { ++failures; std::printf("FAIL: %s\n", name); }
}
static DangerMap map;
static Input Scene() {
    map = DangerMap{};
    Input in{}; in.world.map = &map; in.world.speed = .005f;
    in.settings.leadMs = 0.f; in.nowMs = 10000.; in.frameMs = 20.f;
    return in;
}
static void Shot(Vec2 a, Vec2 b, float end, float half = .04f) {
    auto& lane = map.lanes[map.laneCount++];
    lane.points[0] = a; lane.points[1] = b;
    lane.pointTimesMs[1] = end;
    lane.pointCount = lane.instantCount = 2;
    lane.hitHalf = half; lane.tailAtShotEnd = true; lane.remainingLifeMs = end;
}
static bool Corner(float x,float y,bool) {
    return (x>=-.05f && x<=.65f && std::fabs(y)<=.055f) ||
        (x>=.55f && x<=.65f && y>=-.05f && y<=1.f);
}
static int Turns(const Plan& p) {
    Vec2 previous{}; int count=0;
    for(int i=1;i<p.count;i++) {
        const Vec2 d=Normalize(Sub(p.points[i].pos,p.points[i-1].pos));
        if(LenSq(d)<.5f) continue;
        if(LenSq(previous)>.5f && Dot(d,previous)<.8f) ++count;
        previous=d;
    }
    return count;
}
static bool GoalCircle(const void* context,Vec2 p) {
    const auto& g=*static_cast<const MovementGoal*>(context);
    return Len(Sub(p,g.center))<=g.range;
}
static bool GoalBehindWall(const void* context,Vec2 p) {
    return GoalCircle(context,p) && p.y>=.7f;
}
static bool WallWithOpening(float x,float y,bool) {
    return !(x>=.45f && x<=.65f && y<.6f);
}
static bool GroundAndWall(float x,float y,bool safeWalk) {
    if(x>.4f && x<.65f) return false;
    return !safeWalk || !(std::fabs(x)<.2f && std::fabs(y)<.2f);
}
static void WallAndBombTests() {
    auto in=Scene(); State state{}; Output out{};
    in.world.env.canOccupy=GroundAndWall; in.world.playerOnHazard=true;
    Check(!OccupancyPathClear(in.world,{0.f,0.f},{1.f,0.f}),
        "escaping damaging ground never bypasses an intervening wall");
    Check(OccupancyPathClear(in.world,{0.f,0.f},{0.f,1.f}),
        "ground escape can still move along the wall to safe ground");
    in=Scene(); in.world.speed=.008f; in.zoneCount=1;
    in.zones[0]={{0.f,0.f},3.5f,700.f,1000.f};
    ApplyZonePlanningLimits(in);
    Check(in.settings.maxDistance>3.5f && in.zones[0].radius==3.5f,
        "wide blast expands path budget without changing contact size");
    Evaluate(in,state,out);
    std::printf("Wide bomb: %s wait=%.0f distance=%.2f horizon=%.0f\n",StatusName(out.status),out.waitMs,out.plan.intervention,in.settings.horizonMs);
    Check(state.valid && out.status==Status::Waiting && Validate(in,out.plan),
        "centered wide bomb has a valid delayed escape beyond default micro-dodge cap");
    Check(out.waitMs>=200.f && Len(PositionAt(out.plan,700.f))>3.5f,
        "wide blast exits before impact without departing immediately");
    in=Scene(); state.Reset(); in.world.speed=.002f; in.zoneCount=1;
    in.zones[0]={{0.f,0.f},3.5f,2300.f,2500.f};
    ApplyZonePlanningLimits(in);
    Check(in.settings.horizonMs>=2300.f && in.expandedForZones,
        "known slow-speed bomb escape extends beyond the ordinary 800ms horizon");
    Evaluate(in,state,out);
    std::printf("Slow bomb: %s wait=%.0f distance=%.2f horizon=%.0f\n",StatusName(out.status),out.waitMs,out.plan.intervention,in.settings.horizonMs);
    Check(state.valid && Validate(in,out.plan) && Len(PositionAt(out.plan,2300.f))>3.5f,
        "Slow still clears a wide blast before its real deadline");
    Check(out.waitMs>=480.f,"slow blast escape retains its latest command-scale departure");
    const Plan slow=out.plan;
    in.world.speed=.004f;
    Check(!Validate(in,slow),"live speed change invalidates an extended bomb trajectory");
    in=Scene(); state.Reset(); in.world.speed=.008f; in.zoneCount=2;
    in.zones[0]={{0.f,0.f},3.5f,700.f,1000.f};
    in.zones[1]={{2.f,0.f},1.f,300.f,1000.f};
    in.world.env.canOccupy=WallWithOpening;
    ApplyZonePlanningLimits(in); Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan),"wide bomb escape sweeps the wall and second timed blast together");
    in=Scene(); in.zoneCount=1; in.zones[0]={{15.f,0.f},3.5f,3000.f,3200.f};
    ApplyZonePlanningLimits(in);
    Check(!in.expandedForZones && in.settings.horizonMs==800.f && in.settings.maxDistance==3.f,
        "unrelated distant bombs do not expand ordinary dodge limits");
    in.zones[0].center={}; in.zones[0].startsMs=3500.f;
    ApplyZonePlanningLimits(in);
    Check(!in.expandedForZones,"far-future bomb does not trigger premature planning expansion");
}
static void NativeWalkingTests() {
    auto in=Scene(); in.settings.leadMs=35.f; in.settings.stepMs=50.f;
    Check(PrepareNativeMovement(in,{.1f,0.f}) && in.settings.leadMs==0.f && in.actuationMs==20.f,
        "native walking replacement has no auxiliary command cooldown");
    Output selected{}; selected.status=Status::Moving; selected.velocity={0.f,.005f};
    Vec2 target{};
    Check(NativeMovementTarget(in,selected,target) && std::fabs(target.x)<1e-6f && std::fabs(target.y-.1f)<1e-6f,
        "sideways replacement starts from pre-move origin without subtracting phantom keyboard movement");
    selected.velocity={-.005f,0.f};
    Check(NativeMovementTarget(in,selected,target) && std::fabs(target.x+.1f)<1e-6f,
        "native reverse uses one legal step rather than a two-speed correction");
    selected.velocity={.006f,0.f};
    Check(!NativeMovementTarget(in,selected,target),"native replacement refuses excessive movement speed");
    selected.status=Status::Clear;
    Check(!NativeMovementTarget(in,selected,target),"clear input retains the game's exact original endpoint");
    in=Scene();
    Check(!PrepareNativeMovement(in,{5.f,0.f}),"native walking filter ignores teleports and scripted long moves");
    Check(PrepareNativeMovement(in,{0.f,0.f}) && Len(in.nominal)==0.f,
        "blocked native walking does not fabricate an applied keyboard displacement");
    in=Scene(); const Vec2 rotated={.07071068f,-.07071068f};
    Check(PrepareNativeMovement(in,rotated) && Len(Sub(Mul(in.nominal,in.frameMs),rotated))<1e-5f,
        "native world direction is used directly without another camera rotation");
    in=Scene(); in.frameMs=8.f;
    Check(PrepareNativeMovement(in,{.1f,0.f}) && std::fabs(in.frameMs-20.f)<.001f &&
        Len(in.nominal)<=in.world.speed+1e-6f,
        "20ms native walking during 8ms rendering uses bounded travel time instead of clearing the plan");
    const float preparedFrame=in.frameMs;
    Check(!PrepareNativeMovement(in,{5.f,0.f}) && in.frameMs==preparedFrame,
        "ignored scripted moves leave the prepared timing unchanged");
    for(int scenario=0;scenario<4;scenario++) {
        State state{}; NavigationState nav{}; MovementGoal goal{}; MovementDecision decision{};
        Vec2 position{}; float elapsed=0.f; bool safe=true,delayed=false; int redirects=0;
        for(int frame=0;frame<48;frame++) {
            in=Scene(); in.world.player=position;
            in.frameMs=scenario==3?8.f:scenario==1 && frame%2==0?8.333333f:16.666667f;
            in.world.speed=scenario==2 && frame>=13?.003f:.005f;
            in.nowMs=10000.+elapsed;
            if(elapsed<480.f) Shot({1.2f,-1.2f+.005f*elapsed},{1.2f,1.2f},480.f-elapsed,.08f);
            const Vec2 requested=Add(position,{in.world.speed*(scenario==3?20.f:in.frameMs),0.f});
            if(!PrepareNativeMovement(in,requested)) { safe=false; break; }
            EvaluateMovement(in,true,goal,state,nav,decision);
            if(decision.dodge.waitMs>20.f) delayed=true;
            Vec2 executed=requested;
            if(NativeMovementTarget(in,decision.dodge,executed)) ++redirects;
            if(Len(Sub(executed,position))>in.world.speed*in.frameMs+1e-5f) safe=false;
            if(elapsed<480.f) {
                const float span=std::min(in.frameMs,480.f-elapsed);
                const Vec2 bullet={1.2f,-1.2f+.005f*elapsed};
                const Vec2 a=Sub(bullet,position);
                const Vec2 b=Sub(Add(bullet,{0.f,.005f*span}),Add(position,Mul(Sub(executed,position),span/in.frameMs)));
                if(SweptProjectileContact(a,b,.08f)) safe=false;
            }
            // One native call applies exactly one chosen step. There is no
            // post-update correction and keys cannot undo the selected step.
            position=executed; elapsed+=in.frameMs;
        }
        Check(safe && redirects>0 && delayed,scenario==0?"held walking avoids crossing shot with delayed native replacement":
            scenario==1?"walking stays swept-safe across changing frame lengths":
            scenario==2?"walking revalidates when Slow changes an ongoing escape":
            "walking avoids the crossing shot when native movement uses 20ms and rendering reports 8ms");
    }
}
static void IdleActuatorTests() {
    for(const float frame:{8.333333f,16.666667f,21.4f}) {
        State state{}; NavigationState nav{}; MovementDecision decision{}; MovementGoal goal{};
        auto in=Scene(); in.frameMs=frame;
        Shot({-2.f,0.f},{2.f,0.f},800.f,.1f);
        PrepareAutomaticMovement(in,40.f,0.f,33.4f);
        Check(in.maxCorrectionSpeed==in.world.speed && in.actuationMs<=33.4f,
            "first idle command limits stroke duration without excluding full-speed candidates");
        EvaluateMovement(in,false,goal,state,nav,decision);
        Check(state.valid && Validate(in,decision.dodge.plan) && decision.dodge.waitMs>0.f,
            "standing still finds a late safe escape before any automatic move has ever been issued");
        // Advance the same shot toward its departure without resetting the
        // retained state. Simulate releasing keys before this idle evaluation.
        bool moved=false;
        for(float t=frame;t<400.f;t+=frame) {
            in=Scene(); in.frameMs=frame; in.nowMs+=t;
            Shot({-2.f+.005f*t,0.f},{2.f,0.f},800.f-t,.1f);
            PrepareAutomaticMovement(in,40.f,0.f,33.4f);
            EvaluateMovement(in,false,goal,state,nav,decision);
            if(decision.dodge.status==Status::Moving) {
                const auto step=MovementDisplacement(decision.dodge.velocity,{},in.frameMs);
                moved=Len(step)>0.f && Len(step)<=in.world.speed*33.4f+1e-4f;
                break;
            }
        }
        Check(moved,"idle wait eventually issues a distance-bounded dodge instead of freezing forever");
        in=Scene(); in.frameMs=frame; state.Reset();
        Shot({0.f,0.f},{0.f,0.f},800.f,.1f); map.lanes[0].damageEstimate=10.f;
        PrepareAutomaticMovement(in,40.f,0.f,33.4f);
        EvaluateMovement(in,false,goal,state,nav,decision);
        Check(decision.dodge.status==Status::Recovery && Len(decision.dodge.velocity)>0.f,
            "already-hit stationary player can still execute a recovery escape on the first command");
    }
    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision decision{}; MovementGoal goal{};
    Shot({-2.f,0.f},{2.f,0.f},800.f,.1f);
    PrepareNativeMovement(in,{.1f,0.f});
    EvaluateMovement(in,true,goal,state,nav,decision);
    in.nominal={}; in.frameMs=16.666667f;
    PrepareAutomaticMovement(in,40.f,0.f,33.4f);
    EvaluateMovement(in,false,goal,state,nav,decision);
    Check(state.valid && Validate(in,decision.dodge.plan) && Len(state.plan.nominal)==0.f,
        "key release hands a retained walking plan to a valid stationary dodge in the same evaluation");
}
static void WaypointTests() {
    auto in=Scene(); in.frameMs=40.f; in.actuationMs=40.f;
    State state{}; NavigationState nav{}; MovementDecision out{};
    MovementGoal firing{}; firing.active=true; firing.identity=42; firing.center={-3.f,0.f}; firing.range=1.f;
    firing.context=&firing; firing.contains=GoalCircle;
    WaypointGoal point{true,{3.f,0.f},.2f};
    auto goal=SelectMovementGoal(point,firing);
    Check(goal.waypoint && goal.center.x==3.f && goal.identity!=firing.identity,
        "explicit script waypoint takes ownership over an opposite combat firing zone");
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.dodge.velocity.x>0.f && Validate(in,out.dodge.plan),
        "script travel passes through the same continuous Spacetime safety evaluation");
    for(int i=0;i<20;i++) {
        EvaluateMovement(in,false,goal,state,nav,out);
        in.world.player=Add(in.world.player,Mul(out.dodge.velocity,in.frameMs)); in.nowMs+=in.frameMs;
    }
    Check(goal.contains(goal.context,in.world.player) && Len(out.dodge.velocity)==0.f,
        "script waypoint stops within arrival tolerance instead of walking indefinitely");
    point.active=false; goal=SelectMovementGoal(point,firing);
    Check(!goal.waypoint && goal.identity==42,"clearing a loot waypoint restores the combat goal");
    firing.active=false; goal=SelectMovementGoal(point,firing);
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && Len(out.dodge.velocity)==0.f,"clearing all script goals releases automatic travel");
    in=Scene(); in.frameMs=40.f; in.actuationMs=40.f;
    point={true,{2.f,0.f},.2f}; goal=SelectMovementGoal(point,firing);
    in.world.env.canOccupy=WallWithOpening; state.Reset(); nav.Reset();
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && Validate(in,out.dodge.plan),"waypoint travel finds a checked route around walls");
    in=Scene(); in.frameMs=40.f; in.actuationMs=40.f;
    in.nominal={-.005f,0.f}; state.Reset(); nav.Reset();
    EvaluateMovement(in,true,goal,state,nav,out);
    Check(!out.approaching && out.dodge.velocity.x==in.nominal.x,"keyboard movement takes priority over script travel");
    in.nominal={}; in.world.speed=.0025f; state.Reset(); nav.Reset();
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && Len(out.dodge.velocity)<=.002501f,"script travel reflects live Slow speed");
    in.world.speed=0.f; in.world.movementLocked=true; state.Reset(); nav.Reset();
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.dodge.status==Status::Locked && !out.approaching,"script travel cannot bypass paralysis");
}
static void MovementTests() {
    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision out{};
    MovementGoal goal{}; goal.active=true; goal.identity=1; goal.center={3.f,0.f}; goal.range=4.f;
    goal.context=&goal; goal.contains=GoalCircle;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.dodge.status==Status::Clear && !out.approaching && Len(out.dodge.velocity)==0.f,
        "target assist holds anywhere in firing zone without an inner ring");
    goal.range=1.f; in.frameMs=40.f; in.actuationMs=40.f;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.dodge.velocity.x>.0049f && Validate(in,out.dodge.plan),
        "automatic approach executes a swept-safe finite trajectory from rest");
    Check(Len(Sub(PositionAt(out.dodge.plan,800.f),PositionAt(out.dodge.plan,640.f)))<1e-5f &&
        PositionAt(out.dodge.plan,400.f).x>1.9f,
        "automatic approach validates its continuation and terminal dwell through lookahead");
    const Vec2 autoStep=MovementDisplacement(out.dodge.velocity,{},in.frameMs);
    Check(autoStep.x>.199f && autoStep.x<.201f,"unapplied assist movement is not subtracted as keyboard input");
    Check(Len(MovementDisplacement({.005f,0.f},{.005f,0.f},40.f))==0.f,
        "already applied safe keyboard movement is never applied twice");
    const Vec2 reverse=MovementDisplacement({-.005f,0.f},{.005f,0.f},20.f);
    Check(std::fabs(reverse.x+.2f)<1e-6f,"keyboard reverse dodge compensates the already applied step");
    // Simulate repeated legal automatic commands; no oscillating return to a
    // preferred ring and no movement once the firing region is reached.
    for(int i=0;i<16;i++) {
        EvaluateMovement(in,false,goal,state,nav,out);
        in.world.player=Add(in.world.player,MovementDisplacement(out.dodge.velocity,{},in.frameMs));
        in.nowMs+=40.;
    }
    Check(GoalCircle(&goal,in.world.player) && in.world.player.x<2.002f && !out.approaching,
        "approach reaches nearest firing boundary and stops without overshoot or inner-range retreat");
    in=Scene(); in.frameMs=40.f; in.actuationMs=40.f; nav.Reset(); state.Reset();
    goal.range=1.f;
    in.nominal={-.005f,0.f}; EvaluateMovement(in,true,goal,state,nav,out);
    Check(!out.approaching && out.dodge.velocity.x==in.nominal.x,"keyboard takes priority over target pursuit");
    in.nominal={}; goal.active=false;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(Len(out.dodge.velocity)==0.f && nav.count==0,"expired or cleared goal cancels automatic approach immediately");
    goal.active=true; in.world.speed=.0025f;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && Len(out.dodge.velocity)<=.002501f,"Slow changes automatic approach speed immediately");
    in.world.speed=.01f;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.dodge.velocity.x>.0099f,"Speedy permits a larger live-speed approach command");
    in.world.speed=0.f; in.world.movementLocked=true;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && out.dodge.status==Status::Locked,"paralysis blocks assist and dodge together");
    in=Scene(); in.frameMs=40.f; in.actuationMs=40.f; nav.Reset(); state.Reset();
    Shot({.2f,-1.f},{.2f,1.f},800.f,.03f);
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.dodge.velocity.x>0.f && Validate(in,out.dodge.plan),
        "safe travel crosses a lane instead of assuming a stop inside the incoming projectile");
    map.projectileSourceUnavailable=true;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && out.dodge.status==Status::Incomplete,"missing threat capture never authorizes pursuit");
    in=Scene(); nav.Reset(); state.Reset();
    Shot({-2.f,0.f},{2.f,0.f},800.f);
    Evaluate(in,state,out.dodge);
    goal.center={-3.f,1.f};
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && Validate(in,out.dodge.plan),
        "safe complete goal route replaces a waiting idle dodge");
    goal.active=false;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(state.valid && !out.approaching && Validate(in,out.dodge.plan),"clearing a target restores a needed standalone escape");
    goal.active=true; goal.center={1.f,0.f}; goal.range=3.f;
    in=Scene(); nav.Reset(); state.Reset();
    in.world.player={0.f,1.f};
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && Len(out.dodge.velocity)==0.f,"new safe firing position after dodge is accepted without returning");
    in=Scene(); nav.Reset(); state.Reset();
    goal.contains=GoalBehindWall; in.world.env.canOccupy=WallWithOpening;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && nav.count>0 && nav.route[nav.count-1].y>=.7f && Validate(in,out.dodge.plan),
        "in-range but occluded goal finds a reachable firing opening");
    goal.contains=GoalCircle; goal.center={3.f,0.f}; goal.range=1.f;
    in=Scene(); nav.Reset(); state.Reset(); map.enemyCount=1;
    map.enemies[0].pos={.8f,0.f}; map.enemies[0].radius=.3f;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && Validate(in,out.dodge.plan) &&
        EnemyPathClear(in.world,in.world.player,PositionAt(out.dodge.plan,in.frameMs)),
        "approach routes around the same physical enemy bodies used by dodge");
}
static void AssistRegressionTests() {
    using WeaponProfileMath::LifetimeMs;
    Check(std::fabs(150.f/10000.f*LifetimeMs(220.f,220.f)-3.3f)<.0001f,
        "Pirate King's Cutlass XML speed 150 and lifetime 220ms give 3.3 tiles, not 3300");
    Check(LifetimeMs(.22f,220.f)==220.f && LifetimeMs(220.f,0.f)==220.f &&
        LifetimeMs(.22f,0.f)==220.f,
        "weapon lifetime handles explicit milliseconds and runtime seconds");
    Check(LifetimeMs(0.f,0.f)==0.f && LifetimeMs(1.f,1500.f)==1500.f,
        "missing weapon lifetime stays unknown and explicit units take precedence");
    WeaponProfile profile{}; profile.isParametric=true; profile.lifetimeMs=0.f;
    Check(WeaponProfileMath::HasFlightModel(profile),"parametric range does not require a linear flight lifetime");
    profile.isParametric=false;
    Check(!WeaponProfileMath::HasFlightModel(profile),"linear projectile with unknown lifetime stays unavailable");

    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision decision{};
    MovementGoal goal{}; goal.active=true; goal.identity=44; goal.center={4.f,0.f}; goal.range=3.f;
    goal.context=&goal; goal.contains=GoalCircle;
    goal.arrivalContains=[](const void* context,Vec2 position) {
        const auto& g=*static_cast<const MovementGoal*>(context);
        return Len(Sub(position,g.center))<=g.range-.12f;
    };
    in.world.player={1.05f,0.f};
    EvaluateMovement(in,false,goal,state,nav,decision);
    Check(decision.dodge.status==Status::Clear && !decision.approaching,
        "arrival inset never moves a player already in the firing zone");
    in.world.player={};
    for(int i=0;i<20;i++) {
        in.frameMs=i%2?8.f:16.7f; in.nowMs+=40.f;
        PrepareAutomaticMovement(in,40.f,0.f,56.7f);
        EvaluateMovement(in,false,goal,state,nav,decision);
        if(decision.approaching) in.world.player=Add(in.world.player,MovementDisplacement(decision.dodge.velocity,{},in.frameMs));
    }
    Check(in.world.player.x>1.11f && in.world.player.x<1.13f && !decision.approaching,
        "outside approach finishes just inside the firing boundary and stops");
    bool steady=true;
    for(int i=0;i<12;i++) {
        goal.center.x=4.f+(i%2?.025f:-.025f);
        in.nowMs+=40.f;
        EvaluateMovement(in,false,goal,state,nav,decision);
        if(decision.approaching || Len(decision.dodge.velocity)>1e-6f) steady=false;
    }
    Check(steady,"small target position jitter does not repeatedly restart an arrived approach");
    auto a=Scene(); a.frameMs=16.f; PrepareAutomaticMovement(a,40.f,0.f,56.f);
    auto b=Scene(); b.frameMs=21.f; PrepareAutomaticMovement(b,40.f,0.f,56.f);
    Check(a.settings.stepMs==40.f && b.settings.stepMs==40.f,
        "render timing jitter leaves automatic command spacing unchanged");
    in=Scene(); state.Reset(); nav.Reset(); goal={};
    Shot({1.2f,-1.2f},{1.2f,1.2f},480.f,.08f);
    PrepareNativeMovement(in,{.1f,0.f});
    EvaluateMovement(in,true,goal,state,nav,decision);
    in.nominal.y=.000002f;
    EvaluateMovement(in,true,goal,state,nav,decision);
    Check(decision.dodge.reused && Validate(in,decision.dodge.plan),
        "small held-direction rounding retains a fully revalidated route");
    in.nominal={0.f,in.world.speed};
    EvaluateMovement(in,true,goal,state,nav,decision);
    Check(!decision.dodge.reused,"a real keyboard turn cannot retain the previous intent");
}

static void FrameMovementTests() {
    DodgeRuntime::MovementFrameBudget budget;
    Check(budget.Available(0.)==0.f,"automatic frame actuator cannot run outside game update");
    budget.Begin(100.,16.f);
    Check(budget.Claim(100.,16.f) && !budget.Claim(100.,16.f),"one automatic move per update");
    budget.End(); budget.Begin(108.,8.f);
    Check(budget.Available(108.)==8.f,"120fps movement uses eight milliseconds, not a 40ms jump");
    budget.Native(108.);
    Check(!budget.Claim(108.,8.f),"native walking consumes the automatic allowance");
    budget.End(); budget.Native(115.); budget.Begin(116.,8.f);
    Check(budget.Available(116.)==1.f,"key release cannot double-spend a late native walking step");
    budget.End(); budget.Begin(10000.,16.f);
    Check(budget.Available(10000.)==50.f,"long stalls never bank a large catch-up displacement");
    Check(!budget.Claim(10000.,51.f) && !budget.Claim(10000.,std::numeric_limits<float>::quiet_NaN()),
        "invalid and oversized frame claims fail");

    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision decision{};
    WaypointGoal waypoint{true,{10.f,0.f},.2f};
    const auto goal=SelectMovementGoal(waypoint,{});
    bool smooth=true; float expected=0.f;
    for(int i=0;i<75;i++) {
        const float frame=i%3==0?8.f:i%3==1?16.f:23.f;
        in.world.speed=i<25?.005f:i<50?.0025f:.008f;
        PrepareFrameMovement(in,frame);
        EvaluateMovement(in,false,goal,state,nav,decision);
        const Vec2 step=MovementDisplacement(decision.dodge.velocity,{},in.frameMs);
        expected+=in.world.speed*frame;
        smooth &= decision.approaching && std::fabs(Len(step)-in.world.speed*frame)<1e-5f &&
            std::fabs(step.y)<1e-6f;
        in.world.player=Add(in.world.player,step); in.nowMs+=frame;
    }
    Check(smooth && std::fabs(in.world.player.x-expected)<1e-4f,
        "clear approach moves every variable-length frame at full live speed through Slow and Speedy");
    in.world.speed=0.f; in.world.movementLocked=true;
    PrepareFrameMovement(in,16.f); EvaluateMovement(in,false,goal,state,nav,decision);
    Check(Len(decision.dodge.velocity)==0.f,"continuous movement stops immediately for paralysis");
    Check(!PrepareFrameMovement(in,0.f),"spent update never becomes an automatic motion proposal");

    in=Scene(); state.Reset(); nav.Reset();
    PrepareFrameMovement(in,8.f);
    Check(in.frameMs==8.f && in.settings.stepMs==40.f && in.settings.leadMs==0.f && in.actuationMs==0.f,
        "planning resolution is separate from continuous execution cadence");
    EvaluateMovement(in,false,{},state,nav,decision);
    Check(decision.dodge.status==Status::Clear && Len(decision.dodge.velocity)==0.f,
        "removing the throttle never moves a safe stationary player");
    Shot({-1.f,0.f},{1.f,0.f},400.f,.08f);
    EvaluateMovement(in,false,{},state,nav,decision);
    Check(state.valid && decision.dodge.waitMs>0.f && Len(decision.dodge.velocity)==0.f,
        "continuous actuator preserves late departure rather than moving immediately");
    const Plan retained=state.plan;
    bool safe=true,moved=false;
    for(int i=0;i<80;i++) {
        const float frame=i%2?8.f:16.f;
        PrepareFrameMovement(in,frame);
        const float age=static_cast<float>(in.nowMs-retained.epochMs);
        map.laneCount=0;
        if(age<400.f) Shot({-1.f+.005f*age,0.f},{1.f,0.f},400.f-age,.08f);
        EvaluateMovement(in,false,{},state,nav,decision);
        const Vec2 step=Mul(decision.dodge.velocity,frame);
        if(Len(step)>1e-6f) moved=true;
        safe &= Len(step)<=in.world.speed*frame+1e-4f &&
            decision.dodge.status!=Status::Recovery && decision.dodge.status!=Status::NoPlan &&
            decision.dodge.status!=Status::Incomplete;
        in.world.player=Add(in.world.player,step); in.nowMs+=frame;
    }
    Check(safe && moved,"variable-frame stationary dodge executes safely without a cooldown or overspeed");
}

static void IntegratedTravelTests() {
    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision out{};
    MovementGoal goal{}; goal.active=true; goal.identity=81; goal.center={3.f,0.f}; goal.range=1.f;
    goal.context=&goal; goal.contains=GoalCircle;
    PrepareFrameMovement(in,20.f);
    Shot({.6f,-.6f},{.6f,3.4f},800.f,.08f);
    Shot({0.f,-.2f},{0.f,3.8f},800.f,.08f); // waiting here is unsafe; a real side route is needed
    EvaluateMovement(in,false,goal,state,nav,out);
    bool lateral=false;
    for(int i=0;i<out.dodge.plan.count;i++) lateral |= std::fabs(out.dodge.plan.points[i].pos.y)>.01f;
    Check(out.approaching && out.detouring && Len(out.dodge.velocity)>.0049f &&
        lateral && Validate(in,out.dodge.plan),
        "bullet on walking path produces a safe full-speed detour toward the goal");
    const auto chosen=out.dodge.plan;
    in.world.player=Add(in.world.player,Mul(out.dodge.velocity,20.f)); in.nowMs+=20.;
    map.laneCount=0; Shot({.6f,-.5f},{.6f,3.4f},780.f,.08f);
    Shot({0.f,-.1f},{0.f,3.8f},780.f,.08f);
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.detouring && out.dodge.reused && out.dodge.plan.epochMs==chosen.epochMs &&
        Validate(in,out.dodge.plan),"safe detour keeps its side and rejoin deadline across updates");
    bool safe=true,progress=false;
    for(int i=0;i<45;i++) {
        const float age=static_cast<float>(in.nowMs-chosen.epochMs);
        map.laneCount=0;
        if(age<800.f) Shot({.6f,-.6f+.005f*age},{.6f,3.4f},800.f-age,.08f);
        if(age<800.f) Shot({0.f,-.2f+.005f*age},{0.f,3.8f},800.f-age,.08f);
        EvaluateMovement(in,false,goal,state,nav,out);
        if(out.approaching) safe &= Validate(in,out.dodge.plan);
        safe &= out.dodge.status!=Status::Recovery && out.dodge.status!=Status::NoPlan &&
            out.dodge.status!=Status::Incomplete && Len(out.dodge.velocity)<=in.world.speed+1e-5f;
        in.world.player=Add(in.world.player,Mul(out.dodge.velocity,in.frameMs)); in.nowMs+=in.frameMs;
        progress |= in.world.player.x>1.f;
    }
    Check(safe && progress && GoalCircle(&goal,in.world.player),
        "travel dodges the crossing bullet, rejoins and reaches the firing zone without getting stuck");
    Check(Len(out.dodge.velocity)==0.f,"integrated travel stops after reaching a safe firing zone");

    in=Scene(); state.Reset(); nav.Reset();
    PrepareFrameMovement(in,20.f);
    Shot({-.2f,0.f},{.8f,0.f},200.f,.08f);
    state.valid=true; state.plan.speed=in.world.speed; state.plan.epochMs=in.nowMs;
    state.plan.count=3; state.plan.points[0]={{},0.f}; state.plan.points[1]={{0.f,.2f},40.f};
    state.plan.points[2]={{0.f,.2f},800.f};
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && out.dodge.reused && out.dodge.velocity.y>.0049f,
        "active emergency escape keeps priority over a new travel direction");
    goal.active=false; EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && out.dodge.reused && state.valid,
        "clearing a goal never cancels an executing necessary escape");

    in=Scene(); state.Reset(); nav.Reset(); goal.active=true;
    PrepareFrameMovement(in,20.f); EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && nav.travelValid,"clear approach caches its finite continuation");
    map.projectileSourceUnavailable=true; EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && Len(out.dodge.velocity)==0.f,
        "cached continuation cannot bypass unavailable projectile capture");
    map.projectileSourceUnavailable=false; goal.active=false;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!nav.travelValid && Len(out.dodge.velocity)==0.f,"cancelling travel discards its cached continuation");

    in=Scene(); state.Reset(); nav.Reset(); goal.active=true;
    PrepareFrameMovement(in,20.f); EvaluateMovement(in,false,goal,state,nav,out);
    const auto fast=nav.travel;
    in.world.speed=.0025f; EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.dodge.reused && out.approaching && out.dodge.plan.speed!=fast.speed && Validate(in,out.dodge.plan),
        "Slow rechecks the complete travel continuation, not just its first step");
    in.world.speed=0.f; in.world.movementLocked=true;
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.approaching && out.dodge.status==Status::Locked,"paralysis cancels cached travel execution");

    in=Scene(); state.Reset(); nav.Reset(); PrepareFrameMovement(in,20.f);
    in.zoneCount=1; in.zones[0]={{1.f,0.f},.3f,160.f,600.f};
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.detouring && Validate(in,out.dodge.plan),
        "travel bends around a timed bomb instead of walking into its future blast");
    in=Scene(); state.Reset(); nav.Reset(); PrepareFrameMovement(in,20.f);
    Shot({.6f,-.6f},{.6f,3.4f},800.f,.08f);
    in.world.env.canOccupy=[](float,float y,bool){return std::fabs(y)<.01f;};
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(!out.detouring && (!out.approaching || Validate(in,out.dodge.plan)),
        "blocked corridor never permits a bullet detour through its walls");
}

static void DirectionalTravelTests() {
    auto in=Scene(); State state{}; NavigationState nav{}; MovementDecision out{};
    in.settings.maxSearchMs=4.f;
    in.world.env.canOccupy=WallWithOpening;
    bool safe=true; int stopped=0;
    PrepareNativeMovement(in,{.1f,0.f});
    nav.directional=true; nav.count=1; nav.travelValid=true;
    EvaluateMovement(in,true,{},state,nav,out);
    Vec2 untouched{.1f,0.f};
    Check(out.dodge.status==Status::Clear && !NativeMovementTarget(in,out.dodge,untouched) &&
        !nav.travelValid && nav.count==0,
        "harmless wall preserves native keyboard input and cancels obsolete automatic detours");
    in.settings.avoidHarmlessBlocks=true;
    EvaluateMovement(in,true,{},state,nav,out);
    Check(out.dodge.plan.count>0 && Validate(in,out.dodge.plan),
        "optional block steering restores an executable route around harmless walls");
    in.settings.avoidHarmlessBlocks=false; state.Reset(); nav.Reset();
    in.world.env.canOccupy=[](float,float,bool){return true;};
    in.world.env.isHazard=[](float,float){return false;};
    EvaluateMovement(in,true,{},state,nav,out);
    Check(out.dodge.status==Status::Clear && Len(Sub(out.dodge.velocity,in.nominal))<1e-6f,
        "non-damaging terrain such as water preserves manual walking");
    in.world.env.isHazard=[](float x,float){return x>.4f && x<.7f;};
    Check(!KeyboardIntentSafe(in),"damaging ground still triggers protection during manual walking");
    in.world.env.isHazard=[](float,float){return false;};
    map.enemyCount=1; map.enemies[0]={{.5f,0.f},.3f,true};
    Check(KeyboardIntentSafe(in),"passive solid scenery is not a manual dodge trigger");
    Check(!EnemyPathClear(in.world,{0.f,0.f},{1.f,0.f}),
        "passive solid scenery still constrains physically executable dodge routes");
    map.enemies[0].passiveScenery=false;
    Check(!KeyboardIntentSafe(in),"real enemy bodies remain protected during manual walking");
    const float normalRadius=EnemyAvoidanceRadius(map.enemies[0],in.world.settings);
    in.world.settings.enemyAvoidanceScale=.1f;
    Check(std::fabs(EnemyAvoidanceRadius(map.enemies[0],in.world.settings)-normalRadius*.1f)<1e-6f &&
        EnemyPathClear(in.world,{0.f,.2f},{1.f,.2f}),
        "smaller enemy avoidance size changes the actual swept movement clearance");
    in.world.settings.enemyAvoidanceScale=3.f;
    Check(!EnemyPathClear(in.world,{0.f,1.f},{1.f,1.f}),
        "larger enemy avoidance size rejects movement inside the expanded circle");
    map.enemies[0].passiveScenery=true;
    Check(std::fabs(EnemyAvoidanceRadius(map.enemies[0],in.world.settings)-normalRadius)<1e-6f,
        "enemy slider never inflates harmless solid scenery");
    in.world.settings.enemyAvoidanceScale=1.f;
    map.enemyCount=0;
    Shot({.6f,-.6f},{.6f,3.4f},800.f,.08f);
    Check(!KeyboardIntentSafe(in),"moving projectiles still trigger keyboard protection");
    map.laneCount=0; in.zoneCount=1; in.zones[0]={{.5f,0.f},.3f,80.f,800.f};
    Check(!KeyboardIntentSafe(in),"timed bomb blasts still trigger keyboard protection");
    in.zoneCount=0;
    in.settings.lookRange=32.f; in.settings.horizonMs=4000.f; in.settings.maxDistance=12.f;
    Shot({24.f,0.f},{25.f,0.f},4000.f,.05f);
    Check(LaneInLookRange(in,map.lanes[0]),"expanded look range admits projectiles beyond 16 tiles");
    DebugGeometry::Grid extendedGrid; extendedGrid.Reset({},32.f,.5f);
    Check(extendedGrid.range==32.f,"debug grid respects the expanded look range");
    map.laneCount=0; in.nominal={}; state.Reset(); Evaluate(in,state,out.dodge);
    Check(out.dodge.status==Status::Clear,"4000ms prediction and 12-tile distance remain valid planner settings");
    in.settings.horizonMs=800.f; in.settings.maxDistance=3.f;
    in.nominal={}; EvaluateMovement(in,false,{},state,nav,out);
    Check(!nav.directional && nav.count==0 && Len(out.dodge.velocity)==0.f,
        "releasing keys immediately cancels the directional travel goal");

    in=Scene(); state.Reset(); nav.Reset(); PrepareFrameMovement(in,20.f);
    MovementGoal goal{}; goal.active=true; goal.identity=83; goal.center={3.f,0.f}; goal.range=1.f;
    goal.context=&goal; goal.contains=GoalCircle;
    Shot({.6f,-.6f},{.6f,3.4f},800.f,.08f);
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.approaching && out.detouring && Len(out.dodge.velocity)>.0049f && Validate(in,out.dodge.plan),
        "travel chooses a moving detour before an otherwise safe stationary wait");
    const auto chosen=out.dodge.plan;
    in.world.player=Add(in.world.player,Mul(out.dodge.velocity,20.f)); in.nowMs+=20.f;
    map.laneCount=0; Shot({.6f,-.5f},{.6f,3.4f},780.f,.08f);
    EvaluateMovement(in,false,goal,state,nav,out);
    Check(out.dodge.reused && out.dodge.plan.epochMs==chosen.epochMs,
        "moving detour retains its forward segment and turn deadline across updates");

    in=Scene(); state.Reset(); nav.Reset(); in.world.env.canOccupy=WallWithOpening;
    in.settings.maxSearchMs=4.f;
    PrepareFrameMovement(in,20.f); goal.center={10.f,0.f}; goal.range=.2f;
    safe=true; stopped=0;
    for(int frame=0;frame<90;frame++) {
        EvaluateMovement(in,false,goal,state,nav,out);
        const Vec2 step=Mul(out.dodge.velocity,in.frameMs);
        safe &= OccupancyPathClear(in.world,in.world.player,Add(in.world.player,step));
        stopped+=Len(step)<.001f;
        in.world.player=Add(in.world.player,step); in.nowMs+=in.frameMs;
    }
    Check(safe && in.world.player.x>6.f && stopped<5,
        "distant travel follows corridor turns and renews partial routes without a 200ms arrival pause");
}

static void DenseOpeningTests() {
    for(int scenario=0;scenario<4;scenario++) {
        const bool walking=(scenario%2)!=0,alreadyHit=scenario>=2;
        auto in=Scene(); State state{}; Output out{};
        in.collectDiagnostics=true;
        in.settings.maxSearchMs=4.f; in.settings.recoveryBudgetMs=2.f;
        if(walking) PrepareNativeMovement(in,{.1f,0.f});
        for(int wave=0;wave<6;wave++) for(int row=0;row<80;row++) {
            if(row==43 || row==44) continue;
            const float x=1.1f+wave*.6f,y=(row-40)*.16f;
            Shot({x,y},{x-8.f,y},800.f,.09f);
            auto& lane=map.lanes[map.laneCount-1]; lane.damageEstimate=50.f;
            lane.pointCount=kMaxLanePoints;
            for(int j=0;j<kMaxLanePoints;j++) {
                const float t=800.f*j/(kMaxLanePoints-1);
                lane.pointTimesMs[j]=t; lane.points[j]={x-.01f*t,y};
            }
        }
        Plan witness{}; witness.epochMs=in.nowMs; witness.speed=in.world.speed; witness.nominal=in.nominal;
        witness.count=3; witness.points[0]={{},0.f}; witness.points[1]={{0.f,.56f},112.f};
        witness.points[2]={Add({0.f,.56f},Mul(in.nominal,688.f)),800.f};
        Check(Validate(in,witness),"468-shot field contains a continuously verified micro-opening");
        if(alreadyHit) {
            Shot({},{},800.f,.04f); map.lanes[map.laneCount-1].damageEstimate=10.f;
        }
        const auto started=std::chrono::steady_clock::now(); Evaluate(in,state,out);
        std::printf("Dense opening %s%s: %s reason=%s hits=%d search=%.2f ms\n",walking?"walking":"idle",alreadyHit?" + contact":"",
            StatusName(out.status),ReasonName(out.reason),out.expectedHits,
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
        if(alreadyHit) --map.laneCount;
        Check(state.valid && out.expectedHits==(alreadyHit?1:0) && (!alreadyHit || out.estimatedDamage==10.f) && Validate(in,out.plan),
            "bounded search finds the safe opening in six dense moving waves");
        if(walking && !alreadyHit) {
            const std::vector<LaneThreat> captured(map.lanes,map.lanes+map.laneCount);
            NavigationState nav{}; MovementDecision decision{};
            float elapsed=0.f; bool safe=true;
            for(int frame=0;frame<60;frame++) {
                map.laneCount=elapsed<800.f?static_cast<int>(captured.size()):0;
                for(int i=0;i<map.laneCount;i++) {
                    auto& lane=map.lanes[i]; lane=captured[i]; lane.remainingLifeMs=800.f-elapsed;
                    for(int j=0;j<kMaxLanePoints;j++) {
                        const float t=lane.remainingLifeMs*j/(kMaxLanePoints-1);
                        lane.pointTimesMs[j]=t;
                        lane.points[j]={captured[i].points[0].x-.01f*(elapsed+t),captured[i].points[0].y};
                    }
                }
                in.nowMs=10000.+elapsed; in.frameMs=frame%2?16.f:23.f;
                const Vec2 start=in.world.player,requested=Add(start,{in.world.speed*in.frameMs,0.f});
                PrepareNativeMovement(in,requested);
                EvaluateMovement(in,true,{},state,nav,decision);
                Vec2 executed=requested; NativeMovementTarget(in,decision.dodge,executed);
                safe &= Len(Sub(executed,start))<=in.world.speed*in.frameMs+1e-5f;
                for(int i=0;i<map.laneCount;i++) {
                    const float dt=std::min(in.frameMs,map.lanes[i].remainingLifeMs);
                    const Vec2 a=Sub(map.lanes[i].points[0],start);
                    const Vec2 b=Sub(Add(map.lanes[i].points[0],{-.01f*dt,0.f}),
                        Add(start,Mul(Sub(executed,start),dt/in.frameMs)));
                    safe &= !SweptProjectileContact(a,b,.09f);
                }
                in.world.player=executed; elapsed+=in.frameMs;
            }
            Check(safe && in.world.player.x>2.f,
                "native walking executes through all six dense waves without contact or overspeed over variable frames");
        }
    }
}

static void MatchingTimeTests() {
    float entry=-1.f;
    float leave=-1.f;
    Check(SweptProjectileContact({-2.f,0.f},{2.f,0.f},.5f,&entry,&leave) &&
        entry==.375f && leave==.625f,"continuous recovery residence uses exact contact entry and exit");
    Check(SweptProjectileContact({.45f,.45f},{.45f,.45f},.5f) && std::hypot(.45f,.45f)>.55f,
        "square corner contact lies outside a circle scaled to 1.10");
    Check(SweptProjectileContact({-2.f,.49f},{2.f,.49f},.5f,&entry) && std::fabs(entry-.375f)<1e-6f,
        "fast edge crossing reports its exact per-axis entry time");
    Check(!SweptProjectileContact({-2.f,.5f},{2.f,.5f},.5f),
        "native strict threshold does not invent a hit along the outer boundary");
    Check(SweptProjectileContact({-1.f,-1.f},{1.f,1.f},.5f,&entry) && entry==.25f,
        "diagonal crossing uses the square's entry time rather than the circle's later entry");
    DebugGeometry::Grid squareGrid;
    squareGrid.Reset({},1.f,.1f);
    squareGrid.Segment({}, {},.5f,0.f,0.f,false,false,true);
    bool paintedCorner=false;
    for(int y=0;y<squareGrid.height;y++) for(int x=0;x<squareGrid.width;x++) {
        const auto center=squareGrid.Center(x,y);
        if(center.x>.4f && center.x<.5f && center.y>.4f && center.y<.5f)
            paintedCorner |= squareGrid.At(x,y).arrivalMs==0.f;
    }
    Check(paintedCorner,"debug footprint includes the same corner that the installed game can hit");
    bool matchingSquareRaster=true;
    for(const Vec2 end: {Vec2{1.5f,.8f},Vec2{.8f,1.5f},Vec2{1.5f,0.f},Vec2{0.f,1.5f}}) {
        squareGrid.Reset({},3.f,.0625f);
        const Vec2 start=Mul(end,-1.f);
        squareGrid.Segment(start,end,.18f,0.f,800.f,false,false,true);
        for(int y=0;y<squareGrid.height;y++) for(int x=0;x<squareGrid.width;x++) {
            const Vec2 center=squareGrid.Center(x,y);
            float exactEntry=0.f;
            const bool expected=LenSq(center)<=9.f &&
                SweptProjectileContact(Sub(start,center),Sub(end,center),.18f,&exactEntry);
            const float arrival=squareGrid.At(x,y).arrivalMs;
            matchingSquareRaster &= expected==(arrival>=0.f) && (!expected || std::fabs(arrival-800.f*exactEntry)<.001f);
        }
    }
    Check(matchingSquareRaster,"scanline square grid matches continuous contacts and arrival times for diagonal and axial shots");
    Check(DebugGeometry::GridOpacity(99.)==1.f && DebugGeometry::GridOpacity(101.)>.98f &&
        DebugGeometry::GridOpacity(250.)==0.f,"dense grid fades gradually as its snapshot ages");
    const Vec2 walkingMarker=DebugGeometry::PlayerAt({}, {2.f,3.f},{.005f,0.f},1000.,200.f);
    Check(std::fabs(walkingMarker.x-3.f)<1e-6f && walkingMarker.y==3.f,
        "normal walking debug marker plots live velocity at the labeled future time");
    Plan markerPlan{}; markerPlan.epochMs=1000.; markerPlan.count=3;
    markerPlan.points[0]={{},0.f}; markerPlan.points[1]={{},100.f}; markerPlan.points[2]={{1.f,0.f},300.f};
    Check(Len(DebugGeometry::PlayerAt(markerPlan,{}, {},1000.,50.f))==0.f &&
        std::fabs(DebugGeometry::PlayerAt(markerPlan,{}, {},1000.,200.f).x-.5f)<1e-6f,
        "player trajectory markers include the chosen wait before the timed crossing");
    for(float bulletCrossMs:{50.f,200.f,350.f}) {
        auto in=Scene(); State state{}; Output out{};
        PrepareNativeMovement(in,{.1f,0.f});
        Shot({1.f,-.005f*bulletCrossMs},{1.f,.005f*(800.f-bulletCrossMs)},800.f,.08f);
        Evaluate(in,state,out);
        Check((out.status==Status::Clear)==(bulletCrossMs!=200.f),
            "same crossing geometry is unsafe only when player and bullet arrive together");
    }
    for(float speed:{.0025f,.005f,.01f}) {
        auto in=Scene(); State state{}; Output out{};
        in.world.speed=speed; in.frameMs=40.f;
        PrepareNativeMovement(in,{speed*10.f,0.f});
        Check(std::fabs(in.frameMs-10.f)<.001f && std::fabs(Len(in.nominal)-speed)<1e-6f,
            "short native step during long rendering predicts the full live walking speed");
        Shot({1.f,-1.f},{1.f,3.f},800.f,.08f);
        Evaluate(in,state,out);
        Check((out.status==Status::Clear)==(speed!=.005f),
            "Slow and Speedy change collision arrival timing on the same walking line");
    }
    auto in=Scene(); State state{}; Output out{};
    in.world.player={0.f,1.f}; in.nominal={0.f,-.005f};
    Shot({-1.f,0.f},{1.f,0.f},100.f,.08f);
    auto& lane=map.lanes[0]; lane.tailAtShotEnd=false; lane.remainingLifeMs=800.f;
    lane.hasLinearMotion=true; lane.linearVelocity={.02f,0.f};
    Evaluate(in,state,out);
    Check(out.status==Status::Clear,"verified moving shot leaves its old trail open after sampled coverage ends");
    Vec2 p{};
    Check(SampleProjectile(lane,400.f,p)==SampleStatus::Known && std::fabs(p.x-7.f)<1e-5f,
        "debug and planner extrapolate the same verified projectile position in time");
    in.world.player={3.f,1.f}; state.Reset(); Evaluate(in,state,out);
    Check(out.status!=Status::Clear,"collision after captured samples is detected at the extrapolated moving head");
    in.world.player={3.f,0.f}; in.settings.lookRange=.5f;
    Check(LaneInLookRange(in,lane),"look range admits a verified future head entering from beyond the captured trail");
    int drawn=0;
    DebugGeometry::ForEachSegment(lane,100.f,400.f,[&](Vec2 a,Vec2 b,float start,float end) {
        if(std::fabs(a.x-1.f)<1e-5f && std::fabs(b.x-7.f)<1e-5f && start==100.f && end==400.f) ++drawn;
    });
    Check(drawn==1,"world grid shows the same timed continuation used in collision checks");
    Check(SampleProjectile(lane,800.f,p)==SampleStatus::Expired,
        "verified constant motion never extends a projectile past its lifetime");
    lane.hasLinearMotion=false;
    Check(SampleProjectile(lane,400.f,p)==SampleStatus::Unknown,
        "unknown or curved motion is not silently extrapolated as a straight shot");

    in=Scene(); state.Reset(); NavigationState nav{}; MovementDecision decision{};
    MovementGoal goal{}; goal.active=true; goal.identity=82; goal.center={3.f,0.f}; goal.range=1.f;
    goal.contains=GoalCircle; goal.context=&goal;
    in.world.env.canOccupy=[](float,float y,bool){return std::fabs(y)<.01f;};
    PrepareFrameMovement(in,10.f);
    Shot({.6f,-.6f},{.6f,3.4f},800.f,.08f);
    EvaluateMovement(in,false,goal,state,nav,decision);
    const auto departure=decision.dodge.plan.departureMs;
    const auto epoch=decision.dodge.plan.epochMs;
    Check(decision.approaching && !decision.detouring && decision.dodge.waitMs>0.f &&
        Validate(in,decision.dodge.plan),"corridor travel uses a timed opening rather than going around the bullet trail");
    bool retained=true,safe=true;
    for(int i=0;i<70;i++) {
        const float age=static_cast<float>(in.nowMs-epoch);
        map.laneCount=0;
        Shot({.6f,-.6f+.005f*age},{.6f,3.4f},800.f-age,.08f);
        EvaluateMovement(in,false,goal,state,nav,decision);
        if(age<departure) retained &= decision.dodge.plan.epochMs==epoch &&
            decision.dodge.plan.departureMs==departure;
        if(decision.approaching) safe &= Validate(in,decision.dodge.plan);
        safe &= std::fabs(decision.dodge.velocity.y)<1e-6f;
        in.world.player=Add(in.world.player,Mul(decision.dodge.velocity,in.frameMs)); in.nowMs+=in.frameMs;
    }
    Check(retained && safe && GoalCircle(&goal,in.world.player),
        "timed crossing retains its release deadline, passes through and arrives without endless waiting");
}

static void CaptureAndRammingTests() {
    auto in=Scene();
    in.world.settings.hitScale=1.f; in.world.speed=.008f;
    Check(PrepareNativeMovement(in,{.16f,0.f}),"ramming fixture is an actual bounded native walking command");
    Shot({.13f,0.f},{.13f,0.f},800.f,.025f);
    Output failed{}; failed.status=Status::NoPlan; failed.reason=Reason::SearchBudget;
    Check(ProtectImmediateStep(in,failed) && failed.status==Status::Recovery && failed.unknownDamage,
        "failed full search still replaces an immediately colliding walking step with an honest safe prefix");
    Vec2 target{};
    Check(NativeMovementTarget(in,failed,target) && !SweptProjectileContact(
        Sub(map.lanes[0].points[0],in.world.player),Sub(map.lanes[0].points[0],target),.025f),
        "native fallback endpoint does not ram the bullet or exceed live speed");
    failed={}; failed.status=Status::Incomplete;
    map.lanes[0].points[0]=map.lanes[0].points[1]={.8f,0.f};
    Check(!ProtectImmediateStep(in,failed),"distant danger does not activate immediate walking guard");
    map.lanes[0].points[0]=map.lanes[0].points[1]={0.f,0.f};
    Check(!ProtectImmediateStep(in,failed),"already overlapping is not falsely described as a collision-free prefix");
    map.projectileSourceUnavailable=true;
    Check(!ProtectImmediateStep(in,failed),"missing projectile data cannot certify a safe walking prefix");

    in=Scene(); in.world.settings.hitScale=1.f; in.nominal={.008f,0.f}; in.world.speed=.008f;
    in.settings.maxDistance=.01f; in.settings.maxExpansions=1;
    Shot({.13f,0.f},{.13f,0.f},800.f,.025f); map.lanes[0].damageEstimate=100.f;
    State state{}; Output out{}; Evaluate(in,state,out);
    Check(out.expectedHits==0 && state.valid && Validate(in,out.plan) && Len(out.velocity)<1e-6f,
        "tiny dodge distance budget permits a full hold instead of resuming walking into a stationary projectile");
    map.lanes[0].points[0]=map.lanes[0].points[1]={3.f,0.f}; state.Reset(); Evaluate(in,state,out);
    Check(Len(Sub(out.velocity,in.nominal))<1e-6f,
        "unrestricted emergency hold does not cancel safe walking early for distant danger");

    in=Scene(); float dx=0.f,dy=0.f;
    Check(AoeCapturePolicy::ThrowLanding(6.f,7.f,dx,dy),"Throw landing decodes without a second position");
    in.world.player={6.f,7.f}; in.world.speed=.004f; in.zoneCount=1;
    in.zones[0]={{dx,dy},3.5f,AoeCapturePolicy::ShowEffectDurationMs(0.f,true)-35.f,1700.f};
    ApplyZonePlanningLimits(in); state.Reset(); Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan) && Len(Sub(PositionAt(out.plan,1465.f),{6.f,7.f}))>3.5f,
        "default Throw warning produces a delayed escape clear before the 1500ms blast");
    Check(out.waitMs>0.f && AoeCapturePolicy::ShowEffectDurationMs(.05f,true)==50.f &&
        AoeCapturePolicy::ShowEffectDurationMs(2300.f,true)==2300.f,
        "throw capture preserves explicit short/long flight times and latest-departure behavior");

    const float times[]={0.f,30.f,60.f},xs[]={0.f,.3f,.3f},ys[]={0.f,0.f,.3f};
    Vec2 phase{}; int index=-1;
    Check(FractionalPathAnchor(times,xs,ys,3,20.f,index,phase) && index==0 && std::fabs(phase.x-.2f)<1e-6f,
        "curved cache phase keeps the turn 10ms ahead instead of snapping to a sample");
    Check(!FractionalPathAnchor(times,xs,ys,3,60.f,index,phase),"expired cache falls back to fresh trajectory capture");

    in=Scene(); Shot({1.f,0.f},{-1.f,0.f},200.f);
    auto capture=std::make_unique<Replay::Capture>();
    capture->current.Capture(in,out); capture->previous=capture->current;
    capture->current.map.lanes[0].points[0]={.8f,0.f};
    auto audit=Replay::Compare(capture->previous.map,capture->current.map,{},20.f);
    Check(audit.matched==1 && audit.diverged==0 && audit.maxError<1e-6f,
        "prediction audit compares matching projectile IDs at matching time");
    capture->current.map.lanes[0].points[0].y=.2f;
    audit=Replay::Compare(capture->previous.map,capture->current.map,{},20.f);
    Check(audit.diverged==1 && audit.nearError>.19f,"live lateral prediction error is recorded without inflating contact size");
    capture->current.map.lanes[0].ownerObjId=1;
    Check(Replay::Compare(capture->previous.map,capture->current.map,{},20.f).matched==0,
        "prediction audit does not confuse different owners sharing a bullet ID");
    FILE* file=nullptr;
#ifdef _WIN32
    tmpfile_s(&file);
#else
    file=std::tmpfile();
#endif
    bool roundtrip=file && std::fwrite(capture.get(),sizeof(*capture),1,file)==1;
    if(file) {
        std::rewind(file); auto read=std::make_unique<Replay::Capture>();
        roundtrip=roundtrip && std::fread(read.get(),sizeof(*read),1,file)==1 && read->Compatible() &&
            !read->terrainRecorded && read->current.input.world.map==nullptr &&
            read->current.Restore().world.map==&read->current.map && read->current.map.laneCount==1;
        std::fclose(file);
    }
    Check(roundtrip,"binary replay round-trips sanitized geometry and restores its own map pointer");
}

static int ReplayFiles(int argc,char** argv) {
    for(int i=2;i<argc;i++) {
        auto capture=std::make_unique<Replay::Capture>(); FILE* file=nullptr;
#ifdef _WIN32
        fopen_s(&file,argv[i],"rb");
#else
        file=std::fopen(argv[i],"rb");
#endif
        if(!file) { std::printf("Cannot open %s\n",argv[i]); return 2; }
        const bool ok=std::fread(capture.get(),sizeof(*capture),1,file)==1 && capture->Compatible();
        std::fclose(file);
        if(!ok) { std::printf("Wrong/incomplete replay build: %s\n",argv[i]); return 2; }
        const auto& f=capture->current;
        Input in=f.Restore(); State state{}; Output out{};
        in.settings.maxSearchMs=0.f; in.settings.recoveryBudgetMs=0.f;
        Evaluate(in,state,out);
        std::printf("%s\nGEOMETRY ONLY: terrain and target-goal predicates not recorded\n"
            "shots=%d zones=%d native=%d selected=%d applied=%d rejected=%d nearError=%.4f\n"
            "captured=%s damage=%.0f hits=%d chord=(%.4f,%.4f); unlimited=%s damage=%.0f hits=%d\n",
            argv[i],f.map.laneCount,in.zoneCount,f.native,f.selected,f.applied,f.rejected,capture->audit.nearError,
            StatusName(f.output.status),f.output.estimatedDamage,f.output.expectedHits,f.command.x,f.command.y,
            StatusName(out.status),out.estimatedDamage,out.expectedHits);
    }
    return 0;
}
int main(int argc,char** argv) {
    if(argc>1 && std::strcmp(argv[1],"--replay")==0) return ReplayFiles(argc,argv);
    CaptureAndRammingTests();
    MatchingTimeTests();
    DirectionalTravelTests();
    DenseOpeningTests();
    IntegratedTravelTests();
    FrameMovementTests();
    MovementTests();
    WaypointTests();
    WallAndBombTests();
    NativeWalkingTests();
    IdleActuatorTests();
    AssistRegressionTests();
    const auto started = std::chrono::steady_clock::now();
    State state{}; Output out{};
    auto in = Scene(); Evaluate(in, state, out);
    Check(out.status == Status::Clear && !state.valid && Len(out.velocity) == 0.f, "safe idle does not move");
    in.nominal = {.005f,0.f}; Evaluate(in, state, out);
    Check(out.status == Status::Clear && out.velocity.x == .005f, "safe input is untouched");
    in = Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f);
    Evaluate(in, state, out);
    std::printf("Single shot: %s wait=%.1f distance=%.3f expansions=%d\n",StatusName(out.status),out.waitMs,out.plan.intervention,out.expansions);
    Check(state.valid && out.status == Status::Waiting, "find a delayed escape");
    Check(out.waitMs >= 180.f && out.waitMs < 400.f, "wait until shortly before crossing");
    Check(out.plan.intervention < .65f, "small movement for single shot");
    Check(Validate(in, out.plan), "entire generated trajectory is swept safe");
    const Plan original = out.plan;
    if(state.valid) {
        in.nowMs += 40.;
        for(int k=0;k<map.laneCount;k++) {
            map.lanes[k].points[0].x += .2f;
            map.lanes[k].pointTimesMs[1] -= 40.f;
            map.lanes[k].remainingLifeMs -= 40.f;
        }
        Evaluate(in,state,out);
        Check(out.reused && out.plan.epochMs == original.epochMs, "retain absolute departure across frames");
        Check(std::fabs(out.waitMs - (original.departureMs-40.f)) < 1.f, "departure countdown advances");
        in.world.speed *= .5f;
        Check(!Validate(in,original), "Slow invalidates old timing");
        Evaluate(in,state,out);
        Check(!out.reused, "Slow triggers a new solve");
    }
    in.world.speed = 0.f; in.world.movementLocked = true;
    Evaluate(in,state,out);
    Check(out.status == Status::Locked && Len(out.velocity)==0.f && !state.valid, "paralysis cancels pending movement");
    in = Scene(); map.limited = true; Evaluate(in,state,out);
    Check(out.status == Status::Incomplete && !state.valid, "overflow never becomes a safe result");
    in = Scene(); map.projectileSourceUnavailable = true; Evaluate(in,state,out);
    Check(out.status == Status::Incomplete, "missing capture is not safe idle");
    in = Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f); in.settings.maxExpansions=0;
    Evaluate(in,state,out);
    Check(out.budgetHit && out.status == Status::Incomplete && !state.valid, "bounded search reports incomplete honestly");
    in = Scene(); in.zoneCount=1; in.zones[0]={{0.f,0.f},.35f,500.f,700.f};
    Evaluate(in,state,out);
    Check(state.valid && out.waitMs >= 300.f && Validate(in,out.plan), "telegraph waits but clears before activation");
    in = Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f); Shot({0.f,-2.f},{0.f,2.f},800.f);
    Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan), "crossing shots checked together");
    in=Scene(); in.world.env.canOccupy=&Corner;
    in.zoneCount=1; in.zones[0]={{0.f,0.f},.7f,650.f,800.f};
    Evaluate(in,state,out);
    std::printf("Corner: %s distance=%.3f turns=%d expansions=%d\n",StatusName(out.status),out.plan.intervention,Turns(out.plan),out.expansions);
    Check(state.valid && Turns(out.plan)>0 && Validate(in,out.plan), "spacetime route turns around a corner");
    Check(state.valid && out.waitMs>250.f, "turning escape also delays departure");
    in=Scene(); in.nominal={.005f,0.f};
    in.zoneCount=1; in.zones[0]={{2.f,0.f},.3f,350.f,500.f};
    Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan), "unsafe moving input receives a timed correction");
    if(state.valid) {
        const auto old=out.plan;
        in.nominal={0.f,-.005f};
        Check(!Validate(in,old), "changed input immediately invalidates cached route");
    }
    in=Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f); Evaluate(in,state,out);
    const auto slowPlan=out.plan;
    in.world.speed=.009f; Evaluate(in,state,out);
    Check(state.valid && !out.reused && out.plan.speed==.009f, "Speedy replans with live increased speed");
    Check(out.plan.departureMs>=slowPlan.departureMs, "higher speed can depart later in same single-shot scene");
    in=Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f);
    in.settings.stepMs=50.f; in.actuationMs=33.4f; in.frameMs=50.f;
    Evaluate(in,state,out);
    std::printf("Cadence: %s distance=%.3f expansions=%d\n",StatusName(out.status),out.plan.intervention,out.expansions);
    Check(state.valid && Validate(in,out.plan), "actuator cadence participates in planning");
    in=Scene(); in.world.speed=std::numeric_limits<float>::quiet_NaN(); Evaluate(in,state,out);
    Check(out.status==Status::Incomplete && !state.valid,"unknown speed never becomes a safe plan");
    in=Scene(); Shot({-1.f,0.f},{1.f,0.f},800.f); map.lanes[0].beam=true;
    map.lanes[0].pointTimesMs[1]=0.f; // actual simultaneous-endpoint sensor format
    in.world.player={0.f,1.f}; in.nominal={0.f,-.005f}; Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan),"simultaneous finite beam blocks crossing");
    in=Scene(); Shot({-2.f,0.f},{2.f,0.f},800.f); Evaluate(in,state,out);
    if(state.valid) {
        Plan delayed=out.plan;
        for(int i=1;i<delayed.count;i++) if(delayed.points[i].timeMs>=delayed.departureMs) delayed.points[i].timeMs+=80.f;
        delayed.departureMs+=80.f;
        Check(!Validate(in,delayed),"moving later than safe departure invalidates complete escape");
    }
    in=Scene(); state.Reset(); Shot({.4f,.4f},{.4f,.4f},800.f,.5f);
    in.collectDiagnostics=true;
    Evaluate(in,state,out);
    Check(out.status!=Status::Clear && out.diagnostics.cornerRejects>0,
        "installed game's square corner is a real contact even outside the former circle");
    in=Scene(); state.Reset(); Shot({-4.f,.51f},{16.f,.51f},800.f,.5f);
    Evaluate(in,state,out);
    Check(out.status==Status::Clear,"fast shot passing outside the actual threshold gets no speed/player cushion");
    in=Scene(); state.Reset(); Shot({-4.f,0.f},{16.f,0.f},800.f,.5f);
    Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan) && out.plan.intervention<=.6f,
        "fast direct shot admits a small point-vs-square escape");
    in=Scene(); state.Reset(); Shot({-2.f,0.f},{2.f,0.f},10.f,.5f);
    Evaluate(in,state,out);
    Check(out.status==Status::Recovery && out.expectedHits==1 && !Validate(in,out.plan),
        "unavoidable fast crossing is reported as damage, never a clear or unscored route");
    in=Scene(); state.Reset(); map.enemyCount=1; map.enemies[0]={{0.f,0.f},.5f};
    Check(!EnemyPathClear(in.world,{-1.f,0.f},{1.f,0.f}),"cannot cross an enemy body even with clear endpoints");
    Check(!EnemyPathClear(in.world,{1.f,0.f},{.2f,0.f}),"cannot enter an enemy body");
    Check(EnemyPathClear(in.world,{.3f,0.f},{.5f,0.f}),"existing overlap allows partial outward escape");
    Check(!EnemyPathClear(in.world,{.3f,0.f},{.1f,0.f}) &&
        !EnemyPathClear(in.world,{.3f,0.f},{-1.f,0.f}),"existing overlap never permits deeper movement or crossing centre");
    in.world.player={.3f,0.f}; Evaluate(in,state,out);
    Check(state.valid && out.status==Status::Moving && Validate(in,out.plan) && out.velocity.x>0.f,
        "starting inside an enemy escapes immediately instead of freezing at zero lead");
    map.enemyCount=2; map.enemies[1]={{1.2f,0.f},.5f};
    Check(!EnemyPathClear(in.world,{.3f,0.f},{1.f,0.f}),"escaping one enemy cannot enter another");
    in=Scene(); state.Reset(); in.settings.lookRange=2.f;
    Shot({-10.f,0.f},{10.f,0.f},800.f,.1f);
    Check(LaneInLookRange(in,map.lanes[0]),"look range includes fast shots entering from outside");
    Shot({5.f,5.f},{6.f,5.f},800.f,.1f);
    Check(!LaneInLookRange(in,map.lanes[1]),"look range excludes paths that stay outside");
    Vec2 sample{};
    Check(SampleProjectile(map.lanes[0],200.f,sample)==SampleStatus::Known && std::fabs(sample.x+5.f)<.001f,
        "debug time slice interpolates the same projectile trajectory");
    Check(SampleProjectile(map.lanes[0],800.f,sample)==SampleStatus::Expired,"expired projectile disappears from time slice");
    map.lanes[0].tailAtShotEnd=false; map.lanes[0].remainingLifeMs=-1.f;
    Check(SampleProjectile(map.lanes[0],900.f,sample)==SampleStatus::Unknown,"unknown future is not drawn as known position");
    in=Scene(); state.Reset(); Shot({0.f,.15f},{0.f,.15f},800.f,.1f);
    Evaluate(in,state,out); Check(out.status==Status::Clear,"default contact size leaves circle near miss safe");
    in.world.settings.hitScale=2.f; Evaluate(in,state,out);
    Check(out.status==Status::Recovery && state.valid && ProjectileRadius(map.lanes[0],in.world.settings)==.2f,
        "contact size changes both radius and recovery decision");
    // Already touching a low-damage shot must not disable avoidance of a second,
    // more damaging shot. The current contact is counted once, then escaped.
    in=Scene(); state.Reset(); Shot({0.f,0.f},{0.f,0.f},800.f,.12f); map.lanes[0].damageEstimate=10.f;
    Shot({-2.f,0.f},{2.f,0.f},800.f,.3f); map.lanes[1].damageEstimate=200.f;
    Evaluate(in,state,out);
    Check(state.valid && out.status==Status::Recovery && out.expectedHits==1 &&
        std::fabs(out.estimatedDamage-10.f)<.001f && !out.unknownDamage && Len(out.velocity)>0.f,
        "unavoidable small hit still escapes and avoids the expensive bullet");
    map.lanes[0].damageEstimate=-1.f; state.Reset(); Evaluate(in,state,out);
    Check(out.status==Status::Recovery && out.unknownDamage && out.estimatedDamage>=1000.f,
        "missing damage metadata never becomes a free hit");
    in=Scene(); state.Reset(); in.nominal={.005f,0.f};
    in.frameMs=20.f; in.actuationMs=20.f; in.settings.stepMs=60.f;
    in.maxCorrectionSpeed=.010f;
    map.enemyCount=1; map.enemies[0]={{2.f,0.f},.5f};
    Evaluate(in,state,out);
    std::printf("Walking enemy: %s reason=%s expansions=%d distance=%.3f\n",StatusName(out.status),ReasonName(out.reason),out.expansions,out.plan.intervention);
    Check(state.valid && Validate(in,out.plan),"walking with native command cadence avoids solid enemy ahead");
    Check(out.status==Status::Waiting && out.waitMs>=60.f && Len(Sub(out.velocity,in.nominal))<1e-6f,
        "enemy approach keeps intended walking until a later safe departure");
    if(state.valid) {
        for(int i=1;i<out.plan.count;i++) Check(EnemyPathClear(in.world,out.plan.points[i-1].pos,out.plan.points[i].pos),
            "every walking route segment respects enemy body");
    }
    in=Scene(); state.Reset(); in.settings.horizonMs=400.f;
    Shot({-3.f,0.f},{1.f,0.f},800.f,.1f); Evaluate(in,state,out);
    Check(out.status==Status::Clear,"short lookahead excludes a collision outside its horizon");
    in.settings.horizonMs=800.f; Evaluate(in,state,out);
    Check(state.valid && Validate(in,out.plan),"longer lookahead discovers and dodges the later collision");
    in=Scene(); state.Reset(); in.settings.leadMs=40.f;
    Shot({-.1f,0.f},{.1f,0.f},20.f,.2f); map.lanes[0].damageEstimate=15.f;
    Shot({-2.f,0.f},{2.f,0.f},800.f,.3f); map.lanes[1].damageEstimate=200.f;
    Evaluate(in,state,out);
    Check(state.valid && out.status==Status::Recovery && out.expectedHits==1 && out.estimatedDamage==15.f,
        "unavoidable hit during command gate still plans to avoid subsequent hits");
    Check(out.waitMs>=200.f && Len(out.velocity)<1e-6f,
        "recovery waits for the later threat when moving now cannot avoid the first hit");
    in=Scene(); state.Reset(); Shot({0.f,0.f},{0.f,0.f},800.f,.15f); map.lanes[0].damageEstimate=10.f;
    map.enemyCount=1; map.enemies[0]={{.9f,0.f},.5f}; Evaluate(in,state,out);
    bool recoveryClear=state.valid;
    for(int i=1;i<out.plan.count;i++) recoveryClear=recoveryClear && EnemyPathClear(in.world,out.plan.points[i-1].pos,out.plan.points[i].pos);
    Check(out.status==Status::Recovery && recoveryClear,"damage recovery never trades enemy body collision for a cheaper bullet");
    // This close approach cannot turn far enough under a 20 ms correction /
    // 60 ms command period. Preserve this limitation as a regression case;
    // do not mark a physically blocked route safe to make walking look fixed.
    in=Scene(); state.Reset(); in.nominal={.005f,0.f}; in.frameMs=20.f;
    in.actuationMs=20.f; in.settings.stepMs=60.f; in.maxCorrectionSpeed=.010f;
    map.enemyCount=1; map.enemies[0]={{1.2f,0.f},.5f}; Evaluate(in,state,out);
    Check(!state.valid && out.status==Status::NoPlan,"insufficient walking control authority cannot authorize passage through enemy");
    in=Scene(); state.Reset(); Shot({-2.f,0.f},{2.f,0.f},800.f,.04f);
    in.settings.maxExpansions=1; // stop before exploring later equivalents
    Evaluate(in,state,out);
    std::printf("Limited search: %s wait=%.0f damage=%.0f distance=%.3f\n",StatusName(out.status),out.waitMs,out.estimatedDamage,out.plan.intervention);
    Check(state.valid && out.status==Status::Waiting && out.waitMs>=300.f && Validate(in,out.plan),
        "budget-limited safe route is postponed instead of starting as soon as a future shot is seen");
    const Plan recoveryWaiting=out.plan;
    in.nowMs+=40.; map.lanes[0].points[0].x+=.2f;
    map.lanes[0].pointTimesMs[1]-=40.f; map.lanes[0].remainingLifeMs-=40.f;
    Evaluate(in,state,out);
    Check(out.reused && out.plan.epochMs==recoveryWaiting.epochMs &&
        std::fabs(out.waitMs-(recoveryWaiting.departureMs-40.f))<1.f,
        "retimed safe route retains its deadline instead of postponing forever");
    DebugGeometry::Disk disk{};
    Check(DebugGeometry::ProjectDisk({1.f,0.f},.5f,0.f,0.f,0.f,60.f,500.f,300.f,disk) &&
        disk.radius==30.f && disk.center.x==560.f,"world circle projects once: half tile at 60 px/tile is 30 px");
    Check(DebugGeometry::ProjectDisk({1.f,0.f},.5f,0.f,0.f,1.5707963f,120.f,500.f,300.f,disk) &&
        disk.radius==60.f && std::fabs(disk.center.y-420.f)<.01f,"rotation preserves radius and zoom scales it linearly");
    Check(!DebugGeometry::ProjectDisk({},.5f,0.f,0.f,0.f,0.f,0.f,0.f,disk),"invalid camera zoom cannot create giant debug markers");
    in=Scene(); Shot({-1.f,.25f},{1.f,.25f},10.f,.1f);
    DebugGeometry::Grid grid; grid.Reset({},2.f); grid.Lane(map.lanes[0],.1f,800.f);
    const auto cellAt=[&](Vec2 p) {
        const int x=static_cast<int>(std::floor(p.x/grid.step))-grid.originX;
        const int y=static_cast<int>(std::floor(p.y/grid.step))-grid.originY;
        return grid.At(x,y);
    };
    Check(std::fabs(cellAt({.25f,.25f}).arrivalMs-5.75f)<.01f,
        "grid analytically colors fast crossings between temporal samples");
    const int oldOrigin=grid.originX;
    grid.Reset({.2f,.2f},2.f);
    Check(grid.originX==oldOrigin && grid.Center(0,0).x==-1.75f,"grid remains world anchored during subcell player movement");
    grid.Lane(map.lanes[0],.1f,4.f);
    Check(cellAt({.25f,.25f}).arrivalMs<0.f,"lookahead clips future grid occupancy");
    map.lanes[0].remainingLifeMs=4.f;
    grid.Reset({},2.f); grid.Lane(map.lanes[0],.1f,800.f);
    Check(cellAt({.25f,.25f}).arrivalMs<0.f,"expiry clips grid paths instead of extending their colors");
    map.lanes[0].remainingLifeMs=800.f; map.lanes[0].beam=true; map.lanes[0].pointTimesMs[1]=0.f;
    grid.Reset({},2.f); grid.Lane(map.lanes[0],.1f,800.f);
    Check(cellAt({.25f,.25f}).arrivalMs==0.f,"entire beam occupies grid simultaneously, not just its head");
    grid.Reset({},2.f);
    grid.Segment({.25f,.25f},{.25f,.25f},.2f,500.f,500.f);
    grid.Segment({.25f,.25f},{.25f,.25f},.2f,100.f,100.f);
    Check(cellAt({.25f,.25f}).arrivalMs==100.f,"overlapping projectile predictions retain earliest arrival");
    grid.Segment({.25f,.25f},{.25f,.25f},.2f,0.f,0.f,true);
    Check(cellAt({.25f,.25f}).enemy && cellAt({.25f,.25f}).arrivalMs==100.f && !cellAt({.75f,.25f}).enemy,
        "enemy occupancy is separate from time colors and uses physical radius");
    const auto nowColor=DebugGeometry::TimeColor(0.f,800.f),futureColor=DebugGeometry::TimeColor(800.f,800.f);
    Check(nowColor.r>nowColor.b && futureColor.b>futureColor.r,"fixed palette encodes near and far times");
    const float fineStep=DebugGeometry::Grid::Resolution(60.f,.04f);
    Check(fineStep*60.f<=1.f && fineStep<=.02f,"resolution follows screen pixels and narrow projectile diameter");
    grid.Reset({},2.f,fineStep);
    grid.Segment({.15f,.15f},{.15f,.15f},.04f,0.f,0.f);
    Check(cellAt({.15f,.15f}).arrivalMs==0.f && cellAt({.21f,.15f}).arrivalMs<0.f,
        "small bullet between half-tile centres is visible without inflating its radius");
    int smallCount=0,largeCount=0;
    for(const auto& cell:grid.cells) if(cell.arrivalMs>=0.f) ++smallCount;
    grid.Reset({},2.f,fineStep); grid.Segment({.15f,.15f},{.15f,.15f},.08f,0.f,0.f);
    for(const auto& cell:grid.cells) if(cell.arrivalMs>=0.f) ++largeCount;
    Check(largeCount>smallCount*3 && largeCount<smallCount*5,"doubling contact radius changes raster area approximately fourfold");
    bool exactCapsules=true;
    for(const Vec2 end: {Vec2{1.4f,1.1f},Vec2{1.4f,-1.1f},Vec2{0.f,1.4f},Vec2{1.4f,0.f}}) {
        for(int direction=0;direction<2;direction++) {
            const Vec2 a=direction?end:Mul(end,-1.f),b=Mul(a,-1.f),delta=Sub(b,a);
            grid.Reset({},2.f,fineStep); grid.Segment(a,b,.04f,0.f,800.f);
            for(int y=0;y<grid.height;y++) for(int x=0;x<grid.width;x++) {
                const Vec2 p=grid.Center(x,y);
                const float u=std::clamp(Dot(Sub(p,a),delta)/LenSq(delta),0.f,1.f);
                const bool contact=LenSq(Sub(p,Add(a,Mul(delta,u))))<=.04f*.04f && LenSq(p)<=4.f;
                exactCapsules=exactCapsules && (grid.At(x,y).arrivalMs>=0.f)==contact;
            }
        }
    }
    Check(exactCapsules,"fine scanline raster matches exact circle sweep in both directions, including diagonal and axis-aligned shots");
    grid.Reset({},2.f,.0078125f);
    grid.Segment({-16.f,0.f},{16.f,0.f},.01f,0.f,800.f);
    Check(cellAt({0.f,0.f}).arrivalMs>399.f && cellAt({0.f,.02f}).arrivalMs<0.f,
        "tiny circles on long fast lanes retain contact precision without square cancellation");
    grid.Reset({},2.f,fineStep);
    const Vec2 anchored=grid.Center(100,100);
    grid.Reset({.213f,.071f},2.f,fineStep);
    const int ax=static_cast<int>(std::floor(anchored.x/grid.step))-grid.originX;
    const int ay=static_cast<int>(std::floor(anchored.y/grid.step))-grid.originY;
    Check(LenSq(Sub(grid.Center(ax,ay),anchored))<1e-10f,"fine texture cells keep fixed world coordinates while the player walks");
    grid.Reset({},16.f,.0078125f);
    Check(grid.width<=DebugGeometry::Grid::maxSide && grid.height<=DebugGeometry::Grid::maxSide &&
        grid.Center(0,0).x<=-15.9f && grid.Center(grid.width-1,0).x>=15.9f,
        "extreme resolution is memory bounded without clipping the requested look range");
    grid.Reset({},2.f,fineStep,{-.5f,-.5f},{.5f,.5f});
    Check(grid.width<=66 && grid.height<=66,"raster skips offscreen world cells");
    grid.Segment({.15f,.15f},{.15f,.15f},.04f,0.f,0.f);
    const int stride=grid.width+7;
    std::vector<uint32_t> pixels(static_cast<size_t>(stride)*grid.height,0x12345678u);
    grid.WritePixels(pixels.data(),stride*sizeof(uint32_t),800.f);
    bool paddedRows=true,transparentEmpty=true,colouredContact=false;
    for(int y=0;y<grid.height;y++) {
        for(int x=grid.width;x<stride;x++) paddedRows=paddedRows && pixels[y*stride+x]==0x12345678u;
        for(int x=0;x<grid.width;x++) {
            if(grid.At(x,y).arrivalMs<0.f) transparentEmpty=transparentEmpty && pixels[y*stride+x]==0;
            else colouredContact=colouredContact || pixels[y*stride+x]==(248u|(113u<<8)|(113u<<16)|(65u<<24));
        }
    }
    Check(paddedRows && transparentEmpty && colouredContact,"RGBA texture respects row pitch, clears empty cells and preserves contact colour");
    const auto rasterStart=std::chrono::steady_clock::now();
    grid.Reset({},16.f,fineStep,{-10.f,-8.f},{10.f,8.f});
    for(int i=0;i<512;i++) {
        const float theta=6.2831853f*i/512.f;
        const Vec2 a{9.f*std::cos(theta),9.f*std::sin(theta)};
        grid.Segment(a,Mul(a,-1.f),.06f,0.f,800.f,false,false,true);
    }
    pixels.resize(static_cast<size_t>(grid.width)*grid.height);
    grid.WritePixels(pixels.data(),grid.width*sizeof(uint32_t),800.f);
    std::printf("Dense debug raster: %dx%d, 512 square crossing lanes, %.2f ms CPU\n",grid.width,grid.height,
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-rasterStart).count());
    const auto warmStart=std::chrono::steady_clock::now();
    for(int frame=0;frame<4;frame++) {
        grid.Reset({},16.f,fineStep,{-10.f,-8.f},{10.f,8.f});
        for(int i=0;i<512;i++) {
            const float theta=6.2831853f*i/512.f;
            const Vec2 a{9.f*std::cos(theta),9.f*std::sin(theta)};
            grid.Segment(a,Mul(a,-1.f),.06f,0.f,800.f,false,false,true);
        }
        grid.WritePixels(pixels.data(),grid.width*sizeof(uint32_t),800.f);
    }
    std::printf("Dense debug raster warm: %.2f ms CPU/frame\n",
        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-warmStart).count()/4.);
    const auto forwardPixels=pixels;
    grid.Reset({},16.f,fineStep,{-10.f,-8.f},{10.f,8.f});
    for(int i=511;i>=0;i--) {
        const float theta=6.2831853f*i/512.f;
        const Vec2 a{9.f*std::cos(theta),9.f*std::sin(theta)};
        grid.Segment(a,Mul(a,-1.f),.06f,0.f,800.f,false,false,true);
    }
    grid.WritePixels(pixels.data(),grid.width*sizeof(uint32_t),800.f);
    Check(pixels==forwardPixels,"dense grid colors are stable when projectile priority order reverses");
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    std::printf("Spacetime: %d checks, %d failures (%.1f ms total)\n",checks,failures,ms);
    return failures ? 1 : 0;
}

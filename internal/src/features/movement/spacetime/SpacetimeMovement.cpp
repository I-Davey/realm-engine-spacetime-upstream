#include "pch-il2cpp.h"
#include "SpacetimeMovement.h"
#include "../udodge/UDodgeCore.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <chrono>
#include <vector>

namespace SpacetimeDodge {
MovementGoal SelectMovementGoal(const WaypointGoal& waypoint,const MovementGoal& firing) {
    if(!waypoint.active) return firing;
    MovementGoal goal{};
    goal.active=true; goal.waypoint=true; goal.identity=uint64_t{1}<<63;
    goal.center=waypoint.position; goal.range=waypoint.radius; goal.context=&waypoint;
    goal.contains=[](const void* context,Vec2 p) {
        const auto& w=*static_cast<const WaypointGoal*>(context);
        return UDodge::Len(UDodge::Sub(p,w.position))<=w.radius;
    };
    return goal;
}
namespace {
using namespace UDodge;
bool Contains(const MovementGoal& g, Vec2 p) {
    return g.active && g.contains && g.contains(g.context, p);
}
bool Arrived(const MovementGoal& g, Vec2 p) {
    return Contains(g,p) && (!g.arrivalContains || g.arrivalContains(g.context,p));
}
bool Walk(const Input& in, Vec2 a, Vec2 b) {
    return OccupancyPathClear(in.world,a,b) && EnemyPathClear(in.world,a,b);
}
bool Stand(const Input& in, Vec2 p) {
    return !Core::EnemyBlocked(in.world,p) && Walk(in,p,p);
}
// Static geometry routes are cached. Projectiles never become permanent
// navigation walls: the full timed proposal is checked separately each update.
void FindRoute(const Input& in, const MovementGoal& g, NavigationState& nav) {
    const auto began=std::chrono::steady_clock::now();
    nav.Reset(); nav.identity=g.identity; nav.center=g.center; nav.range=g.range; nav.searchedMs=in.nowMs;
    nav.entering=true;
    nav.checkpoint=in.world.player; nav.progressMs=in.nowMs;
    const Vec2 start=in.world.player;
    // Find the first usable point along the direct approach, then refine its
    // boundary. No artificial inner radius or preferred point on a ring.
    const Vec2 delta=Sub(g.center,start); const float distance=Len(delta);
    const float reach=std::min(distance,6.f);
    const int samples=std::max(1,static_cast<int>(std::ceil(reach/.2f)));
    float previous=0.f;
    for(int i=1;i<=samples;i++) {
        const float t=reach*i/samples;
        const Vec2 p=Add(start,Mul(Normalize(delta),t));
        if(!Walk(in,start,p)) break;
        if(Stand(in,p) && Arrived(g,p)) {
            float lo=previous,hi=t;
            for(int j=0;j<10;j++) {
                const float mid=(lo+hi)*.5f;
                if(Arrived(g,Add(start,Mul(Normalize(delta),mid)))) hi=mid; else lo=mid;
            }
            nav.route[0]=Add(start,Mul(Normalize(delta),hi)); nav.count=1; nav.complete=true; return;
        }
        previous=t;
    }
    constexpr int radius=18, side=radius*2+1, count=side*side;
    constexpr float cell=.35f;
    struct Entry { float score; int index; bool operator<(const Entry& b) const { return score>b.score; } };
    std::array<float,count> cost; cost.fill(std::numeric_limits<float>::infinity());
    std::array<int,count> parent; parent.fill(-1);
    std::array<bool,count> closed{};
    std::priority_queue<Entry> open;
    const int root=radius*side+radius;
    const auto position=[&](int index){return Add(start,{(index%side-radius)*cell,(index/side-radius)*cell});};
    // Distance to the enclosing range is an admissible lower bound, including
    // occluded firing positions (zero inside range). Prioritize the requested
    // direction without turning projectile trails into navigation walls.
    const auto heuristic=[&](Vec2 p){return std::max(0.f,Len(Sub(g.center,p))-g.range);};
    cost[root]=0.f; open.push({0.f,root});
    int found=-1,best=root,expanded=0; float progress=distance;
    while(!open.empty()) {
        if(in.settings.maxSearchMs>0.f && (++expanded%16)==0 &&
            std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-began).count()>
                std::min(1.f,in.settings.maxSearchMs*.25f)) break;
        const int cur=open.top().index; open.pop();
        if(closed[cur]) continue;
        closed[cur]=true;
        const Vec2 p=position(cur);
        if(cur!=root && Stand(in,p) && Arrived(g,p)) { found=cur; nav.complete=true; break; }
        const float d=Len(Sub(g.center,p));
        if(d<progress && Stand(in,p)) { progress=d; best=cur; }
        const int x=cur%side,y=cur/side;
        for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++) {
            if((!dx && !dy) || x+dx<0 || x+dx>=side || y+dy<0 || y+dy>=side) continue;
            const int next=(y+dy)*side+x+dx;
            const float c=cost[cur]+cell*(dx && dy?1.414214f:1.f);
            if(closed[next] || c>=cost[next] || !Walk(in,p,position(next))) continue;
            cost[next]=c; parent[next]=cur; open.push({c+heuristic(position(next)),next});
        }
    }
    // Partial progress is allowed only for a distant target. If already within
    // range but occluded, never walk into a dead end just to reduce distance.
    if(found<0 && distance>g.range+cell && progress<distance-cell) found=best;
    if(found<0 || found==root) return;
    std::vector<Vec2> reverse;
    for(int i=found;i!=root && i>=0;i=parent[i]) reverse.push_back(position(i));
    if(reverse.size()>nav.route.size()) return;
    for(auto i=reverse.rbegin();i!=reverse.rend();++i) nav.route[nav.count++]=*i;
}

// Follow the local geometry route for a finite duration, then dwell. The
// proposal includes the continuation that makes crossing a projectile lane
// safe; it does not require stopping inside that lane after a single frame.
bool TravelPlan(const Input& in,const NavigationState& nav,Vec2 sidestep,float sideMs,
    float travelMs,float waitMs,Plan& plan,float& progress,float prefixMs=0.f) {
    plan={}; plan.speed=in.world.speed; plan.epochMs=in.nowMs; plan.departureMs=in.settings.leadMs+waitMs;
    plan.points[plan.count++]={in.world.player,0.f};
    float time=plan.departureMs;
    if(time>0.f) plan.points[plan.count++]={in.world.player,time};
    const float end=std::min(in.settings.horizonMs-in.settings.dwellMs,time+travelMs);
    if(end<=time || end<in.settings.leadMs+in.frameMs) return false;
    Vec2 position=in.world.player;
    progress=0.f;
    const auto append=[&](Vec2 destination) {
        const float distance=Len(Sub(destination,position));
        if(distance<=1e-6f) return true;
        const float duration=std::min(distance/in.world.speed,end-time);
        if(duration<=0.f || plan.count>=kMaxPlanPoints-2) return false;
        const Vec2 next=duration>=distance/in.world.speed?destination:
            Add(position,Mul(Normalize(Sub(destination,position)),in.world.speed*duration));
        plan.intervention+=Len(Sub(next,position));
        position=next; time+=duration; plan.points[plan.count++]={position,time};
        return time<end-.001f;
    };
    if(prefixMs>0.f && nav.next<nav.count) {
        const Vec2 delta=Sub(nav.route[nav.next],position);
        if(!append(Add(position,Mul(Normalize(delta),std::min(Len(delta),in.world.speed*prefixMs))))) return false;
    }
    if(sideMs>0.f && !append(Add(position,Mul(sidestep,in.world.speed*sideMs)))) return false;
    if(nav.next<nav.count) progress=Len(Sub(in.world.player,nav.route[nav.next]))-
        Len(Sub(position,nav.route[nav.next]));
    for(int i=nav.next;i<nav.count;i++) {
        const Vec2 before=position;
        const bool more=append(nav.route[i]);
        progress+=Len(Sub(position,before));
        if(!more) break;
    }
    if(plan.intervention<1e-5f) return false;
    plan.points[plan.count++]={position,in.settings.horizonMs};
    return true;
}

bool Approach(const Input& in, const MovementGoal& goal, NavigationState& nav, Output& out) {
    const auto began=std::chrono::steady_clock::now();
    if(!in.world.map || in.world.map->limited || in.world.map->projectileSourceUnavailable ||
       in.world.map->enemyCount<0 || in.world.map->enemyCount>UDodge::kMaxEnemies ||
       in.world.movementLocked || !std::isfinite(in.world.speed) || !(in.world.speed>0.f) ||
       !std::isfinite(in.world.player.x) || !std::isfinite(in.world.player.y) ||
       !std::isfinite(in.frameMs) || !(in.frameMs>0.f)) return false;
    const bool changed=nav.identity!=goal.identity || Len(Sub(nav.center,goal.center))>.35f ||
        std::fabs(nav.range-goal.range)>.01f ||
        (nav.complete && nav.count>0 && !Arrived(goal,nav.route[nav.count-1]));
    while(nav.next<nav.count && Len(Sub(nav.route[nav.next],in.world.player))<.005f) ++nav.next;
    const bool obstructed=nav.next<nav.count && !Walk(in,in.world.player,nav.route[nav.next]);
    const bool arrived=nav.next>=nav.count;
    if(Len(Sub(nav.checkpoint,in.world.player))>.08f) {
        nav.checkpoint=in.world.player; nav.progressMs=in.nowMs;
    }
    const bool stalled=nav.travelValid && in.nowMs-nav.progressMs>350. &&
        in.nowMs>nav.travel.epochMs+nav.travel.departureMs+350.;
    if(changed || obstructed || stalled || (arrived && (nav.count>0 || in.nowMs-nav.searchedMs>=200.)))
        FindRoute(in,goal,nav);
    if(nav.next>=nav.count) return false;
    // Retain the chosen side/turn and absolute timing while still safe. Do not
    // recreate a fresh sidestep each update or postpone its rejoin forever.
    if(nav.travelValid && EvaluateTrajectory(in,nav.travel,out) &&
       (Len(out.velocity)>1e-6f || out.waitMs>0.f)) { out.reused=true; return true; }
    nav.travelValid=false; nav.detouring=false;
    // Shortcut only through checked geometry, preserving corridor turns.
    int next=nav.next;
    while(next+1<nav.count && Walk(in,in.world.player,nav.route[next+1])) ++next;
    nav.next=next;
    const Vec2 waypoint=nav.route[next];
    const auto timedOut=[&] { return in.settings.maxSearchMs>0.f &&
        std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-began).count()>
            in.settings.maxSearchMs*.4f; };
    const auto movingTimedOut=[&] { return in.settings.maxSearchMs>0.f &&
        std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-began).count()>
            in.settings.maxSearchMs*.3f; };
    const auto accept=[&](Vec2 side,float sideMs,float duration,float waitMs=0.f) {
        Plan plan{}; Output proposal{}; float progress=0.f;
        if(!TravelPlan(in,nav,side,sideMs,duration,waitMs,plan,progress) || !EvaluateTrajectory(in,plan,proposal)) return false;
        // A bounded detour must actually rejoin/progress along navigation, not
        // turn a travel request into permanent sideways drift.
        if(sideMs>0.f && progress<.01f) return false;
        // Keep travelling in the requested direction until the latest tested
        // safe turn. A delayed turn is forward motion, not an idle wait.
        if(sideMs>0.f) for(float prefix=std::min(240.f,duration*.5f);prefix>=20.f;prefix-=20.f) {
            if(movingTimedOut()) break;
            Plan delayed{}; Output refined{}; float delayedProgress=0.f;
            if(TravelPlan(in,nav,side,sideMs,duration,waitMs,delayed,delayedProgress,prefix) &&
               delayedProgress>.01f && EvaluateTrajectory(in,delayed,refined)) {
                plan=delayed; proposal=refined; break;
            }
        }
        nav.travel=plan; nav.travelValid=true; nav.detouring=sideMs>0.f; out=proposal; return true;
    };
    const float horizon=in.settings.horizonMs-in.settings.leadMs-in.settings.dwellMs;
    if(accept({},0.f,horizon)) return true;
    const Vec2 forward=Normalize(Sub(waypoint,in.world.player));
    // Small symmetric alternatives first, including a turn back onto the
    // route. Every edge and the terminal dwell use the existing continuous
    // projectile, bomb, wall and body checks. Reserve time for emergency dodge.
    for(float sideMs:{40.f,80.f,160.f,240.f}) {
        if(in.world.speed*sideMs>in.settings.maxDistance || movingTimedOut()) break;
        for(float angle:{.3926991f,-.3926991f,.7853982f,-.7853982f,1.5707963f,-1.5707963f}) {
            if(movingTimedOut()) break;
            const Vec2 side{forward.x*std::cos(angle)-forward.y*std::sin(angle),
                forward.x*std::sin(angle)+forward.y*std::cos(angle)};
            if(accept(side,sideMs,horizon)) return true;
        }
    }
    // For a travel request, safe full-speed progress precedes stopping. Wait
    // when a moving route cannot fit; retained timing prevents sliding waits.
    for(float waitMs:{20.f,40.f,80.f,120.f,160.f}) {
        if(timedOut()) return false;
        if(accept({},0.f,horizon,waitMs)) return true;
    }
    // A longer route may be blocked by a later wave. Shorter progress remains
    // useful only when its stopped endpoint is safe for the entire horizon.
    for(float duration:{horizon*.5f,horizon*.25f,in.frameMs}) {
        if(timedOut()) break;
        if(accept({},0.f,duration)) return true;
    }
    return false;
}
}

Vec2 MovementDisplacement(Vec2 selected, Vec2 applied, float commandMs) {
    return UDodge::Mul(UDodge::Sub(selected,applied),commandMs);
}

void PrepareAutomaticMovement(Input& in,float commandIntervalMs,float commandDelayMs,float availableMs) {
    const float frame=in.frameMs;
    in.nominal={};
    // A steady command interval prevents render jitter changing the route's
    // control lattice and the size of every automatic movement proposal.
    in.settings.stepMs=std::max({10.f,frame,commandIntervalMs});
    in.settings.leadMs=std::max(0.f,commandDelayMs);
    in.frameMs=in.settings.stepMs;
    in.actuationMs=std::min(in.settings.stepMs,std::max(1.f,availableMs));
    // Distance is already limited by the stroke duration. Reducing velocity
    // as well excluded every full-speed heading on the first idle command.
    in.maxCorrectionSpeed=in.world.speed;
}

bool PrepareFrameMovement(Input& in,float availableMs) {
    if(!std::isfinite(availableMs) || availableMs<1.f) return false;
    in.nominal={};
    in.frameMs=std::min(50.f,availableMs);
    // Search resolution and execution cadence are separate. A 40ms search
    // edge can execute across several game updates without artificial stops.
    in.settings.stepMs=40.f;
    in.settings.leadMs=0.f;
    in.actuationMs=0.f;
    in.maxCorrectionSpeed=in.world.speed;
    return true;
}

bool PrepareNativeMovement(Input& in,Vec2 requested) {
    using namespace UDodge;
    if(!std::isfinite(in.world.speed) || in.world.speed<=0.f ||
       !std::isfinite(in.frameMs) || in.frameMs<=0.f || in.frameMs>50.f ||
       !std::isfinite(requested.x) || !std::isfinite(requested.y)) return false;
    const Vec2 delta=Sub(requested,in.world.player);
    const float distance=Len(delta);
    // Unity render delta is not necessarily the duration used by native
    // movement. A bounded world step supplies its own minimum travel time at
    // verified speed. Do not reject normal 20ms moves during an 8ms render.
    // Long scripted moves/teleports remain outside the walking filter.
    if(!std::isfinite(distance) || distance>in.world.speed*50.f+.002f) return false;
    // The game's movement slice and the render frame can have different
    // durations in either direction. Project held movement at live speed,
    // using distance/speed for this command's time, never a slower future
    // velocity manufactured by dividing by an unrelated render duration.
    if(distance>1e-6f) {
        in.frameMs=std::clamp(distance/in.world.speed,1.f,50.f);
        in.nominal=Mul(Normalize(delta),in.world.speed);
    } else in.nominal={};
    in.settings.leadMs=0.f;
    in.settings.stepMs=std::max(10.f,in.frameMs);
    in.actuationMs=in.settings.stepMs;
    in.maxCorrectionSpeed=in.world.speed*2.f;
    return true;
}
bool NativeMovementTarget(const Input& in,const Output& out,Vec2& target) {
    using namespace UDodge;
    if(out.status!=Status::Moving && out.status!=Status::Recovery) return false;
    if(Len(Sub(out.velocity,in.nominal))<=1e-6f) return false;
    const Vec2 displacement=Mul(out.velocity,in.frameMs);
    if(!std::isfinite(displacement.x) || !std::isfinite(displacement.y) ||
       Len(displacement)>in.world.speed*in.frameMs+1e-5f) return false;
    target=Add(in.world.player,displacement);
    return true;
}

void EvaluateMovement(const Input& in, bool keyboard, const MovementGoal& goal,
    State& state, NavigationState& nav, MovementDecision& out) {
    const auto began=std::chrono::steady_clock::now();
    out=MovementDecision{};
    if(keyboard && !in.settings.avoidHarmlessBlocks && KeyboardIntentSafe(in)) {
        // Native movement owns harmless collisions. A wall, prop, or water
        // must not manufacture a dodge or keep an obsolete detour alive.
        state.Reset(); nav.Reset(); out.dodge.status=Status::Clear;
        out.dodge.velocity=in.nominal; out.action="following keyboard / no harmful threat"; return;
    }
    if(keyboard && UDodge::Len(in.nominal)>1e-6f && in.world.speed>0.f && !in.world.movementLocked) {
        // A held direction is a travel goal too. Keep its world anchor through
        // a detour instead of inventing a new goal on every lateral step.
        const Vec2 direction=UDodge::Normalize(in.nominal);
        const bool changed=!nav.directional || UDodge::Dot(direction,nav.direction)<.995f;
        if(changed) nav.Reset();
        if(state.valid && !Validate(in,state.plan)) state.Reset();
        if(state.valid) {
            Evaluate(in,state,out.dodge);
            if(out.dodge.reused && out.dodge.status==Status::Moving) {
                out.action="finishing escape / keyboard"; return;
            }
        }
        if(!nav.travelValid) {
            Plan baseline{}; baseline.count=2; baseline.epochMs=in.nowMs;
            baseline.speed=in.world.speed; baseline.nominal=in.nominal;
            baseline.points[0]={in.world.player,0.f};
            baseline.points[1]={UDodge::Add(in.world.player,UDodge::Mul(in.nominal,in.settings.horizonMs)),in.settings.horizonMs};
            if(EvaluateTrajectory(in,baseline,out.dodge)) {
                out.dodge.status=Status::Clear; out.dodge.velocity=in.nominal;
                nav.Reset(); state.Reset(); out.action="following keyboard direction"; return;
            }
        }
        const float reach=std::max(2.f,std::min(6.f,in.world.speed*in.settings.horizonMs));
        const Vec2 center=changed || nav.count==0 || UDodge::Dot(UDodge::Sub(nav.center,in.world.player),direction)<.5f?
            UDodge::Add(in.world.player,UDodge::Mul(direction,reach)):nav.center;
        MovementGoal heading{}; heading.active=true; heading.identity=uint64_t{1}<<62;
        heading.center=center; heading.range=.2f; heading.context=&heading;
        heading.contains=[](const void* context,Vec2 p) {
            const auto& g=*static_cast<const MovementGoal*>(context);
            return UDodge::Len(UDodge::Sub(p,g.center))<=g.range;
        };
        Input travel=in; travel.nominal={};
        if(in.settings.maxSearchMs>0.f) travel.settings.maxSearchMs=std::max(.01f,in.settings.maxSearchMs-
            std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-began).count());
        Output proposal{};
        const bool found=Approach(travel,heading,nav,proposal);
        nav.directional=true; nav.direction=direction;
        if(found) {
            out.dodge=proposal; out.dodge.plan.nominal=in.nominal;
            const float age=static_cast<float>(in.nowMs-proposal.plan.epochMs);
            out.dodge.waitMs=0.f;
            for(int i=1;i<proposal.plan.count;i++) {
                const auto& a=proposal.plan.points[i-1]; const auto& b=proposal.plan.points[i];
                if(b.timeMs<=age || b.timeMs<=a.timeMs) continue;
                const Vec2 v=UDodge::Mul(UDodge::Sub(b.pos,a.pos),1.f/(b.timeMs-a.timeMs));
                if(UDodge::Len(UDodge::Sub(v,in.nominal))>1e-5f) {
                    out.dodge.waitMs=std::max(0.f,a.timeMs-age); break;
                }
            }
            // Native endpoint replacement receives the selected absolute
            // velocity, including a deliberate stop. No second actuator.
            out.dodge.status=UDodge::Len(UDodge::Sub(proposal.velocity,in.nominal))>1e-6f?Status::Moving:Status::Clear;
            out.detouring=nav.detouring; state.Reset();
            out.action=nav.detouring?"routing through / keyboard":"following route / keyboard"; return;
        }
        Input fallback=in;
        if(in.settings.maxSearchMs>0.f) fallback.settings.maxSearchMs=std::max(.01f,in.settings.maxSearchMs-
            std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-began).count());
        Evaluate(fallback,state,out.dodge); out.action="safety fallback / keyboard"; return;
    }
    if(nav.directional) nav.Reset();
    if(keyboard || !goal.active || !goal.contains || !std::isfinite(goal.center.x) ||
       !std::isfinite(goal.center.y) || !std::isfinite(goal.range) || goal.range<=0.f) {
        nav.Reset(); Evaluate(in,state,out.dodge);
        out.action=keyboard?"keyboard / dodge":"holding / dodge"; return;
    }
    // An automatic goal never pretends to be movement already applied by keys.
    Input automatic=in; automatic.nominal={};
    const bool finishingEntry=nav.entering && nav.identity==goal.identity && !Arrived(goal,in.world.player);
    const bool holding=Contains(goal,in.world.player) && !finishingEntry;
    // Keep an executing escape stable. A waiting idle dodge can yield to a
    // completely validated travel route that already avoids its threat.
    bool evaluated=false;
    // Invalidating a retained escape must not launch a full idle search before
    // travel gets its chance. That consumed the budget and stranded goals.
    if(!holding && state.valid && !Validate(automatic,state.plan)) state.Reset();
    if(state.valid || holding) {
        Evaluate(automatic,state,out.dodge); evaluated=true;
        if((out.dodge.status==Status::Moving && (out.dodge.reused || holding)) || out.dodge.status==Status::Locked) {
            nav.travelValid=false;
            out.action="dodging / retaining escape"; return;
        }
        if(holding) { nav.Reset(); out.action=goal.waypoint?"holding waypoint / dodge":"holding firing position / dodge"; return; }
    }
    Output approach{};
    const auto remaining=[&] {
        return in.settings.maxSearchMs-std::chrono::duration<float,std::milli>(
            std::chrono::steady_clock::now()-began).count();
    };
    if(in.settings.maxSearchMs>0.f) automatic.settings.maxSearchMs=std::max(.01f,remaining());
    if((in.settings.maxSearchMs<=0.f || remaining()>0.f) && Approach(automatic,goal,nav,approach)) {
        out.dodge=approach; out.approaching=true; out.detouring=nav.detouring;
        out.action=nav.detouring?"safe detour / continuing goal":goal.waypoint?"travelling to script waypoint":"approaching firing zone";
        state.Reset(); return;
    }
    if(!evaluated) {
        if(in.settings.maxSearchMs>0.f) automatic.settings.maxSearchMs=std::max(.01f,remaining());
        Evaluate(automatic,state,out.dodge);
    }
    out.action=out.dodge.status==Status::Clear?"waiting for safe approach":"dodging / approach blocked";
}
}

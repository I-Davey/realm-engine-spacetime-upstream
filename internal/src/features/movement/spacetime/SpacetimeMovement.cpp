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
    nav.Reset(); nav.identity=g.identity; nav.center=g.center; nav.range=g.range;
    nav.entering=true;
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
    // Native replacement is continuous across calls. Frame jitter must not
    // change the search lattice or introduce a fictitious off portion.
    in.settings.stepMs=40.f;
    in.actuationMs=0.f;
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

bool ResolveNativeMovementTarget(const Input& in,Output& out,bool proposalUsable,
    bool retainControl,Vec2 requested,Vec2& target) {
    Vec2 candidate{};
    if(proposalUsable && NativeMovementTarget(in,out,candidate) &&
       UDodge::OccupancyPathClear(in.world,in.world.player,candidate) &&
       EnemyPathClear(in.world,in.world.player,candidate)) { target=candidate; return true; }
    Output emergency=out;
    emergency.status=Status::Incomplete;
    if(!ProtectImmediateStep(in,emergency,retainControl)) return false;
    out=emergency;
    candidate=UDodge::Add(in.world.player,UDodge::Mul(out.velocity,in.frameMs));
    if(UDodge::Len(UDodge::Sub(candidate,requested))<=1e-6f) return false;
    target=candidate; return true;
}

void EvaluateMovement(const Input& in,bool keyboard,const MovementGoal& supplied,
    State& state,NavigationState& nav,MovementDecision& out) {
    out=MovementDecision{};
    Input request=in;
    request.guidance={};
    request.guidance.manual=keyboard;
    MovementGoal goal=supplied;
    if(keyboard && LenSq(in.nominal)>1e-12f) {
        const Vec2 direction=Normalize(in.nominal);
        const bool changed=!nav.directional || Dot(direction,nav.direction)<.995f;
        if(changed) nav.Reset();
        goal={}; goal.active=true; goal.identity=uint64_t{1}<<62; goal.range=.2f;
        const float reach=std::max(2.f,std::min(6.f,in.world.speed*in.settings.horizonMs));
        goal.center=changed || nav.count==0 || Dot(Sub(nav.center,in.world.player),direction)<.5f?
            Add(in.world.player,Mul(direction,reach)):nav.center;
        goal.context=&goal; goal.contains=[](const void* c,Vec2 p) {
            const auto& g=*static_cast<const MovementGoal*>(c);
            return Len(Sub(p,g.center))<=g.range;
        };
        nav.direction=direction;
    } else if(keyboard) goal={};
    if(!keyboard && nav.directional) nav.Reset();
    const bool validGoal=goal.active && goal.contains && std::isfinite(goal.center.x) &&
        std::isfinite(goal.center.y) && std::isfinite(goal.range) && goal.range>0.f;
    const bool holding=validGoal && Contains(goal,in.world.player) &&
        !(nav.entering && nav.identity==goal.identity && !Arrived(goal,in.world.player));
    if(validGoal && !holding) {
        const bool changed=nav.identity!=goal.identity || Len(Sub(nav.center,goal.center))>.35f ||
            std::fabs(nav.range-goal.range)>.01f ||
            (nav.complete && nav.count>0 && !Arrived(goal,nav.route[nav.count-1]));
        while(nav.next<nav.count && Len(Sub(nav.route[nav.next],in.world.player))<.005f) ++nav.next;
        const bool blocked=nav.next<nav.count && !Walk(in,in.world.player,nav.route[nav.next]);
        if(changed || blocked || nav.next>=nav.count) FindRoute(in,goal,nav);
        while(nav.next+1<nav.count && Walk(in,in.world.player,nav.route[nav.next+1])) ++nav.next;
        request.guidance.active=nav.next<nav.count;
        request.guidance.identity=goal.identity;
        for(int i=nav.next;i<nav.count;i++) request.guidance.points[request.guidance.count++]=nav.route[i];
    } else nav.Reset();
    nav.directional=keyboard && validGoal;
    if(nav.directional) nav.direction=Normalize(in.nominal);
    // One retained temporal trajectory for walking, approach, holding and escape.
    // Cancelling or reaching a goal must not keep executing its cached travel.
    // The idle objective immediately reassesses any remaining danger.
    if(!request.guidance.active && state.goalIdentity!=0) state.Reset();
    Evaluate(request,state,out.dodge);
    out.approaching=!keyboard && request.guidance.active &&
        state.goalIdentity==request.guidance.identity &&
        (out.dodge.status==Status::Moving || out.dodge.status==Status::Waiting);
    out.overrideActive=keyboard && out.dodge.status!=Status::Clear;
    nav.overrideActive=out.overrideActive;
    if(keyboard && out.dodge.status==Status::Clear) nav.Reset();
    out.detouring=request.guidance.active && Len(out.dodge.velocity)>1e-6f &&
        Dot(Normalize(out.dodge.velocity),Normalize(Sub(request.guidance.points[0],in.world.player)))<.98f;
    if(request.guidance.active) for(int i=1;i<out.dodge.plan.count;i++) {
        const Vec2 segment=Sub(out.dodge.plan.points[i].pos,out.dodge.plan.points[i-1].pos);
        if(LenSq(segment)>1e-10f && Dot(Normalize(segment),
            Normalize(Sub(request.guidance.points[0],in.world.player)))<.98f) out.detouring=true;
    }
    nav.detouring=out.detouring;
    out.action=out.dodge.status==Status::Recovery?"unified movement / minimizing damage":
        out.overrideActive?"unified movement / protecting requested direction":
        out.approaching?"unified movement / following goal":keyboard?"following safe keyboard input":"holding / dodge";
}
}

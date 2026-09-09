#pragma once
#include "SpacetimeCore.h"
#include <type_traits>

namespace SpacetimeDodge::Replay {
struct Audit {
    int matched=0,nearMatched=0,diverged=0;
    float maxError=0.f,nearError=0.f;
};
inline Audit Compare(const UDodge::DangerMap& previous,const UDodge::DangerMap& current,
    Vec2 player,float elapsed) {
    Audit result{};
    if(!std::isfinite(elapsed) || elapsed<1.f || elapsed>100.f) return result;
    for(int i=0;i<current.laneCount;i++) {
        const auto& now=current.lanes[i];
        if(now.provisional || now.beam || now.pointCount<1) continue;
        for(int j=0;j<previous.laneCount;j++) {
            const auto& old=previous.lanes[j];
            if(old.provisional || old.beam || old.bulletId!=now.bulletId ||
                old.attackerObjId!=now.attackerObjId || old.ownerObjId!=now.ownerObjId) continue;
            Vec2 expected{};
            if(SampleProjectile(old,elapsed,expected)!=SampleStatus::Known) break;
            const float error=UDodge::Len(UDodge::Sub(expected,now.points[0]));
            ++result.matched; result.maxError=std::max(result.maxError,error);
            if(UDodge::Len(UDodge::Sub(now.points[0],player))<=3.f) {
                ++result.nearMatched; result.nearError=std::max(result.nearError,error);
                if(error>std::max(.03f,now.hitHalf*.5f)) ++result.diverged;
            }
            break;
        }
    }
    return result;
}
struct Frame {
    Input input{};
    UDodge::DangerMap map{};
    Output output{};
    Vec2 requested{},command{};
    bool native=false,selected=false,applied=false,rejected=false,feedbackKnown=false;
    // World terrain callbacks and goal predicates cannot be serialized. Replay
    // is explicitly projectile/body/timed-zone geometry, not a wall proof.
    void Capture(const Input& in,const Output& out) {
        input=in; map=*in.world.map; output=out;
        input.world.map=nullptr; input.world.env={};
    }
    Input Restore() const { Input in=input; in.world.map=&map; return in; }
};
struct Capture {
    uint64_t magic=0x53545245504C3230ull; // STREPL20, same-build ABI only
    uint32_t bytes=sizeof(Capture),version=20;
    bool terrainRecorded=false,hasPrevious=false;
    Audit audit{};
    Frame previous{},current{};
    bool Compatible() const { return magic==0x53545245504C3230ull && version==20 && bytes==sizeof(Capture); }
};
static_assert(std::is_trivially_copyable_v<Capture>);
}

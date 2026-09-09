#pragma once
#include "SpacetimeCore.h"
#include "../../../game/math/W2S.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <vector>
#include <cstdint>

// Drawing helpers only: no collision verdicts and no mutation of planner state.
namespace SpacetimeDodge::DebugGeometry {
struct Disk { Vec2 center{}; float radius=0.f; };
struct Color { int r,g,b; };

inline bool ProjectDisk(Vec2 center,float radius,float camX,float camY,float angle,
                        float pixelsPerTile,float cx,float cy,Disk& disk) {
    if(!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(radius) || radius<0.f ||
       !std::isfinite(pixelsPerTile) || pixelsPerTile<=0.f || !std::isfinite(angle) ||
       !std::isfinite(camX) || !std::isfinite(camY) || !std::isfinite(cx) || !std::isfinite(cy)) return false;
    W2S(center.x,center.y,disk.center.x,disk.center.y,camX,camY,angle,pixelsPerTile,cx,cy);
    disk.radius=radius*pixelsPerTile; // zoom already IS pixels per tile
    return std::isfinite(disk.center.x) && std::isfinite(disk.center.y) && std::isfinite(disk.radius);
}

inline Color TimeColor(float timeMs,float horizonMs) {
    const float fraction=std::clamp(timeMs/std::max(1.f,horizonMs),0.f,1.f);
    const Color now{248,113,113},middle{251,191,90},future{96,165,250};
    const Color a=fraction<.5f?now:middle,b=fraction<.5f?middle:future;
    const float t=fraction<.5f?fraction*2.f:(fraction-.5f)*2.f;
    return {static_cast<int>(a.r+(b.r-a.r)*t),static_cast<int>(a.g+(b.g-a.g)*t),
            static_cast<int>(a.b+(b.b-a.b)*t)};
}

inline Vec2 PlayerAt(const Plan& plan,Vec2 origin,Vec2 nominal,double nowMs,float futureMs) {
    return plan.count>=2?PositionAt(plan,static_cast<float>(nowMs-plan.epochMs)+futureMs):
        UDodge::Add(origin,UDodge::Mul(nominal,futureMs));
}
inline float GridOpacity(double ageMs) {
    return static_cast<float>(std::clamp((250.-ageMs)/150.,0.,1.));
}

// Draw captured segments, never reconstruct a fast/curved shot by coarse
// sampling. Clip both position and time to the selected lookahead and expiry.
template<class Emit>
void ForEachSegment(const UDodge::LaneThreat& lane,float fromMs,float horizonMs,Emit emit) {
    if(lane.beam || lane.pointCount<1 || fromMs>=horizonMs) return;
    const float until=std::min(horizonMs,lane.remainingLifeMs>=0.f?lane.remainingLifeMs:horizonMs);
    for(int j=1;j<lane.pointCount;j++) {
        const float a=lane.pointTimesMs[j-1],b=lane.pointTimesMs[j];
        const float lo=std::max(fromMs,a),hi=std::min(until,b);
        if(b<=a || hi<=lo) continue;
        const Vec2 delta=UDodge::Sub(lane.points[j],lane.points[j-1]);
        emit(UDodge::Add(lane.points[j-1],UDodge::Mul(delta,(lo-a)/(b-a))),
             UDodge::Add(lane.points[j-1],UDodge::Mul(delta,(hi-a)/(b-a))),lo,hi);
    }
    const float lo=std::max(fromMs,lane.pointTimesMs[lane.pointCount-1]);
    Vec2 a{},b{};
    if(until>lo && ProjectLinearTail(lane,lo,a) && ProjectLinearTail(lane,until,b)) emit(a,b,lo,until);
}

// World-anchored texels, sampled at their centres with the matching swept square
// or capsule. Resolution only affects drawing, never the planner's contact size.
struct Grid {
    static constexpr int maxSide=2049;
    struct Cell { float arrivalMs=-1.f; bool enemy=false; };
    std::vector<Cell> cells;
    int originX=0,originY=0,width=0,height=0;
    Vec2 player{};
    float range=0.f,step=.5f;

    // Stable binary subdivisions avoid the grid swimming as zoom changes.
    // Aim for <=1 screen pixel and at least four samples across a bullet.
    static float Resolution(float pixelsPerTile,float smallestRadius) {
        float result=.0625f;
        const float target=std::min(1.f/std::max(1.f,pixelsPerTile),std::max(.0078125f,smallestRadius*.5f));
        while(result>target && result>.0078125f) result*=.5f;
        return result;
    }

    Vec2 Center(int x,int y) const { return {(originX+x+.5f)*step,(originY+y+.5f)*step}; }
    const Cell& At(int x,int y) const { return cells[y*width+x]; }
    void Reset(Vec2 pos,float radius,float requestedStep=.5f,
               Vec2 viewMin={-1e6f,-1e6f},Vec2 viewMax={1e6f,1e6f}) {
        player=pos; range=std::clamp(radius,0.f,32.f); step=std::clamp(requestedStep,.0078125f,.5f);
        const Vec2 lo{std::max(pos.x-range,viewMin.x),std::max(pos.y-range,viewMin.y)};
        const Vec2 hi{std::min(pos.x+range,viewMax.x),std::min(pos.y+range,viewMax.y)};
        if(lo.x>hi.x || lo.y>hi.y) { width=height=0; cells.clear(); return; }
        // Bound memory at extreme zoom / resolution without cropping threats.
        while(std::ceil((hi.x-lo.x)/step)+1>maxSide || std::ceil((hi.y-lo.y)/step)+1>maxSide) step*=2.f;
        originX=static_cast<int>(std::floor(lo.x/step));
        originY=static_cast<int>(std::floor(lo.y/step));
        width=static_cast<int>(std::floor(hi.x/step))-originX+1;
        height=static_cast<int>(std::floor(hi.y/step))-originY+1;
        cells.assign(static_cast<size_t>(width)*height,Cell{});
    }
    void Segment(Vec2 a,Vec2 b,float radius,float ta,float tb,bool enemy=false,bool beam=false,bool square=false) {
        if(width==0 || height==0 || !std::isfinite(radius) || radius<0.f) return;
        const int x0=std::max(0,static_cast<int>(std::floor((std::min(a.x,b.x)-radius)/step))-originX);
        const int x1=std::min(width-1,static_cast<int>(std::floor((std::max(a.x,b.x)+radius)/step))-originX);
        const int y0=std::max(0,static_cast<int>(std::floor((std::min(a.y,b.y)-radius)/step))-originY);
        const int y1=std::min(height-1,static_cast<int>(std::floor((std::max(a.y,b.y)+radius)/step))-originY);
        const Vec2 delta=UDodge::Sub(b,a);
        const float speedSq=UDodge::LenSq(delta);
        const float inverseSpeedSq=1.f/std::max(1e-12f,speedSq);
        const bool crossesRows=std::fabs(delta.y)>1e-8f;
        const float inverseY=1.f/(crossesRows?delta.y:1.f);
        for(int y=y0;y<=y1;y++) {
          // Clip the swept axis to this row's radius band. A diagonal lane
          // visits its narrow capsule, not every pixel in its bounding square.
          int rowX0=x0,rowX1=x1;
          if(crossesRows) {
            const float cy=Center(0,y).y;
            float u=(cy-radius-a.y)*inverseY,v=(cy+radius-a.y)*inverseY;
            if(u>v) std::swap(u,v);
            u=std::clamp(u,0.f,1.f); v=std::clamp(v,0.f,1.f);
            const float ax=a.x+delta.x*u,bx=a.x+delta.x*v;
            rowX0=std::max(x0,static_cast<int>(std::floor((std::min(ax,bx)-radius)/step))-originX);
            rowX1=std::min(x1,static_cast<int>(std::floor((std::max(ax,bx)+radius)/step))-originX);
          }
          for(int x=rowX0;x<=rowX1;x++) {
            const Vec2 center=Center(x,y);
            if(UDodge::LenSq(UDodge::Sub(center,player))>range*range) continue;
            const Vec2 relative=UDodge::Sub(a,center);
            float entry=0.f;
            const float c=UDodge::LenSq(relative)-radius*radius;
            if(square) {
                if(!SweptProjectileContact(relative,UDodge::Add(relative,delta),radius,&entry)) continue;
            } else if(c>0.f) {
                if(speedSq<1e-12f) continue;
                const float dot=UDodge::Dot(relative,delta);
                // Cross-product distance avoids subtracting two huge squares
                // for a tiny bullet on a long lane (loss of contact precision).
                const float cross=relative.x*delta.y-relative.y*delta.x;
                const float chord=radius*radius-cross*cross*inverseSpeedSq;
                if(chord<0.f) continue;
                entry=-dot*inverseSpeedSq-std::sqrt(chord*inverseSpeedSq);
                if(entry<0.f || entry>1.f) continue;
            }
            auto& cell=cells[y*width+x];
            if(enemy) { cell.enemy=true; continue; }
            const float arrival=beam?ta:ta+entry*(tb-ta);
            if(cell.arrivalMs<0.f || arrival<cell.arrivalMs) cell.arrivalMs=arrival;
          }
        }
    }
    // DXGI R8G8B8A8. Respect the mapped texture's padded row stride.
    void WritePixels(void* destination,size_t pitch,float horizonMs) const {
        for(int y=0;y<height;y++) {
            auto* row=reinterpret_cast<uint32_t*>(static_cast<unsigned char*>(destination)+pitch*y);
            for(int x=0;x<width;x++) {
                const auto& cell=At(x,y);
                if(cell.arrivalMs>=0.f) {
                    const auto c=TimeColor(cell.arrivalMs,horizonMs);
                    const auto alpha=static_cast<uint32_t>(65.f-50.f*std::clamp(cell.arrivalMs/std::max(1.f,horizonMs),0.f,1.f));
                    row[x]=static_cast<uint32_t>(c.r|(c.g<<8)|(c.b<<16)|(alpha<<24));
                } else row[x]=cell.enemy?(185u|(115u<<8)|(225u<<16)|(28u<<24)):0u;
            }
        }
    }
    void Lane(const UDodge::LaneThreat& lane,float radius,float horizonMs) {
        if(lane.pointCount<1 || lane.remainingLifeMs==0.f) return;
        if(lane.beam) { Segment(lane.points[0],lane.points[lane.pointCount-1],radius,0.f,0.f,false,true); return; }
        Vec2 current{};
        if(SampleProjectile(lane,0.f,current)==SampleStatus::Known) Segment(current,current,radius,0.f,0.f,false,false,true);
        ForEachSegment(lane,0.f,horizonMs,[&](Vec2 a,Vec2 b,float ta,float tb) { Segment(a,b,radius,ta,tb,false,false,true); });
    }
};
}

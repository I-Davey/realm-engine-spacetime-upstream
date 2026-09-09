#pragma once
#include "UDodgeTypes.h"
#include <cmath>
namespace UDodge {
// Preserve fractional phase on a sampled curved trajectory. Snapping time to
// the nearest sample and translating the curve changes its future direction.
inline bool FractionalPathAnchor(const float* times,const float* xs,const float* ys,
    int count,float time,int& index,Vec2& position) {
    if(count<2 || !std::isfinite(time) || time<times[0] || time>=times[count-1]) return false;
    for(int i=0;i<count-1;i++) {
        const float a=times[i],b=times[i+1];
        if(!std::isfinite(a) || !std::isfinite(b) || b<=a || time<a || time>=b) continue;
        const float t=(time-a)/(b-a);
        position={xs[i]+(xs[i+1]-xs[i])*t,ys[i]+(ys[i+1]-ys[i])*t}; index=i;
        return std::isfinite(position.x) && std::isfinite(position.y);
    }
    return false;
}
}

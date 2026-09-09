#pragma once
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

// Shared by the live overlay and the offline ImGui rendering fixture.
namespace SpacetimeDodge::DebugStyle {
constexpr ImU32 ink=IM_COL32(228,237,245,255), muted=IM_COL32(143,163,181,255);
constexpr ImU32 mint=IM_COL32(94,234,190,255), amber=IM_COL32(251,191,90,255);
constexpr ImU32 coral=IM_COL32(248,113,113,255);
inline void Text(ImDrawList* draw,ImVec2 p,float size,ImU32 color,const char* text) {
    draw->AddText(ImGui::GetFont(),size,p,color,text);
}
inline void Hud(ImDrawList* draw,ImVec2 pos,float width,const char* status,const char* detail,
                ImU32 accent,float speed,int shots,bool preview=false) {
    const float height=88.f;
    draw->AddRectFilled({pos.x,pos.y+3.f},{pos.x+width,pos.y+height+3.f},IM_COL32(0,0,0,38),12.f);
    draw->AddRectFilled(pos,{pos.x+width,pos.y+height},IM_COL32(13,22,32,232),10.f);
    draw->AddRect(pos,{pos.x+width,pos.y+height},IM_COL32(152,182,204,38),10.f);
    draw->AddCircleFilled({pos.x+16.f,pos.y+18.f},3.f,accent);
    Text(draw,{pos.x+26.f,pos.y+10.f},12.f,muted,preview?"SPACETIME  /  PREVIEW":"SPACETIME");
    Text(draw,{pos.x+width-46.f,pos.y+10.f},11.f,muted,"v4.19");
    Text(draw,{pos.x+13.f,pos.y+29.f},17.f,ink,status);
    Text(draw,{pos.x+13.f,pos.y+51.f},12.f,muted,detail);
    const float y=pos.y+74.f;
    Text(draw,{pos.x+13.f,y-3.f},10.f,muted,"NOW");
    draw->AddRectFilledMultiColor({pos.x+41.f,y},{pos.x+69.f,y+3.f},coral,amber,amber,coral);
    const ImU32 blue=IM_COL32(96,165,250,100);
    draw->AddRectFilledMultiColor({pos.x+69.f,y},{pos.x+97.f,y+3.f},amber,blue,blue,amber);
    Text(draw,{pos.x+104.f,y-3.f},10.f,muted,"LATER");
    char metrics[64]; std::snprintf(metrics,sizeof(metrics),"%.1f tiles/s   %d shots",speed,shots);
    const float textWidth=ImGui::GetFont()->CalcTextSizeA(10.f,1e6f,0.f,metrics).x;
    Text(draw,{pos.x+width-13.f-textWidth,y-3.f},10.f,muted,metrics);
}
// These are direction glyphs, not a scaled promise of future distance. The
// optional full trajectory is drawn separately and never smoothed through walls.
inline void Guidance(ImDrawList* draw,ImVec2 player,ImVec2 intent,ImVec2 selected,bool executable,bool preview) {
    const auto unit=[](ImVec2 v) {
        const float n=std::sqrt(v.x*v.x+v.y*v.y);
        return n>1e-5f?ImVec2(v.x/n,v.y/n):ImVec2(0.f,0.f);
    };
    const ImVec2 wanted=unit(intent),move=unit(selected);
    if(wanted.x || wanted.y) for(float t:{18.f,24.f,30.f,36.f,42.f})
        draw->AddCircleFilled({player.x+wanted.x*t,player.y+wanted.y*t},1.3f,IM_COL32(203,218,235,115),8);
    if(executable && (move.x || move.y)) {
        const ImU32 color=preview?IM_COL32(143,163,181,190):mint;
        const ImVec2 a{player.x+move.x*11.f,player.y+move.y*11.f};
        const ImVec2 b{player.x+move.x*32.f,player.y+move.y*32.f};
        draw->AddLine(a,b,IM_COL32(6,16,24,210),5.f);
        draw->AddLine(a,b,color,2.f);
        draw->AddLine(b,{b.x-move.x*6.f-move.y*4.f,b.y-move.y*6.f+move.x*4.f},color,2.f);
        draw->AddLine(b,{b.x-move.x*6.f+move.y*4.f,b.y-move.y*6.f-move.x*4.f},color,2.f);
    }
    draw->AddCircle(player,6.f,IM_COL32(5,15,25,190),24,2.f);
    draw->AddCircleFilled(player,2.f,ink,12);
}
}

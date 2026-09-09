#include "../src/features/movement/spacetime/SpacetimeDebugStyle.h"
#include "../src/features/movement/spacetime/SpacetimeDebugGeometry.h"
#include <vector>
#include <fstream>
using namespace SpacetimeDodge;
// Offline fixture: actual overlay primitives and production grid over a
// synthetic arena. This is visual QA, not a screenshot of live gameplay.
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    constexpr int width=1024,height=640;
    ImGui::CreateContext(); auto& io=ImGui::GetIO(); io.DisplaySize={width,height};
    io.IniFilename=nullptr;
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf",16.f);
    unsigned char* atlas=nullptr; int aw=0,ah=0;
    io.Fonts->GetTexDataAsRGBA32(&atlas,&aw,&ah); io.Fonts->SetTexID(1);
    ImGui::NewFrame(); auto* draw=ImGui::GetBackgroundDrawList();
    draw->AddRectFilled({0,0},{width,height},IM_COL32(25,31,33,255));
    for(int y=0;y<height;y+=32) for(int x=0;x<width;x+=32) {
        const int shade=(x/32+y/32)%3;
        draw->AddRectFilled({float(x+1),float(y+1)},{float(x+31),float(y+31)},IM_COL32(35+shade*3,42+shade*3,43+shade*3,255));
    }
    const auto project=[](UDodge::Vec2 p){return ImVec2(512.f+p.x*52.f,285.f+p.y*52.f);};
    DebugGeometry::Grid grid; grid.Reset({},9.f,.03125f);
    for(int i=0;i<7;i++) grid.Segment({-4.f+i*.6f,-3.2f},{-2.f+i*.6f,4.8f},.10f,0.f,800.f,false,false,true);
    for(int i=0;i<5;i++) grid.Segment({3.2f,-1.8f+i*.6f},{-3.8f,-.8f+i*.6f},.09f,0.f,800.f,false,false,true);
    std::vector<unsigned> texture(static_cast<size_t>(grid.width)*grid.height);
    grid.WritePixels(texture.data(),grid.width*sizeof(unsigned),800.f);
    const float lo=grid.originX*grid.step,hi=lo+grid.width*grid.step;
    draw->AddImage(2,project({lo,lo}),project({hi,hi}));
    for(int i=0;i<7;i++) {
        const auto p=project({-4.f+i*.6f,-3.2f});
        draw->AddQuadFilled({p.x,p.y-6},{p.x+5,p.y},{p.x,p.y+6},{p.x-5,p.y},IM_COL32(255,211,105,255));
    }
    for(int i=0;i<5;i++) {
        const auto p=project({3.2f,-1.8f+i*.6f});
        draw->AddQuadFilled({p.x,p.y-5},{p.x+5,p.y},{p.x,p.y+5},{p.x-5,p.y},IM_COL32(199,220,255,255));
    }
    const ImVec2 player=project({});
    draw->AddRectFilled({player.x-7,player.y-15},{player.x+7,player.y+6},IM_COL32(92,137,167,255),2.f);
    DebugStyle::Guidance(draw,player,{1,0},{.7f,-.7f},true,false);
    DebugStyle::Hud(draw,{16,536},338,"Taking a safe detour","Keyboard intent  /  live speed",DebugStyle::mint,8.6f,12);
    DebugStyle::Text(draw,{24,22},18,DebugStyle::ink,"Movement & threat overlay");
    DebugStyle::Text(draw,{24,49},12,DebugStyle::muted,"Synthetic arena / actual ImGui overlay rendering");
    DebugStyle::Hud(draw,{670,420},338,"Waiting for an opening","Resume in 80 ms",DebugStyle::amber,8.6f,12);
    DebugStyle::Hud(draw,{670,536},338,"No validated passage","search budget exhausted",DebugStyle::coral,8.6f,258);
    ImGui::Render();
    std::vector<unsigned char> pixels(width*height*3,0);
    const auto edge=[](ImVec2 a,ImVec2 b,ImVec2 p){return (p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x);};
    for(auto* list:ImGui::GetDrawData()->CmdLists) for(const auto& cmd:list->CmdBuffer) {
        if(cmd.UserCallback) continue;
        const bool gridTexture=cmd.GetTexID()==2;
        const unsigned char* tex=gridTexture?reinterpret_cast<const unsigned char*>(texture.data()):atlas;
        const int tw=gridTexture?grid.width:aw,th=gridTexture?grid.height:ah;
        for(unsigned i=0;i<cmd.ElemCount;i+=3) {
            const auto& a=list->VtxBuffer[cmd.VtxOffset+list->IdxBuffer[cmd.IdxOffset+i]];
            const auto& b=list->VtxBuffer[cmd.VtxOffset+list->IdxBuffer[cmd.IdxOffset+i+1]];
            const auto& c=list->VtxBuffer[cmd.VtxOffset+list->IdxBuffer[cmd.IdxOffset+i+2]];
            const float area=edge(a.pos,b.pos,c.pos); if(std::fabs(area)<1e-7f) continue;
            const int x0=std::max(0,int(std::max(cmd.ClipRect.x,std::floor(std::min({a.pos.x,b.pos.x,c.pos.x})))));
            const int y0=std::max(0,int(std::max(cmd.ClipRect.y,std::floor(std::min({a.pos.y,b.pos.y,c.pos.y})))));
            const int x1=std::min(width-1,int(std::min(cmd.ClipRect.z,std::ceil(std::max({a.pos.x,b.pos.x,c.pos.x})))));
            const int y1=std::min(height-1,int(std::min(cmd.ClipRect.w,std::ceil(std::max({a.pos.y,b.pos.y,c.pos.y})))));
            for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) {
                const ImVec2 p{x+.5f,y+.5f};
                const float u=edge(b.pos,c.pos,p)/area,v=edge(c.pos,a.pos,p)/area,w=1.f-u-v;
                if(u<0.f || v<0.f || w<0.f) continue;
                const int tx=std::clamp(int((a.uv.x*u+b.uv.x*v+c.uv.x*w)*tw),0,tw-1);
                const int ty=std::clamp(int((a.uv.y*u+b.uv.y*v+c.uv.y*w)*th),0,th-1);
                const auto* sample=tex+4*(ty*tw+tx);
                const float alpha=sample[3]/255.f*((a.col>>24)*u+(b.col>>24)*v+(c.col>>24)*w)/255.f;
                auto* dest=pixels.data()+3*(y*width+x);
                for(int channel=0;channel<3;channel++) {
                    const float color=((a.col>>(channel*8)&255)*u+(b.col>>(channel*8)&255)*v+(c.col>>(channel*8)&255)*w)*sample[channel]/255.f;
                    dest[channel]=static_cast<unsigned char>(color*alpha+dest[channel]*(1.f-alpha));
                }
            }
        }
    }
    std::ofstream file(argv[1],std::ios::binary); file<<"P6\n"<<width<<" "<<height<<"\n255\n";
    file.write(reinterpret_cast<const char*>(pixels.data()),pixels.size());
    ImGui::DestroyContext(); return file?0:1;
}

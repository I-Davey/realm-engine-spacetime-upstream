#pragma once
#include <cstdint>
struct ImDrawList;
namespace TargetAssist {
void SetEnabled(bool enabled); bool IsEnabled();
void SetDebugOverlay(bool enabled); bool GetDebugOverlay();
void SetRangeSafetyFactor(float factor); float GetRangeSafetyFactor();
void SetScriptTarget(int32_t objectId); int32_t GetTargetId(); void ClearTarget();
void SetSelectKey(int key); int GetSelectKey();
void SetClearKey(int key);
bool OwnsAim();
// Firing-zone snapshots expire across player/world changes and after 150 ms.
struct FiringZone {
    bool active=false; int32_t targetId=0; void* player=nullptr; void* world=nullptr; uint64_t capturedMs=0;
    float targetX=0.f,targetY=0.f,velocityX=0.f,velocityY=0.f;
    float range=0.f,projectileSpeed=0.f,lifetimeSeconds=0.f; bool parametric=false;
};
FiringZone GetFiringZone(void* player);
bool CanHitFrom(const FiringZone& zone,float x,float y,float inset=0.f);
void Tick(bool menuVisible); void RenderSettings();
void RenderOverlay(ImDrawList*,float,float,float,float,float,float);
void ProcessMiddleClick(float,float,float,float,float,float,float,float);
}

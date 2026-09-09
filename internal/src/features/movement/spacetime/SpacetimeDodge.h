#pragma once
namespace SpacetimeDodge {
void SetEnabled(bool enabled);
bool IsEnabled();
void OnEnter();
void Tick(void* player,float x,float y,float dt);
void BeginGameUpdate();
void RenderSettings();
void RenderDebugOverlay(float camX,float camY,float angle,float zoom,float cx,float cy);
void ReleaseDebugResources(); // render thread, before DirectX teardown
void SetDebugOverlay(bool enabled);
bool GetDebugOverlay();
void SetShadowMode(bool enabled);
bool GetShadowMode();
void SetLookRange(float tiles); float GetLookRange();
void SetContactScale(float scale); float GetContactScale();
void SetHorizonMs(float ms); float GetHorizonMs();
void SetMaxDistance(float tiles); float GetMaxDistance();
void SetEnemyScale(float scale); float GetEnemyScale();
void SetAvoidBlocks(bool enabled); bool GetAvoidBlocks();
void SetSearchBudgetMs(float ms); float GetSearchBudgetMs();
}

#pragma once
namespace DodgeRuntime {
bool EnsureResolved();
float GetDeltaTime();
float GetMoveSpeedMul(void* player);
float GetTilesPerSec(void* player);
bool CallMoveTo(void* player,float x,float y);
void Reset();

// Spacetime's opt-in native input filter and single-update movement budget.
using MovementFilter=bool(*)(void*,float,float,float&,float&);
using MovementFeedback=void(*)(bool,float,float);
bool EnsureMovementFilter(void* player,MovementFilter filter,MovementFeedback feedback);
bool MovementFilterInstalled();
void UninstallMovementFilter();
void BeginGameUpdate();
void EndGameUpdate();
float GetFrameMoveMs();
double GetMovementTimeMs();
void AccountNativeMovement();
float GetVerifiedTilesPerSec(void* player);
void ObserveSpeed(float x,float y,float dtMs);
float GetObservedTilesPerSec();
enum class MoveResult { Applied, Deferred, Rejected };
MoveResult CallMoveToFrame(void* player,float x,float y,float durationMs);
}

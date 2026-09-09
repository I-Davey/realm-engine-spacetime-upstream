#pragma once
#include "SpacetimeCore.h"

namespace SpacetimeDodge {
// Goal providers describe acceptable positions; they never execute movement.
// The context is immutable and lives for this synchronous evaluation only.
struct MovementGoal {
    bool active = false;
    uint64_t identity = 0;
    Vec2 center{};
    float range = 0.f; // navigation hint; contains remains authoritative
    const void* context = nullptr;
    bool (*contains)(const void*, Vec2) = nullptr;
    // Optional arrival inset. It is used only after approaching from outside;
    // a player already in the firing zone is never pulled toward an inner ring.
    bool (*arrivalContains)(const void*, Vec2) = nullptr;
    bool waypoint = false;
};
struct WaypointGoal { bool active=false; Vec2 position{}; float radius=.2f; };
// Explicit travel/loot goals take precedence over a combat firing zone. The
// waypoint context must live for the duration of EvaluateMovement.
MovementGoal SelectMovementGoal(const WaypointGoal& waypoint,const MovementGoal& firing);
struct NavigationState {
    // Reports the temporal planner's ownership to the final native guard.
    // This is not another movement controller or a timed override latch.
    bool overrideActive = false;
    std::array<Vec2, 128> route{};
    int count = 0, next = 0;
    bool complete = false;
    bool entering = false;
    uint64_t identity = 0;
    Vec2 center{};
    float range = 0.f;
    bool detouring = false;
    bool directional = false;
    Vec2 direction{};
    void Reset() { *this = NavigationState{}; }
};
struct MovementDecision {
    Output dodge{};
    bool overrideActive = false;
    bool approaching = false;
    bool detouring = false;
    const char* action = "holding";
};
// Keyboard input is already represented by in.nominal. Automatic intentions
// start from rest, have a finite stop, and are executed by the same host.
void EvaluateMovement(const Input& in, bool keyboard, const MovementGoal& goal,
    State& dodge, NavigationState& navigation, MovementDecision& out);
// The only conversion from selected velocity to an automatic native command.
// Applied keyboard input is compensated; unapplied automatic intent is not.
Vec2 MovementDisplacement(Vec2 selected, Vec2 applied, float commandMs);
void PrepareAutomaticMovement(Input& in,float commandIntervalMs,float commandDelayMs,float availableMs);
// Continuous future controls, executed as one bounded slice this update.
bool PrepareFrameMovement(Input& in,float availableMs);
bool PrepareNativeMovement(Input& in,Vec2 requested);
bool NativeMovementTarget(const Input& in,const Output& out,Vec2& target);
// A rejected proposal must not silently execute unsafe keys. Recheck the
// requested slice and choose a safe or least-damage emergency replacement.
bool ResolveNativeMovementTarget(const Input& in,Output& out,bool proposalUsable,
    bool retainControl,Vec2 requested,Vec2& target);
}

# Spacetime dodge

Spacetime is an experimental movement planner. It preserves safe manual movement
or a stationary position, and searches for a short timed route when intervention
is needed. Routes can wait, move at the player's live speed, turn, and stop.
When its bounded search cannot find a collision-free route, recovery compares
predicted damage rather than abandoning movement.

## Upstream integration

This implementation is based directly on upstream `main` at `565c3b0`. It does
not require another fork's commits or additional dodge engines. Existing native
mode IDs `0..7` are unchanged; Spacetime uses `8`. Configuration from a fork that
used a different numeric ID must select Spacetime again through the UI.

The shared UDodge sensors provide projectile trajectories, enemies, terrain,
and timed AoEs. Spacetime opts into its collision threshold and fractional
trajectory phase through capture settings. Existing algorithms retain their
default contact model and actuator. Bomb capture improvements apply to the
shared observation stream.

Target Assist supplies a coherent firing-zone snapshot and aim point. It has
no movement actuator, legacy route planner, reloadable DLL policy, or inner
combat ring. Spacetime consumes the zone, giving explicit navigation waypoints
priority. The farmer scripts select Spacetime and clear their goals on stop.
The existing firing implementation remains responsible for shooting.

The native movement hook filters the game's requested local-player movement
before it executes. Automated movement uses one bounded game-update budget.
Slow, Speedy, paralysis and other current speed effects are read through the
game's speed calculation; unavailable speed data is distinguished from zero.

## Controls

Select **Spacetime** under Auto Dodge. Its controls include the threat overlay,
preview-only mode, projectile look range and contact multiplier, prediction
lookahead, movement distance, enemy clearance, harmless-block steering and the
route calculation budget. Numeric/boolean values are sent before enabling the
mode, and edits update the overlay and planner.

Enable Target Assist and middle-click an enemy to select it. Escape clears
selection. Shift bypasses movement intervention. The target range ring shows
weapon reach; the planner also checks interception and line of sight.

## Build and test from a fresh checkout

Follow the repository's `SETUP.md` to provide the normal generated IL2CPP
headers in `internal/src/game/generated`. Those game-derived headers are not
committed. No generated headers or libraries from another fork are required.
The native project includes the new sources and their shared dependencies.

In a Visual Studio developer shell, build to a separate output directory:

```powershell
msbuild internal/il2cpp-dll-injection.sln /p:Configuration=Release /p:Platform=x64 /p:OutDir=C:\build\spacetime\ /m
```

Do not build directly into a DLL watched by a running deployment process. This
build produces a test artifact and does not install or restart the game.

The planner regression harness requires Python and a C++ compiler, but no game,
injector, or generated IL2CPP headers. It compiles the production planner:

```powershell
python internal/tests/run_spacetime_tests.py --all
```

It covers fast projectiles, moving-player contact, late departure, turning
routes, dense fields, unavoidable contact recovery, bombs, speed changes,
movement budgets and shared UDodge regressions. CI runs this harness on Windows.

Client checks:

```powershell
cd client
npm ci --ignore-scripts
npm run build:sdk
npm run typecheck
npm run gen:packets
npm test
```

`gen:packets` normalizes generated artifact line endings on Windows checkouts;
it should not change their committed contents.

## Limits

The solver is bounded by its frame budget and captured data. It does not prove
that every dense-field situation is survivable. Sampled curved trajectories
have a shorter reliable horizon than the largest configurable lookahead.
The full DLL build verifies integration; live collision timing and server
behavior still require in-game testing.

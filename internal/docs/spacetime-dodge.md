# Spacetime dodge

Spacetime is an experimental movement planner. It preserves safe standing or
manual movement and searches for a short timed route when intervention is needed.
Routes can wait, move at the player's live speed, turn, and stop. If the bounded
search finds no collision-free route, recovery compares predicted damage.

## Use

Select **Spacetime** under Auto Dodge. Hold the configured override key (Shift by default) to bypass movement intervention.
Use **Preview only** to inspect decisions without applying them.

Moving and Stationary profiles are independently adjustable:

| Setting | Moving default | Stationary default |
| --- | --- | --- |
| Projectile look range | 3.5 tiles | 10 tiles |
| Contact multiplier | 1.0 | 0.95 |
| Prediction | 1475 ms | 4000 ms |
| Dodge distance budget | 4.5 tiles | 6.5 tiles |
| Enemy clearance multiplier | 0.2 | 0.1 |
| Search budget | 8 ms | 8 ms |

Harmless-block steering defaults off; the overlay and 60 FPS cap default on.
Existing saved values are preserved; Reset to defaults applies this preset.
Profiles follow keyboard, waypoint or firing-zone movement intent, so briefly
stopping for a dodge does not flip profiles. Profile changes invalidate the
retained temporal plan. Both profiles affect the planner and debug drawing.

Auto Dodge exposes the shared hold-to-override key and an optional overlay toggle
key (unbound by default). Target Assist exposes select/toggle and clear keys,
defaulting to middle mouse and Escape. These work while the game has focus.
The feature toggle hotkey remains at the top of each dashboard card.

The controls adjust projectile look range, contact size, prediction lookahead,
movement distance, enemy clearance, harmless-block steering, and calculation
budget. Settings are sent before activation and changes update the planner and
its overlay. Known timed bombs can extend the selected lookahead and distance
budget to allow an escape; the in-game settings show the effective values.

The projectile model uses a point player, square contact thresholds for ordinary
shots, and capsules for beams. The world grid colors contact times from red
(now) through amber to blue (end of lookahead). Drawing resolution does not
change the continuous collision checks.

## Target Assist and scripts

Enable Target Assist and middle-click an enemy to select it. Escape clears the
selection. Fire once to calibrate weapon reach. The range ring marks reach;
firing-zone evaluation also checks interception and line of sight.

Target Assist supplies a firing zone and aim point. Spacetime selects and
executes movement, prioritizing explicit navigation waypoints. The existing
firing implementation remains responsible for shooting.

Scripts can select the mode with `RealmEngine.dodge.setMode('spacetime')`, supply
waypoints with `navigateToPosition`, and select a firing-zone target with
`lockEnemy`. Clear waypoints and enemy locks when stopping a script.

Realm Farmer and Dead Church Farmer default to Unified. To opt into Spacetime,
change `this.dodgeMode = 'unified'` to `this.dodgeMode = 'spacetime'` in the shared
Farmer constructor before starting either script. The chosen mode is reapplied
on map changes. Script aim locks require the Auto Aim plugin to be enabled.

## Runtime and diagnostics

Shared UDodge sensors supply projectile trajectories, enemies, terrain and timed
areas of effect. The native movement hook evaluates requested walking before it
executes. Walking, automatic travel and dodge share one retained temporal plan.
Static navigation supplies waypoints; moving bullets are evaluated in time.
Manual route scoring rewards continued progress along the held direction beyond
its temporary waypoint, and searches forward directions first. A blocked long
continuation can still advance to a verified safe stop. A safe forward continuation
can end a retained hold; cancelling a goal cancels its cached travel. Ownership
has no fixed timeout, and changing direction allows a safe retreat.
Automated movement uses one game-update budget. Current speed effects,
including Slow, Speedy and paralysis, are read through the game's speed calculation.

Recovery with zero predicted hits is labelled Emergency steering: its finite
checked interval is not a certified complete route or evidence of actual damage.

In-game debug settings control the grid, collision outlines, projectile paths
and selected route. **Record diagnostic replays** is off by default and independent
of overlay visibility. When enabled, it captures failures and bomb situations to
up to eight rotating `%LOCALAPPDATA%/RealmEngine-Spacetime-replayNN.bin` files.
Recording is enabled for the current DLL session. Captures can be inspected with:

```powershell
python internal/tests/run_spacetime_tests.py --replay <capture-file>
```

## Build and test

Follow [SETUP.md](../../SETUP.md) to generate the IL2CPP headers in
`internal/src/game/generated`. In a Visual Studio developer shell:

```powershell
msbuild internal/il2cpp-dll-injection.sln /t:Rebuild /p:Configuration=Release /p:Platform=x64 /p:OutDir=C:\build\spacetime\ /m
```

Use a separate output directory; do not link into a DLL watched by deployment.

The native regression runner compiles the production planner with Python and
a C++ compiler. It runs without the game or generated IL2CPP headers:

```powershell
python internal/tests/run_spacetime_tests.py --all
```

Client validation:

```powershell
cd client
npm ci --ignore-scripts
npm run build:sdk
npm run typecheck
npm run gen:packets
npm test
```

## Limits

The solver is bounded by its frame budget and captured data. Sampled curved
trajectories have a shorter reliable horizon than the maximum configurable
lookahead. Recovery ranks estimated damage and cannot guarantee survival.
Compilation and synthetic regressions do not validate live collision timing
or server behavior; these require in-game testing.

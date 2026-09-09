# Spacetime dodge

Spacetime is an experimental movement planner. It preserves safe standing or
manual movement and searches for a short timed route when intervention is needed.
Routes can wait, move at the player's live speed, turn, and stop. If the bounded
search finds no collision-free route, recovery compares predicted damage.

## Use

Select **Spacetime** under Auto Dodge. Hold Shift to bypass movement intervention.
Use **Preview only** to inspect decisions without applying them.

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
executes. Automated movement uses one game-update budget. Current speed effects,
including Slow, Speedy and paralysis, are read through the game's speed calculation.

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

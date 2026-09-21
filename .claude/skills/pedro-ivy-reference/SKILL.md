---
name: pedro-ivy-reference
description: "Authoritative Pedro Pathing 3 and Ivy command-framework API reference (imports, Command Builder, compositions, PedroCommands follow/hold, PoseFactory, Paths, interpolation, Follower in OpModes) plus Foresight and Ivy research notes. Load before writing or changing any autonomous path, Ivy command, or Follower code."
---

## 8. Current API Syntax (authoritative)

Verified against pedropathing.com on **2026-09-20**. Where this section disagrees with §5 or §6, this section wins. §5 documents the old repo as history; §6 is conceptual research notes.

### 8.1 Imports

```java
// Ivy
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.*;                  // Class API only
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;   // instant, infinite, waitMs, waitUntil, onInterrupt, conditional, branch, match, lazy
import static com.pedropathing.ivy.groups.Groups.*;       // sequential, parallel, race, deadline, repeat, loop
import static com.pedropathing.ivy.pedro.PedroCommands.*; // follow, hold

// Pedro 3
import static com.pedropathing.api.Paths.*;               // line(), curve(), path()
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
```

### 8.2 Install

- **Ivy**: `implementation 'com.pedropathing:ivy:1.0.0'` in `build.dependencies.gradle`.
- **Pedro**: Quickstart repo (`https://github.com/Pedro-Pathing/Quickstart.git`), or manual — add `maven { url 'https://repo.dairy.foundation/releases/' }` to `repositories {}`, then `implementation 'com.pedropathing:revhub:x.y.z'` and `implementation 'com.pedropathing:tuning:1.0.0'`, then copy the quickstart's `pedro` package in.

### 8.3 Command Builder API (the default)

```java
Command myFirstCommand = Command.build()
    .setStart(() -> {})                 // Runnable
    .setExecute(() -> {})               // Runnable
    .setDone(() -> true)                // BooleanSupplier
    .setEnd(endCondition -> {})         // Consumer<EndCondition>
    .requiring(/* requirements */)
    .setInterruptedBehavior(/* ... */)
    .setBlockedBehavior(/* ... */)
    .setConflictBehavior(/* ... */)
    .setPriority(/* int */);
```

All methods optional. Method references work: `.setStart(arm::setRaisedArmTarget)`.

Parameterized/reusable commands are a **static method returning a built command** — not the Class API:

```java
public static Command raiseArm(double target) {
    return Command.build()
        .setStart(() -> pid.setTarget(target))
        .setExecute(() -> armMotor.setPower(pid.calculate(armMotor.getCurrentPosition())))
        .setDone(() -> Math.abs(target - armMotor.getCurrentPosition()) < 10)
        .setEnd(endCondition -> armMotor.setPower(0))
        .requiring(armMotor);
}
```

Each call yields a fresh, stateless, safely composable command.

### 8.4 Compositions

| Function form | Method form |
| --- | --- |
| `sequential(c1, c2, c3)` | `c1.then(c2).then(c3)` / `c1.then(c2, c3)` |
| `parallel(c1, c2, c3)` | `c1.with(c2).with(c3)` / `c1.with(c2, c3)` |
| `race(c1, c2, c3)` | `c1.raceWith(c2)` / `c1.raceWith(c2, c3)` |
| `deadline(deadlineCmd, c1, c2)` | — |
| `repeat(command, 3)` (also takes an `IntSupplier`) | — |
| `loop(command)` — never finishes; cap it with a race | — |

Compositions are themselves Commands and nest arbitrarily.

### 8.5 Utility commands (`Commands.*`)

```java
instant(() -> shooter.activate());                     // runs once, finishes immediately
infinite(() -> currentColor = sensor.readColor());     // runs until cancelled/interrupted
waitMs(500);
waitUntil(() -> armMotor.getCurrentPosition() > 100);  // BooleanSupplier
onInterrupt(() -> armMotor.setPower(0));               // runs forever, callback on interrupt (cleanup)
```

### 8.6 Control-flow commands

Conditions are evaluated **once at start**, not continuously and not at schedule time.

```java
conditional(() -> armIsRaised, lowerArmCommand, raiseArmCommand);   // if/else

LinkedHashMap<BooleanSupplier, Command> cases = new LinkedHashMap<>();
cases.put(() -> position == Position.HIGH, moveToHigh);
branch(cases);          // if/else-if; insertion order = priority; no match → finishes immediately

EnumMap<ArmState, Command> cases = new EnumMap<>(ArmState.class);
cases.put(ArmState.RAISED, raiseCommand);
match(() -> currentArmState, cases);   // switch on enum; Supplier<T> must match the EnumMap key type

lazy(() -> { Pose t = getTargetPose(); return Command.build()/*...*/; });  // defers construction to start
```

### 8.7 Decorators (return a new command; original unchanged)

```java
infinite(() -> intake.setPower(1.0)).until(intake::isFull);  // = race with waitUntil
raiseArmCommand.unless(() -> armIsAlreadyRaised);            // skipped entirely if true at start
someCommand.proxy();   // runs through the Scheduler independently; can outlive its parent composition
                       // and be cancelled/inspected via Scheduler.cancel() / Scheduler.isScheduled()
```

### 8.8 Scheduler (static)

```java
Scheduler.reset();        // MUST run before each OpMode — the scheduler is static, state carries over
Scheduler.schedule(cmd);  // or schedule(cmd, cmd2);  or cmd.schedule();
Scheduler.execute();      // once per loop — nothing runs without this
Scheduler.isRunning(cmd); // actively executing
Scheduler.isScheduled(cmd); // running, queued, or suspended; also cmd.isScheduled()
Scheduler.cancel(cmd);    // also cmd.cancel()
```

Everything scheduled runs **in parallel by default** — sequencing requires an explicit `sequential`.

### 8.9 Class API (only for genuinely stateful commands)

Implement `Command` and supply **all** of: `start()`, `execute()`, `done()`, `end(EndCondition)`, `requirements()` (`Set<Object>`), `priority()`, `interruptedBehavior()`, `blockedBehavior()`, `conflictBehavior()`. Builder defaults to replicate:

| Method | Default |
| --- | --- |
| `requirements()` | empty |
| `priority()` | `0` |
| `interruptedBehavior()` | `InterruptedBehavior.END` |
| `blockedBehavior()` | `BlockedBehavior.CANCEL` |
| `conflictBehavior()` | `ConflictBehavior.OVERRIDE` |
| `start()` / `execute()` / `end()` | do nothing |
| `done()` | `false` (runs forever) |

Class instances hold state across runs — construct a new one per schedule.

### 8.10 PedroCommands (Ivy ↔ Pedro 3)

```java
Command goToBasket = follow(follower, basketPath);   // ends when follower.following() == false
Command holdHere   = hold(follower);                 // hold current position
Command holdAtGoal = hold(follower, p.of(120, 80, 0));
```

`follow()` uses whatever interpolation the Path was built with. Composes like anything else:

```java
Command auto = sequential(
    follow(follower, shootPreloads),
    Shooter.shoot(),
    instant(() -> claw.open()),
    parallel(follow(follower, pickupCloseSpikemark), instant(() -> intake.activate()))
);
```

### 8.11 PoseFactory

```java
PoseFactory factory = PoseFactory.degrees();   // or PoseFactory.radians()
factory = factory.mirrorX(70.75);              // field is 141.5 x 141.5 → mirrors across center
factory = factory.mirrorY(70.75);
Pose pose = factory.of(x, y, heading);         // heading unit matches how the factory was created
```

Mirroring applies to **every** pose the factory creates. On-the-fly `new Pose(x, y, Math.toRadians(90))` exists but is radians-only and not recommended for autos.

### 8.12 Paths

```java
Paths.line(Pose start, Pose end)
Paths.curve(Pose... poses)   // first = start, last = end, middle poses = Bézier control points
Paths.path(Path... paths)    // concatenation
```

Recommended usage is **methods that return paths**, not fields — this is also the form the visualizer emits:

```java
private Path startToScore() { return line(startPose, scorePose).linear(startPose, scorePose); }
private Path park()         { return line(scorePose, parkPose).linear(scorePose, parkPose); }
private Path compound()     { return path(startToScore(), park()); }
```

### 8.13 Interpolation

Default is **tangent** (robot faces travel direction). Each type takes a Pose or a raw heading in radians.

```java
path.tangent();
path.linear(start, end);
path.constant(start);
path.facingPoint(end);                 // Pose or Vector2D
path.heading(Interpolator.piecewise()
        .until(0.5, Interpolator.tangent)
        .until(1, Interpolator.linear(start, end)));   // t normalized 0 → 1
```

Set per sub-path, or on the concatenated path — the latter **overrides** every sub-path's interpolation rather than merging.

### 8.14 Follower in OpModes

Auto skeleton:

```java
@Override public void init() {
    Scheduler.reset();
    follower = Constants.create(hardwareMap);
    follower.setPose(startPose);
    follower.update();
}
@Override public void start() { schedule(autoRoutine()); }
@Override public void loop()  { follower.update(); Scheduler.execute(); }
```

- In this repo the skeleton's `init()`/`start()`/`loop()` map onto `CommandOpMode`'s `initialize()`/`onStart()`/`execute()` (those Android `OpMode` methods are final here); `Scheduler.reset()`/`Scheduler.execute()` are already called by `CommandOpMode`.
- Follower creation is **`Constants.create(hardwareMap)`**.
- TeleOp robot-centric: `follower.manual(forward, lateral, turn)` + `follower.update()` every loop.
- Field-centric: `DrivePowers powers = ManualDrive.fieldCentric(fwd, lat, turn, follower.pose().heading()); follower.manual(powers);`
- Anti-push hold: `ManualDrive.driveOrHold(follower, fwd, lat, turn)` or `ManualDrive.driveOrHold(follower, powers)`.
- Localization: `follower.pose()` → `.x()`, `.y()`, `.heading()`. **Always inches and radians** — convert with `Math.toDegrees()`. `follower.setPose(pose)` overrides (only with a relocalization source or a known corner). `follower.mode()` reports follow state.
- Auto → TeleOp pose handoff is not built in: use a static storage class (e.g. `OpModeStorage.autonomousEndPose`), written in the auto's `stop()`, read in TeleOp's `start()`.

### 8.15 Sources

- https://pedropathing.com/docs/ivy , /command-builder , /command-compositions , /utilities-and-decorators , /class-api , /creating-opmodes , /pedro-commands
- https://pedropathing.com/docs/pathing/installation , /guide/path-creation , /guide/auto-usage , /guide/teleop-usage , /reference/api , /reference/posefactory , /reference/interpolation

## 6. Pedro Pathing / Pedro 3 / Ivy

### What Pedro Pathing is
- FTC autonomous path-following library from Team 10158 ("Scott's Bots"), an alternative to RoadRunner.
- **GVF (Guiding Vector Field) feedforward + feedback**, **time-independent formulation** (accounts for loop-time lag rather than assuming fixed timesteps), plus a brake profile from identified system dynamics. Paths built from **Bézier curves**.
- Benefits: faster execution, disturbance correction, on-the-fly path creation, web visualizer (visualizer.pedropathing.com).
- BSD 3-Clause. github.com/Pedro-Pathing (PedroPathing, Docs, Quickstart, AutoTune).
- pedropathing.com hosts three sub-products: **Pedro Pathing**, **Ivy**, **Visualizer**.

### Pedro 3 (rewrite)
- **Foresight** — new core following algorithm replacing Predictive Braking. Benchmark claim: +36% top speed on team 12808's robot in a "Line Test."
- **Pose Factory** — `PoseFactory.degrees()` / `.radians()`, bulk ops like `mirrorX(mirrorLine)` / `mirrorY(...)` applied to every pose the factory creates (e.g. mirroring across a 141.5×141.5 field center).
- **Paths API redesign** — interpolation settable per-sub-path or whole-path.
- **AutoTune** — robot-hosted webpage for Drivetrain / Localizer / Algorithm tuning.
- **Built-in Logging** — Loggers attached to the Follower auto-log `FollowerLog` on every `follower.update()`.

### Foresight (`ForesightConfig`)

**Deceleration**
- `naturalForwardDeceleration` / `naturalStrafeDeceleration` (units/s², required `ConfigVar<Double>`) — max braking power forward/back vs. strafing, measured from your own drivetrain. Used for coast distance and excess-velocity estimates.
- `maxDecelerationConstraint` (units/s², default `Constraint.NONE`) — caps deceleration to avoid tipping. If too restrictive, the robot may not stop exactly on target.
- Loop sequence: (1) coast, (2) distance calculation, (3) constraint application, (4) velocity adjustment.

**Foresight Options**
- `headingDriveRatio` (default `0.5`) — heading correction vs. forward movement priority.
- `brakeAtEnd` (default `true`) — brake before the very end vs. keep coasting; affects only the final point.
- `translationalDeviationTolerance` (default `2.5` in) / `headingDeviationTolerance` (default `Math.toRadians(11.25)`) — on-track thresholds; exceeding prioritizes correction over continuing.
- `cosineScale` (default `false`) — reduces forward speed as error grows; zero forward speed past 2× tolerance. No effect below tolerance.

**Path Constraints**
- `brakeAggression` (default `1.0`) — >1 allows overshoot (useful when a wall helps stop, faster paths); <1 biases undershoot.
- `maxBrakingPower` (default `0.2`) — power opposing motion; too high risks burning the Control Hub, too low may not stop in time.
- `maxAccelerationConstraint` (units/s²) — caps target-velocity ramp-up while coasting (pre-braking only).
- `maxVelocityConstraint` (units/s) — caps target coasting velocity. Related: `maxPathSpeed` as a fraction of current max (e.g. 0.75). If both set, the lower wins.
- `maxDecelerationConstraint` (units/s²) — caps target-velocity decrease while coasting.
- `maxDecelerationScale` (default `Constraint.NONE`) — deceleration limit as a *fraction* of natural deceleration; more restrictive of the two applies.
- `coastDownToVelocity` (default `0.0`) — speed Max Deceleration slows to instead of a full stop (only when `maxDecelerationConstraint` is finite).

### Ivy — command-based framework
- Minimal command-based control-flow framework for FTC, modeled on WPILib's command-based docs. Declarative: describe *what*, not *how*.
- **Install**: see §8 — the docs show a single artifact, `implementation 'com.pedropathing:ivy:1.0.0'`. The `ivy:core:1.1.0` + `ivy:pedro:1.1.0` pair recorded earlier is stale.
- **Command lifecycle**: `start()` (once on schedule), `execute()` (every loop), `done()` (true to stop), `end(endCondition)` (finish/interrupt/suspend). Scheduler uses cooperative multitasking on one thread.
- **Requirements & priorities**: commands declare requirements (subsystems/objects); conflicts resolved by int priority (default 0) plus:
  - `InterruptedBehavior` — `END` (default) or `SUSPEND`.
  - `BlockedBehavior` — `CANCEL` (default) or `QUEUE`.
  - `ConflictBehavior` — `OVERRIDE` (default), `CANCEL`, or `QUEUE`.
- **Two APIs**: Command Builder API (`Command.build()` + `.setStart()/.setExecute()/.setDone()/.setEnd()/.requiring()/.setPriority()`, all optional — recommended default) and the Class API (when internal state is needed). Reusable parameterized commands via static methods returning a built `Command` keeps them stateless.
- **Compositions** (nestable): `sequential(...)`/`.then(...)`, `parallel(...)`/`.with(...)`, `race(...)`/`.raceWith(...)`, `deadline(deadlineCommand, ...)`, `repeat(command, n)` (n can be an `IntSupplier`), `loop(command)` (never finishes — wrap in a race).
- **Pedro Commands**: `import static com.pedropathing.ivy.pedro.PedroCommands.*;` → `follow(follower, path)` (finishes when `follower.following()` goes false, uses the Path's own interpolation), `hold(follower)` / `hold(follower, pose)`. These are normal Ivy `Command`s and compose freely.
- Other observed factories in practice: `instant(Runnable)`, `waitMs(double)`, `waitUntil(BooleanSupplier)`, `infinite(Runnable)`, `conditional(BooleanSupplier, CommandBuilder, CommandBuilder)`, plus `Scheduler.reset()/execute()`.

### White paper
Zenodo DOI `10.5281/zenodo.20725318` — linked from the homepage; **not successfully fetched** (timeouts). Retry for the GVF + Foresight math.

### Sources
- https://pedropathing.com/ , /docs/pathing , /docs/pathing/pedro3
- /docs/pathing/reference/deceleration, /foresightoptions, /pathconstraints, /posefactory
- /docs/ivy , /what-are-commands , /requirements-and-priorities , /command-builder , /command-compositions , /pedro-commands , /creating-opmodes

### Gaps
- White paper full text; Ivy Requirements & Priorities page, Example Repos; Pedro pages on setting-up-auto, path-following, follow-state, debugging, coordinates, end constraints, Bezier curves, types of velocities, logging, controllers, tuning/constants.
- (Ivy Utilities & Decorators, Class API, Scheduling/OpMode use, Pedro Commands, Path Creation, Interpolation, PoseFactory, TeleOp/Auto usage and Installation have now been read — see §8.)

---


# CLAUDE.md — Electrolights Biobuzz

Project context for FTC BIOBUZZ (2026–2027 season) robot code. Consolidated from stored notes.

---

## 1. Who / Context

- **Mithun**, 19.
- FTC (FIRST Tech Challenge) robotics across software (Java, autonomous routines), CAD (Onshape), and 3D-printed fabrication.
- CAD: importing parts from an FTC parts library into Onshape. Chose PETG over PLA for a large turret/launcher gear, with print settings tuned for durability.
- Prior-season auto tuning: PedroPathing path x-coordinates across gate intake and partner paths (`Red24BallSpamLinear.java`).

### Side project — FTC deploy tool
- Fork/remake of **Pusher** (github.com/PzmuV1517/Pusher, Go-based FTC deployment CLI).
- Goal: Pusher's Wi-Fi/ADB connectivity automation + hot-reload speed comparable to **Sloth**.
- Mac first; Linux and Windows later.

### Unrelated note
- High-school **iGEM** (synthetic biology) team formed; debating a project among oncology, bioremediation, infectious diseases, biomanufacturing, therapeutics.

---

## 2. BIOBUZZ — 2026–2027 FTC Game

### Season overview
- **BIOBUZZ™ presented by RTX**, part of the FIRST® CANOPY™ theme (biodiversity/nature) shared across FLL, FTC, FRC.
- Kickoff/reveal: Sept 12, 2026, 12:00pm ET. Complete Competition Manual (V1) same day; V0 preview earlier covered non-game-specific info.
- "Pollen" revealed early (after 2026 Worlds) so teams could practice via "Skill Builders" (7 mini-games) and ecosystem-partner (AndyMark/goBILDA/REV/Studica) "StarterBot Base" kits.
- Season Q&A opens Sept 28, 2026, 12:00pm ET (Lead Coach account on FIRST Dashboard).
- 2026-27 manual philosophy shift: rules emphasize "spirit of the rule" / intended gameplay over exhaustive scenarios; volunteers get good-faith judgment latitude. New **Competition Integrity Contract (CIC)** in §1.5.
- Manual: 16 sections, 173 pages.

### Gameplay
- 2 alliances of 2 teams. Two scoring elements:
  - **Pollen** — yellow, ~2.8 in / 7.1 cm balls, either alliance may use. 40 per match.
  - **Nectar** — red or blue, ~3.6 in / 9.1 cm balls, color-locked per **G408** (red robots only red Nectar). 8 red + 8 blue per match.
- **HIVE** (one per alliance): pivoting, bi-stable structure with 2 **CELLS**, one on each end; only one faces up at a time. Launching enough elements into the upward CELL causes a **HIVE TIP** — flips the Hive, scores, and unlocks staged Nectar (1 per tip from 5 staged in the Alliance Area; all remaining Nectar unlocks at T-60s).
- **FLOWER** structures: robots place Pollen/Nectar via a retrieval opening. Points for placing Nectar in a Flower before the opponent, and for "owning" a Flower (top-most Nectar in it).
- Match: setup → **30s AUTO** → 8s transition → **2:00 TELEOP** → reset. Final scoring assessed once elements/robots are at rest.

### Scoring (partial)
- HIVE TIP ≈ 20 pts (third-party analysis of the manual table).
- Owned Flower element ≈ 2 pts.
- Elements left in the upward CELL at match end score (assessed at rest, not live).
- **RP**: WIN = 3, TIE = 1, plus POLLINATOR 1 RP, POLLINATOR 2 RP (hive-tip thresholds), and SWARM RP (LEAVE + PARK at/above threshold). Non-Championship thresholds: SWARM = 16 pts, POLLINATOR 1 = 4 tips, POLLINATOR 2 = 7 tips. Championship/Regional thresholds were TBA.
- ⚠️ Full Table 10-2 not extracted — re-check the manual PDF for exact values before strategy/design decisions.

### Field
- Standard 144 in. footprint, soft foam tiles (am-2499 or equiv.), perimeter (am-0481 reference).
- Elements: 2 Hives (each 2 Cells, **AprilTag clusters on the Cell undersides** for ID), 4 Flowers, Loading Zones, Gardens, Alliance Areas (~97 in. × 54 in.).
- **AprilTags**: 3.25 in. squares, **36h11** family, 4 distinct tags per Hive Cell cluster.
- Default dimension tolerance ±1 in. (±2.5 cm) unless noted.

### Season structure / advancement
- Register via FIRST Dashboard. Qualifying/League Tournament → (optional) Super Qualifying → Regional Championship → FIRST Championship / Premier Events; some regions have Pilot events.
- Advance only from events in the assigned HOME REGION; one reassignment per season if isolated.
- Advancement points: Qualification Phase Performance (normal-distribution formula on rank) + Alliance Selection + Playoff (40/20/10/5 for 1st–4th) + Team Judged Awards (Inspire worth most: 60/30/15).
- Registration cutoff for regional advancement-slot allocation: **Nov 17, 2026**.

### Awards
- Inspire (top overall); MCI (Design/Innovate/Control); TA (Connect/Reach/Sustain); Think (documentation/portfolio); Judges' Choice.
- **Portfolio**: max 15 pages + 1 cover, US Letter or A4, <15MB digital; only progress since **Jan 1, 2026**.
- Initial Interview: min 10 minutes; first 5 min reserved for uninterrupted team presentation.
- Innovate Award sponsored by RTX this season.

### Sources
- https://ftc-resources.firstinspires.org/ftc/game
- https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ Competition Manual - V1.htm
- https://www.firstinspires.org/first-canopy
- https://community.firstinspires.org

### Gaps
- Full Table 10-2; Section 12 robot construction rules (size/weight, motor/actuator counts); full G-rules beyond G408; Championship RP thresholds.

---

## 3. New Robot — Mechanisms

- **Intake** — motor-powered.
- **Transfer** — motor-powered, **bang-bang control** (not proportional).
- **Latch** — servo, **two positions only** (open/close), must hold whichever position it's set to.
- **Turret** — **4 servos**, each driving its own gear, all 4 meshing on the same axis. Single-axis (yaw only), geared/multi-servo drive.
- **Shooter** — dual-motor.
- **Limelight** — floor-facing, neural-network or color-based CV, for ball detection.
- **Arducam** — second camera, AprilTag Hive detection (see §5).

---

## 4. Codebase Structure

### Canonical directory tree (from Techineers' TeamCode repo)
**Keep this structure identical for Biobuzz** — only logic, variable names, class names, and subsystem implementations change.

```
src/main/java/org/firstinspires/ftc/teamcode/
├── readme.md
├── auto/                          # Autonomous OpModes
│   ├── BlueClose18Auto.java
│   ├── RedClose18Auto.java
│   ├── RedClose21Auto.java
│   ├── RedFarAuto.java
│   └── FixedRedClose18Auto.java
├── robot/                         # Core robot class & configuration
│   ├── Robot.java                 # Main robot class (instantiates all subsystems)
│   ├── RobotConfig.java           # Robot configuration constants
│   └── RobotTeleop.java           # Teleop-specific robot logic
├── subsystems/                    # Hardware subsystems
│   ├── DrivetrainHardware.java    # Drive motors, odometry
│   ├── Intake.java
│   ├── Limelight.java             # Vision/targeting sensor
│   ├── Shooter.java
│   ├── Turret.java
│   └── [Other subsystems as needed]
├── teleop/                        # Teleop OpModes organized by context
│   ├── competition/
│   │   ├── BlueCloseTeleop.java
│   │   └── RedCloseTeleop.java
│   ├── practice/
│   │   ├── PracticeBlueTeleop.java
│   │   └── PracticeRedTeleop.java
│   └── testing/
│       ├── DriveTeleop.java
│       ├── FilmTeleop.java
│       ├── LimelightTest.java
│       └── Tuning.java
├── pedroPathing/                  # Pedro Pathing library config
│   ├── Constants.java             # Path constants (close-field)
│   ├── FarConstants.java          # Path constants (far-field)
│   └── Tuning.java                # Tuning OpMode for path weights
├── controlSystems/                # Utility control algorithms
│   ├── KalmanFilter.java
│   ├── LookUpTable.java           # e.g. shooter distance→power
│   └── PIDController.java
└── util/                          # Utility enums & helpers
    ├── Alliance.java              # Blue/Red enum
    ├── FieldConstants.java        # Goal positions, etc.
    ├── RelocalizePosition.java
    ├── StartPosition.java
    └── Teleop.java                # Teleop mode enum
```

### Structure principles
1. **robot/** is the hub — instantiates all subsystems; never put game logic here.
2. **subsystems/** are isolated, reusable hardware abstractions — no game logic.
3. **auto/** contains only OpModes; one auto per file.
4. **teleop/** separates competition / practice / testing by folder.
5. **pedroPathing/** holds library config & constants.
6. **controlSystems/** holds reusable algorithms.
7. **util/** holds enums and constants — no logic.

### Biobuzz package decisions
- Planned top-level: `Robot.java`, `Tele.java`, `Close.java`/`Far.java`, `ClosePaths.java`/`FarPaths.java`.
- `subsystem/`: `Intake.java`, `Transfer.java`, `Latch.java`, `Turret.java`, `Shooter.java`. Placeholder `vision/` and `auto/` packages to fill in later.
- **No separate `calc/` package.** SOTM math, interpolation, and angle math live inside their own subsystem files (`Shooter.java` holds its SOTM/interpolation; `Turret.java` holds its angle math).
- `util/` holds only hardware-adjacent/reusable helpers: `CachedMotor.java`, `LowPassFilter.java`, `KalmanFilter.java` — **fresh implementations, not copied from the old repo**.

---

## 5. Architecture Patterns (carry across seasons)

Old repo layout (messy, being reorganized): `org.firstinspires.ftc.teamcode` root held top-level OpModes (`Close`, `Far`, `Tele`, `Robot`) and per-auto path classes; sub-packages `pedro`, `subsystem`, `util`, `test`, `variant`.

### `CachedMotor` (implements `DcMotorEx`)
Wraps a real `DcMotorEx` and short-circuits redundant hardware writes/reads:
- `setPower()` clamps to [-1,1]; skips the real call if within 0.01 of the cached value. Same idea for `setVelocity`, `setTargetPosition`, `setZeroPowerBehavior`.
- `getCurrent(unit)` caches raw reads for `currentCacheTimeMS` (default 200ms) and runs them through a `ScalingLowPassFilter` (static `@Config`-tunable `filterCoefficient`/`filterFactor`/`filterScaleFactor`).
- Constructed from an existing `DcMotorEx` or `new CachedMotor(hardwareMap, "name")`.
- Pattern (dedupe writes + cached/filtered current) is meant to be reused every season.

### Filter family
`Filter` abstract class (`update()/getValue()/getRawValue()/forceSet()`) with `ScalingLowPassFilter` (nonlinear low-pass, used in CachedMotor) and `Kalman` (scalar Kalman using Pedro's `KalmanFilterParameters`).

### `CommandOpMode`
Abstract base extending `OpMode`, thinly wrapping Ivy's static `Scheduler`: `schedule(Command...)`, `loop()` → `Scheduler.execute()`, `stop()`/`reset()` → `Scheduler.reset()`. All autos extend this.

### Alliance-variant pattern
Base OpModes (`Close`, `Far`, `Tele`) take an `Alliance` enum (`BLUE`/`RED`) and mirror poses/targets internally (`Pose.mirror()`). A `variant` package holds one-line `@Autonomous`/`@TeleOp` subclasses (`RedFar extends Far`, etc.) calling `super(Alliance.X)` — keeps alliance switching out of the main classes.

### Subsystem class shape
- `@Config` annotated; hardware fields `private final CachedMotor`/`Servo`.
- Tunable constants as `public static double` (dashboard-tunable).
- Plain action methods: `set(double)`, `in()`, `out()`, `off()`, plus sensor getters.
- Matching Ivy command wrappers returning `CommandBuilder` via `Commands.instant(this::method)`.
- Shooter-style subsystems add `periodic()` (feedforward + P velocity loop), `atTarget()`, and distance→velocity interpolation (incl. a `BilinearInterpolation` `forPose` variant using `smile.interpolation`).
- Turret-style: servos in lockstep, linear position↔radians mapping (`ppr`), `face(targetPose, robotPose)` computing/clamping/setting yaw.

### `Robot` class
Composition root: constructs all subsystems + the Pedro `Follower` (**`Constants.create(hardwareMap)`** — the old `Constants.createFollower()` form is wrong), bulk-caching hub setup, `periodic()` (updates follower, calls subsystem periodics, tracks loop timing), and higher-level composite `CommandBuilder` actions (`shoot(Pose)`, `intake()`) built from `Groups.sequential(...)`, often `.raceWith(Commands.infinite(...))` for background tasks (e.g. turret tracking during a shoot sequence).

### ⚠️ Syntax sourcing
Stop citing old-repo-specific syntax (Ivy command names, `CachedMotor` API, etc.). **§8 is the authoritative API reference** — verified against pedropathing.com on 2026-09-20. The notes below are historical context only.

- `FollowPath` (old repo's custom command wrapping a `PathChain`/`Follower`, with early-finish t-value like `new FollowPath(f, path, .95)`) is **fully obsolete** — that repo was on a beta. Current replacement: `PedroCommands.follow(follower, path[, holdEnd])` and `PedroCommands.hold(follower[, pose])`, via `import static com.pedropathing.ivy.pedro.PedroCommands.*;`. Unconfirmed whether current `follow()` has an early-finish-at-t equivalent.
- Old path files mixed legacy Pedro API (`f.pathBuilder().addPath(new BezierCurve(...))...`) with Pedro 3 (`Paths.line(...)`, `PoseFactory.degrees().mirrorX(...)`), with incomplete stubs. Treat the legacy calls as dead; prefer `Paths`/`PoseFactory`.

### Other old-repo files (context only)
`Artifact` enum (per-color RGB calibration, game-specific), `RGBLight` util (`.orange()`, `.green()`, `.violet()`, `.blue()`), `TriTuple` util, test OpModes (`TestDrive`, `ShootTest` — standalone `@TeleOp(group="Tests")` for isolated bring-up).

---

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

## 7. Arducam Hive Vision

Second camera (Arducam), separate from the floor-facing Limelight. Purpose: AprilTag-detect whether the alliance Hive is up or tipping, using the Hive Cell AprilTag clusters (4 distinct 36h11 tags per cluster, on the Cell undersides).

**Given constraints**
- Camera is **physically rotated to the right** to maximize vertical FOV — mount decision is made, don't re-derive.
- Detection must be **heavily lightweight**; runs on the **Control Hub** (no coprocessor), so inefficient settings directly cost FPS/looptime.
- Must return a value usable from auto classes; class belongs in `util/`; use current Ivy syntax, cross-checked against official FTC vision docs.

### Camera legality / hardware
- FTC only requires a **UVC** webcam — not restricted to Logitech; that's just what FIRST pre-calibrated. Arducam USB UVC modules (OV9782, IMX219, IMX291 boards) are UVC-compliant and work with VisionPortal like any webcam.
- Trade-off: no entry in the SDK's `builtinwebcamcalibrations.xml`, so accurate **pose** would need manual lens-intrinsics calibration (AprilTag Camera Calibration utility OpMode). Irrelevant here — see below.
- Two webcams on one Control Hub: both into the CH USB 3.0 port via a hub (see "AprilTag Switchable Cameras" / Vision MultiPortal sample). Under MJPEG, resolutions below ~432×240 can degrade one stream — bench-test the actual combo. (Limelight is a network coprocessor, so it doesn't compete for CH USB bandwidth.)

### Key finding: skip pose, keep ID-only
- The SDK only computes full pose (ftcPose/robotPose, 30+ fields) for a tag **if that tag has size metadata in the AprilTagLibrary**. Without sized entries, detections return **ID + decisionMargin only** — no pose math runs.
- Since the util only needs "which cluster is visible," this is the single biggest lightweight win: **never** call `setLensIntrinsics()` or pass a sized tag library. Also means **no Arducam lens calibration is needed at all** for this util.

### Rotation
- SDK 8.2+ defaults to `SENSOR_NATIVE` — no frame rotation. FTC docs say forcing stream rotation has "rather high" overhead.
- Since the camera is already physically rotated, **do not** request software rotation — leave `SENSOR_NATIVE`, pay zero cost. Only consequence: ftcPose X/Y axes would be swapped — irrelevant, pose is never read.

### CPU / looptime levers (ordered by real cost)
1. **Resolution** — biggest lever. 320×240 or 424×240 is plenty for ID at on-robot range; 640×480 only for pose/long range (not needed).
2. **Decimation** — `aprilTagProcessor.setDecimation(n)`, changeable mid-match. SDK sample (C920 @ 640×480): decimation 2 ≈ 10 FPS / ~10 ft on a 2" tag; decimation 3 (default) ≈ 30 FPS / ~4 ft. Higher decimation likely fine at close fixed range — bench-test at actual distance.
3. **Stream format: MJPEG**, not YUY2 (YUY2 caps out or drops one of two webcams above ~640×360).
4. **`enableLiveView(false)`** — free win, no competition downside.
5. **Disable draw overlays** — `setDrawAxes(false)`, `setDrawCubeProjection(false)`, `setDrawTagOutline(false)`. Meaningless without pose anyway.
6. **`visionPortal.setProcessorEnabled(aprilTag, false/true)`** — cheap, non-blocking toggle; pause detection during phases where Hive state isn't checked (e.g. mid-path-follow), re-enable before the check. Safe from an Ivy `instant()`.
7. **`stopStreaming()`/`resumeStreaming()`** — bigger win but **blocking**. Only at clear phase boundaries, never in a tight loop or `periodic()`.
- Note: `getDetections()` is a cheap accessor over already-computed results — throttling *calls* saves nothing. The cost is the background detection pass per frame, which is why 1–4 matter.

### Design landed on
- Class **`HiveVision`**, package `util/`, `@Config`-annotated (hardware-adjacent reusable helper).
- Owns its own `VisionPortal` + `AprilTagProcessor` scoped to the Arducam's `WebcamName`.
- **No pose config at all** — overlays off + decimation only; no `.setTagLibrary(sizedTags)`, no `.setLensIntrinsics(...)`.
- `update()` — cheap poll over `getDetections()`, counts ID matches against two tunable `int[]` ID sets (one per Hive Cell), filters below a tunable `MIN_DECISION_MARGIN`, sets/returns a 3-state `HiveState` enum: `CELL_A_UP`, `CELL_B_UP`, `TIPPING` (both-or-neither cluster visible = unresolved/mid-flip rather than trusting a stale read).
- `update()` called once per loop from `Robot.periodic()` (matching the `Shooter`/`Turret` `periodic()` convention), so any command can read `getState()` without re-triggering work.
- Exposed to autos as a plain getter `getState()`, consumed via Ivy `waitUntil(BooleanSupplier)` / `conditional(...)` — no command wrapper needed; it's a sensor, not an actuator.

### ⚠️ Open TODOs
- **Exact tag IDs for BIOBUZZ Hive Cells not yet pulled** from the manual's tag-assignment table — current arrays are placeholders. Confirm before relying on this.
- Confirm whether "underside" placement means the visible cluster belongs to the **currently-up** or **currently-down** cell — this flips which state the util should report as tipped/scored. Field geometry not fully nailed down.

### Sources
- ftc-docs AprilTag: apriltag-intro, apriltag-pose, apriltag-library, visionportal-cpu-and-bandwidth, apriltag-advanced-use, visionportal-previews
- ftc-docs: configuring-external-webcam, vision-multiportal
- https://ironreignrobotics.org/docs/vision-and-localization-strategy/ (community; CPU/threading model, decimation table, worked webcam subsystem example — cross-checked against official docs)
- https://docs.arducam.com/UVC-Camera/USB2-UVC-Camera-Kit/Specs-and-Selection-Guide/ (UVC compliance)
- https://pedropathing.com/docs/ivy/creating-opmodes (confirms current Ivy syntax)

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

- Follower creation is **`Constants.create(hardwareMap)`**.
- TeleOp robot-centric: `follower.manual(forward, lateral, turn)` + `follower.update()` every loop.
- Field-centric: `DrivePowers powers = ManualDrive.fieldCentric(fwd, lat, turn, follower.pose().heading()); follower.manual(powers);`
- Anti-push hold: `ManualDrive.driveOrHold(follower, fwd, lat, turn)` or `ManualDrive.driveOrHold(follower, powers)`.
- Localization: `follower.pose()` → `.x()`, `.y()`, `.heading()`. **Always inches and radians** — convert with `Math.toDegrees()`. `follower.setPose(pose)` overrides (only with a relocalization source or a known corner). `follower.mode()` reports follow state.
- Auto → TeleOp pose handoff is not built in: use a static storage class (e.g. `OpModeStorage.autonomousEndPose`), written in the auto's `stop()`, read in TeleOp's `start()`.

### 8.15 Sources

- https://pedropathing.com/docs/ivy , /command-builder , /command-compositions , /utilities-and-decorators , /class-api , /creating-opmodes , /pedro-commands
- https://pedropathing.com/docs/pathing/installation , /guide/path-creation , /guide/auto-usage , /guide/teleop-usage , /reference/api , /reference/posefactory , /reference/interpolation

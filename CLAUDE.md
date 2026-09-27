# CLAUDE.md — Electrolights Biobuzz

Project context for FTC BIOBUZZ (2026–2027 season) robot code. Consolidated from stored notes.


### Side project — FTC deploy tool
- Fork/remake of **Pusher** (github.com/PzmuV1517/Pusher, Go-based FTC deployment CLI).
- Goal: Pusher's Wi-Fi/ADB connectivity automation + hot-reload speed comparable to **Sloth**.
- Mac first; Linux and Windows later.
---

## 2. BIOBUZZ — 2026–2027 FTC Game

Season overview, gameplay, scoring/RP thresholds, field, advancement and awards live in the `biobuzz-game-rules` skill (`.claude/skills/biobuzz-game-rules/SKILL.md`). Load it for strategy, auto-routine, or mechanism-design decisions.

---

## 3. New Robot — Mechanisms

- **Intake** — single motor-powered.
- **Transfer** — dual motor-powered, **bang-bang control** (not proportional).
- **Latch** — servo, **two positions only** (open/close), must hold whichever position it's set to.
- **Turret** — **4 servos**, each driving its own gear, all 4 meshing on the same axis. Single-axis (yaw only), geared/multi-servo drive. All servos share a wire so in code will appear as 1 servo.
- **Shooter** — single-motor flywheel (feedforward + P velocity loop).
- **Limelight** *(planned, not in code yet)* — floor-facing, neural-network or color-based CV, for ball detection.
- **Arducam** *(planned, not in code yet)* — second camera, AprilTag Hive detection (see §6).

---

## 4. Codebase Structure

### Biobuzz package decisions
- **No separate `calc/` package.** SOTM math, interpolation, and angle math live inside their own subsystem files (`Shooter.java` holds its SOTM/interpolation; `Turret.java` holds its angle math).
- `util/` holds only hardware-adjacent/reusable helpers — **fresh implementations, not copied from the old repo**. Top level: `Alliance`, `CommandOpMode` (used by every OpMode). Subpackages: `hardware/` (`CachedMotor`), `filter/` (`Filter`, `ScalingLowPassFilter`, `KalmanFilter` — still an empty stub), `input/` (`GamepadEx`, `BooleanSwitch`, `FloatSupplier`), `data/` (`Tuple`, `TriTuple`).

### Hardware names
All hardware-map device names live in **`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`** (`@Configurable`, `public static String`). Subsystems and `Robot` read from it; `pedro/Constants` should too once the Follower is filled in. Never hard-code a name string elsewhere. Saved Control Hub configurations are in `configs/*.xml` (repo root) — names in `HardwareNames` must match the active one. Pedro's tuner procedures (`pedro/procedures/*`) keep their own template defaults (e.g. `"pinpoint"`) to avoid upstream merge conflicts.

| Constant | Config name | Device |
|---|---|---|
| `FRONT_LEFT` / `FRONT_RIGHT` / `BACK_LEFT` / `BACK_RIGHT` | `frontLeft` / `frontRight` / `backLeft` / `backRight` | Drive motors (Expansion Hub) |
| `PINPOINT` | `pinpoint` | goBILDA Pinpoint |
| `IMU` | `imu` | Control Hub IMU |
| `INTAKE` | `intakeMotor` | Intake motor |
| `TRANSFER_1` / `TRANSFER_2` | `transferMotor` / `transferMotor2` | Transfer motors |
| `SHOOTER` | `launchingmotor` | Flywheel motor |
| `HOOD` | `hoodServo` | Hood servo |
| `LATCH` | `stopperServo` | Latch servo |
| `TURRET` | `turretServo1` | Turret servos (4 on one signal wire → one device on servo port 3; the XML configs also list `turretServo2`–`4`, unused in code) |

Cameras are not listed yet (vision is planned, see §6). Keep this table in sync when `HardwareNames` changes.

---

## 5. Architecture Patterns (carry across seasons)

### `CommandOpMode`
Abstract base extending `LinearOpMode` (so **`init()`/`start()`/`loop()`/`stop()` are final — never override them**), thinly wrapping Ivy's static `Scheduler`. Lifecycle hooks to override: `preInit()`, `initialize()` (abstract), `initializeLoop()`, `onStart()`, `execute()`, `end()`. `runOpMode()` calls `reset()` first (the Scheduler is static), then per active loop runs `execute()` **followed by** `Scheduler.execute()`, and always calls `reset()` + `end()` on exit. `schedule(Command...)` queues commands; the Scheduler only runs after START, so anything scheduled in `initialize()` starts then. All autos and teleops extend this.

### Alliance-variant pattern
Base OpModes (`Close`, `Far`, `Tele`) take an `Alliance` enum (`BLUE`/`RED`) and mirror poses/targets internally (via `PoseFactory.mirrorX(...)`, see `auto/paths/*Paths.poses(Alliance)`). One-line `@Autonomous`/`@TeleOp` subclasses (`auto/red/redFar`, `tele/teleBlue`, etc.) call `super(Alliance.X)` — keeps alliance switching out of the main classes.

### Subsystem class shape
- `@Config` annotated; hardware fields `private final CachedMotor`/`Servo`.
- Hardware-map device names come from `HardwareNames` (see §4 Hardware names) — never hard-code a name string or add a per-subsystem `*_NAME` constant.
- Tunable constants as `public static double` (dashboard-tunable).
- State-based motor subsystems (`Intake`, `Transfer`): a nested state enum, one tunable `public static` power per state, one method per state, plus `getState()` / `atState(state)`. `Intake` and `Transfer` both have exactly four states — `INTAKE` (on), `OUTTAKE`, `SLOW`, `STOPPED` (off) — with methods `intake()`, `outtake()`, `slow()`, `stop()`. `SLOW` is a slow reverse; `Robot.outtake()` (jam clearing) uses `intake.slow()` + `transfer.slow()`. Don't add other intake/transfer modes or parameterized control methods (no `intake(boolean ...)` / `intakeControl(...)`).
- Other subsystems use plain action methods plus sensor getters.
- **No Ivy command wrappers and no `requiring(...)` on subsystems** — subsystems stay plain classes; sequences that use them are built in `Robot` (below).
- Subsystems with per-loop work expose `update()` (called from `Robot.update()`); most have none.
- Shooter-style subsystems add `update()` (feedforward + P velocity loop), `atTarget()`, and distance→velocity interpolation (incl. a `BilinearInterpolation` `forPose` variant using `smile.interpolation`).
- Turret-style: servos in lockstep, linear position↔radians mapping (`ppr`), `face(targetPose, robotPose)` computing/clamping/setting yaw.

### `Robot` class
Composition root: constructs all subsystems + the Pedro `Follower` (**`Constants.create(hardwareMap)`** — the old `Constants.createFollower()` form is wrong), bulk-caching hub setup, and `update()` (clears the bulk cache, updates the follower and subsystems, tracks loop timing — call it once per loop, first). The important actions live here as methods: plain composites (`drive(...)`, `shootAll()`/`stopShootAll()`, `outtake()`/`stopOuttake()`) and Ivy commands built from `Groups.sequential(...)` / `Commands.instant` / `waitMs` (`shootOne()`), often `.raceWith(Commands.infinite(...))` for background tasks (e.g. turret tracking during a shoot sequence). OpModes call these; they do not build sequences themselves. Tunable timings for these sequences (`SHOOT_ONE_MS`) are `public static` on `Robot`.

### TeleOp shape (`tele/Tele.java` is the reference)
- `initialize()`: build `Robot` and `GamepadEx` objects, register trigger buttons (`gp.left_trigger_button(FloatSupplier::isPress)`), schedule **one** `infinite(...)` that runs `robot.update()`, the drive call and telemetry.
- `initializeLoop()`: also call `gp.update()`, so a button held through START does not register as a press.
- `execute()`: `gp.update()`, then plain `if (gp.x.isRisingEdge()) ...` checks that either call `robot.*` directly or `schedule(robot.someCommand())`. Held inputs are handled as a rising-edge "start" plus a falling-edge "stop" (e.g. right trigger: `robot.shootAll()` / `robot.stopShootAll()`); continuous inputs (sticks, turret slew) use `isTrue()` / `getVal()`.
- Do **not** add button-binding helpers or per-control `infinite` commands with requirements/priorities; keep the teleop nearly logic-free.
- `GamepadEx.update()` must run once per loop. A derived switch (`.risingEdge()`, `.and(...)`) updates its source, so use `gp.apply(...)` to swap rather than registering both.

### Auto shape (`auto/Close.java`, `auto/Far.java`)
`initialize()` builds `Robot` and sets the alliance-mirrored start pose from `*Paths.startPose(a)`; `initializeLoop()` shows pose/alliance; `onStart()` schedules `infinite(robot::update)` **first** (the Scheduler runs commands in scheduling order), then the `routine()` — a `sequential(...)` of `follow(...)` / `robot.*` commands. Poses come from `*Paths.poses(a)` so RED is mirrored automatically. `end()` calls `robot.stop()`. `routine()` and the start pose are still placeholders (TODO).

### Syntax sourcing
Stop citing old-repo-specific syntax (Ivy command names, `CachedMotor` API, etc.). The authoritative Pedro 3 / Ivy API reference is the `pedro-ivy-reference` skill (`.claude/skills/pedro-ivy-reference/SKILL.md`) — load it before writing Ivy/Pedro code. Follower creation is `Constants.create(hardwareMap)`. Pedro poses are always inches and radians.

---

## 6. Arducam Hive Vision (planned)

**Vision is removed from the code for now** — no camera classes, no camera names in `HardwareNames`, nothing in `Robot`. It will be added back later. When re-added, put the camera name(s) in `HardwareNames` and the §4 table. The earlier `ArduVision` was also written against the pre-12.0 SDK: in SDK 12.0 `AprilTagDetection` is abstract, `id`/`decisionMargin` live on `AprilTagSingleDetection`, and there is a new `AprilTagClusterDetection` (the Hive uses tag clusters).

Design notes for when it comes back — constraints, findings, CPU levers and open TODOs — are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/CLAUDE.md` (loads when working in `vision/`). Camera rotation is physical — keep `SENSOR_NATIVE`, never request software rotation; detection is AprilTag-ID-only (no pose, no lens intrinsics).

---

## 7. Repo notes

- `FtcRobotController/` is the vendor SDK module — do not edit unless intentionally patching the SDK; team code goes in `TeamCode/`.
- There is no unit test suite; verify by deploying to the Control Hub or running the Pedro tuning OpModes.

## Git workflow (team-specific — see `TeamCode/.../teamcode/gitInstructions.md` for full detail)

- `main` mirrors Pedro's upstream template branch (currently `upstream/pedro3`, tracked via a second `upstream` remote pointing at `Pedro-Pathing/Quickstart`) and is not where feature work happens. Feature/season work happens on branches like `Biobuzz` or `Offseason-2026`, which periodically merge `main` down to pick up template/Pedro updates.
- Commit message format: `[day] [type of day] [tag]: short desc`, e.g. `D6 meet feat: added slew rate`.
  - Day: sequential meet/day counter for the season.
  - Type of day: `comp` | `meet` | `scrim`.
  - Tag: `test` | `feat` | `fix` | `chore` | `docs`.

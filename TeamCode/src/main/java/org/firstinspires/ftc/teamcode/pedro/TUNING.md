# Pedro Follower tuning: goal, findings and status

Written 2026-09-26 (pre first meet). This records what we were trying to do with Pedro 3's tuners, what we
learned about the API, what was built, and why it was rolled back. None of the code described here was
committed, so this file is the only record of it.

## Current status

- `pedro/Constants.java` holds the old `Pedro3+Ivy` values (last season's robot, forward translational kP = 0)
  and `create()` builds a real Follower. They are placeholders until re-tuned on this robot.
- `pedro/Tuning.java` registers Mecanum, Pinpoint, Foresight and Tests, so they show on the AutoTune page.
  Paste each tuner's Java tab into `Constants.java`, keeping the `HardwareNames` references for device names.

## What we were trying to accomplish

1. **Get a real Follower.** `Robot` builds it with `Constants.create(hardwareMap)`, and TeleOp needs it for
   field-centric driving, the pose handoff from auto, and turret/hood/flywheel aiming (all of which read
   `follower.pose()` and `follower.velocity()`).
2. **Tune it from scratch on this robot.** Assume nothing is tuned: motor directions, Pinpoint pod
   offsets/directions and every Foresight gain have to be measured with Pedro's AutoTune.
3. **Make tuning low-effort.** Run each tuner once and have its result land in `Constants.java` in Android
   Studio, without copy-pasting between runs. Hard requirement from the team: **tuned values live in the
   source (git), never saved on the robot and loaded from there**, so what the robot runs always matches
   `Constants.java`.

## How Pedro 3 tuning works (AutoTune)

- Tuning is a web page served by the robot: deploy, join the robot's Wi-Fi, open
  **http://192.168.43.1:10158**. Tuners are **not** Driver Station OpModes.
- The classes in `pedro/procedures/` are the tuning steps. One only shows up on the page when `Tuning`
  has a `@Tuner public static Procedure name()` method returning it.
- `Tuning.java` is blank **on purpose**: Pedro's own "Pedro Pathing 3.0.0" template commit (`f96afa6`,
  upstream `pedro3`) removed the `@Tuner` methods and left `// Tuners go here`. We got that when `main` was
  merged into `Biobuzz`. The docs expect each team to add the tuners it uses.
- When a tuner finishes, its **Java** tab shows a config block to paste into `Constants.java`.

Registration, per the docs (https://pedropathing.com/docs/pathing/tuning):

```java
@Tuner public static Procedure mecanumTuner()  { return new MecanumTuner(); }
@Tuner public static Procedure pinpointTuner() { return new PinpointTuner(); }
@Tuner public static Procedure foresightTuner() {
    return new ForesightTuner(h -> new PinpointLocalizer(h, Constants.localizerConfig),
                              h -> new Mecanum(h, Constants.drivetrainConfig));
}
@Tuner public static Procedure tests() {
    return new Tests(h -> new Mecanum(h, Constants.drivetrainConfig),
                     h -> new PinpointLocalizer(h, Constants.localizerConfig),
                     () -> new Foresight(Constants.foresightConfig)); // docs pass null until Foresight is tuned
}
```

### Tuner order and dependencies

| Step | Procedure | Finds | Reads `Constants`? |
|---|---|---|---|
| 1 | Mecanum | Motor directions (asks for motor names on the page) | No |
| 2 | Pinpoint | Pod offsets, pod directions, pod type (asks for the device name) | No |
| 3 | Tests → Localization | Check: push forward → x goes up, strafe left → y goes up | drivetrain + localizer |
| 4 | Foresight | Max velocities, decelerations, brake coefficients, heading/translational kP, coast/brake kV | drivetrain + localizer |
| 5 | Tests → Line / Curve / Hold / Interpolation | Path-following check | all three configs |

Because Foresight (step 4) builds the drivetrain and localizer from `Constants`, the Mecanum and Pinpoint
results must be in `Constants` (and deployed) before it runs. That dependency is why "paste once at the
end" doesn't work.

## API findings (Pedro `core` 3.0.1 / `revhub` 3.0.1 / `tuning` 1.0.1)

- **Follower:** `new Follower(Localizer, Drivetrain, Algorithm)`, built as
  `new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig))`.
- **Configs** (`MecanumConfig`, `PinpointConfig`, `ForesightConfig`) take a lambda `c -> { c.field.set(...); }`.
  Fields are `ConfigVar`s: `set()` can be called any time, and `get()` throws
  `IllegalStateException("Config variable has not been set")` for a required value nobody set.
  Construction never validates, so an empty config loads fine and fails only when read.
- **Required values** (no default):
  - `MecanumConfig`: the 4 motor names and 4 directions.
  - `PinpointConfig`: `name`, `xPodOffset`, `yPodOffset`, `xPodDirection`, `yPodDirection`.
  - `ForesightConfig`: `headingFeedback`, `forwardTranslational`, `strafeTranslational`, `brake`, `coast`,
    `linearBrakeCoefficients`, `quadraticBrakeCoefficients`, `headingBrakeCoefficients`,
    `maxAchievableForward/StrafeVelocity`, `naturalForward/StrafeDeceleration`.
  - `new Foresight(config)` reads the natural decelerations in its constructor, so an untuned
    `foresightConfig` makes `Constants.create()` throw.
- **Naming:** the docs and `MecanumTuner`'s output call the drivetrain config `drivetrainConfig`. Use that
  name so pasted snippets compile.
- **Hardware names** should come from `HardwareNames` (`FRONT_LEFT`, `PINPOINT`, …), not the strings the
  tuner prints.
- **Tests procedure:** its default selection is Line Test, which needs the Foresight algorithm. If `tests()`
  passes `null`, the default selection throws until you switch the dropdown to Localization.
- **Getting results out in code:** `Procedure.result(...)` is `final`, and the collected results are only
  exposed through a package-private `resultSnapshot()` (used by the AutoTune server). The only way to read
  them from our code is reflection on that method, which could break on any Pedro update.

## What was built (and rolled back)

1. **Registered the tuners** in `Tuning.java` as above, and filled `Constants.java` with values copied from
   the old `Pedro3+Ivy` branch (`pedroPathing/Constants.java`, commit `c1856d8`). Those numbers were never
   measured on this robot, and its forward translational kP was 0, so they were only a placeholder.
2. **Auto-save on the robot.** Each tuner saved its results to `/sdcard/FIRST/pedro/tuning.properties` on
   the hub, and `Constants` loaded that file over its own values at startup. **Rejected:** the robot could
   silently run different values from `Constants.java`.
3. **Auto-pull into Android Studio.** `Constants.java` was reset to untuned, with
   `// AUTOTUNE:MECANUM|PINPOINT|FORESIGHT:BEGIN/END` marker blocks. Then:
   - Each tuner (subclassed in `Tuning.java`) turned its results into Java lines and left them in
     `/sdcard/FIRST/pedro/autotune/<SECTION>.txt`. That folder was a drop-off only; the robot never read it.
   - Each tuner also applied its result in memory until the app restarted, so Foresight could run right
     after Mecanum and Pinpoint without a redeploy.
   - A Gradle task, `pullTuning` (`TeamCode/autotune.gradle`, hooked to run before every build), pulled
     the files over adb, replaced the lines between the markers and deleted the files from the hub.
   - It was tested only with a fake adb: splicing, re-splicing and compiling worked. It never ran against
     the real hub.

   **Rolled back** at the team's request: everything returned to the template stubs.

## Open decisions / next steps

- **Pick a tuning workflow:**
  - **Plain (docs):** register the tuners, then paste the Java tab into `Constants.java` after Mecanum +
    Pinpoint, redeploy, run Foresight, and paste again.
  - **Automated:** rebuild approach 3 above, accepting the reflection dependency on Pedro internals.
- **Fill in `Constants.java`** with the tuned configs and `create()` returning the real Follower. Until then,
  TeleOp and auto can't run.
- **Before the first meet**, at minimum: Mecanum directions and Pinpoint offsets. TeleOp driving and turret
  aiming depend on them. Foresight only matters for auto paths.

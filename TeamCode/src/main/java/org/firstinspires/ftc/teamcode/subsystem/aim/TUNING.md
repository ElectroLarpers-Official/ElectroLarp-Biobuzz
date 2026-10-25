# Shooter / Turret Tuning Guide

Which number to change, where it lives, and what it fixes.

> **Testing on the old (Decode) robot.** Every number borrowed from the old robot is marked `// TODO(new robot): old-robot value` in the source. When the new robot is ready, search the project for `TODO(new robot)` and retune each one (Android Studio: Edit > Find > Find in Files). Everything marked `public static` can be changed live in Panels, but **Panels changes are lost on restart: copy the final value back into the Java file.**

Files (all under `teamcode/`):

| File | What's in it |
|---|---|
| `subsystem/Shooter.java` | Flywheel speed loop (PIDF: `kP` / `kI` / `kD` / `kF`, optional bang-bang while shooting) |
| `subsystem/Hood.java` | Hood servo limits |
| `subsystem/Turret.java` | Turret pivot position on the robot, servo write threshold |
| `subsystem/aim/shot/ShooterConstants.java` | Where the HIVE cells are on the field, which cell to aim at, under-HIVE hold |
| `subsystem/aim/shot/ShooterCalc.java` | The shot solver: shot-map / tuned-point lookup, flywheel and hood models, lead, hood comp. **All solver tunables live here** |
| `subsystem/aim/turret/TurretCalc.java` | Turret gearing, angle limits, slew, per-alliance trim, yaw feedforward. **All turret tunables live here** |
| `subsystem/aim/shot/Braking.java` | Braking model used to predict where the robot stops |
| `subsystem/aim/shot/ShotCheck.java` | Can this shot go in from here? Aim / flywheel tolerances for "ready" |
| `subsystem/aim/table/HiveAimTable.java` | The simulated shot map (data in `HiveAimTableData.java`, generated — don't edit) |
| `subsystem/aim/table/TunedShotTable.java` / `TunedShotPoints.java` | Hand-tuned shots that override the map near them |
| `util/control/PIDController.java` | The PIDF math the flywheel uses |

Test OpModes (group "Shooter Test" / "Turret Test"): `FlywheelPIDFTest`, `HoodServoPositionTuning`, `ShotPointTuner`, `TurretTrackingTest`.

---

## Retune order (new robot / big mechanical change)

Do these in order. Each step assumes the ones before it are right.

### 1. Turret geometry — `TurretCalc`, `Turret`
The turret angle is turned into a servo position from the gearing: `servo position = SERVO_CENTER ± (turret deg / 1.731) / SERVO_RANGE_DEG`. 1.731 = (48/15) × (33/61) turret degrees per servo degree.

| Number | Default | Change when |
|---|---|---|
| `TurretCalc.SERVO_RANGE_DEG` | 355 | Set to the rotation the Axons are programmed for over position 0..1. If the turret turns too far / not far enough by a growing amount as the angle grows, this (or the gear teeth) is wrong. |
| `TurretCalc.SERVO_CENTER` | 0.5 | Servo position with the turret pointing straight ahead. |
| `TurretCalc.SERVO_REVERSED` | false | Set true if a positive angle turns the turret clockwise (positive should be counter-clockwise, to the robot's left). |
| `SERVO_GEAR_TEETH` / `STAGE1_DRIVEN_TEETH` / `STAGE2_DRIVE_TEETH` / `TURRET_RING_TEETH` | 48 / 15 / 33 / 61 | Only if the gears change. |
| `TurretCalc.MIN_ANGLE` / `MAX_ANGLE` | -224.75 / 224.75 | Soft limits (degrees from straight ahead), e.g. for wiring. Must stay inside what the servos reach (about ±307° with the defaults). |
| `Turret.TURRET_FORWARD_OFFSET` / `TURRET_STRAFE_OFFSET` | 0 / 0 | Turret pivot distance from the Pinpoint/Pedro pose point, inches (forward +, left +). 0 from CAD: the turret axis is on the robot's centre of rotation (middle of the four wheel axles), and the Pinpoint pod offsets in `pedro/Constants` are measured from that point. If the pod offsets move the pose point, change these to match. |
| `Turret.SERVO_THRESHOLD` | 0.0002 | Only writes the servo if the position moved more than this. Raise if the turret buzzes while still; lower if it moves in visible steps. |

Use `TurretTrackingTest` (needs only the Pinpoint and the turret servo): push/spin the robot by hand and the turret should keep pointing at the cell.

### 2. Flywheel speed loop — `Shooter`
Meta Infinity's PIDF on **motor RPM**: power = (`kF` × target RPM / 5200 + `kP` × error + `kI` × integral + `kD` × derivative) × 12 V / battery voltage. Tune it in `FlywheelPIDFTest`.

| Number | Default | Change when |
|---|---|---|
| `kF` | 1.1 | Feedforward. `kF = 1` is full power at 5200 motor RPM. Tune first with `kP = 0`: the wheel should settle close to target on its own. |
| `kP` | 0.0006 | Power per RPM of error. Raise if it recovers slowly after a shot; lower if it oscillates. |
| `kI` / `kD` | 0 / 0 | Usually leave at 0. |
| `BANG_BANG_WHILE_SHOOTING` | false | Turn on to use full power (`BANG_POWER`, 1.0) whenever the wheel is more than `DEADBAND_RPM` (15) under target while shooting. Off = PIDF the whole time. |
| `VELOCITY_TOLERANCE` | 100 ticks/s | How close counts as "at speed" on telemetry. Shooting uses ShotCheck's own tolerance (section 10). |
| `ENCODER_REVERSED` / `MOTORS_REVERSED` | true / false | Flip `ENCODER_REVERSED` if the RPM reads negative while the wheel spins the right way. |
| `WRITE_THRESHOLD` | 0.005 | Only writes motor power when it changes by more than this. |

### 3. Flywheel and hood models — `ShooterCalc`, `Hood`
These turn the solver's answer (ball exit speed, launch angle) into hardware commands.

- **Ball speed → flywheel:** belt model. Flywheel rev/s = exit speed / `FLYWHEEL_EXIT_EFFICIENCY` / (π × `FLYWHEEL_DIAMETER_IN` × 0.0254), then × `FLYWHEEL_PULLEY_TEETH`/`MOTOR_PULLEY_TEETH` (40/20) × `MOTOR_TICKS_PER_REV` (28). With the defaults that is about 570 motor RPM per m/s. **`FLYWHEEL_EXIT_EFFICIENCY` (0.88) is the number to calibrate**: measure ball exit speed (high-fps video) at a few RPMs.
- **Launch angle → hood servo:** 33T servo gear → 200T hood, goBILDA servo with 300° over 0..1. `position = HOOD_SERVO_POS_AT_LAUNCH_MIN + (deg − LAUNCH_MIN_DEG) × (200/33) / 300`. Position 0 = flattest angle; 80° ≈ 0.747. Check it with `HoodServoPositionTuning` and a protractor. If 43.02° isn't on the flat stop, adjust `HOOD_SERVO_POS_AT_LAUNCH_MIN` (0). If 80° is off, check `HOOD_SERVO_RANGE_DEG` (300).
- **`LAUNCH_MIN_DEG` / `LAUNCH_MAX_DEG`** (43.01765° / 80°): the real hood's launch-angle range. The solver never asks for more.
- **`Hood.HOOD_MIN_POS` / `HOOD_MAX_POS`** (0.0 / 0.75): servo clamps, so the hood never hits its hard stops. `Hood.START_LAUNCH_DEG` (80) is where it goes at init.

### 4. Where you're aiming — `ShooterConstants`
| Number | Default | Change when |
|---|---|---|
| `USE_MAP_AIM_POINTS` | true | Aim at the window centres from the field CAD (`MAP_*`). Keep this on while the shot map is on: the map's shots assume the turret points at these. |
| `MAP_RED_TOP_X/Y`, `MAP_RED_BOTTOM_X/Y`, `MAP_BLUE_TOP_X/Y`, `MAP_BLUE_BOTTOM_X/Y` | red (59.25, 88.45) / (59.25, 55.55), blue (84.75, 88.45) / (84.75, 55.55) | CAD window centres, Pedro inches. TOP = far cell (high y), BOTTOM = audience cell (low y). |
| `HIVE_RED_TOP_X/Y`, `HIVE_RED_BOTTOM_X/Y`, `HIVE_BLUE_TOP_X/Y`, `HIVE_BLUE_BOTTOM_X/Y` | see file | Measured aim points. **Only used when `USE_MAP_AIM_POINTS` is off.** Consistent left/right miss at **one** cell only → move that cell. |
| `HIVE_SIDE_SPLIT_Y` | 72 | Robot y where it switches from the audience cell to the far cell. |
| `HIVE_SIDE_HYSTERESIS` | 3 | Raise if the aim flips between cells when sitting near the middle. |

### 5. Stationary shots — tuned points first, then trims
Shoot from a standstill first. Don't touch the moving-shot numbers until these land.

With the shot map on (default), the launch angle and speed at every spot come from the map or from your tuned points (see **Shot map** and **Tuned shot points** below). The best fix for a spot that misses is a tuned point there. The trims below shift every shot:

| Symptom | Number | Direction |
|---|---|---|
| Short/long at **every** spot | `verticalShift` (0) | Ticks/s added to every solved flywheel command (not to the under-HIVE hold). |
| Every tuned point needs the same RPM / hood change (new batch of balls) | `TunedShotTable.RPM_TRIM` / `HOOD_TRIM` (0 / 0) | Added to every tuned point. |
| Map shots short/long everywhere | `FLYWHEEL_EXIT_EFFICIENCY` (0.88) | Step 3: calibrate it rather than trimming. |
| Want a slightly higher / steeper arc everywhere | `hiveHeightTrim` (0 in) / `hiveEntryTrimDeg` (0°) | Added to the solver's score height / entry angle. More negative entry = steeper. |
| Release height wrong | `launchHeight` (11.0 in) | Shooter exit above the tiles (CAD: 10.4–11.6 in). Used for tuned-point flight times, the geometry shot check and the map-off aim height. Map shots don't use it. |

Only with the shot map **off** (`USE_AIM_TABLE = false`): `hiveAimHeight` (65 in) is the aim height and `hiveEntryDeg` (-50°) the entry angle; if balls bounce off the back of the HIVE and out, try -40.

### 6. Turret aim — `TurretCalc`
| Symptom | Number | Direction |
|---|---|---|
| Consistently left/right on **red** | `turretOffset` (15) | Degrees added to the turret command. **Old-robot value.** |
| Consistently left/right on **blue** | `turretOffset2` (10) | Same, blue only. **Old-robot value.** |
| Aim lags behind while the robot spins in place | `yawFeedforwardGain` (0.145) | Raise. Aim *leads* (swings past) while spinning → lower. |
| Turret too slow to track | `maxSlewNormalDegPerSec` (400) | Raise. Turret overshoots/jerks → lower. |
| Wrap-around sweep too slow | `maxSlewWrapDegPerSec` (540) | Raise. Jerky wrap → lower. |
| Turret sits at a different point during a wrap | `wrapServoOffset` / `servoOffset` (0 / 0) | Servo units (0–1). All 4 servos share one wire, so this shifts the whole turret. Leave at 0 unless you need it. |

### 7. Shooting while moving — `ShooterCalc`
Only after steps 5 and 6 are solid.

| Symptom | Number | Direction |
|---|---|---|
| Misses **behind** the direction of travel (not enough lead) | `leadGain` (1.0) | Raise. |
| Misses **ahead** of travel (too much lead) | `leadGain` | Lower. |
| Lead is right in range but wrong sideways | `sotmFactor` (1.0) | Scales the turret's angle away from the plain goal bearing while moving (lead plus the forward projection), not the flywheel. |
| Moving shots short/long only (not stationary) | `sotmOffset` (0) | Ticks/s removed from the flywheel only while moving (projected speed ≥ `sotmMinSpeed`, or while the braking model is on). |
| Lead consistently too small/large at all speeds | `dragTimeFactor` (1) | Scales the flight time the lead uses. |
| Time from latch-open to ball leaving is different | `feederDelay` (0.1 s) | Measure on high-fps video. |
| Turret still arriving when the ball leaves | `turretSettleTime` (0.03 s) | Raise. |
| Turret jitters when turning in place (fake speed) | `sotmMinSpeed` (12 in/s) | Raise. Below this (projected) speed no lead is applied (except while the braking model is on); the release point is still projected forward. |
| Misses while driving a curve | `compensateTurning` (true) | Leave on. Tune `leadGain` first. |

Accel filtering (usually leave alone): `accelFilterAlphaDecel` (0.5), `accelFilterAlphaAccel` (0.3), `positiveAccelGain` (0.5), `slopeWindowSeconds` (0.15), `maxProjectedSpeed` (125), `accelClamp` (400).

### 8. Shooting while braking — `ShooterCalc`, `Braking`
| Symptom | Number | Direction |
|---|---|---|
| Misses while braking into a shot, fine at constant speed | `Braking.horizonSeconds` (0.5) | Main knob. Sweep it live. |
| Braking model kicks in during normal driving | `revAmpedBrakeThreshold` (-100) | More negative. |
| Braking model drops out mid-stop | `revAmpedReleaseThreshold` (-75) | Toward -10 holds it longer. |
| Want it always on / never on | `revAmpedMode` | `2` = always, `1` = auto (default), `0` = off. |
| New drivetrain | `Braking.coefA` / `coefB` (0.00097587 / 0.11091) | **Still the old robot's.** Take the forward values from Pedro AutoTune: `quadraticBrakeCoefficients` → `coefA`, `linearBrakeCoefficients` → `coefB`. (`pedro/Constants` currently has quadratic `diag(0.00147, 0.00138)` and linear `diag(0.106, 0.087)`; check which entry is forward before copying.) |

Check `RevAmped braking active` on telemetry to see when it's on.

### 9. Rapid-fire (burst) — `ShooterCalc`
The burst comp is **off by default** (`burstCompEnabled = false`). Turn it on if balls 2+ of a burst fall short.
While feeding, it re-aims the hood for the slower ball the sagging flywheel throws. By default (`burstCompUseMeasured = true`) the sag is measured every loop from the flywheel's real speed vs its target, so there is no sag number to tune. The hood moves toward the angle that needs the least speed (for every current tuned point that means steeper), and if the ball is too slow to reach at any angle it stops at that angle instead of jumping to a hood limit. Watch `lastBurstSag` / `lastBurstHoodDeltaDeg` in Panels during a burst.

| Symptom | Number | Direction |
|---|---|---|
| Ball 1 lands, balls 2+ fall short | `burstCompEnabled` → `true` | Measured sag, nothing else to set. |
| Hood twitches on small dips | `burstCompDeadband` (0.01) | Raise (fraction of target speed ignored). |
| Prefer a fixed sag | `burstCompUseMeasured` → `false`, then `burstSpeedSag` (0.08) | 300 RPM drop at ~3700 RPM ≈ 0.08. |
| Ball 1 affected by the comp | `burstCompDelayMs` (30) | Raise (must stay under the time between balls). |

The comp's timer starts every time the robot starts shooting (`Robot.startShooting()`: right-trigger hold, and `Robot.shootCycle()` on A and in auto).

`velCompEnabled` (false) with `velCompToward` / `velCompAway` (0 / 0) adds a hood offset (servo units) when driving toward/away from the goal. Leave it off unless moving shots miss short/long in one direction only and stationary shots are fine.

### 10. Shot check (can it go in?) — `ShotCheck`
Every loop the planned shot is checked. **Shot** on telemetry shows `READY` or the reason it can't go in. With `Tele.REQUIRE_SHOT_ALLOWED` on (default), **TeleOp only shoots while the shot can go in**: holding the right trigger shoots, shooting pauses if the shot becomes impossible and resumes on its own. The gamepad rumbles while you try to shoot a shot that can't go in. Turn `REQUIRE_SHOT_ALLOWED` off to test shooting from anywhere.

With the shot map or tuned points in use (normal HIVE play), the check asks the map whether the spot scores:

| Reason on telemetry | What it means |
|---|---|
| shot map: no scoring shot from here | The map has no shot from this spot (or you're more than `OFF_GRID_MARGIN_IN`, 2 in, off the map grid). |
| shot map: shot too tight | The map's exit-speed window here is not over `MIN_SPEED_WINDOW_MPS`. Too risky. |
| no tuned shot point close enough | `MAP_FALLBACK` is off and no tuned point is within `MAX_POINT_DIST_IN`. |
| under the HIVE: holding fixed RPM / hood | See **Under-HIVE hold** below. |
| too far: flywheel can't spin that fast | Needs more than `MAX_FLYWHEEL_TPS`. |
| no trajectory reaches the aim point | The solver found no shot. |
| other alliance's HIVE is in the way | Only if `ALLOW_OTHER_HIVE_DEPENDENT` is off and this spot scores only with the other HIVE tilted one way. |
| turret can't rotate that far | Aim is outside `MIN_ANGLE`/`MAX_ANGLE` (can't happen with the default ±224.75). |

Only if neither a tuned point nor the map gives a shot (map off or failed to load) does it re-fly the ball against the window geometry instead, with the extra reasons: behind the cell opening, too close (ball still rising), too far to the side, opening narrower than the ball, HIVE frame in the way, past the flywheel speed fit.

| Number | Default | Change when |
|---|---|---|
| `BALL_DIAMETER` | 2.8 in (POLLEN) | 3.6 for NECTAR. |
| `MAX_FLYWHEEL_TPS` | 2700 (5800 rpm × 28 / 60) | Run the flywheel at full power and read its speed off telemetry. |
| `MIN_SPEED_WINDOW_MPS` | 0.075 m/s | Spots with a speed window this tight or tighter are refused (about 22% of scoring spots). 0 = no minimum. |
| `TABLE_SPEED_WINDOW_USE` | 0.8 | Share of the map's speed window the flywheel may use; the rest is left for hood error. |
| `ALLOW_OTHER_HIVE_DEPENDENT` | true | Off = refuse spots that only score with the other HIVE tilted a particular way. |
| `APERTURE_MARGIN` | 0.5 in | Allowed shots still miss the edges → raise. Too many spots rejected → lower. |
| `WINDOW_*`, `*_HIVE_*`, `HIVE_BOTTOM/TOP` | manual / visualizer | Field geometry for the fallback geometry check. Correct from the field CAD if a check is clearly wrong. |

The **geometry check** only runs when the shot came from neither a tuned point nor the map (tuned points switched off). It checks flywheel / turret limits, ball falling at the window, coming from the cell's side, the other HIVE in the way, and the window's room for error.

**Ready to shoot** (`Robot.readyToShoot()`) = the shot can go in **and** the turret command is within the turret tolerance **and** the flywheel is within the flywheel tolerance. It is shown on telemetry (`Shot: READY`) but does **not** stop TeleOp from shooting; only "can it go in" does. The turret part only checks that the turret command has caught up with the target (the servo has no feedback), not that the turret is really there.
- Turret tolerance = how much room the window leaves beside the ball, from where you are. The window looks narrower from the side.
- Flywheel tolerance = 0.8 × the map's speed window here, in ticks/s. Tuned points use `TunedShotTable.TUNED_SPEED_WINDOW_MPS` (0.15 m/s).

Watch **Tolerance turret deg / flywheel tps** on telemetry. With the defaults:
- Turret: ≈ 15° at 30 in, ≈ 8° at 60 in, ≈ 5° at 100 in (straight on).
- Flywheel: ≈ 48 ticks/s for a typical map spot, ≈ 32 ticks/s at a tuned point.

The HIVE shots are steep lobs, where a small speed error moves the ball a lot, so the flywheel loop (step 2) has to hold speed tightly.

---

## Shot map — `HiveAimTable`
For every release point on a 2 in grid (x and y from 10.5 to 132.5 in) and each cell (red/blue × audience/far), the map stores the launch angle and exit speed that score, the flight time and how much exit-speed error the shot tolerates. It came from a drag + spin simulation of the real POLLEN and the HIVE. The solver reads it at the release point it predicts (including shoot-on-the-move), so you don't fit any range curves.

| Number | Default | Change when |
|---|---|---|
| `ShooterCalc.USE_AIM_TABLE` | **false** (off since 2026-10-25: tuned points replaced it) | On = use the map again (also set `MAP_FALLBACK` on to use it where no tuned point is close). The map is not even loaded while off. |
| `HiveAimTable.MAX_BLEND_SPREAD_DEG` | 4.0° | Neighbouring grid spots are blended only when their launch angles are this close; across a bigger jump (lob vs direct shot) the nearest spot is used. |
| `HiveAimTable.OFF_GRID_MARGIN_IN` | 2.0 in | Further than this outside the grid counts as "no shot". |

`HiveAimTableData.java` is **generated** by `tools/export_hive_aim_table.py` from the simulation's map files. Never edit it by hand; re-run the script after re-running the simulation. Telemetry line **Aim table** shows the map's launch angle, speed and speed window here.

## Tuned shot points — `TunedShotTable` + `ShotPointTuner`
A tuned point is a spot where you found an RPM and hood that score. Within `MAX_POINT_DIST_IN` (18 in) of a tuned point, tuned points replace the map: up to `BLEND_POINTS` (4) nearest points are blended (weight 1 / distance^`BLEND_POWER`, 2), and closer than `SNAP_IN` (0.75 in) the point is used exactly. Shoot-on-the-move, the turret and the shot check keep working with them. Telemetry line **Shot source** shows `TUNED (...)` or `MAP (...)`.

Workflow:
1. Run **`ShotPointTuner`** (Shooter Test). In init, right bumper picks the alliance. The robot starts at `Tele.START_X / START_Y / START_HEADING_DEG`.
2. Drive or push the robot to a spot. RPM and hood start at what TeleOp would shoot there and follow it as you move.
3. Right trigger shoots (never blocked here), left trigger intakes, Y toggles the flywheel. Adjust:
   - d-pad up/down = RPM ± `RPM_STEP` (25); right/left bumper = ± `RPM_BIG_STEP` (100)
   - d-pad right/left = hood ± `HOOD_STEP` (0.005, up = steeper)
   - START = back to following the prediction
4. When the shots go in, press **A** to save the point (the turret position, your RPM and the hood position actually sent, which `Hood` caps at 0.75 even though the tuner lets you nudge to 1.0). It goes to `FIRST/shotpoints.csv` on the Hub, is used by the tuner's prediction right away, and by TeleOp the next time it inits. B deletes the nearest file point; X reloads the file. Saving under the HIVE is refused.
5. When a set is good: press **BACK** to print the points as Java lines (Driver Station + logcat tag `ShotPoints`), paste them into `TunedShotPoints.POINTS`, commit, then **hold BACK** for `CLEAR_HOLD_S` (3 s) to clear the file.

| Number | Default | Change when |
|---|---|---|
| `USE_TUNED_POINTS` | true | Off = shot map only. |
| `MAP_FALLBACK` | **false** | Off = no shot where no tuned point is close enough (`NO_TUNED_POINT`); flywheel and hood still follow the nearest tuned point, and driver 2's override (LT) can still shoot. On = shot map there (needs `USE_AIM_TABLE`). |
| `MAX_POINT_DIST_IN` | 18 in | How far a tuned point reaches. |
| `ONE_CELL_FOR_ALL` | true | Every point is stored for the red audience cell and mirrored to the other three: red far = flipped across y = 72, blue audience = flipped across x = 72, blue far = both. Off = tune cells separately. |
| `MIRROR_BLUE` | true | Only with `ONE_CELL_FOR_ALL` off: blue uses the red points turned 180°. |
| `REPLACE_RADIUS_IN` | 3 in | Saving removes file points this close; a file point hides code points this close. |
| `RPM_TRIM` / `HOOD_TRIM` | 0 / 0 | Shift every tuned point (e.g. a new batch of balls). |

## Under-HIVE hold — `ShooterConstants`
While the robot pose is inside the box under both HIVEs (`UNDER_HIVE_X_MIN/MAX` 47.27–96.73, `UNDER_HIVE_Y_MIN/MAX` 46.7–97.3, Pedro inches), the solver is skipped:
- The flywheel holds `UNDER_HIVE_RPM` (3300 motor RPM) and the hood holds `UNDER_HIVE_LAUNCH_DEG` (72°), so they're ready on the way out.
- The turret keeps pointing at the cell.
- Shooting is blocked (`Shot: under the HIVE ...`).

The map has no shots in there anyway. Turn it off with `UNDER_HIVE_HOLD = false`.

---

## Telemetry to watch (TeleOp)
- **Shot** — `READY`, or why the shot can't go in (section 10).
- **Tolerance turret deg / flywheel tps** — how much aim/speed error this spot allows.
- **Shot source** — `TUNED` (which points) or `MAP`.
- **Aim table** — the map's launch angle, exit speed and speed window here; `NO SHOT HERE` if it doesn't score.
- **Aim** — which target is selected (HIVE far / audience cell), and `UNDER HIVE` when the hold is on.
- **Range now / projected / virtual** — if projected ≠ now while standing still, the velocity is noisy.
- **Score height / entry** — what the solver was given (with the map on, a "virtual" height with drag folded in, usually above the real window).
- **Flight time**, **Solver iters / converged** — solver health.
- **Speed projected / aAlongV** — what the braking logic sees.
- **Hood comp burst deg / vel** — how much the hood comps are adding.
- **Shooter** — motor RPM / target RPM, at speed / spinning up / off, mode (PIDF / BANG_BANG / OFF) and power. Not reaching target → step 2. Panels graphs: Motor RPM, Target RPM, RPM error, Shooter power, Battery V.

# Shooter / Turret Tuning Guide

Which number to change, where it lives, and what it fixes.

> **Testing on the old (Decode) robot.** Every number borrowed from the old robot is marked `// TODO(new robot): old-robot value` in the source. When the new robot is ready, search the project for `TODO(new robot)` and retune each one (Android Studio: Edit > Find > Find in Files). Everything marked `public static` can be changed live in Panels, but **Panels changes are lost on restart: copy the final value back into the Java file.**

Files (all under `teamcode/`):

| File | What's in it |
|---|---|
| `subsystem/Shooter.java` | Flywheel velocity loop (kS / kF / kP) |
| `subsystem/Hood.java` | Hood servo limits |
| `subsystem/Turret.java` | Turret pivot position on the robot, servo write threshold |
| `subsystem/control/ShooterConstants.java` | Where the goals / HIVE cells are on the field |
| `subsystem/control/pasted/ShooterCalc.java` | The shot solver (pasted HIVE logic): aim height, entry angle, lead, hood comp. **All solver tunables live here** |
| `subsystem/control/pasted/TurretCalc.java` | Turret angle limits, slew, per-alliance trim, yaw feedforward. **All turret tunables live here** |
| `subsystem/control/Braking.java` | Braking model used to predict where the robot stops |
| `subsystem/control/shotcheck/ShotCheck.java` | Can this shot go in from here? Aim / flywheel tolerances for "ready" |

### Two versions of the shooter logic
| Package | What it is |
|---|---|
| `subsystem/control/pasted/` | The copy-pasted HIVE logic as-is. Its `ShotCheck` is a stand-in: every shot allowed, no rumble, "ready" = flywheel within `Shooter.VELOCITY_TOLERANCE`. |
| `subsystem/control/shotcheck/` | **Default.** The same solver and turret math (these classes extend the pasted ones) plus the shot-possible check, computed tolerances and rumble (section 10). |

**To switch:** in `Robot.java` (3 imports) and `tele/Tele.java` (2 imports), replace `control.shotcheck.` with `control.pasted.` (or back). Nothing else changes. Both versions share the same tunables, so Panels shows one `ShooterCalc` and one `TurretCalc` group, and tuning carries over when you switch. `Braking` and `ShooterConstants` are shared by both.

If you change the pasted solver's math, copy the change into `shotcheck/ShooterCalc.solve()` too. It is the pasted `solve()` plus four lines at the end that record the ShotCheck inputs.

---

## Retune order (new robot / big mechanical change)

Do these in order. Each step assumes the ones before it are right.

### 1. Turret geometry — `TurretCalc`, `Turret`
| Number | Default | Change when |
|---|---|---|
| `TurretCalc.MIN_ANGLE` / `MAX_ANGLE` | -224.75 / 224.75 | Command 0.05 and 0.95 servo position and measure the turret angle at each. The servo range is mapped linearly between these two. If the turret overshoots/undershoots a commanded angle by a growing amount, these are wrong. |
| `Turret.TURRET_FORWARD_OFFSET` / `TURRET_STRAFE_OFFSET` | -0.526 / 0 | From CAD: turret pivot distance from the Pinpoint/Pedro pose point, inches (forward +, left +). |
| `Turret.SERVO_THRESHOLD` | 0.0002 | Only writes the servo if the position moved more than this. Raise if the turret buzzes while still; lower if it moves in visible steps. |

### 2. Flywheel speed loop — `Shooter`
| Number | Default | Change when |
|---|---|---|
| `kS` | 0.2 | Power to just overcome friction. |
| `kF` | 0.000449 | Power per tick/s. Tune first with kP = 0: flywheel should settle close to target on its own. |
| `kP` | 0.009 | Raise if it recovers slowly after a shot; lower if it oscillates. Power can go negative, so kP also brakes the wheel down when the target drops. |
| `VELOCITY_TOLERANCE` | 100 | How close counts as `atTarget()`. |

### 3. Hood & flywheel maps (code edits, not Panels) — `pasted/ShooterCalc`
These convert the physics answer into hardware commands. **They are still the old robot's fits.**
- **Ball speed → ticks/s:** `tps = -16.19*v² + 449.11*v - 964.9` (v in m/s). Refit by measuring ball exit speed at several flywheel speeds.
- **Hood angle → servo position:** `hoodTime = 0.01625*deg - 0.6`. Refit by measuring the launch angle at several servo positions.
- **`HOOD_MIN` / `HOOD_MAX`** (46° / 80°, `private static final`): the real hood's launch-angle limits.
- `Hood.HOOD_MIN_POS` / `HOOD_MAX_POS` (0.05 / 1.0): servo clamps, so the hood never hits its hard stops.

### 4. Where you're aiming — `ShooterConstants`
| Number | Default | Change when |
|---|---|---|
| `HIVE_RED_TOP_X/Y`, `HIVE_RED_BOTTOM_X/Y`, `HIVE_BLUE_TOP_X/Y`, `HIVE_BLUE_BOTTOM_X/Y` | see file | Aim point for each HIVE cell (Pedro inches). Consistent left/right miss at **one** cell only → move that cell. |
| `HIVE_SIDE_SPLIT_Y` | 72 | Robot y where it switches from audience cell to far cell. |
| `HIVE_SIDE_HYSTERESIS` | 3 | Raise if the aim flips between cells when sitting near the middle. |
| `GOAL_POS_RED` / `GOAL_POS_BLUE` | (140, 140.5) / (3, 140.5) | DECODE goal, only used when `hiveMode = false`. |

### 5. Stationary shots — `ShooterCalc`
Shoot from a standstill at several ranges first. Don't touch the moving-shot numbers until these land.

| Symptom | Number | Direction |
|---|---|---|
| Short/long at **every** range | `hiveHeightTrim` (0) | Short → raise. Long → lower. |
| Short/long, trim doesn't fix it | `launchHeight` (14) | Measure shooter exit height above the tiles. |
| Balls bounce off the back of the HIVE and out | `hiveEntryDeg` (-50) | Try -40 (steeper drop onto the floor). |
| Aim height is wrong for the real HIVE | `hiveAimHeight` (65) | Window centroid height above the tiles, inches. |
| Only short at close range / only long far | Step 3 maps are wrong | Refit the maps; don't fight it with trims. |
| Constant speed error everywhere | `verticalShift` (0) | Ticks/s added to every flywheel command. |

`hiveMode` (true) switches between HIVE and the old DECODE curves. With it off, `targetHeight()`/`entryAngle()` use the DECODE regression and `SCORE_ANGLE`.

### 6. Turret aim — `TurretCalc`
| Symptom | Number | Direction |
|---|---|---|
| Consistently left/right on **red** | `turretOffset` (15) | Degrees added to the turret command. |
| Consistently left/right on **blue** | `turretOffset2` (10) | Same, blue only. |
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
| Lead is right in range but wrong sideways | `sotmFactor` (1.0) | Scales only the turret's lead angle. |
| Moving shots short/long only (not stationary) | `sotmOffset` (0) | Ticks/s removed from the flywheel only while moving. |
| Time from latch-open to ball leaving is different | `feederDelay` (0.1 s) | Measure on high-fps video. |
| Turret still arriving when the ball leaves | `turretSettleTime` (0.03 s) | Raise. |
| Turret jitters when turning in place (fake speed) | `sotmMinSpeed` (12 in/s) | Raise. Below this speed the shot is treated as stationary. |
| Misses while driving a curve | `compensateTurning` (true) | Leave on. Tune `leadGain` first. |

Accel filtering (usually leave alone): `accelFilterAlphaDecel` (0.5), `accelFilterAlphaAccel` (0.3), `positiveAccelGain` (0.5), `slopeWindowSeconds` (0.15), `maxProjectedSpeed` (125), `accelClamp` (400).

### 8. Shooting while braking — `ShooterCalc`, `Braking`
| Symptom | Number | Direction |
|---|---|---|
| Misses while braking into a shot, fine at constant speed | `Braking.horizonSeconds` (0.5) | Main knob. Sweep it live. |
| Braking model kicks in during normal driving | `revAmpedBrakeThreshold` (-100) | More negative. |
| Braking model drops out mid-stop | `revAmpedReleaseThreshold` (-75) | Toward -10 holds it longer. |
| Want it always on (auto) | `revAmpedMode` | `2` = always, `1` = auto (teleop), `0` = off. |
| New drivetrain | `Braking.coefA` / `coefB` (0.00097587 / 0.11091) | **Still the old robot's.** Take the forward values from Pedro AutoTune: `quadraticBrakeCoefficients` → `coefA`, `linearBrakeCoefficients` → `coefB`. |

Check `RevAmped braking active` on telemetry to see when it's on.

### 9. Rapid-fire (shootAll) — `ShooterCalc`
| Symptom | Number | Direction |
|---|---|---|
| Ball 1 lands, balls 2+ fall short | `burstSpeedSag` (0.03) | Raise. Log flywheel TPS during a burst to measure the real drop. |
| Balls 2+ now go long | `burstSpeedSag` | Lower. |
| Ball 1 affected by the comp | `burstCompDelayMs` (30) | Raise (must stay under the time between balls). |
| Turn it off | `burstCompEnabled` | `false`. |

`velCompToward` / `velCompAway` (0 / 0) add a hood offset when driving toward/away from the goal. Leave them at 0 unless moving shots miss short/long in one direction only and stationary shots are fine.

### 10. Shot check (can it go in?) — `shotcheck/ShotCheck` (shotcheck version only)
Every loop the planned shot is re-flown in field coordinates and checked. **Shot** on telemetry shows `READY` or the reason it can't go in; the gamepad rumbles while you shoot a shot that can't go in (it still fires).

| Reason on telemetry | What it means |
|---|---|
| behind the cell opening | The cell you're aimed at doesn't face you (beside the HIVE, or past the cell). Drive to the cell's end of the field. |
| too close: ball still rising | Right under/in front of the cell; even max hood can't drop it in. Back off. |
| no trajectory reaches the aim point | Too close for any hood angle. |
| too far: past the flywheel speed fit | Needs a ball faster than the fitted speed map covers (top of its parabola). |
| too far: flywheel can't spin that fast | Needs more than `MAX_FLYWHEEL_TPS`. |
| too far to the side / opening narrower than the ball | Too edge-on to the opening. |
| other alliance's HIVE / frame in the way | The arc hits the other HIVE or a frame end plate. |

| Number | Default | Change when |
|---|---|---|
| `BALL_DIAMETER` | 5.0 (DECODE ball) | **2.8 for POLLEN on the new robot** (3.6 for NECTAR). |
| `MAX_FLYWHEEL_TPS` | 2380 (placeholder) | Run the flywheel at full power and read its speed off telemetry. |
| `APERTURE_MARGIN` | 0.5 in | Allowed shots still miss the edges → raise. Too many spots rejected → lower. |
| `MAX_INCIDENCE_DEG` | 70 | Steep side shots bounce off the cell's side walls → lower. |
| `WINDOW_*`, `*_HIVE_*`, `FRAME_*`, `HIVE_BOTTOM/TOP` | manual / visualizer | Field geometry. Correct from the field CAD if a check is clearly wrong. |

**Ready to shoot** (`Robot.readyToShoot()`) = the shot can go in **and** the turret command is within the turret tolerance **and** the flywheel is within the flywheel tolerance. Both tolerances are computed each loop from how much room the opening leaves around the ball from where you are, so they already account for range, side angle and shoot-on-the-move. Watch **Tolerance turret deg / flywheel tps** on telemetry. Typical values for HIVE lobs: turret 3–20°, flywheel **±30–55 ticks/s**. The HIVE shots are steep lobs, where a small speed error moves the ball a lot, so the flywheel loop (step 2) has to hold speed tightly.

Where a stationary RED robot can shoot from (old-robot numbers, DECODE ball; 6 in per character, top = far wall, left = x 0). `.` ok, `#` frame, `b` behind the opening, `c` too close, `X` no trajectory, `e` too edge-on, `H` blue HIVE in the way. BLUE is the mirror image.
```
  117 ........................
  111 ...............H........
  105 ........cccc..HHHHHHH...
   99 .......ccXXc.HHHHHHHH...
   93 ......ecXXXccHHHHHHHH...
   87 bbbbbbbc########bbbbbbbb
   81 bbbbbbbc########bbbbbbbb
   75 bbbbbbbb########bbbbbbbb
   69 bbbbbbbb########bbbbbbbb
   63 bbbbbbbc########bbbbbbbb
   57 bbbbbbbc########bbbbbbbb
   51 ......ecXXXccHHHHHHHH...
   45 .......ccXXccHHHHHHHH...
   39 ........cccc..HHHHHHH...
   33 ........................
```
(Rows above 117 and below 33 are all `.`.)

---

## Telemetry to watch (TeleOp)
- **Shot** — `READY`, or why the shot can't go in (section 10).
- **Tolerance turret deg / flywheel tps** — how much aim/speed error this spot allows.
- **Range now / projected / virtual** — if projected ≠ now while standing still, the velocity is noisy.
- **Aim** — which target is selected (HIVE far / audience / DECODE).
- **Flight time**, **Solver iters / converged** — solver health. Not converging → maps from step 3 are off.
- **Speed projected / aAlongV** — what the braking logic sees.
- **Hood comp burst deg / vel** — how much the hood comps are adding.
- **Shooter x / y ticks/s** — flywheel tracking. Not reaching target → step 2.

# ElectroLarp-Biobuzz project index

Generated 2026-11-27, source HEAD `7e77c7c9d1fb5dd3929f9bdb5c76f0d839859efe`. Source files were inspected directly.

## Coverage

- 197 tracked non-sensitive paths inventoried.
- 67 TeamCode Java files, 70 SDK/vendor Java files.
- 811 method/constructor declarations; 154 candidate internal dependency edges.
- [Interactive mind map](mindmap.html), [Mermaid mind map](MINDMAP.md), [machine-readable inventory and symbols](index.json).
- [Regeneration script](../../scripts/generate_context_index.py): run `python scripts/generate_context_index.py` from this checkout. It writes local documents only; Atlas refresh is a separate explicit operation.

## Architecture

`redClose` / `blueClose` and `redFar` / `blueFar` select an alliance and inherit autonomous behavior.
`teleRed` / `teleBlue` select an alliance and inherit `Tele` controls.
`CommandOpMode` runs the Ivy scheduler inside the FTC LinearOpMode lifecycle.
`Robot` constructs the follower and six mechanisms, and owns coordinated shooting and aiming.
Each `Robot.update()` clears bulk caches, updates the follower, updates aiming, then updates the shooter.
`Constants.create()` builds a PulseFollower using Pinpoint localization, Foresight and Mecanum (or NoDrivetrain).
`ShooterCalc` selects tuned/table shots and computes targets; `TurretCalc` maps turret aim; `ShotCheck` gates shooting.
`OpModeStorage.endPose` carries the last pose into a later OpMode when enabled.

## Current source facts

- Intake and transfer run together in `Intake`; the transfer motor is optional. There is no separate Transfer class.
- Mechanism actions are ordinary methods in this snapshot. `Robot.shootCycle()` wraps startShooting / delay / stopShooting in Ivy commands.
- Close autonomous uses Ivy groups and electropathing follow/pose callbacks. No direct pulse-lumen dependency is declared.
- Far autonomous and its start pose are placeholders. Close and Far use different alliance-mirroring conventions; verify geometry before field use.
- Flower and latch are optional hardware. Turret supports the temporary four-servo configuration.
- TunedShotTable combines committed points with a Control Hub CSV; this index does not read that device file.
- Historical tuning prose can be stale: current `Tuning.java` registers four tuners despite an older section saying it is blank.

## Reading routes

| Task | Start here |
|---|---|
| Autonomous paths / flower callback | auto/Close.java, auto/paths/ClosePaths.java, subsystem/Flower.java |
| Transfer / intake behavior | subsystem/Intake.java, Robot.startShooting / stopShooting |
| Driver controls | tele/Tele.java, util/input/GamepadEx.java |
| Shot accuracy | subsystem/aim/shot/ShooterCalc.java, subsystem/aim/table/TunedShotTable.java |
| Flywheel control | subsystem/Shooter.java, util/control/PIDController.java, test/FlywheelPIDFTest.java |
| Turret geometry | subsystem/Turret.java, subsystem/aim/turret/TurretCalc.java |
| Localization / drive | pedro/Constants.java, pedro/Tuning.java, pedro/procedures/ |
| Pose handoff / lifecycle | util/CommandOpMode.java, util/OpModeStorage.java |
| Device configuration | HardwareNames.java, configs/LightningLM1.xml |
| Team workflow | docs/git.md, .githooks/commit-msg, .github/workflows/ |

Paths in this table are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, except build/config/workflow paths.

## Direct dependencies

- `org.firstinspires.ftc:Inspection:12.0.0`
- `org.firstinspires.ftc:Blocks:12.0.0`
- `org.firstinspires.ftc:RobotCore:12.0.0`
- `org.firstinspires.ftc:RobotServer:12.0.0`
- `org.firstinspires.ftc:OnBotJava:12.0.0`
- `org.firstinspires.ftc:Hardware:12.0.0`
- `org.firstinspires.ftc:FtcCommon:12.0.0`
- `org.firstinspires.ftc:Vision:12.0.0`
- `androidx.appcompat:appcompat:1.2.0`
- `com.pedropathing:revhub:3.0.2`
- `com.pedropathing:tuning:1.0.1`
- `com.pedropathing.ivy:pedro:1.1.1`
- `com.pedropathing:telemetry:1.0.0`
- `org.electrolightsftc:pulse-electropathing:2.1.2`
- `org.electrolightsftc:pulse-lumen:2.1.0`
- `dev.frozenmilk.sinister:Sloth:0.3.2`
- `com.bylazar.sloth:fullpanels:0.3.2+1.0.13`

## Build and repository structure

- `TeamCode/`: robot behavior, mechanisms, tuners, tests, Android resources.
- `FtcRobotController/`: FTC application host, SDK samples and utility OpModes; indexed separately as vendor support.
- `settings.gradle`: includes both Android modules.
- `build.gradle`, `build.common.gradle`, `build.dependencies.gradle`: plugins, shared Android configuration, declared dependencies.
- `TeamCode/build.gradle`: team module, Sloth plugin and project-local Git settings during Gradle evaluation.
- `configs/`: robot hardware configuration XML; `libs/`: bundled support artifacts; `doc/`: upstream/legal material.
- `.github/workflows/` and `.githooks/`: commit rules and pull-request automation.

## Validation and limits

Inventory and symbols were generated from tracked files, with source hashes for freshness checks.
Comments are removed before declaration extraction, preserving source line numbers. Signatures and dependency edges are heuristic.
No compile, deployment, robot run or dependency resolution was performed for this documentation task.
Local settings, credentials, keystores, hidden tool state, build output and binary contents are excluded.
Atlas stores generated summaries and symbol outlines, not complete raw source files. Re-read current sources before editing.

## TeamCode file and symbol catalog

### HardwareNames.java

Central robot configuration device names.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java); 44 lines; SHA-256 `1409aab0675e626cbe69f50a0db84d089dab8977bfe88da7b07c16f7277df8e9`.

Types: `HardwareNames` (class, line 6)

- L7: `private HardwareNames()`

Internal dependency candidates: none detected

### Robot.java

Owns hardware and follower; updates aiming and flywheel; coordinates shooting, manual fallback and pose reset.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java); 304 lines; SHA-256 `eb5b3074f17bfc12fd4935da2c9afef202b92c96ba07b518873088116cac76a8`.

Types: `Robot` (class, line 36)

- L71: `public Robot(HardwareMap hardwareMap, Alliance alliance)`
- L92: `public void update()`
- L108: `public void setDriverSticks(double forward, double lateral, double turn, double forwardOffsetRad)`
- L112: `public double stickBlendDeg()`
- L114: `private void logShot(long nowNanos)`
- L141: `public void manualLatch(boolean open)`
- L145: `public void setTurretOffsetOverride(double offsetDeg)`
- L149: `public void setFixedTurretAngle(double angleDeg)`
- L153: `public Double fixedTurretAngleDeg()`
- L155: `private double solveTurret(double angleDeg, double angularVelocity)`
- L164: `private void updateAiming()`
- L224: `public boolean isManual()`
- L226: `public boolean isTurretForward()`
- L227: `public void setTurretForward(boolean on)`
- L229: `public void setManual(boolean on)`
- L233: `public double manualTurretDeg()`
- L234: `public void nudgeManualTurret(double deg)`
- L239: `public static Pose resetPose(Alliance alliance)`
- L243: `public void resetPose()`
- L247: `public boolean underHive()`
- L252: `public void startTeleOpDrive()`
- L257: `public void startShooting()`
- L264: `public void stopShooting()`
- L272: `public boolean isShooting()`
- L277: `public Command shootCycle()`
- L285: `public void flower(boolean down)`
- L289: `public void stop()`
- L300: `public double loopMs()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Drivetrain.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/StickLead.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/FollowerLocalization.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ShotLogger.java`

### auto/Close.java

Near-side autonomous sequence: shooting, pollen loading, flower approach and retreat.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java); 165 lines; SHA-256 `28eb886e3c424d105daed5eee1663c85b7939d5cd4d435587823ef6ebb541e6c`.

Types: `Close` (class, line 25)

- L42: `public Close(Alliance a)`
- L47: `public void initialize()`
- L57: `public void initializeLoop()`
- L65: `public void onStart()`
- L73: `public void execute()`
- L105: `public void end()`
- L113: `private Command routine()`
- L148: `private Command shootCycle()`
- L161: `private Command shootCycle2()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`

### auto/OneTip.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/OneTip.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/OneTip.java); 84 lines; SHA-256 `bf3cc6dd95a2ccaaf61e9409faad8035efb9a2922aa6f1773244cfa9d1d14b7d`.

Types: `OneTip` (class, line 19)

- L27: `public OneTip(Alliance a)`
- L32: `public void initialize()`
- L40: `public void initializeLoop()`
- L47: `public void onStart()`
- L53: `private void holdRpm()`
- L58: `public void execute()`
- L63: `public void end()`
- L71: `private Command routine()`
- L80: `private Command shootCycle()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/OneTipPaths.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`

### auto/blue/blue1tip.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blue1tip.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blue1tip.java); 13 lines; SHA-256 `3c54b2ceff69dff045b1cf6ce5474cafc88b0cedd0bc13a179c489845e628080`.

Types: `blue1tip` (class, line 9)

- L10: `public blue1tip()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/OneTip.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/blue/blueClose.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueClose.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueClose.java); 13 lines; SHA-256 `740d69129f0313df7955deb8b7903a1349e87fa7129cb8b718d39dd82f8c9d93`.

Types: `blueClose` (class, line 9)

- L10: `public blueClose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/paths/ClosePaths.java

Near-side paths, heading interpolation, mirrored poses and flower pose callback.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java); 199 lines; SHA-256 `02d1114f22de7d978097e00228ab1d5c8b0873cd702e286cb9edf23c523074fc`.

Types: `ClosePaths` (class, line 19), `Poses` (class, line 160)

- L20: `private ClosePaths()`
- L24: `public static PoseFactory poses(Alliance a)`
- L29: `public static Pose startPose(Alliance a)`
- L35: `public static Path intakeLoading(Alliance a)`
- L43: `public static Path goNearFlower(Alliance a, Command openFlower)`
- L54: `public static Path approachFlower(Alliance a)`
- L58: `public static Path finishFlower(Alliance a)`
- L63: `private static Path flowerLeg(Alliance a, boolean approach)`
- L77: `public static Path park(Alliance a)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/paths/OneTipPaths.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/OneTipPaths.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/OneTipPaths.java); 60 lines; SHA-256 `fb339ea3e6f2dd3366fd26f1ce84ca057a8b5c3e1046cced5eaadfc0b2f328dd`.

Types: `OneTipPaths` (class, line 11), `Poses` (class, line 45)

- L12: `private OneTipPaths()`
- L19: `public static PoseFactory poses(Alliance a)`
- L25: `private static Pose of(Alliance a, double x, double y, double headingDeg)`
- L30: `public static Pose startPose(Alliance a)`
- L34: `public static Path path1(Alliance a)`
- L39: `public static Path path2(Alliance a)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/red/red1tip.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/red1tip.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/red1tip.java); 13 lines; SHA-256 `efbcff506bde42a9d86e375b9ab891c664a3735412b8451a4f3bd8f01e428670`.

Types: `red1tip` (class, line 9)

- L10: `public red1tip()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/OneTip.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/red/redClose.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redClose.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redClose.java); 13 lines; SHA-256 `489f1823321533c36319bd4766448fb70337474b62b4ae21f6efc2a12b3d20ab`.

Types: `redClose` (class, line 9)

- L10: `public redClose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### pedro/Constants.java

Builds PulseFollower from Pinpoint localization, Mecanum or NoDrivetrain, and Foresight configuration.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java); 96 lines; SHA-256 `c2d437440fe3bd744a8653196d6434b98168393cb433e0a19f104291e24f9352`.

Types: `Constants` (class, line 23)

- L83: `public static PulseFollower create(HardwareMap h)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java`

### pedro/NoDrivetrain.java

Drivetrain fallback when the four drive motors are absent.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java); 16 lines; SHA-256 `4e98c00b715b2fa89c0dc41923a448b916fe7e481212c2a03f5cea64f9ea0bd9`.

Types: `NoDrivetrain` (class, line 9)

- L10: `public void drive(DrivePowers powers, boolean useBrake)`
- L11: `public double maxScaling(DrivePowers a, DrivePowers b)`
- L12: `public void stop()`
- L13: `public void stop(boolean useBrake)`
- L15: `public double interpolateVelocity(double a, double b, double c)`

Internal dependency candidates: none detected

### pedro/Tuning.java

Registers AutoTune procedures: Mecanum, Pinpoint, Foresight and Tests.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Tuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Tuning.java); 42 lines; SHA-256 `7c61f05fc65a658ff6347e176d45b5fb2f75496d9570357bc50bfaf97124db61`.

Types: `Tuning` (class, line 17)

- L19: `public static Procedure mecanumTuner()`
- L24: `public static Procedure pinpointTuner()`
- L29: `public static Procedure foresightTuner()`
- L36: `public static Procedure tests()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ForesightTuner.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/MecanumTuner.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/PinpointTuner.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/Tests.java`

### pedro/procedures/ForesightTuner.java

AutoTune procedure: ForesightTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ForesightTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ForesightTuner.java); 1253 lines; SHA-256 `ed707ce17eb7eab9a331b58a1c7601ca800c6b112cac92d39ce3f96d9a80c0ca`.

Types: `ForesightTuner` (class, line 22), `ForwardVelocity` (class, line 125), `StrafeVelocity` (class, line 183), `ForwardDeceleration` (class, line 241), `StrafeDeceleration` (class, line 332), `HeadingBraking` (class, line 423), `State` (enum, line 540), `HeadingTuner` (class, line 568), `ForwardBraking` (class, line 681), `State` (enum, line 830), `StrafeBraking` (class, line 849), `State` (enum, line 998), `ForwardTranslational` (class, line 1017), `StrafeTranslational` (class, line 1137)

- L26: `public ForesightTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L33: `public void run() throws InterruptedException`
- L132: `public ForwardVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance)`
- L140: `protected Double runTuningOpMode() throws InterruptedException`
- L190: `public StrafeVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance)`
- L198: `protected Double runTuningOpMode() throws InterruptedException`
- L252: `public ForwardDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity)`
- L261: `protected Double runTuningOpMode() throws InterruptedException`
- L343: `public StrafeDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity)`
- L352: `protected Double runTuningOpMode() throws InterruptedException`
- L450: `public HeadingBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L458: `protected List<Double> runTuningOpMode() throws InterruptedException`
- L546: `private static double[] biasedGradient( int count, double max, double min, double bias )`
- L588: `public HeadingTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L595: `protected Double runTuningOpMode() throws InterruptedException`
- L644: `private double calculatekP(double alpha)`
- L650: `private void systemIdentification()`
- L707: `public ForwardBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double headingLinear, double headingQuadratic, double headingKP, double distance)`
- L719: `protected List<Double> runTuningOpMode() throws InterruptedException`
- L785: `private double getHeadingPower(Localizer localizer)`
- L795: `private void drive(Drivetrain drivetrain, Localizer localizer)`
- L799: `private void brake(Drivetrain drivetrain, Localizer localizer)`
- L813: `private void collectTrialData(Localizer localizer, Drivetrain drivetrain)`
- L837: `private static double[] biasedGradient(int count, double max, double min, double bias)`
- L875: `public StrafeBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double headingLinear, double headingQuadratic, double headingKP, double distance)`
- L887: `protected List<Double> runTuningOpMode() throws InterruptedException`
- L954: `private double getHeadingPower(Localizer localizer)`
- L963: `private void drive(Drivetrain drivetrain, Localizer localizer)`
- L967: `private void brake(Drivetrain drivetrain, Localizer localizer)`
- L981: `private void collectTrialData(Localizer localizer, Drivetrain drivetrain)`
- L1005: `private static double[] biasedGradient(int count, double max, double min, double bias)`
- L1039: `public ForwardTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L1046: `protected List<Double> runTuningOpMode() throws InterruptedException`
- L1100: `private double calculatekP(double alpha)`
- L1106: `private void systemIdentification()`
- L1157: `public StrafeTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L1164: `protected List<Double> runTuningOpMode() throws InterruptedException`
- L1217: `private double calculatekP(double alpha)`
- L1223: `private void systemIdentification()`

Internal dependency candidates: none detected

### pedro/procedures/MecanumTuner.java

AutoTune procedure: MecanumTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/MecanumTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/MecanumTuner.java); 96 lines; SHA-256 `78626e97be611a2190552370c5b4a1e393e38ff9831cdd206b428137020830c3`.

Types: `Direction` (enum, line 7), `MecanumTuner` (class, line 12), `SpinMotor` (class, line 77)

- L13: `public MecanumTuner()`
- L18: `public void run() throws InterruptedException`
- L54: `private Direction testMotor(Wheel wheel, String displayName, String hardwareName) throws InterruptedException`
- L80: `public SpinMotor(String displayName, String hardwareName)`
- L87: `protected Void runTuningOpMode()`

Internal dependency candidates: none detected

### pedro/procedures/OTOSTuner.java

AutoTune procedure: OTOSTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/OTOSTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/OTOSTuner.java); 210 lines; SHA-256 `3c8898501af8e1fbea6b9f6169a8f0e666dc8fe67a3ffc41fb9220fda1c80572`.

Types: `OTOSTuner` (class, line 15), `OTOSLinearScalar` (class, line 69), `OTOSAngularScalar` (class, line 113), `OTOSOffsets` (class, line 166)

- L16: `public OTOSTuner()`
- L21: `public void run() throws InterruptedException`
- L73: `public OTOSLinearScalar(String name, double distance)`
- L83: `protected Double runTuningOpMode()`
- L118: `public OTOSAngularScalar(String name, int turns)`
- L129: `protected Double runTuningOpMode()`
- L170: `public OTOSOffsets(String name, double linearScalar, double angularScalar)`
- L181: `protected List<Double> runTuningOpMode()`

Internal dependency candidates: none detected

### pedro/procedures/OctoQuadTuner.java

AutoTune procedure: OctoQuadTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/OctoQuadTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/OctoQuadTuner.java); 350 lines; SHA-256 `be98632e9388a766188b4eb54d025b55e006ea8fd7e186e36683a2fd4183a2e9`.

Types: `OctoQuadTuner` (class, line 16), `PodType` (enum, line 17), `OctoQuadHeadingScalar` (class, line 91), `OctoQuadCustomPodScalar` (class, line 140), `OctoQuadForwardDirection` (class, line 188), `OctoQuadStrafeDirection` (class, line 237), `OctoQuadOffsets` (class, line 287)

- L26: `public OctoQuadTuner()`
- L31: `public void run() throws InterruptedException`
- L99: `public OctoQuadHeadingScalar(String name, int turns, int xPodPort, int yPodPort)`
- L111: `protected Double runTuningOpMode() throws InterruptedException`
- L145: `public OctoQuadCustomPodScalar(Double distance, String name, int xPodPort, int yPodPort)`
- L157: `protected Double runTuningOpMode()`
- L195: `public OctoQuadForwardDirection(String name, OctoQuadTuner.PodType podType, double customPodScalar, double headingScalar, int xPodPort, int yPodPort)`
- L210: `protected Boolean runTuningOpMode()`
- L244: `public OctoQuadStrafeDirection(String name, OctoQuadTuner.PodType podType, double customPodScalar, double headingScalar, int xPodPort, int yPodPort)`
- L259: `protected Boolean runTuningOpMode()`
- L296: `public OctoQuadOffsets(String name, OctoQuadTuner.PodType podType, double customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed, double headingScalar, int xPodPort, int yPodPort)`
- L313: `protected List<Double> runTuningOpMode()`

Internal dependency candidates: none detected

### pedro/procedures/PinpointTuner.java

AutoTune procedure: PinpointTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/PinpointTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/PinpointTuner.java); 268 lines; SHA-256 `ffffcc842c736f1d18879cbde2e12824eb463bf93ac6f405322e09e7be1bf157`.

Types: `PinpointTuner` (class, line 14), `PodType` (enum, line 15), `PinpointCustomPodScalar` (class, line 72), `PinpointForwardDirection` (class, line 109), `PinpointStrafeDirection` (class, line 154), `PinpointOffsets` (class, line 198)

- L20: `public PinpointTuner()`
- L25: `public void run() throws InterruptedException`
- L77: `public PinpointCustomPodScalar(Double distance, String name)`
- L87: `protected Double runTuningOpMode() throws InterruptedException`
- L114: `public PinpointForwardDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar)`
- L125: `protected Boolean runTuningOpMode() throws InterruptedException`
- L159: `public PinpointStrafeDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar)`
- L170: `protected Boolean runTuningOpMode() throws InterruptedException`
- L205: `public PinpointOffsets(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed)`
- L220: `protected List<Double> runTuningOpMode() throws InterruptedException`

Internal dependency candidates: none detected

### pedro/procedures/Tests.java

AutoTune procedure: Tests.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/Tests.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/Tests.java); 485 lines; SHA-256 `1da8c76374932336672833f84b526e1d104d0974deffb075f47e33ad6fbfd0be`.

Types: `Tests` (class, line 25), `Test` (enum, line 26), `TestsHold` (class, line 132), `TestsLine` (class, line 156), `TestsCurve` (class, line 198), `TestsInterpolation` (class, line 240), `TestsLocalization` (class, line 282), `TestsOdometry` (class, line 313), `Test` (enum, line 317), `TestsDriving` (class, line 442), `TestsPose` (class, line 461)

- L49: `public Tests(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction, Supplier<Algorithm> algorithmSupplier)`
- L57: `public void run() throws InterruptedException`
- L135: `public TestsHold(Function<HardwareMap, Follower> followerFunction)`
- L141: `public Boolean runTuningOpMode() throws InterruptedException`
- L160: `public TestsLine(Function<HardwareMap, Follower> followerFunction, double distance)`
- L167: `public Boolean runTuningOpMode() throws InterruptedException`
- L202: `public TestsCurve(Function<HardwareMap, Follower> followerFunction, double distance)`
- L209: `public Boolean runTuningOpMode() throws InterruptedException`
- L244: `public TestsInterpolation(Function<HardwareMap, Follower> followerFunction, double distance)`
- L251: `public Boolean runTuningOpMode() throws InterruptedException`
- L286: `public TestsLocalization(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction)`
- L293: `public Boolean runTuningOpMode() throws InterruptedException`
- L336: `public TestsOdometry(Function<HardwareMap, Drivetrain> drivetrainFunction, Function<HardwareMap, Localizer> localizerFunction)`
- L343: `public Boolean runTuningOpMode() throws InterruptedException`
- L445: `public TestsDriving(Function<HardwareMap, Drivetrain> drivetrainFunction)`
- L451: `public Boolean runTuningOpMode() throws InterruptedException`
- L464: `public TestsPose(Function<HardwareMap, Localizer> localizerFunction)`
- L470: `public Boolean runTuningOpMode() throws InterruptedException`

Internal dependency candidates: none detected

### pedro/procedures/ThreeWheelIMUTuner.java

AutoTune procedure: ThreeWheelIMUTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ThreeWheelIMUTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ThreeWheelIMUTuner.java); 328 lines; SHA-256 `f807403dbb40b5c591ea90df056ee916712c68facfc044aab5da3b44bd8aace8`.

Types: `ThreeWheelIMUTuner` (class, line 20), `ThreeWheelIMUResolution` (class, line 202), `ThreeWheelIMUOffsets` (class, line 240), `ThreeWheelIMUTurn` (class, line 290)

- L31: `public ThreeWheelIMUTuner()`
- L36: `public void run() throws InterruptedException`
- L155: `private List<Double> measure(String pod, double distance) throws InterruptedException`
- L216: `protected List<Double> runTuningOpMode()`
- L263: `protected List<Double> runTuningOpMode()`
- L302: `protected Double runTuningOpMode()`

Internal dependency candidates: none detected

### pedro/procedures/ThreeWheelTuner.java

AutoTune procedure: ThreeWheelTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ThreeWheelTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ThreeWheelTuner.java); 290 lines; SHA-256 `06d2a249a98213a5d1a62e0e438fc342d5f504815cdd2e93a1f6265c5c0118d6`.

Types: `ThreeWheelTuner` (class, line 18), `ThreeWheelResolution` (class, line 176), `ThreeWheelOffsets` (class, line 214), `ThreeWheelTurn` (class, line 258)

- L24: `public ThreeWheelTuner()`
- L29: `public void run() throws InterruptedException`
- L130: `private List<Double> measure(String pod, double distance) throws InterruptedException`
- L190: `protected List<Double> runTuningOpMode()`
- L237: `protected List<Double> runTuningOpMode()`
- L270: `protected Double runTuningOpMode()`

Internal dependency candidates: none detected

### pedro/procedures/TwoWheelTuner.java

AutoTune procedure: TwoWheelTuner.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/TwoWheelTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/TwoWheelTuner.java); 404 lines; SHA-256 `67d107243b66bfdd5cf37ca80c8978fa67e9c4173f74ff9a5f458d9141de1383`.

Types: `TwoWheelTuner` (class, line 15), `TwoWheelSetup` (class, line 141), `TwoWheelForwardResolution` (class, line 164), `TwoWheelStrafeResolution` (class, line 211), `TwoWheelForwardDirection` (class, line 255), `TwoWheelStrafeDirection` (class, line 297), `TwoWheelOffsets` (class, line 339)

- L17: `public TwoWheelTuner()`
- L22: `public void run() throws InterruptedException`
- L180: `protected Double runTuningOpMode()`
- L227: `protected Double runTuningOpMode()`
- L274: `protected Boolean runTuningOpMode()`
- L316: `protected Boolean runTuningOpMode()`
- L369: `protected List<Double> runTuningOpMode()`

Internal dependency candidates: none detected

### subsystem/Drivetrain.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Drivetrain.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Drivetrain.java); 225 lines; SHA-256 `dbca14d209c7f1c59d71dec109d57a5c0aecc4c2c7aed5bddb62ddcab4437104`.

Types: `Drivetrain` (class, line 21)

- L45: `public Drivetrain(HardwareMap hardwareMap, PulseFollower follower, Alliance alliance)`
- L54: `public void startTeleOpDrive()`
- L58: `public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)`
- L66: `public void setPowers(double lf, double lr, double rf, double rr)`
- L93: `public void arcadeDrive(double x, double y, double rx, float DZ, float angleZero, float maxPower)`
- L173: `public void setFieldCentric(boolean fieldCentric)`
- L177: `public boolean isFieldCentric()`
- L181: `private double forwardOffsetDeg()`
- L185: `private Vector2D smoothGamepadAngle(double x, double y, double zeroAngle)`
- L197: `public void arcadeDrive(double x, double y, double rx)`
- L201: `public void arcadeDrive(Gamepad gamepad)`
- L205: `public double getLambda(double s, double alpha)`
- L216: `public void addTelemetry(Telemetry telemetry)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/RotMatrix.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorMagnitude.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorPrint.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorTransform.java`

### subsystem/Flower.java

Optional flower mechanism servo.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java); 35 lines; SHA-256 `53e70e739e4fcd6afd98b7e237e4212ee28b2f6ac3532caada7b69762688e53c`.

Types: `Flower` (class, line 10)

- L17: `public Flower(HardwareMap hw)`
- L21: `public void setDown(boolean state)`
- L26: `public void update()`
- L28: `public void stop()`
- L30: `public void toggle()`
- L32: `public boolean isDown()`
- L34: `public boolean isConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/Hood.java

Hood servo position and launch-angle control.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java); 51 lines; SHA-256 `a2fe1a3944e838a59ff3b1e50e83478c91d6843494eecc6bd36a3e6a2251291b`.

Types: `Hood` (class, line 13)

- L27: `public Hood(HardwareMap hw)`
- L37: `public void setLaunchAngle(double launchDeg)`
- L41: `public void setPosition(double position)`
- L50: `public double position()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`

### subsystem/Intake.java

Intake plus optional belted transfer motor, controlled together.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java); 79 lines; SHA-256 `9a95ff875f55f50fc058d0af9477f235ac6758ea0d7c0e7d11c7d5f344571d33`.

Types: `Intake` (class, line 13), `IntakeState` (enum, line 21)

- L32: `public Intake(HardwareMap hw)`
- L43: `private void setPower(double power)`
- L48: `public void on()`
- L53: `public void reverse()`
- L58: `public void slowReverse()`
- L63: `public void off()`
- L68: `public IntakeState getState()`
- L72: `public boolean atState(IntakeState state)`
- L76: `public boolean hasTransferMotor()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/hardware/CachedMotor.java`

### subsystem/Latch.java

Optional shooting gate servo.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java); 37 lines; SHA-256 `eebfe5b5b9cf813600d4db6f0a89e602c33f3b4ea4f733a2fa3bebed2fed7a07`.

Types: `Latch` (class, line 10)

- L17: `public Latch(HardwareMap hw)`
- L23: `public void open()`
- L29: `public void close()`
- L35: `public boolean isOpen()`
- L36: `public boolean isConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/Shooter.java

Dual flywheel motors, encoder feedback, PIDF and optional shooting bang-bang control.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java); 149 lines; SHA-256 `9ba26560e450d8adbd406a75b6d06558f9e31cf8c54d9e7ac0eebe520cf36467`.

Types: `Shooter` (class, line 19), `Mode` (enum, line 43)

- L60: `public Shooter(HardwareMap hw)`
- L72: `public void update()`
- L107: `private void writePower(double p)`
- L121: `private void readVoltage()`
- L127: `public void setTargetVelocity(double ticksPerSecond)`
- L128: `public void setTargetRpm(double motorRpm)`
- L130: `public void setShooting(boolean shooting)`
- L131: `public void partyMode()`
- L132: `public void stop()`
- L139: `public boolean isSpinning()`
- L140: `public boolean atTarget()`
- L141: `public double velocity()`
- L142: `public double targetVelocity()`
- L143: `public double rpm()`
- L144: `public double targetRpm()`
- L145: `public double power()`
- L146: `public double voltage()`
- L147: `public Mode mode()`
- L148: `public boolean isShooting()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/PIDController.java`

### subsystem/Turret.java

Turret servo hardware, optional four-servo operation and robot-to-turret geometry.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java); 187 lines; SHA-256 `e1a7583c7bef3946450a50f45c48d0a9014f6314f4c1e7b658dc25423a0bc143`.

Types: `Turret` (class, line 17)

- L30: `public Turret(HardwareMap hw)`
- L42: `private static Servo.Direction direction(boolean reversed)`
- L46: `private boolean applyDirection(ServoImplEx servo, int i, boolean reversed)`
- L53: `public static Pose getTurretPose(Pose robotPose)`
- L64: `public static Vector2D getTurretToGoalVector(Pose turretPose, Pose goal)`
- L70: `public boolean isConnected()`
- L71: `public List<String> missing()`
- L73: `public void setPosition(double pos)`
- L101: `public Turret(HardwareMap hw, boolean fourServos)`
- L117: `private ServoImplEx extraServo(HardwareMap hw, String name, boolean reversed)`
- L130: `public int servoCount()`
- L149: `public void setPositionAll(double pos)`
- L170: `public void setExtraServosEnabled(boolean enabled)`
- L176: `private void setEnabled(ServoImplEx servo, boolean enabled)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/aim/shot/Braking.java

Stopping prediction used by aiming.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java); 88 lines; SHA-256 `49fb386f0041d510dc6de74c2090ea07755eac87b100fcde8c17a9ba1cdecebe`.

Types: `Braking` (class, line 6)

- L18: `public static double lambertW(double x)`
- L37: `private static double computeX(double t)`
- L42: `private static void refreshFactors()`
- L60: `private static void ensureFresh()`
- L68: `public static double speedAfter(double currentSpeed)`
- L76: `public static double distanceAfter(double currentSpeed)`
- L84: `public static double impliedDecel(double currentSpeed)`

Internal dependency candidates: none detected

### subsystem/aim/shot/ShooterCalc.java

Selects and solves shots; tuned-point/map lookup, motion prediction and hardware target conversion.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java); 430 lines; SHA-256 `862d5579d0ad876c4a5b11d106da3ab90c3c72de4b5a7eed8b89dab592102441`.

Types: `ShooterCalc` (class, line 18)

- L68: `private static void queryTuned(double range, double bearing)`
- L80: `public static double flightTime(double range, double hoodPosition)`
- L89: `public static void resetProjectionFilter()`
- L125: `public static double ballSpeedForMotorRpm(double motorRpm)`
- L168: `public static double hoodServoPosition(double launchDeg)`
- L172: `public static double launchDegForHoodPosition(double position)`
- L178: `public static double normalizeDegrees(double degrees)`
- L186: `public static Pose goal(Alliance alliance, double robotY)`
- L210: `public static boolean hiveAimFar()`
- L218: `private static double[] solveMeta(double robotHeading, double x, double goalTheta, Vector2D robotVel)`
- L259: `public static double[] solve( double robotHeading, Vector2D robotToGoalVector, Vector2D robotVel, double robotAngularVelocity)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### subsystem/aim/shot/ShooterConstants.java

Field/HIVE geometry, target selection constants and under-HIVE behavior.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java); 56 lines; SHA-256 `fc230eb564588091665cf91cffa8535640c1fff35440034fdda4467866ba27e3`.

Types: `ShooterConstants` (class, line 7)

- L21: `public static Pose hiveAimPoint(boolean red, boolean far)`
- L30: `public static double hiveAimX(boolean red, boolean far)`
- L34: `public static double hiveAimY(boolean red, boolean far)`
- L49: `public static boolean underHive(double x, double y)`

Internal dependency candidates: none detected

### subsystem/aim/table/HiveCell.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java); 11 lines; SHA-256 `3fce1a3147678e2f5f559cadb5ac273e2dc93489fedc4f9f75bee18ba3f91074`.

Types: `HiveCell` (enum, line 4)

- L7: `public static HiveCell of(boolean red, boolean far)`

Internal dependency candidates: none detected

### subsystem/aim/table/TunedShotPoints.java

Committed calibrated shot points.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java); 63 lines; SHA-256 `27903efe84fd0cef93020cc6e1454b05dec279a5cf2135a4f569cc1e672cf76f`.

Types: `TunedShotPoints` (class, line 17)

- L18: `private TunedShotPoints()`
- L60: `private static TunedShotTable.Point p(HiveCell cell, double x, double y, double rpm, double hood)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`

### subsystem/aim/table/TunedShotTable.java

Blends hand-tuned points; merges committed points with Control Hub shotpoints.csv; mirrors between cells.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java); 305 lines; SHA-256 `9afbb0a55bf0cf36c24f2284640fa9b74fd2d5662e13d4a82785130d3c18c5a9`.

Types: `TunedShotTable` (class, line 40), `Point` (class, line 65), `Shot` (class, line 84)

- L70: `public Point(HiveCell cell, double x, double y, double rpm, double hood, boolean fromFile)`
- L74: `public String javaLine()`
- L97: `private TunedShotTable()`
- L100: `public static synchronized void load()`
- L131: `private static void rebuild()`
- L143: `private static void ensureLoaded()`
- L145: `public static File file()`
- L146: `public static String lastError()`
- L149: `private static HiveCell storedCell(HiveCell cell)`
- L158: `private static boolean isRed(HiveCell c)`
- L159: `private static boolean isFar(HiveCell c)`
- L163: `private static double storedX(HiveCell cell, double x)`
- L166: `private static double storedY(HiveCell cell, double y)`
- L178: `public static synchronized boolean lookup(HiveCell cell, double x, double y, Shot out)`
- L226: `public static synchronized Point save(HiveCell cell, double x, double y, double rpm, double hood) throws IOException`
- L243: `public static synchronized Point nearest(HiveCell cell, double x, double y)`
- L259: `public static double distance(HiveCell cell, double x, double y, Point p)`
- L264: `public static synchronized boolean delete(Point p) throws IOException`
- L272: `public static synchronized void clearFile() throws IOException`
- L279: `public static synchronized List<Point> filePoints()`
- L280: `public static synchronized int count(boolean fromFile)`
- L288: `public static synchronized List<String> printJava()`
- L296: `private static void writeFile() throws IOException`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java`

### subsystem/aim/turret/StickLead.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/StickLead.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/StickLead.java); 176 lines; SHA-256 `1d7a253a1d044a89d5eff48389eed2fa0a8b47499a671e2709f9f939373733b8`.

Types: `StickLead` (class, line 26)

- L65: `public void setSticks(double forward, double lateral, double turn, double offsetRad, boolean robotCentric)`
- L75: `public void clearSticks()`
- L80: `public void track(double dtSec)`
- L106: `private static double weight(double age)`
- L122: `public void apply(double solverAimDeg, double aimRange, double heading, double vx, double vy, double measuredOmega, double leadSec)`
- L171: `public double aimDeg()`
- L173: `public double omega()`
- L175: `public double changeDeg()`

Internal dependency candidates: none detected

### subsystem/aim/turret/TurretCalc.java

Turret servo gearing, angular limits, slew, alliance offsets and yaw compensation.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java); 251 lines; SHA-256 `7017807c993279dd9dcd5322d75e028d1089e93e7fac1b7a891f0204f094991b`.

Types: `TurretCalc` (class, line 11)

- L63: `public static double turretDegPerServoDeg()`
- L68: `public static double servoPositionFor(double turretDeg)`
- L87: `public static void reset()`
- L101: `public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance)`
- L107: `public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, double offsetDeg)`
- L122: `public static double solveWithoutWrap(double commandedAngleDeg, double robotAngularVelocityRadPerSec, double offsetDeg)`
- L135: `private static double targetAngle(double commandedAngleDeg, double robotAngularVelocityRadPerSec, double offsetDeg)`
- L142: `public static double leadSec()`
- L148: `public static double commandDeg()`
- L150: `public static boolean outOfRange()`
- L152: `public static double aimErrorDeg()`
- L154: `private static double servoPosition(double targetDeg)`
- L165: `private static double normalizeDegrees(double degrees)`
- L169: `private static double getClosestValidTurretAngle(double relativeGoalDegrees)`
- L206: `private static double slewTurret(double target)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### tele/Tele.java

Two-driver controls, field-centric drive, shooting gates, manual presets and live trims.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java); 202 lines; SHA-256 `e65462be00cd1bb70100cd8fbeb3d65905b0e730251e0d00d00c88487f998766`.

Types: `Tele` (class, line 18)

- L32: `public Tele(Alliance a)`
- L39: `public void initialize()`
- L76: `public void initializeLoop()`
- L87: `public void execute()`
- L100: `public void end()`
- L111: `private void driver1()`
- L142: `private void driver2(double dt)`
- L177: `private double forwardOffsetRad()`
- L181: `private double turretOffset()`
- L182: `private void setTurretOffset(double deg)`
- L188: `private boolean usingSavedPose()`
- L190: `private Pose startPose()`
- L194: `private String notConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Drivetrain.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### tele/teleBlue.java

Alliance-specific TeleOp registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/teleBlue.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/teleBlue.java); 12 lines; SHA-256 `b7ef3ea40433ee90ab896b9c08b31b2fcd7b88ab199039503721bb8e65d79737`.

Types: `teleBlue` (class, line 8)

- L9: `public teleBlue()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### tele/teleRed.java

Alliance-specific TeleOp registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/teleRed.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/teleRed.java); 12 lines; SHA-256 `b886d30d4179a5093320a24659c1668a4375e282de433682767b007a8722a6e4`.

Types: `teleRed` (class, line 8)

- L9: `public teleRed()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### test/FlowerTest.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlowerTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlowerTest.java); 65 lines; SHA-256 `23a1e29836820edefb4e2dae9699145829b9e4641815c1b0a151d2d7e611a28b`.

Types: `FlowerTest` (class, line 17)

- L28: `public void initialize()`
- L35: `public void initializeLoop()`
- L41: `public void execute()`
- L51: `private void moveTo(double target)`
- L57: `private void show()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/FlywheelPIDFTest.java

Bench OpMode for intake, flywheel PIDF and solver-driven hood targets.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlywheelPIDFTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlywheelPIDFTest.java); 177 lines; SHA-256 `85ac6e4a11d28ed291e7b8b141669404d2bc51d8e2dc16e0309abf0945ed8fc0`.

Types: `FlywheelPIDFTest` (class, line 42)

- L68: `public void initialize()`
- L90: `public void initializeLoop()`
- L100: `public void execute()`
- L124: `private boolean showProblem()`
- L131: `private void runShooter()`
- L162: `private void solveForPose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/HoodServoPositionTuning.java

Hood calibration OpMode.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/HoodServoPositionTuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/HoodServoPositionTuning.java); 61 lines; SHA-256 `f96b8537d199d700a8749234343a73d7cd33a7ebdc55572e8c1fa03e6b9b7a60`.

Types: `HoodServoPositionTuning` (class, line 21)

- L28: `public void initialize()`
- L36: `public void initializeLoop()`
- L42: `public void execute()`
- L55: `private void show()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/LatchPositionTuning.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LatchPositionTuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LatchPositionTuning.java); 123 lines; SHA-256 `0d1c0bcce6a9d13ac43688a9d0c6a5c8a86bb820720d56ef0c6b9a94b65678f6`.

Types: `LatchPositionTuning` (class, line 32)

- L46: `public void initialize()`
- L54: `public void initializeLoop()`
- L60: `public void execute()`
- L85: `private void nudge(double step)`
- L90: `private void moveTo(double pos, String why)`
- L96: `private void save(boolean asOpen)`
- L107: `private static double clamp(double pos)`
- L111: `private void show()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/ShotPointTuner.java

Drive, tune, save and export calibrated shot points.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/ShotPointTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/ShotPointTuner.java); 286 lines; SHA-256 `a18e54bc4b69ef90b3b333288c6a2dd1c0b802882db7fc0dc291c48c58cce344`.

Types: `ShotPointTuner` (class, line 54)

- L82: `public void initialize()`
- L97: `public void initializeLoop()`
- L110: `public void onStart()`
- L116: `public void execute()`
- L136: `private void aim()`
- L154: `private void handleButtons()`
- L202: `private void nudgeTurret(double degrees)`
- L207: `private void nudge(double rpm, double hood)`
- L213: `private void save()`
- L229: `private void deleteNearest()`
- L247: `private void showTelemetry()`
- L283: `public void end()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveCell.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/FollowerLocalization.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/TurretCenter.java

Turret centering OpMode.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretCenter.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretCenter.java); 44 lines; SHA-256 `868e014e037734dfa9513749cc4bca26d04c4375152b2c3696a34084faf7903b`.

Types: `TurretCenter` (class, line 15)

- L19: `public void initialize()`
- L25: `public void initializeLoop()`
- L30: `public void execute()`
- L34: `private void hold()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`

### test/TurretTest.java

Turret hardware test OpMode.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretTest.java); 305 lines; SHA-256 `6d6c97bc1409e19a721ff50fc6251985970c451917282e93bada68548aa1d59d`.

Types: `TurretTest` (class, line 55), `Mode` (enum, line 70)

- L90: `public void initialize()`
- L122: `public void initializeLoop()`
- L132: `public void execute()`
- L174: `private boolean showNoServos()`
- L183: `private void commonControls()`
- L190: `private int activeServoCount()`
- L195: `private void setSingleServo(boolean single)`
- L201: `private void setTurret(double pos)`
- L227: `private void smoothTowardTarget(double dt)`
- L248: `private static double servoUnits(double turretDeg)`
- L252: `private void resetPose()`
- L258: `private void track()`
- L269: `private static double turretDegFor(double servoPos)`
- L274: `private void show()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/TurretTrackingTest.java

Checks turret tracking using localization.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretTrackingTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretTrackingTest.java); 167 lines; SHA-256 `11c67e88423a123dff13dc08b179f439c3faf01671063d4c32d05e12819c3095`.

Types: `TurretTrackingTest` (class, line 38), `Aim` (enum, line 42)

- L56: `public void initialize()`
- L80: `public void initializeLoop()`
- L91: `public void execute()`
- L104: `private boolean showProblem()`
- L111: `private void pickAlliance()`
- L116: `private void resetPose()`
- L121: `private Pose goal(Alliance a, double robotY)`
- L133: `private void track()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### util/Alliance.java

Shared alliance selection.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java); 6 lines; SHA-256 `b58fbc8f05e4ab1fd99ae1eaf85e3c66b7229547faa5eba37fad118389920cca`.

Types: `Alliance` (enum, line 3)


Internal dependency candidates: none detected

### util/CommandOpMode.java

LinearOpMode lifecycle driving the Ivy scheduler; resets scheduler before final end().

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java); 54 lines; SHA-256 `1ed9713cc408d1d61d14aeb738192b37072e45b2f829911f73f2bece099640d9`.

Types: `CommandOpMode` (class, line 7)

- L9: `public void runOpMode() throws InterruptedException`
- L35: `public void reset()`
- L39: `public void schedule(Command... commands)`
- L43: `public abstract void initialize()`
- L45: `public void initializeLoop()`
- L47: `public void execute()`
- L49: `public void onStart()`
- L51: `public void end()`
- L53: `public void preInit()`

Internal dependency candidates: none detected

### util/FollowerLocalization.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/FollowerLocalization.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/FollowerLocalization.java); 33 lines; SHA-256 `8cadecc6866b4f0181ec84415184be2e6c1acf417f555eb3dd0049e6f060ffcc`.

Types: `FollowerLocalization` (class, line 22)

- L23: `private FollowerLocalization()`
- L25: `public static void update(PulseFollower follower)`

Internal dependency candidates: none detected

### util/OpModeStorage.java

Pose handoff between OpModes.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java); 9 lines; SHA-256 `cd4cf3ba0275a02eccc48c2037c2744a12e4728b0b106ae4aa28c5af9dbd500a`.

Types: `OpModeStorage` (class, line 5)

- L6: `private OpModeStorage()`

Internal dependency candidates: none detected

### util/ShotLogger.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ShotLogger.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ShotLogger.java); 149 lines; SHA-256 `0372a558443254991843ccba5bff1676b3742e946d229cccae0590252e6ad743`.

Types: `ShotLogger` (class, line 24)

- L51: `public double[] row()`
- L54: `public void commit(boolean shooting)`
- L72: `private void flushRunUp(double now)`
- L80: `private void append(double[] from, int offset)`
- L87: `public int rowCount()`
- L93: `public File save()`
- L104: `public void run()`
- L117: `private static File newFile()`

Internal dependency candidates: none detected

### util/TeleTelemetry.java

Driver Station and Panels telemetry presentation.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java); 100 lines; SHA-256 `15476a818d2bf6339218e4d4491624fad4e9abbc72b94a10c0f1d872d2d22b9f`.

Types: `TeleTelemetry` (class, line 11)

- L17: `public TeleTelemetry(Telemetry telemetry, Robot robot, Alliance a)`
- L23: `public void update(double turretOffset)`
- L29: `public static String shotSourceText()`
- L35: `private String driver2Text(double turretOffset)`
- L44: `private void driverStation(double turretOffset)`
- L76: `private void panels(double turretOffset)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### util/control/PIDController.java

Reusable PID feedback math.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/PIDController.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/PIDController.java); 86 lines; SHA-256 `fdab1d6e234984cabf4d464fc9ba911580513cc3b2f592b46e06769e8d6968e7`.

Types: `PIDController` (class, line 6)

- L12: `public PIDController(double p, double i, double d, double f, boolean velocityMode)`
- L22: `public void setDeadband(double deadband)`
- L26: `public double update(double target, double current)`
- L30: `public double update(double target, double current, double voltage)`
- L74: `public void reset()`
- L80: `public void setGains(double p, double i, double d, double f)`

Internal dependency candidates: none detected

### util/data/TriTuple.java

Three-value data container.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/TriTuple.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/TriTuple.java); 10 lines; SHA-256 `b296aa82d725f02df08ac3abd4f1bf943a0fdea3c52d720dcf647ec2ea810ca1`.

Types: `TriTuple` (class, line 3)

- L6: `public TriTuple(T1 val1, T2 val2, T3 val3)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/Tuple.java`

### util/data/Tuple.java

Two-value data container.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/Tuple.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/Tuple.java); 11 lines; SHA-256 `5ccd9012c0f3e23d94fe2ba0f48f263af50d05a0c2cff2af62b69187933ab313`.

Types: `Tuple` (class, line 3)

- L7: `public Tuple(T1 val1, T2 val2)`

Internal dependency candidates: none detected

### util/filter/Filter.java

Numeric filter contract.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/Filter.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/Filter.java); 8 lines; SHA-256 `9f02e877efc8ad0988ba26bf1de9893b0f7cfc1660903188bf85bbcc35fce8ea`.

Types: `Filter` (class, line 3)

- L4: `public abstract double update(double value)`
- L5: `public abstract double getValue()`
- L6: `public abstract double getRawValue()`
- L7: `public abstract void forceSet(double value)`

Internal dependency candidates: none detected

### util/filter/ScalingLowPassFilter.java

Scaling low-pass signal filtering.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/ScalingLowPassFilter.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/ScalingLowPassFilter.java); 89 lines; SHA-256 `460ba060439a436da81301c87318ac347ad0fb3b35366f71c623a31248f89ed4`.

Types: `ScalingLowPassFilter` (class, line 7)

- L15: `public ScalingLowPassFilter(double coeff, double factor, double scaleFactor)`
- L21: `public ScalingLowPassFilter(TriTuple<Double, Double, Double> values)`
- L25: `public ScalingLowPassFilter(double factor, double scaleFactor)`
- L29: `public ScalingLowPassFilter(double scaleFactor)`
- L33: `public void setValues(double coeff, double factor, double scaleFactor)`
- L39: `public void setValues(TriTuple<Double, Double, Double> values)`
- L43: `public void setCoefficient(double coeff)`
- L47: `public void setFactor(double factor)`
- L51: `public void setScaleFactor(double scaleFactor)`
- L55: `public double update(double value)`
- L76: `public double getValue()`
- L80: `public double getRawValue()`
- L85: `public void forceSet(double value)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/TriTuple.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/Filter.java`

### util/hardware/CachedMotor.java

Motor hardware wrapper with cached writes.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/hardware/CachedMotor.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/hardware/CachedMotor.java); 434 lines; SHA-256 `8ad84567181284e935bd99951041ac358d94c10910c2786bb561de16556e32d3`.

Types: `CachedMotor` (class, line 21)

- L49: `public CachedMotor(DcMotorEx motor)`
- L56: `public CachedMotor(HardwareMap hardwareMap, String name)`
- L64: `private void _init()`
- L77: `public void setOnlyQuickPower(boolean onlyQuickPower)`
- L82: `public void setMotorEnable()`
- L87: `public void setMotorDisable()`
- L92: `public boolean isMotorEnabled()`
- L97: `public void setVelocity(double angularRate)`
- L105: `public void setVelocity(double angularRate, AngleUnit unit)`
- L109: `public double getTargetVelocity()`
- L114: `public double getVelocity()`
- L120: `public double getVelocity(AngleUnit unit)`
- L126: `public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients)`
- L132: `public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException`
- L145: `public void setVelocityPIDFCoefficients(PIDFCoefficients pidfCoefficients)`
- L150: `public void setVelocityPIDFCoefficients(double p, double i, double d, double f)`
- L156: `public void setPositionPIDFCoefficients(double p)`
- L161: `public void setPositionPIDFCoefficients(PIDFCoefficients pidfCoefficients)`
- L165: `public void setPositionPIDFCoefficients(double p, double i, double d, double f)`
- L171: `public PIDCoefficients getPIDCoefficients(RunMode mode)`
- L177: `public PIDFCoefficients getPIDFCoefficients(RunMode mode)`
- L189: `public PIDFCoefficients getVelocityPIDFCoefficients()`
- L193: `public PIDFCoefficients getPositionPIDFCoefficients()`
- L198: `public void setTargetPositionTolerance(int tolerance)`
- L204: `public int getTargetPositionTolerance()`
- L208: `public void setCurrentCacheTime(int currentCacheTimeMS)`
- L212: `public void setCurrentFilter(double coeff, double factor, double scaleFactor)`
- L216: `public void setCurrentFilter(TriTuple<Double, Double, Double> values)`
- L221: `public double getCurrent(CurrentUnit unit)`
- L247: `public double getRawCurrent(CurrentUnit unit)`
- L274: `public double getCurrentAlert(CurrentUnit unit)`
- L279: `public void setCurrentAlert(double current, CurrentUnit unit)`
- L284: `public boolean isOverCurrent()`
- L289: `public MotorConfigurationType getMotorType()`
- L294: `public void setMotorType(MotorConfigurationType motorType)`
- L299: `public DcMotorController getController()`
- L304: `public int getPortNumber()`
- L309: `public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior)`
- L313: `public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior, boolean force)`
- L320: `public ZeroPowerBehavior getZeroPowerBehavior()`
- L326: `public void setPowerFloat()`
- L331: `public boolean getPowerFloat()`
- L336: `public void setTargetPosition(int position)`
- L343: `public int getTargetPosition()`
- L348: `public boolean isBusy()`
- L353: `public int getCurrentPosition()`
- L358: `public void setMode(RunMode mode)`
- L364: `public RunMode getMode()`
- L368: `public void resetEncoder()`
- L374: `public void setDirection(Direction direction)`
- L379: `public void setInverted(boolean inverted)`
- L384: `public Direction getDirection()`
- L388: `public boolean getInverted()`
- L393: `public void setPower(double power)`
- L401: `public double getPower()`
- L406: `public Manufacturer getManufacturer()`
- L411: `public String getDeviceName()`
- L416: `public String getConnectionInfo()`
- L421: `public int getVersion()`
- L426: `public void resetDeviceConfigurationForOpMode()`
- L431: `public void close()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/data/TriTuple.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/filter/ScalingLowPassFilter.java`

### util/input/BooleanSwitch.java

Boolean input state and edge tracking.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/BooleanSwitch.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/BooleanSwitch.java); 114 lines; SHA-256 `e8b9bc37d8302ea1a326074db6d310e703df65bcd1c2cc279ddfc282df2556e8`.

Types: `BooleanSwitch` (class, line 5)

- L14: `public BooleanSwitch(BooleanSupplier decider)`
- L18: `public void update()`
- L29: `public boolean isTrue()`
- L33: `public boolean isRisingEdge()`
- L37: `public boolean isFallingEdge()`
- L41: `public boolean isToggled()`
- L45: `public BooleanSwitch risingEdge()`
- L49: `public BooleanSwitch fallingEdge()`
- L53: `public BooleanSwitch toggled()`
- L57: `public BooleanSwitch and(BooleanSwitch other)`
- L64: `public BooleanSwitch and(BooleanSupplier other)`
- L68: `public BooleanSwitch or(BooleanSwitch other)`
- L75: `public BooleanSwitch or(BooleanSupplier other)`
- L79: `public BooleanSwitch not()`
- L83: `private boolean canPress(long timestamp, long interval)`
- L90: `public boolean canPress(long timeStamp)`
- L95: `public boolean canPressShort(long timeStamp)`
- L100: `public boolean canPress4Short(long timeStamp)`
- L105: `public boolean canPress6Short(long timeStamp)`
- L110: `public BooleanSwitch setUpdateFunction(Runnable r)`

Internal dependency candidates: none detected

### util/input/FloatSupplier.java

Analog input supplier and trigger helpers.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java); 36 lines; SHA-256 `048b718ed9414e794e404e701cec33f1e9787f561f99a9f4be53a44e0a8d2dd8`.

Types: `FloatSupplier` (class, line 5)

- L9: `public FloatSupplier(Supplier<Float> floatSupplier)`
- L13: `public float getVal()`
- L17: `public void update()`
- L21: `public BooleanSwitch clamped(float thresholdMin, float thresholdMax)`
- L25: `public BooleanSwitch greaterThan(float thresholdMin)`
- L29: `public BooleanSwitch lessThan(float thresholdMax)`
- L33: `public BooleanSwitch isPress()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/BooleanSwitch.java`

### util/input/GamepadEx.java

Gamepad inputs with edge-aware buttons and suppliers.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java); 134 lines; SHA-256 `0344c8b606de8c0f644dafffdf1f2c2b9af2215595a4bff1466df56da5e4638d`.

Types: `GamepadEx` (class, line 12)

- L48: `public GamepadEx(Gamepad gamepad)`
- L80: `public void apply(Function<BooleanSwitch, BooleanSwitch> function)`
- L84: `public void left_trigger_button(Function<FloatSupplier, BooleanSwitch> f)`
- L89: `public void right_trigger_button(Function<FloatSupplier, BooleanSwitch> f)`
- L94: `public void left_stick_x_button(Function<FloatSupplier, BooleanSwitch> f)`
- L99: `public void left_stick_y_button(Function<FloatSupplier, BooleanSwitch> f)`
- L104: `public void right_stick_x_button(Function<FloatSupplier, BooleanSwitch> f)`
- L109: `public void right_stick_y_button(Function<FloatSupplier, BooleanSwitch> f)`
- L114: `public Vector2D leftStick()`
- L118: `public Vector2D rightStick()`
- L122: `public void rumble(double ms)`
- L126: `public void addButton(BooleanSwitch button)`
- L130: `public void update()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/BooleanSwitch.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java`

### util/math/RotMatrix.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/RotMatrix.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/RotMatrix.java); 25 lines; SHA-256 `7733f325ed07b7402633c00c8d05dc866bf6c093c23e11fd10b423136b895fc5`.

Types: `RotMatrix` (class, line 9)

- L10: `private RotMatrix()`
- L16: `public static Matrix rotMatrix(double angle)`

Internal dependency candidates: none detected

### util/math/VectorMagnitude.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorMagnitude.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorMagnitude.java); 19 lines; SHA-256 `e3ecd87cf55a2a49a8e39bd39168a9c048f1f8f7c7283fa2c1080732f65367c9`.

Types: `VectorMagnitude` (class, line 9)

- L10: `private VectorMagnitude()`
- L16: `public static double magnitude(Vector2D v)`

Internal dependency candidates: none detected

### util/math/VectorPrint.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorPrint.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorPrint.java); 12 lines; SHA-256 `040e82cb5c1cb3dc976368916362942ad21099fb40c6c28bf1958a037074f533`.

Types: `VectorPrint` (class, line 6)

- L7: `private VectorPrint()`
- L9: `public static String print(Vector2D v)`

Internal dependency candidates: none detected

### util/math/VectorTransform.java

Project support file.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorTransform.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/math/VectorTransform.java); 34 lines; SHA-256 `f1bd720d0288808294fe71d5d3ff43d8a074fdce7e53d81131394583ddf8f7e6`.

Types: `VectorTransform` (class, line 10)

- L11: `private VectorTransform()`
- L19: `public static Vector2D transform(Vector2D v, Matrix matrix)`

Internal dependency candidates: none detected

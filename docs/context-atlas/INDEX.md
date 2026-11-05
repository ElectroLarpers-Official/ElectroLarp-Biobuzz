# ElectroLarp-Biobuzz project index

Generated 2026-11-04, source HEAD `1820e3dc4bb6c76c178f51aaca25c6fdca91d7ce`. Source files were inspected directly.

## Coverage

- 188 tracked non-sensitive paths inventoried.
- 59 TeamCode Java files, 70 SDK/vendor Java files.
- 776 method/constructor declarations; 141 candidate internal dependency edges.
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
- `org.electrolightsftc:pulse-electropathing:2.1.1`
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

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java); 39 lines; SHA-256 `50493a20e75887730df6401b257bf469dc7505fe6be9684e115b48fc4204cd15`.

Types: `HardwareNames` (class, line 6)

- L7: `private HardwareNames()`

Internal dependency candidates: none detected

### Robot.java

Owns hardware and follower; updates aiming and flywheel; coordinates shooting, manual fallback and pose reset.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java); 396 lines; SHA-256 `927e7add9b0e3f3699ab1e80e6d4eda27a46480f54fb2257f84be821e0d99ec9`.

Types: `Robot` (class, line 48), `ManualShot` (enum, line 82)

- L109: `public Robot(HardwareMap hardwareMap, Alliance alliance)`
- L129: `public void update()`
- L142: `private void updateAiming()`
- L199: `public boolean isManual()`
- L201: `public void setManual(boolean on)`
- L205: `public ManualShot manualShot()`
- L206: `public void setManualShot(ManualShot shot)`
- L207: `public double manualTurretDeg()`
- L209: `public void nudgeManualTurret(double deg)`
- L215: `public static Pose resetPose(Alliance alliance)`
- L220: `public void resetPose()`
- L225: `public boolean underHive()`
- L229: `public boolean shotAllowed()`
- L234: `public boolean readyToShoot()`
- L238: `public void drive(double forward, double lateral, double turn, double forwardOffsetRad)`
- L242: `public boolean isPaused()`
- L246: `public boolean pauseFollowing(boolean hold)`
- L259: `public boolean resumeFollowing()`
- L313: `public void clearPause()`
- L317: `private static void collectModifiers(Path path, List<Modifier> inherited, List<List<Modifier>> out)`
- L329: `private static Curve tailOf(Curve curve, double t)`
- L351: `public void startShooting()`
- L359: `public void stopShooting()`
- L367: `public boolean isShooting()`
- L372: `public Command shootCycle()`
- L380: `public void flower(boolean status)`
- L384: `public void stop()`
- L393: `public double loopMs()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/Close.java

Near-side autonomous sequence: shooting, pollen loading, flower approach and retreat.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java); 92 lines; SHA-256 `7a87fb614d99064c7e4901a255e79a8951ea98b90385e45b275a1255c44353f9`.

Types: `Close` (class, line 23)

- L29: `public Close(Alliance a)`
- L34: `public void initialize()`
- L42: `public void initializeLoop()`
- L49: `public void onStart()`
- L57: `public void execute()`
- L63: `public void end()`
- L71: `private Command routine()`
- L87: `private Command shootCycle()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`

### auto/Far.java

Far-side autonomous lifecycle; routine currently a placeholder.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Far.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Far.java); 60 lines; SHA-256 `eff0d4f04083a2d9786476059afb4717aaf302fcea2af0ccd2793d51772033be`.

Types: `Far` (class, line 16)

- L21: `public Far(Alliance a)`
- L24: `public void initialize()`
- L31: `public void initializeLoop()`
- L38: `public void onStart()`
- L47: `public void end()`
- L57: `private Command routine()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/FarPaths.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`

### auto/blue/blueClose.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueClose.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueClose.java); 13 lines; SHA-256 `740d69129f0313df7955deb8b7903a1349e87fa7129cb8b718d39dd82f8c9d93`.

Types: `blueClose` (class, line 9)

- L10: `public blueClose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/blue/blueFar.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueFar.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/blue/blueFar.java); 14 lines; SHA-256 `17926d177ee79b3f377c612d46ba16836c11c70a420f075083e95ae26776cd17`.

Types: `blueFar` (class, line 10)

- L11: `public blueFar()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Far.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/paths/ClosePaths.java

Near-side paths, heading interpolation, mirrored poses and flower pose callback.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/ClosePaths.java); 168 lines; SHA-256 `ff3d44e2ec24a7b2230222ef74863db5b487fa2c679dfae229425c85680561a2`.

Types: `ClosePaths` (class, line 18), `Poses` (class, line 138)

- L19: `private ClosePaths()`
- L23: `public static PoseFactory poses(Alliance a)`
- L28: `public static Pose startPose(Alliance a)`
- L34: `public static Path intakeLoading(Alliance a)`
- L39: `public static Path goNearFlower(Alliance a, Command openFlower)`
- L50: `public static Path goInFlower(Alliance a)`
- L55: `public static Path moveback(Alliance a)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/paths/FarPaths.java

Far-side pose factory and placeholder starting pose; paths not implemented.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/FarPaths.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/paths/FarPaths.java); 23 lines; SHA-256 `a6a74be0b649233647f256240298f2c17e61cb6a8c09bf57a24a586609be78d1`.

Types: `FarPaths` (class, line 8)

- L9: `private FarPaths()`
- L14: `public static PoseFactory poses(Alliance a)`
- L20: `public static Pose startPose(Alliance a)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/red/redClose.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redClose.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redClose.java); 13 lines; SHA-256 `489f1823321533c36319bd4766448fb70337474b62b4ae21f6efc2a12b3d20ab`.

Types: `redClose` (class, line 9)

- L10: `public redClose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Close.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### auto/red/redFar.java

Alliance-specific autonomous registration wrapper.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redFar.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/red/redFar.java); 13 lines; SHA-256 `cdcc1f34bbd5504fad1397d361f55826b81182dac579de728252b5c39e9e74cc`.

Types: `redFar` (class, line 9)

- L10: `public redFar()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/Far.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### pedro/Constants.java

Builds PulseFollower from Pinpoint localization, Mecanum or NoDrivetrain, and Foresight configuration.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java); 101 lines; SHA-256 `603b06bead177ef6b94a5cd36937bdc5a7ea9160a568ed87ac95df7830c8b0e2`.

Types: `Constants` (class, line 28)

- L88: `public static PulseFollower create(HardwareMap h)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java`

### pedro/NoDrivetrain.java

Drivetrain fallback when the four drive motors are absent.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/NoDrivetrain.java); 20 lines; SHA-256 `8fa5e6b75fd99e6588366c708f4145eaeea885e36b646701881959cfdb7cc894`.

Types: `NoDrivetrain` (class, line 13)

- L14: `public void drive(DrivePowers powers, boolean useBrake)`
- L15: `public double maxScaling(DrivePowers a, DrivePowers b)`
- L16: `public void stop()`
- L17: `public void stop(boolean useBrake)`
- L19: `public double interpolateVelocity(double a, double b, double c)`

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

### subsystem/Flower.java

Optional flower mechanism servo.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Flower.java); 23 lines; SHA-256 `6c5f1c7c4c5f2e04cba9406a7fea8ef02225b326b4e2c44ad926548d449c4e38`.

Types: `Flower` (class, line 8)

- L14: `public Flower(HardwareMap hw)`
- L18: `public void toggleFlower(boolean state)`
- L22: `public boolean isConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/Hood.java

Hood servo position and launch-angle control.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java); 47 lines; SHA-256 `30709dc088c931463273764d6b9b16d655c2817d32d40021554322eb917d416d`.

Types: `Hood` (class, line 13)

- L26: `public Hood(HardwareMap hw)`
- L35: `public void setLaunchAngle(double launchDeg)`
- L39: `public void setPosition(double position)`
- L46: `public double position()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`

### subsystem/Intake.java

Intake plus optional belted transfer motor, controlled together.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java); 83 lines; SHA-256 `be11a60dcbe227ad343268840276c0c6cbfeacd37ce483a4c4ce77f4b7cc5878`.

Types: `Intake` (class, line 17), `IntakeState` (enum, line 25)

- L36: `public Intake(HardwareMap hw)`
- L47: `private void setPower(double power)`
- L52: `public void on()`
- L57: `public void reverse()`
- L62: `public void slowReverse()`
- L67: `public void off()`
- L72: `public IntakeState getState()`
- L76: `public boolean atState(IntakeState state)`
- L80: `public boolean hasTransferMotor()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/hardware/CachedMotor.java`

### subsystem/Latch.java

Optional shooting gate servo.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Latch.java); 37 lines; SHA-256 `e883fb0a47f27b6ba190ecf4a2e38baf74d2fb65dc95907ff54b37ea59101e16`.

Types: `Latch` (class, line 10)

- L17: `public Latch(HardwareMap hw)`
- L23: `public void open()`
- L29: `public void close()`
- L35: `public boolean isOpen()`
- L36: `public boolean isConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/Shooter.java

Dual flywheel motors, encoder feedback, PIDF and optional shooting bang-bang control.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java); 154 lines; SHA-256 `5817e302b832dd0671c4d76b5b938c7b1e27597e33f7344bb5b9feb16977d2f2`.

Types: `Shooter` (class, line 29), `Mode` (enum, line 60)

- L73: `public Shooter(HardwareMap hw)`
- L87: `public void update()`
- L114: `private void writePower(double p)`
- L125: `private void readVoltage()`
- L131: `public void setTargetVelocity(double ticksPerSecond)`
- L132: `public void setTargetRpm(double motorRpm)`
- L134: `public void setShooting(boolean shooting)`
- L135: `public void partyMode()`
- L136: `public void stop()`
- L143: `public boolean isSpinning()`
- L144: `public boolean atTarget()`
- L146: `public double velocity()`
- L147: `public double targetVelocity()`
- L148: `public double rpm()`
- L149: `public double targetRpm()`
- L150: `public double power()`
- L151: `public double voltage()`
- L152: `public Mode mode()`
- L153: `public boolean isShooting()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/control/PIDController.java`

### subsystem/Turret.java

Turret servo hardware, optional four-servo operation and robot-to-turret geometry.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java); 171 lines; SHA-256 `d58c2072edcfb5d8651622aa3dd5be2bfd8ff40209b2df559170fe23f3fb4e22`.

Types: `Turret` (class, line 17)

- L37: `public Turret(HardwareMap hw)`
- L49: `private static Servo.Direction direction(boolean reversed)`
- L54: `private boolean applyDirection(ServoImplEx servo, int i, boolean reversed)`
- L61: `public static Pose getTurretPose(Pose robotPose)`
- L72: `public static Vector2D getTurretToGoalVector(Pose turretPose, Pose goal)`
- L78: `public boolean isConnected()`
- L81: `public List<String> missing()`
- L83: `public void setPosition(double pos)`
- L108: `public Turret(HardwareMap hw, boolean fourServos)`
- L120: `private ServoImplEx extraServo(HardwareMap hw, String name, boolean reversed)`
- L133: `public int servoCount()`
- L139: `public void setPositionAll(double pos)`
- L154: `public void setExtraServosEnabled(boolean enabled)`
- L160: `private void setEnabled(ServoImplEx servo, boolean enabled)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`

### subsystem/aim/shot/Braking.java

Stopping prediction used by aiming.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java); 112 lines; SHA-256 `a8e1eecaf4674ab83c3a320ad11952d9faf05cfc2020b51bd323bd60d7fc9825`.

Types: `Braking` (class, line 6)

- L42: `public static double lambertW(double x)`
- L61: `private static double computeX(double t)`
- L66: `private static void refreshFactors()`
- L84: `private static void ensureFresh()`
- L92: `public static double speedAfter(double currentSpeed)`
- L100: `public static double distanceAfter(double currentSpeed)`
- L108: `public static double impliedDecel(double currentSpeed)`

Internal dependency candidates: none detected

### subsystem/aim/shot/ShooterCalc.java

Selects and solves shots; tuned-point/map lookup, motion prediction and hardware target conversion.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java); 715 lines; SHA-256 `35447a7279d66be3a2f46768277d0944633aafdf01cbcd73b114679f31c8a99b`.

Types: `ShooterCalc` (class, line 18), `ShotSource` (enum, line 119)

- L129: `public static boolean aimTableInUse()`
- L131: `public static boolean tunedShotInUse()`
- L133: `public static boolean needsTunedPoint()`
- L134: `public static HiveAimTable.Shot tableShot()`
- L139: `private static void queryAimTable(double range, double bearing)`
- L183: `private static void fillFromTuned(double range, boolean valid)`
- L250: `public static void resetProjectionFilter()`
- L313: `public static double tpsForBallSpeed(double mps)`
- L318: `public static double tpsPerBallSpeed()`
- L322: `public static double maxFitBallSpeedMps()`
- L330: `private static double hoodMinRad()`
- L331: `private static double hoodMaxRad()`
- L343: `public static double hoodServoPosition(double launchDeg)`
- L348: `public static double launchDegForHoodPosition(double position)`
- L355: `public static double ballSpeedForMotorRpm(double motorRpm)`
- L363: `public static double targetHeight(double range)`
- L375: `public static double entryAngle(double range)`
- L390: `public static double[] solveStatic(double range, double height, double entry)`
- L433: `public static double normalizeDegrees(double degrees)`
- L441: `public static Pose goal(Alliance alliance, double robotY)`
- L467: `public static boolean hiveAimFar()`
- L469: `public static double flightTime(double range)`
- L482: `public static double[] solve( double robotHeading, Vector2D robotToGoalVector, Vector2D robotVel, Vector2D robotAccel, double robotAngularVelocity)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/Braking.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### subsystem/aim/shot/ShooterConstants.java

Field/HIVE geometry, target selection constants and under-HIVE behavior.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java); 84 lines; SHA-256 `69eb50c91fd4ffa68268abd21da0d0bc544bb168f38410eb607376d4a208910b`.

Types: `ShooterConstants` (class, line 7)

- L34: `public static Pose hiveAimPoint(boolean red, boolean far)`
- L44: `public static double hiveAimX(boolean red, boolean far)`
- L49: `public static double hiveAimY(boolean red, boolean far)`
- L77: `public static boolean underHive(double x, double y)`

Internal dependency candidates: none detected

### subsystem/aim/shot/ShotCheck.java

Shot feasibility, shooting permission and readiness tolerances.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java); 215 lines; SHA-256 `8df330733a5a5c52b7adbce2a0d39f112994af7ac507a94f8a78d6d58da57e0a`.

Types: `ShotCheck` (class, line 11), `Result` (enum, line 13)

- L75: `private ShotCheck()`
- L77: `public static boolean allowed()`
- L79: `public static String status()`
- L81: `public static double turretToleranceDeg()`
- L82: `public static double flywheelToleranceTps()`
- L84: `public static boolean ready(double flywheelVelocity, double flywheelTarget)`
- L91: `public static void blockUnderHive()`
- L97: `public static boolean check(Alliance alliance, Pose goal)`
- L104: `private static Result evaluate(Alliance alliance, Pose goal, boolean turretInRange)`
- L116: `private static Result evaluateGeometry(Alliance alliance, Pose goal, boolean turretInRange)`
- L176: `private static double lateralRoom(double bearing)`
- L189: `private static Result evaluateTable(boolean turretInRange)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### subsystem/aim/table/HiveAimTable.java

Queries simulated shot-map data by cell and field position.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java); 179 lines; SHA-256 `c97e7d3cddb2ddb50a16bfca9de69a9369e6285cd3fdeaf890f7d9eb9783eff2`.

Types: `HiveAimTable` (class, line 19), `Cell` (enum, line 26), `Shot` (class, line 36)

- L29: `public static Cell of(boolean red, boolean far)`
- L59: `private HiveAimTable()`
- L62: `public static synchronized boolean load()`
- L89: `public static boolean loaded()`
- L90: `public static String loadError()`
- L96: `public static boolean lookup(Cell cell, double x, double y, Shot out)`
- L138: `private static double clamp(double v, double lo, double hi)`
- L140: `private static byte[] inflate(byte[] data, int size) throws DataFormatException`
- L158: `private static byte[] base64(String s)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTableData.java`

### subsystem/aim/table/HiveAimTableData.java

Generated simulated shot-map data; use the generator workflow rather than editing values manually.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTableData.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTableData.java); 28 lines; SHA-256 `6e962344d16e0e9417e6b52ced30052b322ef0edddf2361d374cd7b9f615c986`.

Types: `HiveAimTableData` (class, line 10)

- L11: `private HiveAimTableData()`

Internal dependency candidates: none detected

### subsystem/aim/table/TunedShotPoints.java

Committed calibrated shot points.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java); 52 lines; SHA-256 `d5cc438f65e0d0930e51706b8800673728ce25de434e33cf05dc6571d388465e`.

Types: `TunedShotPoints` (class, line 17)

- L18: `private TunedShotPoints()`
- L49: `private static TunedShotTable.Point p(HiveAimTable.Cell cell, double x, double y, double rpm, double hood)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`

### subsystem/aim/table/TunedShotTable.java

Blends hand-tuned points; merges committed points with Control Hub shotpoints.csv; mirrors between cells.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java); 325 lines; SHA-256 `a3e16dc5d20cf2c6a33d62deca746fe56b5d40fb9c56c6f10a256643e8b8dec4`.

Types: `TunedShotTable` (class, line 40), `Point` (class, line 78), `Shot` (class, line 98)

- L83: `public Point(HiveAimTable.Cell cell, double x, double y, double rpm, double hood, boolean fromFile)`
- L88: `public String javaLine()`
- L113: `private TunedShotTable()`
- L116: `public static synchronized void load()`
- L147: `private static void rebuild()`
- L159: `private static void ensureLoaded()`
- L161: `public static File file()`
- L162: `public static String lastError()`
- L165: `private static HiveAimTable.Cell storedCell(HiveAimTable.Cell cell)`
- L174: `private static boolean isRed(HiveAimTable.Cell c)`
- L175: `private static boolean isFar(HiveAimTable.Cell c)`
- L179: `private static double storedX(HiveAimTable.Cell cell, double x)`
- L182: `private static double storedY(HiveAimTable.Cell cell, double y)`
- L194: `public static synchronized boolean lookup(HiveAimTable.Cell cell, double x, double y, Shot out)`
- L246: `public static synchronized Point save(HiveAimTable.Cell cell, double x, double y, double rpm, double hood) throws IOException`
- L263: `public static synchronized Point nearest(HiveAimTable.Cell cell, double x, double y)`
- L279: `public static double distance(HiveAimTable.Cell cell, double x, double y, Point p)`
- L284: `public static synchronized boolean delete(Point p) throws IOException`
- L292: `public static synchronized void clearFile() throws IOException`
- L299: `public static synchronized List<Point> filePoints()`
- L300: `public static synchronized int count(boolean fromFile)`
- L308: `public static synchronized List<String> printJava()`
- L316: `private static void writeFile() throws IOException`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotPoints.java`

### subsystem/aim/turret/TurretCalc.java

Turret servo gearing, angular limits, slew, alliance offsets and yaw compensation.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java); 216 lines; SHA-256 `dc8a0431bd0cb81a4ab21bcf22fc58f8de2765ec90a22adb16d40aa2c197b14c`.

Types: `TurretCalc` (class, line 10)

- L57: `public static double turretDegPerServoDeg()`
- L62: `public static double servoPositionFor(double turretDeg)`
- L80: `public static void reset()`
- L93: `public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance)`
- L110: `public static boolean outOfRange()`
- L112: `public static double aimErrorDeg()`
- L114: `private static double servoPosition(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance)`
- L130: `private static double normalizeDegrees(double degrees)`
- L134: `private static double getClosestValidTurretAngle(double relativeGoalDegrees)`
- L171: `private static double slewTurret(double target)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

### tele/Tele.java

Two-driver controls, field-centric drive, shooting gates, manual presets and live trims.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java); 220 lines; SHA-256 `1afbe086149ff3a45cfed22cbeae3929dd4713002319002e192ebe8928fe79ff`.

Types: `Tele` (class, line 23)

- L57: `public Tele(Alliance a)`
- L64: `public void initialize()`
- L91: `public void initializeLoop()`
- L102: `public void execute()`
- L115: `public void end()`
- L128: `private void driver1()`
- L160: `private void driver2(double dt)`
- L191: `private boolean canShoot()`
- L196: `private boolean shotOverride()`
- L198: `private double forwardOffsetRad()`
- L202: `private double turretOffset()`
- L203: `private void setTurretOffset(double deg)`
- L207: `private boolean usingSavedPose()`
- L209: `private Pose startPose()`
- L213: `private String notConnected()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

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

### test/FlywheelPIDFTest.java

Bench OpMode for intake, flywheel PIDF and solver-driven hood targets.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlywheelPIDFTest.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/FlywheelPIDFTest.java); 183 lines; SHA-256 `6abd6588439db4513fa3647dd411cf701e51473fffa5cdb3c9906dca5bc02272`.

Types: `FlywheelPIDFTest` (class, line 43)

- L69: `public void initialize()`
- L91: `public void initializeLoop()`
- L101: `public void execute()`
- L125: `private boolean showProblem()`
- L132: `private void runShooter()`
- L166: `private void solveForPose()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/HardwareNames.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Intake.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Shooter.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/HoodServoPositionTuning.java

Hood calibration OpMode.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/HoodServoPositionTuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/HoodServoPositionTuning.java); 61 lines; SHA-256 `f96b8537d199d700a8749234343a73d7cd33a7ebdc55572e8c1fa03e6b9b7a60`.

Types: `HoodServoPositionTuning` (class, line 21)

- L28: `public void initialize()`
- L36: `public void initializeLoop()`
- L42: `public void execute()`
- L55: `private void show()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Hood.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

### test/ShotPointTuner.java

Drive, tune, save and export calibrated shot points.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/ShotPointTuner.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/ShotPointTuner.java); 278 lines; SHA-256 `f373900620cc7194570c639cfd3a8093a1ccd175d2c1c7230d9af26a4dd1b41c`.

Types: `ShotPointTuner` (class, line 53)

- L81: `public void initialize()`
- L95: `public void initializeLoop()`
- L108: `public void onStart()`
- L114: `public void execute()`
- L133: `private void aim()`
- L152: `private void handleButtons()`
- L200: `private void nudge(double rpm, double hood)`
- L206: `private void save()`
- L222: `private void deleteNearest()`
- L240: `private void showTelemetry()`
- L275: `public void end()`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/Turret.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterConstants.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/HiveAimTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/table/TunedShotTable.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/turret/TurretCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tele/Tele.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/CommandOpMode.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/FloatSupplier.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/input/GamepadEx.java`

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

### util/OpModeStorage.java

Pose handoff between OpModes.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/OpModeStorage.java); 9 lines; SHA-256 `cd4cf3ba0275a02eccc48c2037c2744a12e4728b0b106ae4aa28c5af9dbd500a`.

Types: `OpModeStorage` (class, line 5)

- L6: `private OpModeStorage()`

Internal dependency candidates: none detected

### util/TeleTelemetry.java

Driver Station and Panels telemetry presentation.

Source: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/TeleTelemetry.java); 125 lines; SHA-256 `dbc9ac81cd7eb76c1cc1ec4c30ddb96d3c44c1e0bc5f0b3c0d794629c9d3efa4`.

Types: `TeleTelemetry` (class, line 12)

- L18: `public TeleTelemetry(Telemetry telemetry, Robot robot, Alliance a)`
- L24: `public void update(boolean shotOverride, double turretOffset)`
- L30: `public static String shotSourceText()`
- L46: `private String driver2Text(boolean shotOverride, double turretOffset)`
- L55: `private static String aimTableText()`
- L63: `private void driverStation(boolean shotOverride, double turretOffset)`
- L98: `private void panels(boolean shotOverride, double turretOffset)`

Internal dependency candidates: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShooterCalc.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/aim/shot/ShotCheck.java`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Alliance.java`

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

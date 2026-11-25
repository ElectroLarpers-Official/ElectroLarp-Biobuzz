package org.firstinspires.ftc.teamcode.tele;
import static com.pedropathing.ivy.commands.Commands.infinite;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.TeleTelemetry;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

@Configurable
public class Tele extends CommandOpMode {
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;
    public static boolean USE_SAVED_POSE = true;
    public static double RPM_TRIM_STEP = 25, HOOD_TRIM_STEP = 0.005, TURRET_TRIM_STEP_DEG = 1;
    public static double MANUAL_STICK_DEADBAND = 0.05;
    public static double TELEMETRY_PERIOD_MS = 100;

    private final Alliance a;
    private Robot robot;
    private GamepadEx gp1, gp2;
    private TeleTelemetry tele;
    private long lastExecNanos = 0;
    private long lastTelemetryNanos = 0;

    public Tele(Alliance a) {
        this.a = a;
    }

    // ---- Main -------------------------------------------------------------------------------------------

    @Override
    public void initialize() {
       // setTurretOffset(turretOffset()-3);
        // Robot builds the Pulse follower and attaches it to PulseRuntime itself (one follower, one Pinpoint setup).
        robot = new Robot(hardwareMap, a);
        poseFromAuto = usingSavedPose();
        robot.follower.setPose(startPose());
        // Use the auto's end pose once, then clear it so a later TeleOp run doesn't reuse a stale pose.
        OpModeStorage.endPose = null;
        robot.follower.update();
        robot.startTeleOpDrive();   // RevAmped arcade drive owns the drive motors from here

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        gp1.left_trigger_button(FloatSupplier::isPress);
        gp1.right_trigger_button(FloatSupplier::isPress);
        gp2.left_trigger_button(FloatSupplier::isPress);
        gp2.right_trigger_button(FloatSupplier::isPress);


        tele = new TeleTelemetry(telemetry, robot, a);
        schedule(infinite(() -> {
            double forward = -gp1.left_stick_y.getVal();
            double lateral = -gp1.left_stick_x.getVal();
            double turn = -gp1.right_stick_x.getVal();
            // The sticks go to Robot before update(), so the turret's stick blend uses this loop's sticks.
            robot.setDriverSticks(forward, lateral, turn, forwardOffsetRad());
            robot.update();
            robot.drivetrain.arcadeDrive(gamepad1);   // RevAmped arcade drive, writes the motors this loop
            long now = System.nanoTime();
            if ((now - lastTelemetryNanos) / 1e6 >= TELEMETRY_PERIOD_MS) {
                lastTelemetryNanos = now;
                tele.update(turretOffset()); //TODO turn off later
            }
        }));
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Pose source", poseFromAuto ? "last auto" : "START_X/Y");
        telemetry.addData("Not connected (skipped)", notConnected());
        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void execute() {
        gp1.update();
        gp2.update();
        long now = System.nanoTime();
        double dt = lastExecNanos == 0 ? 0.0 : Math.min(0.1, (now - lastExecNanos) / 1e9);
        lastExecNanos = now;

        // Driver 2 first: it can turn manual mode / the shot override on, which driver 1's shooting reads.
        driver2(dt);
        driver1();
    }

    @Override
    public void end() {
        if (robot == null) return;
        robot.stop();
    }

    // ---- Controls --------------------------------------------------------------------------------------------

    /**
     * Driver 1: LT hold = intake; RT hold = shoot; B hold = slow reverse (clear a jam; the bumpers are broken);
     * A = latch closed; Y = latch open; DPAD DOWN = flower open / close toggle; X = pose reset (robot squared in the reset corner, Robot.RESET_X / Y / HEADING_DEG).
     */
    private void driver1() {
        if (gp1.x.isRisingEdge()) {
            robot.resetPose();
            gamepad1.rumble(200);
        }

        if (gp1.left_trigger_button.isRisingEdge()) robot.intake.on();
        if (gp1.left_trigger_button.isFallingEdge()) robot.intake.off();

        // Right trigger held = shoot.
        if (gp1.right_trigger_button.isTrue()) {
            if (!robot.isShooting()) robot.startShooting();
        } else if (robot.isShooting()) {
            robot.stopShooting();
        }

        if (gp1.b.isRisingEdge()) robot.intake.slowReverse();
        if (gp1.b.isFallingEdge()) robot.intake.off();

        if (gp1.dpad_down.isRisingEdge()) robot.flower.toggle();

        if (gp1.a.isRisingEdge()) robot.manualLatch(false);   // A: latch closed
        if (gp1.y.isRisingEdge()) robot.manualLatch(true);    // Y: latch open
    }

    /**
     * Driver 2: d-pad up / down = RPM trim; d-pad right / left = hood trim (right = steeper);
     * LT / RT = turret offset for this alliance (LT = counter-clockwise; the bumpers are broken); BACK = all trims to 0;
     * LEFT STICK CLICK (or A outside manual) = turret straight forward on / off; X = manual mode on / off.
     * Manual mode runs Robot.MANUAL_RPM and Robot.MANUAL_HOOD, and right stick X turns the turret.
     */
    private void driver2(double dt) {
        if (gp2.dpad_up.isRisingEdge()) Robot.LIVE_RPM_TRIM += RPM_TRIM_STEP;
        if (gp2.dpad_down.isRisingEdge()) Robot.LIVE_RPM_TRIM -= RPM_TRIM_STEP;
        if (gp2.dpad_right.isRisingEdge()) Robot.LIVE_HOOD_TRIM += HOOD_TRIM_STEP;   // steeper
        if (gp2.dpad_left.isRisingEdge()) Robot.LIVE_HOOD_TRIM -= HOOD_TRIM_STEP;    // flatter
        if (gp2.left_trigger_button.isRisingEdge()) setTurretOffset(turretOffset() + TURRET_TRIM_STEP_DEG);
        if (gp2.right_trigger_button.isRisingEdge()) setTurretOffset(turretOffset() - TURRET_TRIM_STEP_DEG);
        if (gp2.back.isRisingEdge()) {
            Robot.LIVE_RPM_TRIM = 0;
            Robot.LIVE_HOOD_TRIM = 0;
            setTurretOffset(0);
            gamepad2.rumble(200);
        }
        boolean forwardToggle = gp2.left_stick_button.isRisingEdge()
                || (!robot.isManual() && gp2.a.isRisingEdge());   // A = easy turret forward / tracking toggle
        if (forwardToggle) {
            robot.setTurretForward(!robot.isTurretForward());
            gamepad2.rumble(robot.isTurretForward() ? 500 : 200);
        }
        if (gp2.x.isRisingEdge()) {   // X: manual <-> normal
            robot.setManual(!robot.isManual());
            gamepad2.rumble(robot.isManual() ? 500 : 200);
        }

        if (!robot.isManual()) return;
        double stick = gp2.right_stick_x.getVal();
        if (Math.abs(stick) > MANUAL_STICK_DEADBAND) {
            // stick right = turret clockwise (negative angle)
            robot.nudgeManualTurret(-stick * Robot.MANUAL_TURRET_RATE_DEG_S * dt);
        }
    }

    // ---- Helpers ---------------------------------------------------------------------------------------------

    /** Drivetrain's per-alliance forward offset, for the turret stick blend (setDriverSticks). */
    private double forwardOffsetRad() {
        return Math.toRadians(a == Alliance.BLUE ? Drivetrain.BLUE_FORWARD_OFFSET_DEG : Drivetrain.RED_FORWARD_OFFSET_DEG);
    }

    private double turretOffset() { return a == Alliance.RED ? TurretCalc.turretOffset : TurretCalc.turretOffset2; }
    private void setTurretOffset(double deg) {
        if (a == Alliance.RED) TurretCalc.turretOffset = deg; else TurretCalc.turretOffset2 = deg;
    }

    private boolean poseFromAuto;

    private boolean usingSavedPose() { return USE_SAVED_POSE && OpModeStorage.endPose != null; }

    private Pose startPose() {
        return usingSavedPose() ? OpModeStorage.endPose : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private String notConnected() {
        StringBuilder s = new StringBuilder();
        if (!Constants.drivetrainConnected) s.append("drive motors ");
        for (String name : robot.turret.missing()) s.append(name).append(' ');
        if (!robot.latch.isConnected()) s.append("latch ");
        if (!robot.flower.isConnected()) s.append("flower ");
        return s.length() == 0 ? "none" : s.toString().trim();
    }
}

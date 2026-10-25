package org.firstinspires.ftc.teamcode.tele;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.PulseRuntime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.TeleTelemetry;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

@Configurable
public class Tele extends CommandOpMode {
    /**
     * Rotates the field-centric frame so "stick forward" points away from the driver.
     * TODO: verify on the field which way each alliance's driver faces in Pedro coordinates.
     */
    public static double BLUE_FORWARD_OFFSET_DEG = 0;
    public static double RED_FORWARD_OFFSET_DEG = 180;

    /** Start pose when no saved pose is used, Pedro inches / degrees. */
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;
    /** Start from where the last auto / TeleOp ended instead of START_X/Y. Turn on once autos are running. */
    public static boolean USE_SAVED_POSE = false;

    /** Only shoot when ShotCheck says the shot can go in. Turn off to test shooting from anywhere. */
    public static boolean REQUIRE_SHOT_ALLOWED = true;
    /** Rumble length while trying to shoot a shot that can't go in, ms. */
    public static int CANT_SHOOT_RUMBLE_MS = 250;

    /** Driver 2 trim steps: motor RPM, hood servo position, turret degrees. */
    public static double RPM_TRIM_STEP = 25, HOOD_TRIM_STEP = 0.005, TURRET_TRIM_STEP_DEG = 1;
    /** Driver 2 right stick dead band in manual mode. */
    public static double MANUAL_STICK_DEADBAND = 0.05;

    private final Alliance a;
    private Robot robot;
    private GamepadEx gp1, gp2;
    private Command shootCycle;
    private TeleTelemetry tele;
    private long lastExecNanos = 0;

    public Tele(Alliance a) {
        this.a = a;
    }

    // ---- Main -------------------------------------------------------------------------------------------

    @Override
    public void initialize() {
        // Build the Pulse follower and attach it to PulseRuntime BEFORE constructing Robot,
        // because Robot reads it with PulseRuntime.follower(). This OpMode isn't a PulseOpMode,
        // so nothing attaches the follower for us — we do it here, the one thing PulseOpMode.init
        // would otherwise do. (configurePedro is a PulseOpMode instance method and can't be used here.)
        PulseRuntime.attach((PulseFollower) Constants.create(hardwareMap));
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(startPose());
        robot.follower.update();
        shootCycle = robot.shootCycle();

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        gp1.left_trigger_button(FloatSupplier::isPress);
        gp1.right_trigger_button(FloatSupplier::isPress);
        gp2.left_trigger_button(FloatSupplier::isPress);

        tele = new TeleTelemetry(telemetry, robot, a);
        schedule(infinite(() -> {
            robot.update();
            robot.drive(-gp1.left_stick_y.getVal(), -gp1.left_stick_x.getVal(), -gp1.right_stick_x.getVal(),
                    forwardOffsetRad());
            tele.update(shotOverride(), turretOffset()); //TODO turn off later
        }));
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Pose source", usingSavedPose() ? "auto / last run" : "START_X/Y");
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
        OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    // ---- Controls --------------------------------------------------------------------------------------------

    /**
     * Driver 1: LT hold = intake; RT hold = shoot while the shot can go in; LB hold = slow reverse (clear a jam);
     * A = one shoot cycle; X = pose reset (robot squared in the reset corner, Robot.RESET_X / Y / HEADING_DEG).
     * Rumbles while trying to shoot a shot that can't go in.
     */
    private void driver1() {
        if (gp1.x.isRisingEdge()) {
            robot.resetPose();
            gamepad1.rumble(200);
        }

        if (gp1.left_trigger_button.isRisingEdge()) robot.intake.on();
        if (gp1.left_trigger_button.isFallingEdge()) robot.intake.off();

        // Right trigger held = shoot, but only while the shot can go in. If it stops being possible mid-burst
        // (robot moved), shooting stops; it resumes on its own once the shot is possible again.
        boolean canShoot = canShoot();
        if (gp1.right_trigger_button.isTrue() && canShoot) {
            if (!robot.isShooting()) robot.startShooting();
        } else if (robot.isShooting()) {
            robot.stopShooting();
        }

        if (gp1.left_bumper.isRisingEdge()) robot.intake.slowReverse();
        if (gp1.left_bumper.isFallingEdge()) robot.intake.off();

        if (gp1.a.isRisingEdge() && canShoot && !Scheduler.isScheduled(shootCycle)) schedule(shootCycle);

        boolean tryingToShoot = gp1.right_trigger_button.isTrue() || gp1.a.isTrue();
        if (tryingToShoot && !canShoot && !gamepad1.isRumbling()) gamepad1.rumble(CANT_SHOOT_RUMBLE_MS);
    }

    /**
     * Driver 2: LT hold = shot-check override; d-pad up / down = RPM trim; d-pad right / left = hood trim
     * (steeper / flatter); RB / LB = turret offset for this alliance (+ = counter-clockwise); BACK = all trims to 0;
     * START = manual mode on / off. In manual mode: right stick X turns the turret, A / B / Y = close / mid / far preset.
     */
    private void driver2(double dt) {
        if (gp2.dpad_up.isRisingEdge()) Robot.LIVE_RPM_TRIM += RPM_TRIM_STEP;
        if (gp2.dpad_down.isRisingEdge()) Robot.LIVE_RPM_TRIM -= RPM_TRIM_STEP;
        if (gp2.dpad_right.isRisingEdge()) Robot.LIVE_HOOD_TRIM += HOOD_TRIM_STEP;
        if (gp2.dpad_left.isRisingEdge()) Robot.LIVE_HOOD_TRIM -= HOOD_TRIM_STEP;
        if (gp2.right_bumper.isRisingEdge()) setTurretOffset(turretOffset() + TURRET_TRIM_STEP_DEG);
        if (gp2.left_bumper.isRisingEdge()) setTurretOffset(turretOffset() - TURRET_TRIM_STEP_DEG);
        if (gp2.back.isRisingEdge()) {
            Robot.LIVE_RPM_TRIM = 0;
            Robot.LIVE_HOOD_TRIM = 0;
            setTurretOffset(0);
            gamepad2.rumble(200);
        }
        if (gp2.start.isRisingEdge()) {
            robot.setManual(!robot.isManual());
            gamepad2.rumble(robot.isManual() ? 500 : 200);
        }

        if (!robot.isManual()) return;
        if (gp2.a.isRisingEdge()) robot.setManualShot(Robot.ManualShot.CLOSE);
        if (gp2.b.isRisingEdge()) robot.setManualShot(Robot.ManualShot.MID);
        if (gp2.y.isRisingEdge()) robot.setManualShot(Robot.ManualShot.FAR);
        double stick = gp2.right_stick_x.getVal();
        if (Math.abs(stick) > MANUAL_STICK_DEADBAND) {
            // stick right = turret clockwise (negative angle)
            robot.nudgeManualTurret(-stick * Robot.MANUAL_TURRET_RATE_DEG_S * dt);
        }
    }

    // ---- Helpers ---------------------------------------------------------------------------------------------

    private boolean canShoot() {
        return !REQUIRE_SHOT_ALLOWED || shotOverride() || robot.isManual() || robot.shotAllowed();
    }

    /** Driver 2 holding the left trigger: shoot even where the shot check says no. */
    private boolean shotOverride() { return gp2.left_trigger_button.isTrue(); }

    private double forwardOffsetRad() {
        return Math.toRadians(a == Alliance.BLUE ? BLUE_FORWARD_OFFSET_DEG : RED_FORWARD_OFFSET_DEG);
    }

    private double turretOffset() { return a == Alliance.RED ? TurretCalc.turretOffset : TurretCalc.turretOffset2; }
    private void setTurretOffset(double deg) {
        if (a == Alliance.RED) TurretCalc.turretOffset = deg; else TurretCalc.turretOffset2 = deg;
    }

    private boolean usingSavedPose() { return USE_SAVED_POSE && OpModeStorage.endPose != null; }

    private Pose startPose() {
        return usingSavedPose() ? OpModeStorage.endPose : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private String notConnected() {
        StringBuilder s = new StringBuilder();
        if (!Constants.drivetrainConnected) s.append("drive motors ");
        for (String name : robot.turret.missing()) s.append(name).append(' ');
        if (!robot.latch.isConnected()) s.append("latch ");
        return s.length() == 0 ? "none" : s.toString().trim();
    }
}

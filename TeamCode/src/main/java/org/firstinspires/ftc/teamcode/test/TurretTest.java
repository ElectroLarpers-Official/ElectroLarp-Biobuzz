package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

/**
 * One turret bench test: 1 or 4 servos, and center / set position / track HIVE. Needs only the turret servo(s);
 * tracking also needs the Pinpoint + odo pods. No Robot, follower, drivetrain or shooter is built.
 *
 * <p>Servos: built with {@code new Turret(hw, true)}. With the Y-split, only {@code turretServo1} is in the config.
 * Without it, put the four Axons on their own ports as {@code turretServo1}-{@code turretServo4}; all mode writes
 * every one in the config with {@link Turret#setPositionAll}. X switches to single mode: {@link Turret#setPosition}
 * (turretServo1 only, same as Robot) with the others' PWM turned off so they go limp.
 *
 * <p>Init and run: X = 1 / all servos, dpad left = BLUE HIVE, dpad right = RED HIVE (used by tracking).
 * Run: A = center (0.5), B = set position, Y = track HIVE.
 * Set position: dpad up / down = +/- COARSE_STEP, right bumper / left bumper = +/- FINE_STEP.
 * Tracking: push or spin the robot by hand; back = reset the pose to START_X/Y/HEADING_DEG.
 */
@Configurable
@TeleOp(group = "Turret Full Test")
public class TurretTest extends CommandOpMode {
    /** Pose the robot is reset to at init and on back. Pedro inches / degrees. */
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;
    /** Set-position steps, servo units. */
    public static double COARSE_STEP = 0.01, FINE_STEP = 0.001;

    enum Mode { CENTER, SET_POSITION, TRACK }

    Turret turret;
    boolean singleServo = false;
    double position = Turret.CENTER;
    PinpointLocalizer localizer; // null if the Pinpoint isn't in the config or setup failed: tracking is unavailable
    String localizerProblem = null;
    GamepadEx gp1;

    Mode mode = Mode.CENTER;
    Alliance alliance = Alliance.BLUE;
    double setPosition = Turret.CENTER;
    double commandedDeg = 0;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        turret = new Turret(hardwareMap, true); // turretServo1 + any of turretServo2-4, all centered
        setTurret(Turret.CENTER);

        if (hardwareMap.tryGet(GoBildaPinpointDriver.class, HardwareNames.PINPOINT) == null) {
            localizerProblem = "Pinpoint \"" + HardwareNames.PINPOINT + "\" is not in the active robot config.";
        } else {
            try {
                localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
                resetPose();
            } catch (RuntimeException e) {
                localizerProblem = "Pinpoint setup failed (check Constants.localizerConfig): " + e;
            }
        }
        TurretCalc.reset();
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        if (showNoServos()) return;
        commonControls();
        if (localizer != null) localizer.update();
        setTurret(Turret.CENTER);
        show();
    }

    @Override
    public void execute() {
        gp1.update();
        if (showNoServos()) return;
        commonControls();
        if (gp1.a.isRisingEdge()) mode = Mode.CENTER;
        if (gp1.b.isRisingEdge()) {
            setPosition = position; // start from wherever the turret is now, so switching modes doesn't jump
            mode = Mode.SET_POSITION;
        }
        if (gp1.y.isRisingEdge() && localizer != null) {
            TurretCalc.reset();
            mode = Mode.TRACK;
        }

        if (localizer != null) localizer.update();

        switch (mode) {
            case CENTER:
                setTurret(Turret.CENTER);
                break;
            case SET_POSITION:
                if (gp1.dpad_up.isRisingEdge()) setPosition += COARSE_STEP;
                if (gp1.dpad_down.isRisingEdge()) setPosition -= COARSE_STEP;
                if (gp1.right_bumper.isRisingEdge()) setPosition += FINE_STEP;
                if (gp1.left_bumper.isRisingEdge()) setPosition -= FINE_STEP;
                setPosition = Range.clip(setPosition, 0.0, 1.0);
                setTurret(setPosition);
                break;
            case TRACK:
                if (gp1.back.isRisingEdge()) resetPose();
                track();
                break;
        }
        show();
    }

    private boolean showNoServos() {
        if (turret.servoCount() > 0) return false;
        telemetry.addLine("CANNOT RUN: no turret servo (\"" + HardwareNames.TURRET + "\" to \""
                + HardwareNames.TURRET_4 + "\") is in the active robot config.");
        telemetry.update();
        return true;
    }

    private void commonControls() {
        if (gp1.x.isRisingEdge()) setSingleServo(!singleServo);
        if (gp1.dpad_left.isRisingEdge() && mode != Mode.SET_POSITION) alliance = Alliance.BLUE;
        if (gp1.dpad_right.isRisingEdge() && mode != Mode.SET_POSITION) alliance = Alliance.RED;
    }

    private int activeServoCount() {
        return singleServo ? Math.min(1, turret.servoCount()) : turret.servoCount();
    }

    /** true: only turretServo1 gets a signal, the others' PWM is turned off so they go limp. */
    private void setSingleServo(boolean single) {
        singleServo = single;
        turret.setExtraServosEnabled(!single);
    }

    private void setTurret(double pos) {
        position = pos;
        if (singleServo) turret.setPosition(pos);
        else turret.setPositionAll(pos);
    }

    private void resetPose() {
        localizer.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        localizer.update();
    }

    /** Same aim Robot uses: the turret pose to the alliance's HIVE cell, relative to the robot heading. */
    private void track() {
        Pose pose = localizer.pose();
        Pose turretPose = Turret.getTurretPose(pose);
        Pose goal = ShooterCalc.goal(alliance, pose.y());
        double dx = goal.x() - turretPose.x();
        double dy = goal.y() - turretPose.y();
        commandedDeg = Math.IEEEremainder(Math.toDegrees(Math.atan2(dy, dx) - pose.heading()), 360.0);
        setTurret(TurretCalc.solve(commandedDeg, localizer.velocity().omega, alliance));
    }

    /** Servo position -> turret degrees from straight ahead (inverse of TurretCalc.servoPositionFor). */
    private static double turretDegFor(double servoPos) {
        double servoDeg = (servoPos - TurretCalc.SERVO_CENTER) * TurretCalc.SERVO_RANGE_DEG;
        return (TurretCalc.SERVO_REVERSED ? -1 : 1) * servoDeg * TurretCalc.turretDegPerServoDeg();
    }

    private void show() {
        telemetry.addData("Servos (X)", "%d of %d driven%s", activeServoCount(), turret.servoCount(),
                singleServo && turret.servoCount() > 1 ? " (others limp)" : "");
        telemetry.addData("Mode (A center / B set / Y track)", mode);
        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addData("Turret deg (from servo position)", "%.1f", turretDegFor(position));
        if (mode == Mode.SET_POSITION) {
            telemetry.addLine("dpad up/down +/-" + COARSE_STEP + ", RB/LB +/-" + FINE_STEP);
        }
        telemetry.addData("Alliance (dpad L/R)", alliance);
        if (localizer == null) {
            telemetry.addLine("Tracking unavailable: " + localizerProblem);
        } else {
            Pose pose = localizer.pose();
            telemetry.addData("Pose x / y / heading deg", "%.1f / %.1f / %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        if (mode == Mode.TRACK) {
            telemetry.addData("Tracking", "%s HIVE %s cell (back = reset pose)", alliance,
                    ShooterCalc.hiveAimFar() ? "TOP" : "BOTTOM");
            telemetry.addData("Commanded turret deg", "%.1f", commandedDeg);
            telemetry.addData("Aim error deg", "%.1f", TurretCalc.aimErrorDeg());
        }
        telemetry.update();
    }
}

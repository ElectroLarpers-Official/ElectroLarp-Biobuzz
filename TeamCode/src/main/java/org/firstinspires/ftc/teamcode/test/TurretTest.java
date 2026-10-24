package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

import java.util.ArrayList;
import java.util.List;

/**
 * One turret bench test: 1 or 4 servos, and center / set position / track HIVE. Needs only the turret servo(s);
 * tracking also needs the Pinpoint + odo pods. No Robot, follower, drivetrain or shooter is built.
 *
 * <p>Servos: built with {@code new Turret(hw, true)}. With the Y-split, only {@code turretServo1} is in the config.
 * Without it, put the four Axons on their own ports as {@code turretServo1}-{@code turretServo4}; all mode writes
 * every one in the config with {@link Turret#setPositionAll}. X switches to single mode: {@link Turret#setPosition}
 * (turretServo1 only, same as Robot) with the others' PWM turned off so they go limp.
 * TODO delete (quad turret): the X 1 / all mode and turretServo2-4; each spot is marked "TODO delete" below.
 *
 * <p>Init and run: X = 1 / all servos, dpad left = BLUE HIVE, dpad right = RED HIVE (used by tracking).
 * Run: A = center (0.5), B = set position, Y = track HIVE.
 * Set position: dpad up / down = +/- COARSE_STEP, right bumper / left bumper = +/- FINE_STEP.
 * Tracking: push or spin the robot by hand; back = reset the pose to START_X/Y/HEADING_DEG.
 *
 * <p>Smoothing (test only, on top of TurretCalc's own slew): every position the modes ask for is reached through a
 * speed + acceleration limited move that slows down before it arrives, so nothing snaps. Set position and tracking use
 * MAX_SPEED / MAX_ACCEL; going back to center (A) uses CENTER_SPEED / CENTER_ACCEL. An ACCEL of 0 means no ramp: a
 * slow snap, straight to the target at that speed, starting and stopping at once (center's default). SMOOTH = false
 * sends the raw positions like before (a full-speed snap).
 *
 * <p>Init: the Turret constructor sends center straight away, so it is only built once the turret is already there.
 * Until then the test writes turretServo1-4 itself and moves them to center (CENTER_SPEED / CENTER_ACCEL) from where
 * this test last left them: the servos go limp when an OpMode stops and stay about there. That is only known while the
 * app keeps running (center after an app restart or Sloth reload), and other OpModes (Tele) moving the turret aren't
 * seen, so leave the turret near center by hand in those cases or the first move still jumps.
 */
@Configurable
@TeleOp(group = "Turret Full Test")
public class TurretTest extends CommandOpMode {
    /** Pose the robot is reset to at init and on back. Pedro inches / degrees. */
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;
    /** Set-position steps, servo units. */
    public static double COARSE_STEP = 0.01, FINE_STEP = 0.001;
    /** Smoothing on/off, and its limits in turret degrees (converted to servo units with TurretCalc's gearing).
     *  ACCEL 0 = no ramp: constant-speed snap at the SPEED. */
    public static boolean SMOOTH = true;
    public static double MAX_SPEED_DEG_PER_SEC = 90, MAX_ACCEL_DEG_PER_SEC2 = 180;
    public static double CENTER_SPEED_DEG_PER_SEC = 60, CENTER_ACCEL_DEG_PER_SEC2 = 0;
    /** Longest loop time the smoothing steps over, s (a slow loop moves the turret at most this much time's worth). */
    private static final double MAX_DT = 0.05;
    /** Last servo position this test sent; where the next init's move to center starts. */
    private static double lastSentPosition = Turret.CENTER;

    enum Mode { CENTER, SET_POSITION, TRACK }

    Turret turret; // null until the init move to center is done (its constructor writes center straight away)
    final List<ServoImplEx> servos = new ArrayList<>(); // turretServo1-4 in the config; written directly until then
    double initStartPosition;
    boolean singleServo = false; // TODO delete (quad turret)
    double position = Turret.CENTER; // servo position actually sent (follows target through the smoothing)
    double target = Turret.CENTER;   // servo position the current mode asks for
    double velocity = 0;             // smoothing speed, servo units / s
    final ElapsedTime loopTimer = new ElapsedTime();
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
        // TODO delete: keep only TURRET / REVERSE_SERVO_1 in these two
        String[] names = {HardwareNames.TURRET, HardwareNames.TURRET_2, HardwareNames.TURRET_3, HardwareNames.TURRET_4};
        boolean[] reversed = {Turret.REVERSE_SERVO_1, Turret.REVERSE_SERVO_2, Turret.REVERSE_SERVO_3,
                Turret.REVERSE_SERVO_4};
        for (int i = 0; i < names.length; i++) {
            ServoImplEx servo = hardwareMap.tryGet(ServoImplEx.class, names[i]);
            if (servo == null) continue;
            // Same setup as Turret(hw, true), which applies it again to these same devices when it's built.
            // Keep in sync with Turret (REVERSE_SERVO_1-4) or the servos jump when it's built.
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
            servo.setDirection(reversed[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servos.add(servo);
        }
        position = initStartPosition = lastSentPosition;
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
        if (turret == null) { // START pressed before the init move to center finished: finish it first
            if (localizer != null) localizer.update();
            setTurret(Turret.CENTER);
            show();
            return;
        }
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
        if (!servos.isEmpty()) return false;
        // TODO delete: drop the "to TURRET_4" part
        telemetry.addLine("CANNOT RUN: no turret servo (\"" + HardwareNames.TURRET + "\" to \""
                + HardwareNames.TURRET_4 + "\") is in the active robot config.");
        telemetry.update();
        return true;
    }

    private void commonControls() {
        if (gp1.x.isRisingEdge()) setSingleServo(!singleServo); // TODO delete (quad turret)
        if (gp1.dpad_left.isRisingEdge() && mode != Mode.SET_POSITION) alliance = Alliance.BLUE;
        if (gp1.dpad_right.isRisingEdge() && mode != Mode.SET_POSITION) alliance = Alliance.RED;
    }

    // TODO delete (quad turret): activeServoCount and setSingleServo
    private int activeServoCount() {
        return singleServo ? 1 : servos.size();
    }

    /** true: only turretServo1 gets a signal, the others' PWM is turned off so they go limp. */
    private void setSingleServo(boolean single) {
        singleServo = single;
        if (turret != null) turret.setExtraServosEnabled(!single);
    }

    /** Moves the sent position toward {@code pos} (smoothed unless SMOOTH is off) and writes it to the servo(s). */
    private void setTurret(double pos) {
        target = pos;
        double dt = Math.min(loopTimer.seconds(), MAX_DT);
        loopTimer.reset();
        if (SMOOTH) {
            smoothTowardTarget(dt);
        } else {
            position = target;
            velocity = 0;
        }
        if (turret == null) {
            for (ServoImplEx servo : servos) servo.setPosition(position);
            if (position == Turret.CENTER && velocity == 0) { // at center: build Turret, its center write won't move it
                turret = new Turret(hardwareMap, true); // TODO delete: -> new Turret(hardwareMap)
                if (singleServo) turret.setExtraServosEnabled(false); // TODO delete (quad turret)
            }
        } else if (singleServo) { // TODO delete: keep only turret.setPosition(position) from this if / else
            turret.setPosition(position);
        } else {
            turret.setPositionAll(position);
        }
        lastSentPosition = position;
    }

    /** Speed + acceleration limited step: never faster than the speed that can still stop at the target. With no
     *  acceleration limit (0), it moves at the full speed from the first loop and stops dead on arrival. */
    private void smoothTowardTarget(double dt) {
        boolean center = mode == Mode.CENTER;
        double maxSpeed = servoUnits(center ? CENTER_SPEED_DEG_PER_SEC : MAX_SPEED_DEG_PER_SEC);
        double accel = servoUnits(center ? CENTER_ACCEL_DEG_PER_SEC2 : MAX_ACCEL_DEG_PER_SEC2);
        double error = target - position;
        if (accel <= 0) {
            velocity = Math.signum(error) * maxSpeed;
        } else {
            double wanted = Math.signum(error) * Math.min(maxSpeed, Math.sqrt(2 * accel * Math.abs(error)));
            velocity += Range.clip(wanted - velocity, -accel * dt, accel * dt);
        }
        double step = velocity * dt;
        if (Math.signum(step) == Math.signum(error) && Math.abs(step) >= Math.abs(error)) {
            position = target; // arrives this loop
            velocity = 0;
        } else {
            position += step;
        }
    }

    /** Turret degrees (or deg/s, deg/s^2) -> servo position units. */
    private static double servoUnits(double turretDeg) {
        return turretDeg / TurretCalc.turretDegPerServoDeg() / TurretCalc.SERVO_RANGE_DEG;
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
        if (turret == null) {
            telemetry.addData("Init", "moving to center from %.3f (where this test last left it)", initStartPosition);
        }
        // TODO delete (quad turret)
        telemetry.addData("Servos (X)", "%d of %d driven%s", activeServoCount(), servos.size(),
                singleServo && servos.size() > 1 ? " (others limp)" : "");
        telemetry.addData("Mode (A center / B set / Y track)", mode);
        telemetry.addData("Servo position (sent / target)", "%.3f / %.3f", position, target);
        telemetry.addData("Turret deg (from servo position)", "%.1f", turretDegFor(position));
        telemetry.addData("Smoothing", SMOOTH ? "%.0f turret deg/s, %.1f deg behind target" : "off",
                velocity / servoUnits(1), Math.abs(turretDegFor(target) - turretDegFor(position)));
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

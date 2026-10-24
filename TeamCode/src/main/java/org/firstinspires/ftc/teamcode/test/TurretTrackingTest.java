package org.firstinspires.ftc.teamcode.test;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

/**
 * Turret tracking on a partial robot: needs only the Pinpoint + odo pods and the turret servo.
 * No Robot, Follower, drivetrain, Shooter, Hood, Intake, Transfer or Latch is constructed, so no drive motors
 * have to be in the config or connected. The localizer is built from the same {@code Constants.localizerConfig}
 * the Follower uses, so paste the Pinpoint tuner's block there first.
 *
 * <p>Push or spin the robot by hand; the turret should keep pointing at the chosen HIVE cell. If the Pinpoint or
 * turret servo is missing from the active config, or the localizer config is incomplete, the OpMode shows what
 * is wrong on telemetry instead of throwing.
 *
 * <p>Init and run: dpad left = BLUE HIVE, dpad right = RED HIVE.
 * Run: A = reset pose to the start pose below, Y = toggle tracking (off holds the turret centered),
 * B = cycle which cell (up / down) of that HIVE it tracks: AUTO (picks by robot y, like Tele) / TOP / BOTTOM.
 */
@Configurable
@TeleOp(group = "Turret Test")
public class TurretTrackingTest extends CommandOpMode {
    /** Pose the robot is reset to at init and on A. Pedro inches / degrees. */
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;

    enum Aim { AUTO, TOP, BOTTOM }

    PinpointLocalizer localizer;
    Turret turret;
    GamepadEx gp1;
    String problem = null;

    Alliance alliance = Alliance.BLUE;
    Aim aim = Aim.AUTO;
    boolean tracking = true;
    double commandedDeg, servoPos, goalDistance;
    Pose goal = new Pose(0, 0), turretPose = new Pose(0, 0);

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);

        if (hardwareMap.tryGet(GoBildaPinpointDriver.class, HardwareNames.PINPOINT) == null) {
            problem = "Pinpoint \"" + HardwareNames.PINPOINT + "\" is not in the active robot config (I2C device).";
        } else if (hardwareMap.tryGet(ServoImplEx.class, HardwareNames.TURRET) == null) {
            problem = "Servo \"" + HardwareNames.TURRET + "\" is not in the active robot config.";
        }
        if (problem == null) {
            try {
                localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
                turret = new Turret(hardwareMap);
                resetPose();
            } catch (RuntimeException e) {
                problem = "Setup failed (check Constants.localizerConfig has name, pod offsets and directions): " + e;
            }
        }
        if (problem != null) return;

        TurretCalc.reset();
        schedule(infinite(this::track));
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        if (showProblem()) return;
        pickAlliance();
        localizer.update();
        telemetry.addData("Alliance (dpad L/R)", alliance);
        telemetry.addData("Pose", localizer.pose());
        telemetry.update();
    }

    @Override
    public void execute() {
        gp1.update();
        if (showProblem()) return;
        pickAlliance();
        if (gp1.b.isRisingEdge()) aim = Aim.values()[(aim.ordinal() + 1) % Aim.values().length];
        if (gp1.a.isRisingEdge()) resetPose();
        if (gp1.y.isRisingEdge()) {
            tracking = !tracking;
            if (!tracking) turret.setPosition(0.5);
            TurretCalc.reset();
        }
    }

    private boolean showProblem() {
        if (problem == null) return false;
        telemetry.addLine("CANNOT RUN: " + problem);
        telemetry.update();
        return true;
    }

    private void pickAlliance() {
        if (gp1.dpad_left.isRisingEdge()) alliance = Alliance.BLUE;
        if (gp1.dpad_right.isRisingEdge()) alliance = Alliance.RED;
    }

    private void resetPose() {
        localizer.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        localizer.update();
    }

    private Pose goal(Alliance a, double robotY) {
        Pose auto = ShooterCalc.goal(a, robotY); // always run so its top/bottom latch keeps up
        if (aim == Aim.AUTO) return auto;
        boolean red = a == Alliance.RED;
        if (aim == Aim.TOP) {
            return red ? new Pose(ShooterConstants.HIVE_RED_TOP_X, ShooterConstants.HIVE_RED_TOP_Y)
                    : new Pose(ShooterConstants.HIVE_BLUE_TOP_X, ShooterConstants.HIVE_BLUE_TOP_Y);
        }
        return red ? new Pose(ShooterConstants.HIVE_RED_BOTTOM_X, ShooterConstants.HIVE_RED_BOTTOM_Y)
                : new Pose(ShooterConstants.HIVE_BLUE_BOTTOM_X, ShooterConstants.HIVE_BLUE_BOTTOM_Y);
    }

    private void track() {
        localizer.update();
        Pose pose = localizer.pose();
        double omega = localizer.velocity().omega;
        turretPose = Turret.getTurretPose(pose);
        goal = goal(alliance, pose.y());
        double dx = goal.x() - turretPose.x();
        double dy = goal.y() - turretPose.y();
        goalDistance = Math.hypot(dx, dy);

        // Same angle Robot gets from ShooterCalc.solve(): goal bearing relative to the robot heading, degrees.
        commandedDeg = Math.IEEEremainder(Math.toDegrees(Math.atan2(dy, dx) - pose.heading()), 360.0);

        if (tracking) {
            servoPos = TurretCalc.solve(commandedDeg, omega, alliance);
            turret.setPosition(servoPos);
        }

        telemetry.addData("Alliance (dpad L/R)", alliance);
        telemetry.addData("Tracking (Y)", tracking ? "ON" : "OFF (centered)");
        telemetry.addData("Aim mode (B)", aim);
        telemetry.addData("Tracking", "%s HIVE %s", alliance,
                aim != Aim.AUTO ? aim : ShooterCalc.hiveAimFar() ? "TOP (auto)" : "BOTTOM (auto)");
        telemetry.addData("Pose x / y / heading deg", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret pose x / y", "%.1f / %.1f", turretPose.x(), turretPose.y());
        telemetry.addData("Goal x / y", "%.1f / %.1f", goal.x(), goal.y());
        telemetry.addData("Distance to goal", "%.1f", goalDistance);
        telemetry.addData("Commanded turret deg", "%.1f", commandedDeg);
        telemetry.addData("Servo position", "%.3f", servoPos);
        telemetry.addData("Out of range", TurretCalc.outOfRange());
        telemetry.addData("Aim error deg", "%.1f", TurretCalc.aimErrorDeg());
        telemetry.update();
    }
}

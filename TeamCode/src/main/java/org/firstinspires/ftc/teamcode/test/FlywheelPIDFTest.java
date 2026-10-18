package org.firstinspires.ftc.teamcode.test;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShotCheck;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

import java.util.List;

/**
 * Shooter bench test using the robot's real subsystems: Intake (intake + belted transfer), Shooter (Meta PIDF,
 * bang-bang while feeding) and Hood. No Robot, Follower, drivetrain, turret or latch is constructed.
 * Flywheel gains and directions live in Shooter / Intake and are edited in Panels there.
 *
 * <p>Gamepad 1: A = intake on/off, B = feed (intake on + flywheel in bang-bang, like shooting), Y = shooter on/off,
 * X = auto speed on/off.
 * Auto speed ON: the robot is assumed to sit at ROBOT_X / ROBOT_Y / ROBOT_HEADING_DEG (Pedro inches); target RPM
 * and hood come from the same solver + shot map TeleOp uses. Dpad moves that pose by POSE_STEP (up = +y,
 * right = +x), right bumper switches alliance.
 * Auto speed OFF: dpad up / down = target RPM +/- RPM_STEP, hood stays put.
 */
@Configurable
@TeleOp(group = "Shooter Test")
public class FlywheelPIDFTest extends CommandOpMode {
    /** Flywheel target in motor RPM when auto speed is off. */
    public static double TARGET_RPM = 3000;
    public static double RPM_STEP = 100;

    public static boolean AUTO_SPEED = true;
    /** Pretend robot pose, Pedro inches / degrees. (72, 72) is the field centre (inside the HIVE: no shot there). */
    public static double ROBOT_X = 72, ROBOT_Y = 72, ROBOT_HEADING_DEG = 0;
    public static double POSE_STEP = 6;
    public static boolean RED_ALLIANCE = true;
    /** Also drive the hood to the solver's angle (needs hoodServo in the config). */
    public static boolean AUTO_HOOD = true;

    Intake intake;
    Shooter shooter;
    Hood hood;   // null if hoodServo isn't configured
    List<LynxModule> hubs;
    GamepadEx gp1;
    TelemetryManager panels;
    String problem = null;

    boolean intakeOn = false, feeding = false, shooterOn = false;
    double loopMs = 0;
    long lastLoopNanos = 0;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        for (String name : new String[]{HardwareNames.INTAKE, HardwareNames.SHOOTER, HardwareNames.SHOOTER_2}) {
            if (hardwareMap.tryGet(DcMotorEx.class, name) == null) {
                problem = "Motor \"" + name + "\" is not in the active robot config.";
                return;
            }
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        if (hardwareMap.tryGet(Servo.class, HardwareNames.HOOD) != null) hood = new Hood(hardwareMap);
        ShooterCalc.resetProjectionFilter();   // also decodes the shot-map table

        schedule(infinite(this::runShooter));
    }

    @Override
    public void initializeLoop() {
        if (showProblem()) return;
        telemetry.addLine("A = intake, B = feed, Y = shooter, X = auto speed (dpad moves pose, RB = alliance)");
        telemetry.addData("Auto speed", AUTO_SPEED ? "ON at (" + ROBOT_X + ", " + ROBOT_Y + ")" : "OFF");
        telemetry.addData("Transfer motor", intake.hasTransferMotor() ? "found, runs with intake" : "not in config");
        telemetry.addData("Hood servo", hood != null ? "found" : "not in config (hood stays manual)");
        telemetry.update();
    }

    @Override
    public void execute() {
        if (showProblem()) return;
        for (LynxModule hub : hubs) hub.clearBulkCache();
        gp1.update();

        if (gp1.a.isRisingEdge()) intakeOn = !intakeOn;
        if (gp1.b.isRisingEdge()) feeding = !feeding;
        if (gp1.y.isRisingEdge()) shooterOn = !shooterOn;
        if (gp1.x.isRisingEdge()) AUTO_SPEED = !AUTO_SPEED;
        if (AUTO_SPEED) {
            if (gp1.dpad_up.isRisingEdge()) ROBOT_Y += POSE_STEP;
            if (gp1.dpad_down.isRisingEdge()) ROBOT_Y -= POSE_STEP;
            if (gp1.dpad_right.isRisingEdge()) ROBOT_X += POSE_STEP;
            if (gp1.dpad_left.isRisingEdge()) ROBOT_X -= POSE_STEP;
            if (gp1.right_bumper.isRisingEdge()) RED_ALLIANCE = !RED_ALLIANCE;
        } else {
            if (gp1.dpad_up.isRisingEdge()) TARGET_RPM += RPM_STEP;
            if (gp1.dpad_down.isRisingEdge()) TARGET_RPM = Math.max(0, TARGET_RPM - RPM_STEP);
        }

        if (intakeOn || feeding) intake.intake(); else intake.stop();
        shooter.setFeeding(feeding);
    }

    private boolean showProblem() {
        if (problem == null) return false;
        telemetry.addLine("CANNOT RUN: " + problem);
        telemetry.update();
        return true;
    }

    private void runShooter() {
        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;

        if (AUTO_SPEED) solveForPose();
        if (shooterOn && TARGET_RPM > 0) shooter.setTargetRpm(TARGET_RPM); else shooter.stop();
        shooter.update();

        panels.addData("Motor RPM", shooter.rpm());
        panels.addData("Target RPM", shooterOn ? TARGET_RPM : 0);
        panels.addData("RPM error", shooterOn ? TARGET_RPM - shooter.rpm() : 0);
        panels.addData("Shooter power", shooter.power());
        panels.addData("Battery V", shooter.voltage());
        panels.addData("Loop ms", loopMs);
        panels.addData("Mode", shooter.mode().name());
        panels.addData("Shooter (Y)", shooterOn ? "ON" : "OFF");
        panels.addData("Intake (A)", intakeOn ? "ON" : "OFF");
        panels.addData("Feeding (B)", feeding ? "ON" : "OFF");
        panels.addData("Auto speed (X)", AUTO_SPEED ? "ON" : "OFF");
        if (AUTO_SPEED) {
            panels.addData("Pose", String.format("(%.0f, %.0f) %.0f deg, %s alliance", ROBOT_X, ROBOT_Y,
                    ROBOT_HEADING_DEG, RED_ALLIANCE ? "RED" : "BLUE"));
            panels.addData("Aiming at", ShooterCalc.hiveAimFar() ? "far cell" : "audience cell");
            panels.addData("Range to window (in)", ShooterCalc.lastRange);
            panels.addData("Map launch / exit speed", String.format("%.1f deg / %.2f m/s", ShooterCalc.lastTableLaunchDeg,
                    ShooterCalc.lastTableSpeedMps));
            panels.addData("Shot", ShotCheck.status());
            panels.addData("Hood servo pos", hood != null ? String.format("%.3f", hood.position()) : "no servo");
        }
        panels.update(telemetry); // pushes to Panels and the Driver Station
    }

    /** Same solve TeleOp runs (Robot.updateAiming), for a robot standing still at the configured pose. */
    private void solveForPose() {
        Alliance alliance = RED_ALLIANCE ? Alliance.RED : Alliance.BLUE;
        Pose robotPose = new Pose(ROBOT_X, ROBOT_Y, Math.toRadians(ROBOT_HEADING_DEG));
        Pose turretPose = Turret.getTurretPose(robotPose);
        Pose goal = ShooterCalc.goal(alliance, robotPose.y());
        Vector2D turretToGoal = Turret.getTurretToGoalVector(turretPose, goal);
        if (ShooterConstants.underHive(robotPose.x(), robotPose.y())) {   // same hold as TeleOp
            TARGET_RPM = ShooterConstants.UNDER_HIVE_RPM;
            if (hood != null && AUTO_HOOD) hood.setLaunchAngle(ShooterConstants.UNDER_HIVE_LAUNCH_DEG);
            ShotCheck.blockUnderHive();
            return;
        }
        Double[] result = ShooterCalc.solve(robotPose.heading(), turretToGoal, Vector2D.zero(), Vector2D.zero(), 0.0);
        ShotCheck.check(alliance, goal);
        TARGET_RPM = ShooterCalc.requiredRPM;   // motor RPM (20T motor -> 40T flywheel)
        if (hood != null && AUTO_HOOD) hood.setPosition(result[1]);
    }
}

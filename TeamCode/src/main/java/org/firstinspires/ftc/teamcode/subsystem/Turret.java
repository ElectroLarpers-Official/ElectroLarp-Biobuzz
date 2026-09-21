package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public final class Turret {
    public static String SERVO_NAME = "turret";
    private final ServoImplEx turret;
    private double LAST_POSITION = -1;
    public static double SERVO_THRESHOLD = 0.0012;
    // TODO: replace both with the turret pivot offset from CAD (inches, measured from the robot's pose point; forward +, left +)
    public static double TURRET_FORWARD_OFFSET = -0.52588;
    public static double TURRET_STRAFE_OFFSET = 0;
    private double CENTER = 0.5;

    public Turret(HardwareMap hw) {
        turret = hw.get(ServoImplEx.class, SERVO_NAME);
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret.setPosition(CENTER);
    }

    public static Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.x() + TURRET_FORWARD_OFFSET * cos - TURRET_STRAFE_OFFSET * sin;
        double turretY = robotPose.y() + TURRET_FORWARD_OFFSET * sin + TURRET_STRAFE_OFFSET * cos;

        return new Pose(turretX, turretY, heading);
    }
    public static Vector2D getTurretToGoalVector(Pose turretPose, Alliance alliance) {
        Pose goal = (alliance == Alliance.RED)
                ? ShooterConstants.GOAL_POS_RED
                : ShooterConstants.GOAL_POS_BLUE;
        double dx = goal.x() - turretPose.x();
        double dy = goal.y() - turretPose.y();
        return Vector2D.polar(Math.hypot(dx, dy), Math.atan2(dy, dx));
    }

    public void setPosition(double pos) {
        if (LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            turret.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

}
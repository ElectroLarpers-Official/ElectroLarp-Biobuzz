package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.math.RotMatrix;
import org.firstinspires.ftc.teamcode.util.math.VectorMagnitude;
import org.firstinspires.ftc.teamcode.util.math.VectorPrint;
import org.firstinspires.ftc.teamcode.util.math.VectorTransform;

@Configurable
public final class Drivetrain {
    public static float MAGNITUDE_ZERO = 0.15f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;
    public static double CLAMP_POWER = 0;
    public static double MAX_LAMBDA = 0.95;
    public static double MAX_VELOCITY = 76;
    public static boolean tipCorrection = false;
    public static boolean FIELD_CENTRIC = true;

    public static double RED_FORWARD_OFFSET_DEG = 0;
    public static double BLUE_FORWARD_OFFSET_DEG = 180;

    private final PulseFollower follower;
    private final Alliance alliance;
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final boolean connected;
    private boolean prevVelZero;
    private String dbgRobotVel = "", dbgVoltage = "";
    private double dbgS = Double.NaN, dbgAlpha = Double.NaN, dbgG = Double.NaN, dbgLambda = Double.NaN;

    public Drivetrain(HardwareMap hardwareMap, PulseFollower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        leftFront = hardwareMap.tryGet(DcMotorEx.class, HardwareNames.FRONT_LEFT);
        leftRear = hardwareMap.tryGet(DcMotorEx.class, HardwareNames.BACK_LEFT);
        rightFront = hardwareMap.tryGet(DcMotorEx.class, HardwareNames.FRONT_RIGHT);
        rightRear = hardwareMap.tryGet(DcMotorEx.class, HardwareNames.BACK_RIGHT);
        connected = leftFront != null && leftRear != null && rightFront != null && rightRear != null;
    }
    public void startTeleOpDrive() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        if (!connected) return;
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

    public void setPowers(double lf, double lr, double rf, double rr) {
        if (lf == lr && lr == rf && rf == rr && lf == 0) {
            if (prevVelZero) {
                return;
            }

            prevVelZero = true;
        } else {
            prevVelZero = false;
        }

        if (!connected) return;
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    /**
     * Arcade drive method for controlling the robot using a gamepad.
     * @param x the x-value from the gamepad
     * @param y the y-value from the gamepad
     * @param rx the rotation value from the gamepad
     * @param DZ deadzone value for joystick input
     * @param angleZero the angle at which the gamepad is considered to be at zero
     * @param maxPower the maximum power for the motors
     */
    public void arcadeDrive(double x, double y, double rx, float DZ, float angleZero, float maxPower) {
        double robotHeading;

        if (Math.abs(y) > DZ || Math.abs(x) > DZ || Math.abs(rx) > DZ) {
            Vector2D movementVector = smoothGamepadAngle(x, y, angleZero);

            if (Math.abs(y) > DZ) {
                y = -0.5 * Math.tan(movementVector.y() * 1.12); // Remember, this is reversed!
            } else {
                y = 0;
            }

            if (Math.abs(x) > DZ) {
                x = 0.5 * Math.tan(movementVector.x() * 1.12);
            } else {
                x = 0;
            }

            if (Math.abs(rx) > DZ) {
                rx = 0.5 * Math.tan(rx * 1.12);
            } else {
                rx = 0;
            }

            if (FIELD_CENTRIC) {
                robotHeading = follower.pose().heading() + Math.toRadians(forwardOffsetDeg());
                double cos = Math.cos(-robotHeading);
                double sin = Math.sin(-robotHeading);
                double fieldX = x;
                double fieldY = y;
                x = fieldX * cos - fieldY * sin;
                y = fieldX * sin + fieldY * cos;
            }

            if (tipCorrection) {
                double heading = follower.pose().heading();
                Vector2D voltage = Vector2D.cartesian(x, y);
                Velocity v = follower.velocity();
                Vector2D vel = Vector2D.cartesian(v.vx, v.vy);
                Vector2D robotVel = VectorTransform.transform(vel, RotMatrix.rotMatrix(heading));
                robotVel = Vector2D.cartesian(robotVel.y(), -robotVel.x());
                dbgRobotVel = VectorPrint.print(robotVel);
                dbgVoltage = VectorPrint.print(voltage);
                double alpha = voltage.dot(robotVel)
                        / (VectorMagnitude.magnitude(voltage) * VectorMagnitude.magnitude(robotVel) + 1e-9);

                if (alpha < 0.2) {
                    Vector2D forwardHeadingVector = Vector2D.unit(heading);
                    double parallelVelNormalized = Math.abs(vel.dot(forwardHeadingVector)) / MAX_VELOCITY * 2;
                    double lambda = getLambda(parallelVelNormalized, alpha);
                    Vector2D e2 = Vector2D.cartesian(0, 1);
                    Vector2D forwardVoltage = voltage.projectOnto(e2);
                    Vector2D perpendicularVoltage = voltage.minus(forwardVoltage);
                    Vector2D adjustedVoltage = forwardVoltage.times(lambda).plus(perpendicularVoltage);
                    x = adjustedVoltage.x();
                    y = adjustedVoltage.y();
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPow = (y + x + rx) / denominator;
            double backLeftPow = (y - x + rx) / denominator;
            double frontRightPow = (y - x - rx) / denominator;
            double backRightPow = (y + x - rx) / denominator;

            double highestPower = Math.max(Math.max(frontLeftPow, backLeftPow), Math.max(frontRightPow, backRightPow));

            if (highestPower > maxPower) {
                frontLeftPow *= maxPower / highestPower;
                frontRightPow *= maxPower / highestPower;
                backRightPow *= maxPower / highestPower;
                backLeftPow *= maxPower / highestPower;
            }

            setPowers(frontLeftPow, backLeftPow, frontRightPow, backRightPow);
        } else if (!prevVelZero){
            setPowers(0,0,0,0);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        FIELD_CENTRIC = fieldCentric;
    }

    public boolean isFieldCentric() {
        return FIELD_CENTRIC;
    }

    private double forwardOffsetDeg() {
        return alliance == Alliance.BLUE ? BLUE_FORWARD_OFFSET_DEG : RED_FORWARD_OFFSET_DEG;
    }

    private Vector2D smoothGamepadAngle(double x, double y, double zeroAngle) {
        if (x == 0 || y==0) {
            return Vector2D.cartesian(x,y);
        } else if (Math.abs(y/x) < zeroAngle) {
            return Vector2D.cartesian(x, 0);
        } else if (Math.abs(x/y) < zeroAngle) {
            return Vector2D.cartesian(0,y);
        } else {
            return Vector2D.cartesian(x,y);
        }
    }

    public void arcadeDrive(double x, double y, double rx) {
        arcadeDrive(x, y, rx, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER);
    }

    public void arcadeDrive(Gamepad gamepad) {
        arcadeDrive(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);
    }

    public double getLambda(double s, double alpha) {
        dbgS = s;
        dbgAlpha = alpha;
        double g = (1 - alpha)/2;
        dbgG = g;
        double lambda = 1 - (1 - CLAMP_POWER) * s * s * g * g;
        dbgLambda = lambda;
        if (lambda > MAX_LAMBDA) return 1.0;
        return Math.max(lambda, 0);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (!tipCorrection) return;
        telemetry.addData("robotVel", dbgRobotVel);
        telemetry.addData("voltage", dbgVoltage);
        telemetry.addData("s", dbgS);
        telemetry.addData("alpha", dbgAlpha);
        telemetry.addData("g", dbgG);
        telemetry.addData("lambda", dbgLambda);
    }
}

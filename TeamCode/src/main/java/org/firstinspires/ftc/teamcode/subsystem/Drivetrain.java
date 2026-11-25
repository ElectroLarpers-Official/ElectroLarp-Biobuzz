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

/**
 * TeleOp arcade drive, ported from RevAmped ({@code mechanisms/Drivetrain.arcadeDrive}). The drive logic is
 * copied as-is: deadzone, axis snap, tan response curve, optional tip correction, mecanum mix and max-power cap.
 * Field-centric is the one deliberate change: RevAmped's rotation was broken (see {@link #FIELD_CENTRIC}), so it
 * is fixed here and given a per-alliance forward offset and a Panels switch. Pedro 2 math is written with Pedro 3 types; where Pedro 3 behaves differently,
 * the {@code util/math} helpers ({@link RotMatrix}, {@link VectorTransform},
 * {@link VectorMagnitude}, {@link VectorPrint}) reproduce the Pedro 2 / RevAmped result.
 *
 * Hardware writes are RevAmped's too: {@link #setPowers} sets the four drive motors directly, in the same
 * loop, with no {@code CachedMotor} threshold. That only works while the follower isn't driving them, so TeleOp
 * calls {@code Robot.startTeleOpDrive()}: {@code Robot.update()} then updates only the localizer (RevAmped's
 * {@code Drivetrain.update()} did the same). Autos never call it, so the follower drives them as before.
 * Motor directions are the ones Pedro's {@code Mecanum} set from {@code Constants} (same motor objects).
 * If a drive motor is missing (Pedro uses {@code NoDrivetrain}), the writes are skipped.
 */
@Configurable
public final class Drivetrain {
    public static float MAGNITUDE_ZERO = 0.15f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;
    public static double CLAMP_POWER = 0;
    public static double MAX_LAMBDA = 0.95;
    public static double MAX_VELOCITY = 76;
    public static boolean tipCorrection = false;

    /**
     * Field-centric switch (Panels, read every loop; {@link #setFieldCentric} sets it too). false = robot-centric:
     * stick forward = robot forward. true = stick forward = away from the driver, whatever way the robot faces.
     * Uses the Pinpoint heading, so reset the pose (driver 1 X) if it drifts.
     * Fixed versus RevAmped: RevAmped computed the new y from the already-rotated x, so the stick direction came out
     * wrong at any heading other than 0 / 180 deg, and it had no forward offset, so "forward" was field +X.
     */
    public static boolean FIELD_CENTRIC = false;
    /**
     * Per-alliance offset added to the heading so "stick forward" points away from that alliance's drivers (same
     * convention as the old Biobuzz {@code ManualDrive.fieldCentric} offset; stick forward = field angle -offset).
     * Checked against the BIOBUZZ manual, Section 9 (Fig. 9-1, 9-2, 9.5): the red ALLIANCE AREA is on the left
     * wall and the blue one on the right wall, seen from the audience. In this repo's Pedro frame +X runs red wall
     * to blue wall (red HIVE x 59.25, blue 84.75) and +Y runs away from the audience (audience CELL y 57.5, far
     * 86.5). So red drivers face +X (0 deg) and blue drivers face -X (180 deg).
     */
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

    // Tip-correction debug values (RevAmped sent these to telemetry from inside the drive call).
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

    /** RevAmped's {@code follower.startTeleOpDrive()} with {@code useBrakeModeInTeleOp(true)}: brake at zero power. */
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

    /**
     * Sets the power levels for each of the four motors in the system.
     *
     * @param lf the power level for the left front motor, must be between -1.0 and 1.0
     * @param lr the power level for the left rear motor, must be between -1.0 and 1.0
     * @param rf the power level for the right front motor, must be between -1.0 and 1.0
     * @param rr the power level for the right rear motor, must be between -1.0 and 1.0
     */
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
                // Stick (x = right, y = forward) is in the driver's frame; rotate it by -(heading + offset) into the
                // robot's frame. Both components come from the ORIGINAL x / y (RevAmped overwrote x first).
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

            // Copied as-is from RevAmped: signed max, not Math.abs.
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

    /**
     * Smooths the gamepad angle to meet human standards
     * @param x the x-value
     * @param y the y-value
     * @param zeroAngle the angle at which the gamepad is considered to be at zero
     * @return the smoothed x and y values
     */
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

    /**
     * Adds the tip-correction debug lines RevAmped printed ("robotVel", "voltage", "s", "alpha", "g", "lambda"),
     * using the latest values. Only while {@link #tipCorrection} is on. Called from TeleTelemetry, which is
     * throttled, so the drive loop itself doesn't add lines every cycle.
     */
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

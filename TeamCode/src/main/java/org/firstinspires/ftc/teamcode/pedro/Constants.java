package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.firstinspires.ftc.teamcode.HardwareNames;

/**
 * TODO(new robot): old-robot values. Copied from the old Pedro3+Ivy branch (tuned on last season's robot, commit c1856d8).
 * Re-tune on this robot with AutoTune and paste the results over them.
 * Exception: the Pinpoint pod offsets are from the new robot's CAD (pod directions are still old-robot).
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(HardwareNames.FRONT_LEFT);
                c.backLeftName.set(HardwareNames.BACK_LEFT);
                c.frontRightName.set(HardwareNames.FRONT_RIGHT);
                c.backRightName.set(HardwareNames.BACK_RIGHT);

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set(HardwareNames.PINPOINT);
                // Pod offsets from the new robot's CAD (Oct 2026), inches from the centre of rotation, which is
                // the pose point Pedro tracks. That is the middle of the four wheel axles, and the turret axis
                // sits on it (see Turret.TURRET_FORWARD_OFFSET). Intake end is front.
                // xPodOffset: forward pod's sideways offset, left +. It is 3.87 in left of centre.
                // yPodOffset: strafe pod's forward offset, forward +. It is 5.47 in behind centre.
                c.xPodOffset.set(3.87);
                c.yPodOffset.set(-5.47);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {

                Controller primaryTranslationalForward = Controller.proportional(0.25760027892657744);
                Controller secondaryTranslationalForward = Controller.proportional(0.0951764198571476);
                Controller primaryTranslationalLateral = Controller.proportional(0.37340514667602215);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13796322412751144);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010542930199001517));
                c.brake.set(Controller.proportionalFeedforward(0.008961490669151289));

                c.headingFeedback.set(Controller.proportional(3.4348127048292714));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.061035109599781784, 0.0029728769732944157));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0849998567427187, 0.0944417465344987));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015915370432974978, 9.841829622535892E-4));

                c.maxAchievableForwardVelocity.set(90.6671249800453);
                c.maxAchievableStrafeVelocity.set(76.48499327721291);
                c.naturalForwardDeceleration.set(106.3775078481255);
                c.naturalStrafeDeceleration.set(143.00122616090636);
            }
    );
    public static boolean drivetrainConnected = false;

    public static PulseFollower create(HardwareMap h) {
        drivetrainConnected = h.tryGet(DcMotorEx.class, HardwareNames.FRONT_LEFT) != null
                                && h.tryGet(DcMotorEx.class, HardwareNames.FRONT_RIGHT) != null
                               && h.tryGet(DcMotorEx.class, HardwareNames.BACK_LEFT) != null
                               && h.tryGet(DcMotorEx.class, HardwareNames.BACK_RIGHT) != null;
               Drivetrain drivetrain = drivetrainConnected ? new Mecanum(h, drivetrainConfig) : new NoDrivetrain();
        return new PulseFollower(
                new PinpointLocalizer(h, localizerConfig),
                //new Mecanum(h, drivetrainConfig),
                drivetrain,
                new Foresight(foresightConfig),
                foresightConfig);
    }
}

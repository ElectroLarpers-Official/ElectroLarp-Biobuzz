package org.firstinspires.ftc.teamcode.subsystem.aim.turret;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Stick blend (TeleOp only): lets the turret start moving when the driver moves the sticks, before the robot's
 * measured velocity has changed. It only changes where the turret points. The shot lead (velocity * flight time)
 * and the RPM always use the measured velocity and the current pose.
 *
 * <p>Each loop the sticks are turned into the velocity and turn rate the driver is asking for (stick * top speed).
 * Over the turret lead time T (TurretCalc.leadSec(): Meta's 0.05 s in Meta mode):
 * <ul>
 *   <li>driving: the turret aims from the current position shifted by
 *       STICK_BLEND_DRIVE * (stick velocity - measured velocity) * T,</li>
 *   <li>turning: the turn rate used for the turret's turn lead becomes
 *       measured + STICK_BLEND_TURN * (stick turn rate - measured).</li>
 * </ul>
 * Both are zero when the sticks match what the robot is doing, and both only act for STICK_BLEND_FADE_SEC after
 * the driver last moved the sticks, so holding the sticks against another robot (a pin) stops affecting the aim.
 * There is no cap on the size: a reversal needs a correction as big as a pin makes.
 *
 * <p>Robot feeds it: setSticks() from TeleOp each loop, then track() and apply(). Autos never call setSticks(),
 * so it does nothing there. No hardware and no FTC classes in here.
 */
@Configurable
public final class StickLead {
    /** How much of the driving stick / measured velocity difference is used. 0 = off. Tune by reversing hard
     *  across the goal while shooting: turret late = raise, early = lower. Above 1 is allowed. */
    public static double STICK_BLEND_DRIVE = 2;
    /** The same for the turn stick. 0 = off (the default). */
    public static double STICK_BLEND_TURN = 0;
    /** The stick blend acts for this long after the driver last moved the sticks, seconds, then eases out. Longer
     *  covers slow reversals fully, shorter stops a pin affecting the aim sooner. */
    public static double STICK_BLEND_FADE_SEC = 0.5;
    /** A stick counts as moved when it changes by more than this (stick units, 0..1) within about 0.1 s. */
    public static double STICK_MOVE_THRESHOLD = 0.1;
    /** Robot speed at full stick, in/s. Start values are Pedro's maxAchievableForward / StrafeVelocity (old-robot
     *  numbers): replace them with the speeds telemetry shows at full stick. */
    public static double TOP_FORWARD_IN_S = 100, TOP_STRAFE_IN_S = 80;
    /** Robot turn rate at full turn stick, deg/s. 300 is a placeholder, not measured: replace it with the turn
     *  rate telemetry shows at full stick. */
    public static double TOP_TURN_DEG_S = 300;

    /** After the fade time the blend goes from full to zero over this long, so the turret isn't dropped back in
     *  one step. */
    private static final double EASE_OUT_SEC = 0.15;
    /** Time constant of the slow copy of the sticks that "moved" is measured against, seconds. */
    private static final double STICK_FILTER_SEC = 0.1;
    private static final double NEVER = 1e9;

    // This loop's sticks (robot.drive()'s forward / lateral / turn) and how drive() rotates them.
    private boolean hasSticks = false;
    private double forward, lateral, turn, offsetRad;
    private boolean robotCentric;

    // Slow copies of the sticks, and time since each last moved.
    private boolean filterReady = false;
    private double slowForward, slowLateral, slowTurn;
    private double driveAge = NEVER, turnAge = NEVER;

    // Outputs of apply().
    private double aimDeg, omega, changeDeg;

    /** TeleOp, every loop before Robot.update(): the sticks exactly as given to Robot.drive(). */
    public void setSticks(double forward, double lateral, double turn, double offsetRad, boolean robotCentric) {
        this.forward = forward;
        this.lateral = lateral;
        this.turn = turn;
        this.offsetRad = offsetRad;
        this.robotCentric = robotCentric;
        hasSticks = true;
    }

    /** End of the loop: the sticks are used once. If TeleOp stops sending them (or it's an auto), nothing acts. */
    public void clearSticks() {
        hasSticks = false;
    }

    /** Every loop: updates the "time since the sticks moved" clocks. dtSec = time since the last call. */
    public void track(double dtSec) {
        if (!hasSticks) {
            filterReady = false;
            driveAge = NEVER;
            turnAge = NEVER;
            return;
        }
        if (!filterReady) {   // first loop with sticks: nothing has "moved" yet
            slowForward = forward;
            slowLateral = lateral;
            slowTurn = turn;
            filterReady = true;
        }
        if (dtSec < 0) dtSec = 0;
        boolean driveMoved = Math.hypot(forward - slowForward, lateral - slowLateral) > STICK_MOVE_THRESHOLD;
        boolean turnMoved = Math.abs(turn - slowTurn) > STICK_MOVE_THRESHOLD;
        driveAge = driveMoved ? 0.0 : Math.min(NEVER, driveAge + dtSec);
        turnAge = turnMoved ? 0.0 : Math.min(NEVER, turnAge + dtSec);

        double alpha = 1.0 - Math.exp(-dtSec / STICK_FILTER_SEC);
        slowForward += alpha * (forward - slowForward);
        slowLateral += alpha * (lateral - slowLateral);
        slowTurn += alpha * (turn - slowTurn);
    }

    /** 1 for STICK_BLEND_FADE_SEC after the sticks moved, then down to 0 over EASE_OUT_SEC. */
    private static double weight(double age) {
        if (age <= STICK_BLEND_FADE_SEC) return 1.0;
        return Math.max(0.0, 1.0 - (age - STICK_BLEND_FADE_SEC) / EASE_OUT_SEC);
    }

    /**
     * Works out the turret aim and turn rate with the stick blend applied. Read them with aimDeg() / omega().
     *
     * @param solverAimDeg  turret aim from the solver, degrees from the robot's front (CCW +)
     * @param aimRange      distance to the point the solver aims at, inches
     * @param heading       robot heading, radians
     * @param vx            measured field velocity, in/s
     * @param vy            measured field velocity, in/s
     * @param measuredOmega measured turn rate, rad/s (CCW +)
     * @param leadSec       the turret lead time T, seconds
     */
    public void apply(double solverAimDeg, double aimRange, double heading, double vx, double vy,
                      double measuredOmega, double leadSec) {
        aimDeg = solverAimDeg;
        omega = measuredOmega;
        changeDeg = 0.0;
        if (!hasSticks || leadSec <= 0.0) return;
        double driveGain = STICK_BLEND_DRIVE * weight(driveAge);
        double turnGain = STICK_BLEND_TURN * weight(turnAge);
        if (driveGain == 0.0 && turnGain == 0.0) return;

        // The mecanum drive scales all wheels down when forward + strafe + turn asks for more than full power.
        double scale = 1.0 / Math.max(1.0, Math.abs(forward) + Math.abs(lateral) + Math.abs(turn));

        if (driveGain != 0.0 && aimRange > 1e-6) {
            // Stick -> robot-frame powers, the way Robot.drive() does it (ManualDrive.fieldCentric).
            double rot = robotCentric ? 0.0 : -(heading + offsetRad);
            double powerForward = forward * Math.cos(rot) - lateral * Math.sin(rot);
            double powerLeft = forward * Math.sin(rot) + lateral * Math.cos(rot);
            // -> robot-frame velocity -> field velocity.
            double robotVx = powerForward * scale * TOP_FORWARD_IN_S;
            double robotVy = powerLeft * scale * TOP_STRAFE_IN_S;
            double stickVx = robotVx * Math.cos(heading) - robotVy * Math.sin(heading);
            double stickVy = robotVx * Math.sin(heading) + robotVy * Math.cos(heading);

            // Aim from the shifted position: the vector to the aim point gets shorter by the shift.
            double shiftX = driveGain * (stickVx - vx) * leadSec;
            double shiftY = driveGain * (stickVy - vy) * leadSec;
            double bearing = heading + Math.toRadians(solverAimDeg);
            double toAimX = aimRange * Math.cos(bearing);
            double toAimY = aimRange * Math.sin(bearing);
            double newX = toAimX - shiftX;
            double newY = toAimY - shiftY;
            if (Math.hypot(newX, newY) > 1e-6) {
                // Angle from the old aim direction to the new one (no wrap-around jumps).
                double deltaDeg = Math.toDegrees(Math.atan2(toAimX * newY - toAimY * newX, toAimX * newX + toAimY * newY));
                aimDeg = solverAimDeg + deltaDeg;
                changeDeg += deltaDeg;
            }
        }

        if (turnGain != 0.0) {
            double stickOmega = turn * scale * Math.toRadians(TOP_TURN_DEG_S);
            omega = measuredOmega + turnGain * (stickOmega - measuredOmega);
            // TurretCalc: turret angle -= turn rate * T.
            changeDeg -= Math.toDegrees((omega - measuredOmega) * leadSec);
        }
    }

    /** Turret aim with the stick blend, degrees from the robot's front (CCW +). */
    public double aimDeg() { return aimDeg; }
    /** Turn rate for the turret's turn lead with the stick blend, rad/s. */
    public double omega() { return omega; }
    /** How far the stick blend moved the turret this loop, degrees (telemetry / shot log). 0 = not acting. */
    public double changeDeg() { return changeDeg; }
}

package org.firstinspires.ftc.teamcode.subsystem.control;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Braking {

    /** Quadratic braking coefficient, s^2/in.
     *
     *  From the old robot's Pedro PredictiveBrakingCoefficients(0.18, 0.11091, 0.00097587)
     *  -> (prop, lin, quad). RevAmped's converted value (3.82e-5) is ~26x smaller because it
     *  was fit on a heavier robot with different wheels; only the STRUCTURE here is theirs.
     *
     *  Refit on the new robot (pedro/Constants quadraticBrakeCoefficients is the Foresight
     *  equivalent once AutoTune has been run). */
    public static double coefA = 0.00097587; // TODO(new robot): old-robot value

    /** Linear braking coefficient, seconds. Same source as {@link #coefA}. */
    public static double coefB = 0.11091; // TODO(new robot): old-robot value

    /** Linearisation reference speed, in/s. The Lambert solution is evaluated once at this
     *  speed and scaled linearly by actual speed, so it is most accurate near this value. */
    public static double coefV0 = 72.0; // TODO(new robot): old-robot value

    /**
     * Projection horizon, seconds. THEIRS IS 0.5 - start here and tune to this
     * robot. Note the cached factors are recomputed whenever this changes, so
     * it is safe to sweep live from Panels.
     *
     * This is a TUNING KNOB, not feederDelay + turretSettleTime: the long lookahead acts as
     * implicit turret-lag compensation. Do NOT combine it with an explicit command-rate
     * feedforward, or the same servo lag gets corrected twice.
     */
    public static double horizonSeconds = 0.5; // TODO(new robot): old-robot value

    private static double cachedSpeedFactor = 0.0;
    private static double cachedDistFactor = 0.0;
    private static double cachedA = Double.NaN;
    private static double cachedB = Double.NaN;
    private static double cachedV0 = Double.NaN;
    private static double cachedHorizon = Double.NaN;
    public static double lambertW(double x) {
        if (x < 0 && x < -1.0 / Math.E) return Double.NaN;

        double w = x > 1.0 ? Math.log(x) - Math.log(Math.log(x)) : x;
        if (w < 0) w = 0.0;

        for (int i = 0; i < 3; i++) {
            double ew = Math.exp(w);
            double f = w * ew - x;
            double num = 2.0 * ew * (1.0 + w);
            double denom = num - (f / (w + 1.0));
            if (Math.abs(denom) < 1e-300) break;
            double delta = f / denom;
            w -= delta;
            if (Math.abs(delta) < 1e-15) break;
        }
        return w;
    }

    private static double computeX(double t) {
        double leadingTerm = (2.0 * coefA / coefB) * coefV0;
        return leadingTerm * Math.exp(leadingTerm - (t / coefB));
    }

    private static void refreshFactors() {
        double wx0 = lambertW(computeX(0.0));
        double wxNew = lambertW(computeX(horizonSeconds));

        double ratio = (1.0 / coefV0) * coefB / 2.0 / coefA;
        double upper = (wxNew * wxNew / 2.0) + wxNew;
        double lower = (wx0 * wx0 / 2.0) + wx0;
        double integral = coefB * (lower - upper);

        cachedSpeedFactor = wxNew * ratio;
        cachedDistFactor = integral * ratio;

        cachedA = coefA;
        cachedB = coefB;
        cachedV0 = coefV0;
        cachedHorizon = horizonSeconds;
    }

    private static void ensureFresh() {
        if (cachedA != coefA || cachedB != coefB
                || cachedV0 != coefV0 || cachedHorizon != horizonSeconds
                || Double.isNaN(cachedA)) {
            refreshFactors();
        }
    }

    public static double speedAfter(double currentSpeed) {
        if (currentSpeed <= 1e-3) return 0.0;
        ensureFresh();
        double v = cachedSpeedFactor * currentSpeed;
        if (Double.isNaN(v) || v < 0.0) return 0.0;
        return Math.min(v, currentSpeed);
    }

    public static double distanceAfter(double currentSpeed) {
        if (currentSpeed <= 1e-3) return 0.0;
        ensureFresh();
        double d = cachedDistFactor * currentSpeed;
        if (Double.isNaN(d) || d < 0.0) return 0.0;
        return d;
    }

    public static double impliedDecel(double currentSpeed) {
        if (currentSpeed <= 1e-3 || horizonSeconds <= 1e-6) return 0.0;
        return (speedAfter(currentSpeed) - currentSpeed) / horizonSeconds;
    }
}
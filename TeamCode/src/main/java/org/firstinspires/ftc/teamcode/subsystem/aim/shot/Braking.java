package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Braking {
    public static double coefA = 0.00097587;
    public static double coefB = 0.11091;
    public static double coefV0 = 72.0;
    public static double horizonSeconds = 0.5;

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
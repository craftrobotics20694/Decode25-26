package org.firstinspires.ftc.teamcode.Decode.MathUtils;


public class mathFuncs {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    /**
     * @param value
     * @param divisor
     * @return Positive result of modulus operation
     */
    public static double mod(double value, double divisor) {
        value = value % divisor;
        if (value < 0) {
            value += divisor;
        }
        return (value);
    }

    public static double normalizeToPi(double angle) {
        // First, wrap to [0, 2π)
        angle = mod(angle, (2 * Math.PI));

        // Then shift to [-π, π)
        if (angle >= Math.PI)
            angle -= 2 * Math.PI;
        else if (angle < -Math.PI)
            angle += 2 * Math.PI;

        return angle;
    }
}
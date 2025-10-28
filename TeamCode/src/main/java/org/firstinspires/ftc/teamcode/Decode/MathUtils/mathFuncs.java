package org.firstinspires.ftc.teamcode.Decode.MathUtils;


import com.pedropathing.math.Vector;

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

    public static vector minVector(vector a, vector b){
        return (a.getMagnitude()<b.getMagnitude()) ? a : b;
    }
    public static vector maxVector(vector a, vector b){
        return (a.getMagnitude()<b.getMagnitude()) ? a : b;
    }

    public static double findNormalizingScaling(vector staticVector, vector variableVector, double maxPowerScaling) {
        double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
        double b = staticVector.getXComponent() * variableVector.getXComponent() + staticVector.getYComponent() * variableVector.getYComponent();
        double c = Math.pow(staticVector.getXComponent(), 2) + Math.pow(staticVector.getYComponent(), 2) - Math.pow(maxPowerScaling, 2);
        return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }
}
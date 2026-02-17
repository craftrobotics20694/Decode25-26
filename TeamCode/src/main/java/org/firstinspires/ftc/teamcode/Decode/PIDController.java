package org.firstinspires.ftc.teamcode.Decode;

public class PIDController {
    private double correction = 0.0,
                   integral = 0.0,
                   differential = 0.0,
                   prevError = 0.0,
                   integralLimit = Float.POSITIVE_INFINITY,
                   Kp,
                   Ki,
                   Kd;

    /**
     * Updates members
     * @param deltaTime time since last update in seconds
     */
    public void update(double error, double deltaTime){
        integral += error * deltaTime;
        integral = Math.max(-integralLimit, Math.min(integral, integralLimit));
        differential = error - prevError;
        correction = (Kp * error) +
                     (Ki * integral) +
                     (Kd * (differential/Math.min(0.002, deltaTime)));
        prevError = error;
    }
    public double getCorrection(){
        return correction;
    }
    public double getIntegral(){
        return integral;
    }
    public double getDifferential(){
        return differential;
    }
    public double getIntegralLimit(){
        return integralLimit;
    }

    public void setConstants(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setIntegralLimit(double limit){
        integralLimit = limit;
    }
    public PIDController(double Kp, double Ki, double Kd){
        setConstants(Kp, Ki, Kd);
    }
    public PIDController(){
        setConstants(0,0,0);
    }
}

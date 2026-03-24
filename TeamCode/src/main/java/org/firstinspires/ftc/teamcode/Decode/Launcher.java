package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Decode.PIDController;

public class Launcher {
    public double targetSpeed = 0,
                  launcherSpeed = 0,
                  power = 0.5;
    public PIDController PID = new PIDController(1,0.4,1.5);
    public DcMotor.Direction direction = DcMotor.Direction.REVERSE;
    private final double ticksPerRotation = 28,
                         percentageTolerance = 0.05;
    public double powerHeuristic = 76;
    public int speedLength = 50;
    private double[] speedArray = new double[speedLength];
    private int speedIndex = 0,
                prevPosition = 0;
    private DcMotor launcherMotor;

    /**
     * @param distance horizontal distance to target
     */
    public double getSpeedFor(double distance){
        double speed = ( (0.00261196) * (Math.pow(distance, 2)) - ((-0.161285)*distance) + 37.89638);
        return speed;
    }
    /**
     * Called once per loop
     */
    public void update(double deltaTime){
        launcherSpeed = 0;
        for(int i = 0; i < speedLength; i++){
            launcherSpeed += speedArray[i];
        }
        launcherSpeed /= speedLength;

        speedArray[speedIndex] = ((launcherMotor.getCurrentPosition() - prevPosition) /
                deltaTime) /
                ticksPerRotation;
        speedIndex = (speedIndex + 1) % speedLength;

        PID.update(targetSpeed - launcherSpeed, deltaTime);
        power = Math.max(0, Math.min(power+((PID.getCorrection())/10000), 1));

        prevPosition = launcherMotor.getCurrentPosition();
    }

    /**
     * Sets power variable based on a heuristic and resets integral, so idling doesn't have an effect.<br>
     * Use once at input to begin approach: if gamepadN.buttonWasPressed()
     * @return if launcher is at speed (probably not useful)
     */
    public boolean beginApproach(){
        power = Math.max(0, Math.min(targetSpeed/powerHeuristic, 1));
        PID.resetIntegral();
        return approachSpeed();
    }

    /**
     * Sets motor power to power variable
     * @return if launcher is at speed
     */
    public boolean approachSpeed(){
        launcherMotor.setPower(power);
        return(launcherSpeed > targetSpeed * (1-percentageTolerance));
    }
    public void setPower(double power){
        launcherMotor.setPower(power);
    }
    public double getLauncherSpeed(){return launcherSpeed;}
    public double getTargetSpeed(){return targetSpeed;}
    public double getPower(){return power;}
    public Launcher(DcMotor motor){
        PID.setIntegralLimit(0.4);
        launcherMotor = motor;
        launcherMotor.setDirection(direction);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public Launcher(HardwareMap hardwareMap, String name){
        this(hardwareMap.get(DcMotor.class, name));
    }
}

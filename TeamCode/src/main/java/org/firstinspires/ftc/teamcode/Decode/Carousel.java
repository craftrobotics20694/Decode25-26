package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Decode.PIDController;
public class Carousel {
    public PIDController PID = new PIDController(0.5,0.05, -0.2);
    private final double PIDIntegralLimit = 0.5;
    private DcMotor motor;
    public double position;
    public DcMotor.Direction motorDirection = DcMotor.Direction.REVERSE;
    private boolean liftPosition = false;
    private final double ticksPerPos = 928;
    //tolerance in servo positions (ergo trial and error)
    private final double liftTolerance = 0.05;
    //tolerance in ticks
    private final double carouselTolerance = 0.03;
    private final double basePower = 0.05;
    private Servo lift;
    private final double liftUp = 0.0;
    private final double liftDown = 1.0;

    public Carousel assignMotor(DcMotor motor) {
        this.motor = motor;
        this.motor.setDirection(motorDirection);
        return this;
    }

    public Carousel assignMotor(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        this.motor.setDirection(motorDirection);
        return this;
    }

    public Carousel assignLift(Servo servo) {
        lift = servo;
        return this;
    }

    public Carousel assignLift(HardwareMap hardwareMap, String name) {
        lift = hardwareMap.get(Servo.class, name);
        return this;
    }

    public void resetEncoding() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double distanceToPos(double position) {
        double motorPos = motor.getCurrentPosition() / ticksPerPos;
        return (position - motorPos);
    }


    public boolean approachPosition(double deltaTime, double position) {

        boolean inTolerance = (Math.abs(distanceToPos(position)) < carouselTolerance);
        PID.update(distanceToPos(position), deltaTime);

        motor.setPower(Math.max(-1, Math.min(PID.getCorrection(), 1))/3 + (basePower * Math.signum(PID.getCorrection())));
        return(inTolerance);
    }

    public boolean approachPosition(double deltaTime) {
        return(approachPosition(deltaTime, position));
    }

    public void liftUp() {
        lift.setPosition(liftUp);
        liftPosition = true;
    }

    public void liftDown() {
        lift.setPosition(liftDown);
        liftPosition = false;
    }

    public boolean getLiftPosition(){
        return(liftPosition);
    }

    public void incrementPosition(double increment) {
        position += increment;
    }

    public void setPosition(double position) {
        this.position = position;
    }
    public double getPosition()
    {
        return(position);
    }

    public Carousel(){
        PID.setIntegralLimit(PIDIntegralLimit);
    }
    public Carousel(DcMotor motor){
        assignMotor(motor);
        PID.setIntegralLimit(PIDIntegralLimit);
    }

    public Carousel(HardwareMap hardwareMap, String name){
        assignMotor(hardwareMap, name);
        PID.setIntegralLimit(PIDIntegralLimit);
    }
}
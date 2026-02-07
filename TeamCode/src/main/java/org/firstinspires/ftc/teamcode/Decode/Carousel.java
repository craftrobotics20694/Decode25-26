package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Carousel {
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
    private final double decay = 3;
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


    public boolean approachPosition(double position) {
        boolean inTolerance = (Math.abs(distanceToPos(position)) < carouselTolerance);
        motor.setPower((Math.signum(distanceToPos(position)) * (basePower + (1 - (1 / ((Math.abs(distanceToPos(position)) * decay) + 1))))));
        return(inTolerance);
    }

    public boolean approachPosition() {
        return(approachPosition(position));
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
}
package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Carousel {
    private DcMotor motor;
    public double position;
    private boolean liftPosition = false;
    private final double ticksPerPos = 250;
    //tolerance in servo positions (ergo trial and error)
    private final double liftTolerance = 0.05;
    //tolerance in ticks
    private final int carouselTolerance = 100;
    private final double decay = 0.001;
    private Servo lift;
    private final double liftUp = 0.5;
    private final double liftDown = 0;

    public Carousel assignMotor(DcMotor motor) {
        this.motor = motor;
        return this;
    }

    public Carousel assignMotor(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
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
        motor.setPower((Math.signum(distanceToPos(position) * (1 - (1 / ((Math.abs(distanceToPos(position)) * decay) + 1)))))/3);
        return(Math.abs(distanceToPos(position)) < carouselTolerance);
    }

    public boolean approachPosition() {
        return(approachPosition(position));
    }

    public boolean liftUp() {
        lift.setPosition(liftUp);
        liftPosition = true;
        return(Math.abs(liftUp - lift.getPosition()) < liftTolerance);
    }

    public boolean liftDown() {
        lift.setPosition(liftDown);
        liftPosition = false;
        return(Math.abs(liftDown - lift.getPosition()) < liftTolerance);
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
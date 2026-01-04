package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class carousel {
    private DcMotor motor;
    public double position;
    private final double ticksPerPos = 250;
    private final double tolerance = 0.05;
    private final double decay = 25;
    private Servo lift;
    private final double liftUp = 0.5;
    private final double liftDown = 0;

    public carousel assignMotor(DcMotor motor) {
        this.motor = motor;
        return this;
    }

    public carousel assignMotor(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        return this;
    }

    public carousel assignLift(Servo servo) {
        lift = servo;
        return this;
    }

    public carousel assignLift(HardwareMap hardwareMap, String name) {
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


    public void approachPosition(double position) {
        motor.setPower(-(1 - (1 / ((distanceToPos(position) * decay) + 1))));
    }

    public void approachPosition() {
        approachPosition(position);
    }

    public void liftUp() {
        lift.setPosition(liftUp);
    }

    public void liftDown() {
        lift.setPosition(liftDown);
    }

    public void incrementPosition(double increment) {
        position += increment;
    }

    public void setPosition(double position) {
        this.position = position;
    }
}
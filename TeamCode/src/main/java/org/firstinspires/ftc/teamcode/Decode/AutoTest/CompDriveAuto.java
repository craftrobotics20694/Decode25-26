package org.firstinspires.ftc.teamcode.Decode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class CompDriveAuto extends OpMode {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start(){
        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightFront.setPower(.5);
        rightBack.setPower(.5);
    }

    @Override
    public void loop(){}
}

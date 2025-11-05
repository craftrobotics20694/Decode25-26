package org.firstinspires.ftc.teamcode.Decode.manual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Decode.mecanumConstants;

@TeleOp
public class mecanumManip extends OpMode{
    DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(mecanumConstants.mecanumConstants.leftFrontMotorDirection);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(mecanumConstants.mecanumConstants.leftRearMotorDirection);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(mecanumConstants.mecanumConstants.rightFrontMotorDirection);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(mecanumConstants.mecanumConstants.rightRearMotorDirection);
    }

    @Override
    public void loop(){
        leftFront.setPower(-gamepad1.left_stick_y);
        leftBack.setPower(-gamepad1.left_stick_y);
        rightFront.setPower(-gamepad1.left_stick_y);
        rightBack.setPower(-gamepad1.left_stick_y);
    }
}

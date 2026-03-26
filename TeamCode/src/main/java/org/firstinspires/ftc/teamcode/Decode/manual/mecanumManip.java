package org.firstinspires.ftc.teamcode.Decode.manual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Decode.mecanumConstants;

@TeleOp
public class mecanumManip extends OpMode{
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, carousel;
    private double liftDown = 0.0;
    private double liftUp = 0.0;

    public void init(){
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(mecanumConstants.mecanumConstants.leftFrontMotorDirection);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(mecanumConstants.mecanumConstants.leftRearMotorDirection);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(mecanumConstants.mecanumConstants.rightFrontMotorDirection);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(mecanumConstants.mecanumConstants.rightRearMotorDirection);
    }

    public void loop(){
        leftFront.setPower(-gamepad1.left_stick_y);
        leftBack.setPower(-gamepad1.left_stick_y);
        rightFront.setPower(-gamepad1.left_stick_y);
        rightBack.setPower(-gamepad1.left_stick_y);

        carousel.setPower(gamepad2.right_stick_x);

        if (gamepad1.rightBumperWasPressed()){
            liftDown += 0.05;
        }
        if (gamepad1.leftBumperWasPressed()){
            liftDown -= 0.05;
        }
        if (gamepad1.circleWasPressed()){
            liftUp += 0.05;
        }
        if (gamepad1.triangleWasPressed()){
            liftUp -= 0.05;
        }

        telemetry.addData("Lift Down", liftDown);
        telemetry.addData("Lift Up", liftUp);
        telemetry.addData("Carousel ticks", carousel.getCurrentPosition());
    }
}
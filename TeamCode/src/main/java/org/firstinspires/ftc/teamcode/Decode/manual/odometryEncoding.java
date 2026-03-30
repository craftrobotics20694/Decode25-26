package org.firstinspires.ftc.teamcode.Decode.manual;

import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.*;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Decode.Launcher;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Decode.Carousel;
import org.firstinspires.ftc.teamcode.Decode.mecanumConstants;

@Configurable
@TeleOp
public class odometryEncoding extends OpMode {
    DcMotor leftEncoder, rightEncoder, strafeEncoder;
    @Override
    public void init() {
        leftEncoder = hardwareMap.get(DcMotor.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightFront");
        strafeEncoder = hardwareMap.get(DcMotor.class, "leftBack");
    }

    @Override
    public void loop(){
        telemetry.addData("leftEncoding", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoding", rightEncoder.getCurrentPosition());
        telemetry.addData("strafeEncoding", strafeEncoder.getCurrentPosition());
        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
    }

}
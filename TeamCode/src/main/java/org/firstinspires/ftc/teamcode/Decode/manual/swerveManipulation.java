package org.firstinspires.ftc.teamcode.Decode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs;
import org.firstinspires.ftc.teamcode.Decode.drivetrains.swervePod;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.Vector;

@TeleOp
public class swerveManipulation extends OpMode{
    private swervePod leftPod = new swervePod();
    private swervePod rightPod = new swervePod();
    private DcMotorEx left0, left1, right0, right1;

    private Vector targetVectorLeft = new Vector();
    private Vector targetVectorRight = new Vector();

    private int mode = 0;


    @Override
    public void init(){
        left0  = hardwareMap.get(DcMotorEx.class, "left0");
        left1  = hardwareMap.get(DcMotorEx.class, "left1");
        right0 = hardwareMap.get(DcMotorEx.class, "right0");
        right1 = hardwareMap.get(DcMotorEx.class, "right1");

        leftPod.assignMotors(
                hardwareMap.get(DcMotorEx.class, "left0"),
                hardwareMap.get(DcMotorEx.class, "left1")
                );
        rightPod.assignMotors(
                hardwareMap.get(DcMotorEx.class, "right0"),
                hardwareMap.get(DcMotorEx.class, "right1")
        );
    }
    @Override
    public void loop(){
        telemetry.update();
        targetVectorLeft .setOrthogonalComponents(gamepad1.left_stick_x,-gamepad1.left_stick_y);
        targetVectorRight.setOrthogonalComponents(gamepad1.right_stick_x,-gamepad1.right_stick_y);

        //Stick input can sometimes exceed a magnitude of 1.0
        targetVectorLeft .setMagnitude(mathFuncs.clamp(targetVectorLeft .getMagnitude(),0.0,1.0));
        targetVectorRight.setMagnitude(mathFuncs.clamp(targetVectorRight.getMagnitude(),0.0,1.0));

        switch(mode) {
            case 0:
                left0.setPower(targetVectorLeft.getYComponent());
                left1.setPower(targetVectorLeft.getXComponent());
                break;
            case 1:
                leftPod.turnAndDrive(targetVectorLeft);
                rightPod.turnAndDrive(targetVectorRight);
                break;
            case 2:
                leftPod.turnAndDrive(targetVectorLeft);
                rightPod.turnAndDrive(targetVectorLeft);
                break;
        }

        if (gamepad1.aWasPressed()){ mode = (mode + 1) % 3; }
        if (gamepad1.bWasPressed()){
            leftPod.resetEncoding();
            rightPod.resetEncoding();
        }


        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("leftPodMotor0 ticks", leftPod.getTicks()[0]);
        telemetry.addData("leftPodMotor1 ticks", leftPod.getTicks()[1]);
        telemetry.addData("Mode", mode);

        if (mode == 1){
            double[] debug = leftPod.getDriveComponents(targetVectorLeft);
            String deltaTheta = Double.toString(debug[0]);
            String turnDirection = Double.toString(debug[1]);
            String drivePower = Double.toString(debug[2]);
            String turnPower = Double.toString(debug[3]);
            telemetry.addData("deltaTheta", deltaTheta);
            telemetry.addData("oppositeDeltaTheta", debug[4]);
            telemetry.addData("turnDirection", turnDirection);
            telemetry.addData("drivePower", drivePower);
            telemetry.addData("turnPower", turnPower);
        }
    }
}

package org.firstinspires.ftc.teamcode.Decode.manual;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Decode.drivetrains.Swerve;
import org.firstinspires.ftc.teamcode.Decode.swerveConstants;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs;
import org.firstinspires.ftc.teamcode.Decode.drivetrains.swervePod;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;

@TeleOp
public class swerveManip extends OpMode{
    private Swerve drivetrain;
    private Follower follower;
    private final double startingHeading = Math.toRadians(90);

    private final swervePod
            leftPod  = new swervePod(),
            rightPod = new swervePod();

    private DcMotorEx left0, left1, right0, right1;

    private vector
            leftStickVector  = new vector(),
            rightStickVector = new vector(),
            leftTurnVector   = swerveConstants.swerveConstants.leftPodTurn,
            rightTurnVector  = swerveConstants.swerveConstants.leftPodTurn,
            leftPodTargetVector = new vector(),
            rightPodTargetVector = new vector();

    private int mode = 0;
    
    @Override
    public void init(){
//        follower = swerveConstants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(0,0,startingHeading));
//        follower.update();
        
        left0  = hardwareMap.get(DcMotorEx.class, "left0" );
        left1  = hardwareMap.get(DcMotorEx.class, "left1" );
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
         drivetrain = new Swerve(hardwareMap, swerveConstants.swerveConstants);
    }
    
    @Override
    public void start(){
//        follower.startTeleopDrive();
    }
    
    @Override
    public void loop(){
        telemetry.update();
//        follower.update();
        leftStickVector. setOrthogonalComponents(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        rightStickVector.setOrthogonalComponents(gamepad1.right_stick_x,-gamepad1.right_stick_y);

        //Stick input can sometimes exceed a magnitude of 1.0
        leftStickVector. setMagnitude(mathFuncs.clamp(leftStickVector .getMagnitude(),0.0,1.0));
        rightStickVector.setMagnitude(mathFuncs.clamp(rightStickVector.getMagnitude(),0.0,1.0));

        switch(mode) {
            case 0:
                //Most direct mechanics, each pod relates to a stick, left-right controls yaw rotation, up-down controls drive power
                left0 .setPower(leftStickVector .rotated(Math.toRadians(-45)).getYComponent());
                left1 .setPower(leftStickVector .rotated(Math.toRadians(-45)).getXComponent());
                right0.setPower(rightStickVector.rotated(Math.toRadians(-45)).getYComponent());
                right1.setPower(rightStickVector.rotated(Math.toRadians(-45)).getXComponent());
                break;
            case 1:
                //Utilizing swervePod.turnAndDrive robot will strafe in direction of left stick input
                leftPod .turnAndDrive(leftStickVector);
                rightPod.turnAndDrive(leftStickVector);
                break;
            case 2:
                //Robot-centric strafe and turn, with naive turn logic
                //Set target vector as sum of strafe vector (leftStickVector), and turn vector (leftTurnVector) times right stick input
                leftPodTargetVector  = leftStickVector.plus(leftTurnVector .times(rightStickVector.getXComponent()));
                rightPodTargetVector = leftStickVector.plus(rightTurnVector.times(rightStickVector.getXComponent()));
                double maxMagnitude = Math.max(leftPodTargetVector.getMagnitude(), rightPodTargetVector.getMagnitude());
                leftPodTargetVector = leftPodTargetVector.times(1/maxMagnitude);
                rightPodTargetVector = rightPodTargetVector.times(1/maxMagnitude);

                //Run motors
                leftPod.turnAndDrive(leftPodTargetVector);
                rightPod.turnAndDrive(rightPodTargetVector);
                break;
//            case 3:
//                //Field centric strafe and turn, with advanced turn logic
//                //Mirrors Swerve.calculateDrive
//                leftStickVector.rotateVector(-(follower.getHeading()-startingHeading));
//                rightStickVector.rotateVector(-(follower.getHeading()-startingHeading));
//                leftPod.turnAndDrive(leftStickVector.plus(leftTurnVector));
//                rightPod.turnAndDrive(rightStickVector.plus(rightTurnVector));
//                break;
        }

        if (gamepad1.aWasPressed()){ mode = (mode + 1) % 3; }
        if (gamepad1.bWasPressed()){
            leftPod .resetEncoding();
            rightPod.resetEncoding();
        }

        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("leftPodMotor0 ticks" , leftPod .getTicks()[0]);
        telemetry.addData("leftPodMotor1 ticks" , leftPod .getTicks()[1]);
        telemetry.addData("rightPodMotor0 ticks", rightPod.getTicks()[0]);
        telemetry.addData("rightPodMotor1 ticks", rightPod.getTicks()[1]);
        telemetry.addData("Mode", mode);
    }
}

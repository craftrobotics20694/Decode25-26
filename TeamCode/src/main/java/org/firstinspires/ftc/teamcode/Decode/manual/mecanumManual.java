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
public class mecanumManual extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(9,9, Math.toRadians(90));
    private final Pose redObelisk = new Pose(136, 136, 0);
    private final Pose blueObelisk = new Pose(8, 136, 0);
    private Pose targetObelisk = redObelisk;

    private boolean fieldCentric = false;
    private vector targetVector = new vector();
    private Servo lift;
    private double targetSpeed = 20;
    private final double liftTimeToPosition = 0.8;
    private double startTime,
                   prevTime = -0.0001;
    private boolean canMoveCarousel = false,
                    canShoot,
                    carouselMoving = false,
                    inHalfPosition = false;
    private Carousel carousel = new Carousel();
    private Launcher launcher;
    private DcMotor carouselMotor,  intake, leftFront, leftBack, rightFront, rightBack;
    private int debugVal = 0;

    public void init() {
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel");
        lift = hardwareMap.get(Servo.class, "lift");
        
        carousel.assignMotor(carouselMotor);
        carousel.assignLift(lift);
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher = new Launcher(hardwareMap, "launcher");
        launcher.speedLength = 10;
        launcher.PID.setConstants(1.75,0.3,2.0);
        launcher.PID.setIntegralLimit(0.4);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = mecanumConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    public void loop() {
        //Call these once per loop
        double deltaTime = time - prevTime;
        follower.update();
        launcher.update(deltaTime);

        //Blue controller===================================================
        targetVector.setOrthogonalComponents(gamepad1.left_stick_x,-gamepad1.left_stick_y);

        follower.setTeleOpDrive(
                targetVector.getYComponent(),
                -targetVector.getXComponent(),
                -gamepad1.right_stick_x,
                true
        );

        intake.setPower(gamepad1.right_trigger);

        if(gamepad1.xWasPressed()) fieldCentric = !fieldCentric;

        if(gamepad1.leftBumperWasPressed()){
            carousel.incrementPosition(-1 + mod(carousel.getPosition(), 1));
            inHalfPosition = false;
        }

        if(gamepad1.rightBumperWasPressed()){
            carousel.incrementPosition(1 - mod(carousel.getPosition(), 1));
            inHalfPosition = false;
        }

        if((gamepad1.square && gamepad1.dpadLeftWasPressed())||(gamepad1.squareWasPressed()&&gamepad1.dpad_left)){
            carousel.setPosition(0);
        }

        //Red controller====================================================

        if(gamepad2.leftBumperWasPressed()){
            carousel.incrementPosition(-0.5 - mod(carousel.getPosition(), 1));
            inHalfPosition = true;
        }

        if(gamepad2.rightBumperWasPressed()){
            carousel.incrementPosition(0.5 + mod(carousel.getPosition(), 1));
            inHalfPosition = true;
        }
        if(gamepad2.squareWasPressed()){
            debugVal = 0;
        }

        double distance = Math.sqrt(
                    Math.pow(targetObelisk.getX() - follower.getPose().getX(), 2) +
                    Math.pow(targetObelisk.getY() - follower.getPose().getY(), 2)
                );
        if(Math.abs(gamepad2.right_stick_y)<0.3) {
            launcher.targetSpeed = launcher.getSpeedFor(distance);
            if (gamepad2.circleWasPressed()) {
                canShoot = false;
                launcher.beginApproach();
                debugVal = 1;
            } else if (gamepad2.circle) {
                boolean tempBool = launcher.approachSpeed();
                canShoot = canShoot || tempBool;
            } else {
                launcher.setPower(0);
            }
        }
        else {
            launcher.setPower(-gamepad2.right_stick_y);
            canShoot = true;
        }

        //Carousel shenanigans==============================================

        //Triangle pressed and carousel not moving
        if(gamepad2.triangle && (!carouselMoving) && inHalfPosition && canShoot){
            carousel.liftUp();
            canMoveCarousel = false;
            startTime = time;
        }
        //Circle not pressed (doesn't matter if carousel is moving or not)
        else{
            canMoveCarousel = time > (startTime + liftTimeToPosition);
            carousel.liftDown();
        }

        //Try carousel movement
        if(canMoveCarousel) {
            //Unwinding of carousel
            if (Math.abs(carousel.getPosition()) >= 6) {
                carousel.setPosition(carousel.getPosition() % 3);
            }

            //Moves and checks movement of carousel
            carouselMoving = !carousel.approachPosition(deltaTime);
        }
        else {carouselMotor.setPower(0);}

        //Telemetry
        telemetry.addData("targetSpeed", launcher.targetSpeed);
        telemetry.addData("launcherSpeed", launcher.getLauncherSpeed());
        telemetry.addData("distance", distance);
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Field Centric", fieldCentric);
//        telemetry.addData("Launcher Power", launcher.getPower());
//        telemetry.addData("carouselPos", carousel.position);
//        telemetry.addData("carouselDistance", carousel.distanceToPos(carousel.position));
        telemetry.addData("deltaTime", deltaTime);
//        telemetry.addData("debugVal", debugVal);
//        telemetry.addData("launcherPower", launcher.getPower());
        telemetry.addData("stickY", gamepad1.left_stick_y);
        telemetry.addData("stickX", gamepad1.left_stick_x);

        prevTime = time;
    }
}
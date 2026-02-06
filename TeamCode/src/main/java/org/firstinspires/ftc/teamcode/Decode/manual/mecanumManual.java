package org.firstinspires.ftc.teamcode.Decode.manual;

import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.*;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Decode.Carousel;
import org.firstinspires.ftc.teamcode.Decode.mecanumConstants;

@Configurable
@TeleOp
public class mecanumManual extends OpMode {
    private Follower follower;
    private static double startingHeading = Math.toRadians(90);
    public static Pose startingPose = new Pose(0,0, startingHeading); //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;
    private boolean fieldCentric = false;
    private vector targetVector = new vector();
    private Servo lift;
    private boolean canMoveCarousel = false;
    private boolean carouselMoving = false;
    private Carousel carousel = new Carousel();
    private DcMotor carouselMotor, launcher, intake;

    public void init() {
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel");
        lift = hardwareMap.get(Servo.class, "lift");
        
        carousel.assignMotor(carouselMotor);
        carousel.assignLift(lift);
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);

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
        //Call this once per loop
        follower.update();

        //Blue controller===================================================
        targetVector.setOrthogonalComponents(gamepad1.left_stick_x,-gamepad1.left_stick_y);

        if(fieldCentric) targetVector.rotateVector(-(follower.getHeading() - startingHeading));

        follower.setTeleOpDrive(
                targetVector.getYComponent(),
                targetVector.getXComponent(),
                gamepad1.right_stick_x,
                false
        );

        intake.setPower(gamepad1.right_trigger);

        if(gamepad1.xWasPressed()) fieldCentric = !fieldCentric;

        if(gamepad1.leftBumperWasPressed()){
            carousel.incrementPosition(-1 + mod(carousel.getPosition(), 1));
        }

        if(gamepad1.rightBumperWasPressed()){
            carousel.incrementPosition(1 - mod(carousel.getPosition(), 1));
        }

        //Red controller====================================================

        launcher.setPower(gamepad2.right_stick_y);

        if(gamepad2.leftBumperWasPressed()){
            carousel.incrementPosition(-0.5 - mod(carousel.getPosition(), 1));
        }

        if(gamepad2.rightBumperWasPressed()){
            carousel.incrementPosition(0.5 + mod(carousel.getPosition(), 1));
        }

        //Carousel shenanigans==============================================
        //Circle pressed and carousel not moving
        if(gamepad2.circle && !carouselMoving){
            carousel.liftUp();
            canMoveCarousel = false;
        }
        //Circle not pressed (doesn't matter if carousel is moving or not)
        else{
            canMoveCarousel = carousel.liftDown();
        }

        //Try carousel movement
        if(canMoveCarousel){
            //Unwinding of carousel
            if(Math.abs(carousel.getPosition())>=5){
                carousel.setPosition(carousel.getPosition() % 3);
            }

            //Moves and checks movement of carousel
            carouselMoving = !carousel.approachPosition();
        }

        //Telemetry
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Field Centric", fieldCentric);

        telemetry.addData("carouselPos", carousel.position);
        telemetry.addData("carouselDistance", carousel.distanceToPos(carousel.position));
    }
}
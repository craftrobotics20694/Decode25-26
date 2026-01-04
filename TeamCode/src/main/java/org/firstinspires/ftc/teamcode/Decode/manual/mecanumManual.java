package org.firstinspires.ftc.teamcode.Decode.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Decode.carousel;

import org.firstinspires.ftc.teamcode.Decode.mecanumConstants;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class mecanumManual extends OpMode {
    private Follower follower;
    private static double startingHeading = Math.toRadians(90);
    public static Pose startingPose = new Pose(0,0, startingHeading); //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean fieldCentric = false;
    private vector targetVector = new vector();
    private CRServo lift;
    private DcMotorEx carousel;
    private DcMotorEx launcher, intake;

    public void init() {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        lift = hardwareMap.get(CRServo.class, "lift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = mecanumConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    public void loop() {
        //Call this once per loop
        //carousel.approachPosition();
        follower.update();
        targetVector.setOrthogonalComponents(gamepad1.left_stick_x,-gamepad1.left_stick_y);

        if(fieldCentric) targetVector.rotateVector(-(follower.getHeading() - startingHeading));

        follower.setTeleOpDrive(
                targetVector.getYComponent(),
                targetVector.getXComponent(),
                gamepad1.right_stick_x,
                true
        );

        intake.setPower(gamepad1.right_trigger);

        carousel.setPower(-gamepad2.right_stick_x);

        lift.setPower(gamepad2.left_stick_y);

        launcher.setPower(-gamepad2.right_stick_y);

        if(gamepad1.bWasPressed()) fieldCentric = !fieldCentric;

        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Field Centric", fieldCentric);
//        telemetry.addData("carouselPos", carousel.position);
//        telemetry.addData("carouselDistance", carousel.distanceToPos(carousel.position));
    }
}
package org.firstinspires.ftc.teamcode.Decode.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private Vector targetVector;
    private Panels panels = Panels.INSTANCE;

    @Override
    public void init() {
        follower = mecanumConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        targetVector.setOrthogonalComponents(gamepad1.left_stick_x,-gamepad1.left_stick_y);

        if (!automatedDrive){
            if(fieldCentric) {
                targetVector = targetVector.rotateVector(-(follower.getHeading()-startingHeading));
                follower.setTeleOpDrive(
                        targetVector.getYComponent(),
                        -targetVector.getXComponent(),
                        -gamepad1.right_stick_x,
                        false
                );
            }
            else
                follower.setTeleOpDrive(
                        targetVector.getYComponent(),
                        targetVector.getXComponent(),
                        -gamepad1.right_stick_x,
                        false
                );
        }


        //Stop automated following if the follower is done
        if (gamepad1.aWasPressed()) {
            startingHeading = follower.getHeading();
        }

        //Robot Centric
        if (gamepad1.rightBumperWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Target Theta", targetVector.getTheta());
        telemetry.addData("Target Magnitude", targetVector.getMagnitude());
        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("Field Centric", fieldCentric);
    }
}
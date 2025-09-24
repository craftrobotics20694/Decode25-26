package org.firstinspires.ftc.teamcode.Decode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /* Defines
    *
     */
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .leftRearMotorName("leftBack")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.0)
            .yVelocity(42.29);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("rightBack")
            .forwardEncoderDirection(Encoder.FORWARD)
            .forwardPodY(5.3125)
            .strafeEncoder_HardwareMapName("leftFront")
            .strafeEncoderDirection(Encoder.REVERSE)
            .strafePodX(-6.0625)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                    )
            )
            .forwardTicksToInches(0.003)
            .strafeTicksToInches(0.003);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}

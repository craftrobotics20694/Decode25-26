package org.firstinspires.ftc.teamcode.Decode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .leftRearMotorName("leftBack")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
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

    public static ThreeWheelIMUConstants threeWheelIMUConstants = new ThreeWheelIMUConstants()
            .leftEncoder_HardwareMapName("leftFront")
            .leftPodY(7.5625)
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoder_HardwareMapName("rightFront")
            .rightPodY(-7.5625)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoder_HardwareMapName("leftBack")
            .strafePodX(-7.1875)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardTicksToInches(0.003)
            .strafeTicksToInches(0.003);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(threeWheelIMUConstants)
                .build();
    }
}

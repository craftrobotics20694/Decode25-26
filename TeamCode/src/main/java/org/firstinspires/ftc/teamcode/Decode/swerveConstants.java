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

import org.firstinspires.ftc.teamcode.Decode.drivetrains.Swerve;
import org.firstinspires.ftc.teamcode.Decode.drivetrains.SwerveConstants;

public class swerveConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /* Defines
     *
     */
    public static SwerveConstants swerveConstants = new SwerveConstants()
            .xVelocity(61.0)
            .yVelocity(42.29)
            .setLeftPod(-1,0)
            .setRightPod(1,0);

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
                .setDrivetrain(new Swerve(hardwareMap, swerveConstants))
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}

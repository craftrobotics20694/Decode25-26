package org.firstinspires.ftc.teamcode.Decode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Math.abs;

import android.graphics.Interpolator;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="Firing sequence", group = "Autonomous")
public class SimpleShoot extends OpMode{
    //Define things
    DcMotorEx launch1;
    DcMotorEx launch2;
    boolean Team = false
    double distancex;
    private Follower follower;
    private Pose RedObelisk = new Pose(125,125, Math.toRadians(45));
    private Pose BlueObelisk = new Pose(60, 85, Math.toRadians(135));
    Pose triangleTipRed = new Pose(72,72, Math.toRadians(45));
    Pose triangleTipBlue = new Pose(72,72, Math.toRadians(135));
    private Path topOfTriangleRed;
    private Path topOfTriangleBlue;
    private int pathState;
    //Make the firing sequence of the code
    @Override
    public void init(){launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
    launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
    };
    public void loop(){
        launch1.setPower(0.2);
        launch2.setPower(0.2);
    }
    public void buildPaths() {
        topOfTriangleRed = new Path(new BezierLine(follower.getPose(), triangleTipRed));
        topOfTriangleRed.setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(45)));
        topOfTriangleBlue = new Path(new BezierLine(follower.getPose(), triangleTipBlue));
        topOfTriangleBlue.setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(135)));
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(Team = false) {
                    follower.followPath(topOfTriangleRed);
                    setPathState(1);
                }
                    break;

            case 1:
                follower.followPath(topOfTriangleBlue);
                setPathState(2);
                break;
    //release from the loop and continue auto mode

    // SEPERATE ATTEMPT


}

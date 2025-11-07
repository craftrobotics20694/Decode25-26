package org.firstinspires.ftc.teamcode.Decode.autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.tools.javac.code.Attribute;


import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="Projectile", group = "Autonomous")
public class FiringCalc extends OpMode{
    double rx;
    double ry;
    double rz;
    double tx;
    double ty;
    double tz;
    double g_val = 9.81;
    DcMotorEx launch1;
    DcMotorEx launch2;
    private Follower follower;
    private Pose RedObelisk = new Pose(125,125, Math.toRadians(53));
    private Pose BlueObelisk = new Pose(60, 85, Math.toRadians(135));
    private Path Shooting;

    // Imported Override system so code stops screaming
    @Override
    public void init(){launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
    };
    public void loop(){
        launch1.setPower(0.2);
        launch2.setPower(0.2);
    }
    // Turn to face obelisk
    Turningpath = follower.pathbuilder()
    // Find distance

    // Increase power based on distance the correct amount

    //
    vector botpos;
    
    }
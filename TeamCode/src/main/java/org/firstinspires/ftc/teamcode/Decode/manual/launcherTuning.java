package org.firstinspires.ftc.teamcode.Decode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Decode.Carousel;
import org.firstinspires.ftc.teamcode.Decode.PIDController;

@TeleOp
public class launcherTuning extends OpMode {
    private int inverseMagnitude = 3;
    private final double ticksPerRotation = 28;
    private double targetSpeed = 50;
    private double launcherChange = 0;
    private final int speedLength = 50;
    private double[] speedArray = new double[speedLength];
    private int speedIndex = 0;
    private double launcherSpeed = 0;
    private double power = 0.5;
    private double prevTime = -0.0001;
    private double Kp = 1.0,
    Ki = 0.5,
    Kd = 0.2;
    private Carousel carousel = new Carousel();
    private PIDController pidController = new PIDController();
    private DcMotor launcherMotor;
    private double prevPosition;
    @Override
    public void init(){
        launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidController.setIntegralLimit(1.0);
        for(int i = 0; i < speedLength; i++){
            speedArray[i] = 0;
        }
    }

    @Override
    public void loop(){
        //Launcher speed is average speed of last 20 loops
        launcherSpeed = 0;
        for(int i = 0; i < speedLength; i++){
            launcherSpeed += speedArray[i];
        }
        launcherSpeed /= speedLength;

        speedArray[speedIndex] = ((launcherMotor.getCurrentPosition() - prevPosition) /
                                 (time - prevTime)) /
                                 ticksPerRotation;
        speedIndex = (speedIndex + 1) % speedLength;

        pidController.update(targetSpeed - launcherSpeed, time - prevTime);
        power = Math.max(0, Math.min(power+((pidController.getCorrection()*(time - prevTime))/100), 1));

        if (gamepad2.dpadUpWasPressed()){
            Kp += 0.05;
        }
        if(gamepad2.dpadDownWasPressed()){
            Kp -= 0.05;
        }

        if(gamepad2.dpadRightWasPressed()){
            Ki += 0.05;
        }
        if(gamepad2.dpadLeftWasPressed()){
            Ki -= 0.05;
        }

        if(gamepad2.rightBumperWasPressed()){
            Kd += 0.05;
        }
        if(gamepad2.leftBumperWasPressed()){
            Kd -= 0.05;
        }

        if(gamepad1.rightBumperWasPressed()){
            targetSpeed += 5;
        }
        if(gamepad1.leftBumperWasPressed()){
            targetSpeed -= 5;
        }

        if(gamepad2.triangleWasPressed()){
            pidController.setIntegralLimit(pidController.getIntegralLimit()+0.05);
        }
        if(gamepad2.crossWasPressed()){
            pidController.setIntegralLimit(pidController.getIntegralLimit()-0.05);
        }

        if(gamepad2.circleWasPressed()){
            pidController.resetIntegral();
            power = 0.5;
        }

        if(gamepad2.circle){
            launcherMotor.setPower(power);
        }
        else{launcherMotor.setPower(0);}

        telemetry.addData("deltaTime", time-prevTime);
        telemetry.addData("contoller y", gamepad2.right_stick_y);
        telemetry.addData("Launcher Ticks", launcherMotor.getCurrentPosition());
        telemetry.addData("Launcher Speed", launcherSpeed);
        telemetry.addData("Target Speed", targetSpeed);
        telemetry.addData("Error", targetSpeed - launcherSpeed);
        telemetry.addData("Power", power);
        telemetry.addData("Power change", (pidController.getCorrection()*(time - prevTime))/100);
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("Integral Limit", pidController.getIntegralLimit());
        prevPosition = launcherMotor.getCurrentPosition();
        prevTime = time;
        pidController.setConstants(Kp, Ki, Kd);
    }
}
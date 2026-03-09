package org.firstinspires.ftc.teamcode.Decode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Decode.Launcher;

@TeleOp
public class launcherTuning extends OpMode {
    private double prevTime = -0.0001,
                   lastUpdateTime;
    private double Kp = 1.75,
    Ki = 0.3,
    Kd = 2.0;
    boolean canShoot = false; //delete, used for debugging
    private Launcher launcher;
    @Override
    public void init(){
        launcher = new Launcher(hardwareMap, "launcher");
        launcher.speedLength = 10;
        launcher.PID.setIntegralLimit(0.4);
    }

    @Override
    public void loop(){
        if (time - lastUpdateTime >= 0.02) {
            double deltaTime = time - prevTime;
            launcher.update(deltaTime);
            lastUpdateTime = time;

            if (gamepad2.dpadUpWasPressed()) {
                Kp += 0.05;
            }
            if (gamepad2.dpadDownWasPressed()) {
                Kp -= 0.05;
            }

            if (gamepad2.dpadRightWasPressed()) {
                Ki += 0.05;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                Ki -= 0.05;
            }

            if (gamepad2.rightBumperWasPressed()) {
                Kd += 0.05;
            }
            if (gamepad2.leftBumperWasPressed()) {
                Kd -= 0.05;
            }

            if (gamepad1.rightBumperWasPressed()) {
                launcher.targetSpeed += 0.5;
            }
            if (gamepad1.leftBumperWasPressed()) {
                launcher.targetSpeed -= 0.5;
            }

            if (gamepad2.triangleWasPressed()) {
                launcher.PID.setIntegralLimit(launcher.PID.getIntegralLimit() + 0.05);
            }
            if (gamepad2.crossWasPressed()) {
                launcher.PID.setIntegralLimit(launcher.PID.getIntegralLimit() - 0.05);
            }

            //launcher.PID.setConstants(Kp, Ki, Kd);

            if (Math.abs(gamepad2.right_stick_y) < 0.3) {
                if (gamepad2.circleWasPressed()) {
                    canShoot = false;
                    launcher.beginApproach();
                } else if (gamepad2.circle) {
                    boolean tempBool = launcher.approachSpeed();
                    canShoot = canShoot || tempBool;
                } else {
                    launcher.setPower(0);
                }
            } else {
                launcher.setPower(-gamepad2.right_stick_y);
                canShoot = true;
            }

            telemetry.addData("deltaTime", time - prevTime);
            telemetry.addData("contoller y", gamepad2.right_stick_y);
            telemetry.addData("Launcher Speed", launcher.getLauncherSpeed());
            telemetry.addData("Target Speed", launcher.getTargetSpeed());
            telemetry.addData("Error", launcher.getTargetSpeed() - launcher.getLauncherSpeed());
            telemetry.addData("Power var", launcher.getPower());
            telemetry.addData("Power change", (launcher.PID.getCorrection() * (deltaTime)) / 100);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Integral Limit", launcher.PID.getIntegralLimit());
            telemetry.addData("deltaTime", deltaTime);
            prevTime = time;
            launcher.PID.setConstants(Kp, Ki, Kd);
        }
    }
}
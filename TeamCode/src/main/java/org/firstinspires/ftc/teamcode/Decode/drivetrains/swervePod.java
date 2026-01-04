package org.firstinspires.ftc.teamcode.Decode.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class swervePod {
    private HardwareMap hardwareMap;

    private DcMotorEx motor0, motor1;
    private vector heading = new vector();
    private double radiansPerTick = 1692;

    //Tangent to robot center in direction of positive radian turning
    public double startingHeading = Math.toRadians(90);
    public double driveDecay = 25;

    /**
     * Assigns motors to prexisting motor objects
     * @param motor0 assignment for motor0
     * @param motor1 assignment for motor1
     * @return Returns the swervePod object
     */
    public swervePod assignMotors(DcMotorEx motor0, DcMotorEx motor1){
        this.motor0 = motor0;
        this.motor1 = motor1;
        return this;
    }

    /**
     * Assigns motors to this swerve object according to hardwareMap
     * @param hardwareMap
     * @param name0
     * @param name1
     * @return
     */
    public swervePod assignMotors(HardwareMap hardwareMap, String name0, String name1){
        motor0 = hardwareMap.get(DcMotorEx.class, name0);
        motor1 = hardwareMap.get(DcMotorEx.class, name1);
        return this;
    }

    /**
     * Resets both motors encoding to a position of 0
     */
    public void resetEncoding(){
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateHeading(){
        heading.setComponents(1,((motor0.getCurrentPosition() - motor1.getCurrentPosition())/radiansPerTick) + startingHeading);
    }

    /**
     * @return int[2] containing motor positions
     */
    public int[] getTicks(){
        int[] ticks = new int[2];
        ticks[0] = motor0.getCurrentPosition();
        ticks[1] = motor1.getCurrentPosition();
        return ticks;
    }

    /**
     * Applies the necessary motor powers for the swerve drive to apporach and drive a given vector<br>
     * @param driveVector Desired direction of wheel, and magnitude of drive power (left stick input or auto equivalent)
     */
    public void turnAndDrive(vector driveVector){
        double[] drivePowers = getDrivePowers(driveVector);

        motor0.setPower(drivePowers[0]);
        motor1.setPower(drivePowers[1]);
    }

    /**
     * Returns the necessary motor powers for the swerve drive to apporach and drive a given vector<br>
     * - Note that logic needs to be implemented in the drivetrain class
     * @param driveVector the desired vector
     * @return
     */
    public double[] getDrivePowers(vector driveVector){
        updateHeading();
        driveVector.setMagnitude(mathFuncs.clamp(driveVector.getMagnitude(), -1, 1));

        double[] drivePowers = new double[2];
        double deltaTheta         = mathFuncs.normalizeToPi(driveVector.getTheta()                  - heading.getTheta());
        double oppositeDeltaTheta = mathFuncs.normalizeToPi(driveVector.rotated(Math.PI).getTheta() - heading.getTheta());

        double turnDirection, drivePower, turnPower;
        if(Math.abs(deltaTheta) < Math.toRadians(90)){
            turnDirection = Math.signum(deltaTheta);
            drivePower = driveVector.getMagnitude()/((Math.abs(deltaTheta) * driveDecay) + 1);
            turnPower = (driveVector.getMagnitude() - drivePower) * turnDirection;
        }
        else{
            turnDirection = Math.signum(oppositeDeltaTheta);
            drivePower = -driveVector.getMagnitude()/((Math.abs(oppositeDeltaTheta) * driveDecay) + 1);
            turnPower  = (driveVector.getMagnitude() + drivePower) * turnDirection;
        }

        drivePowers[0] = drivePower + turnPower;
        drivePowers[1] = drivePower - turnPower;

        return drivePowers;
    }

    //Used for debugging
    public double[] getDriveComponents(vector driveVector){
        updateHeading();
        driveVector.setMagnitude(mathFuncs.clamp(driveVector.getMagnitude(), -1, 1));

        double[] driveComponents = new double[5];
        double deltaTheta         = mathFuncs.normalizeToPi(driveVector.getTheta()                  - heading.getTheta());
        double oppositeDeltaTheta = mathFuncs.normalizeToPi(driveVector.rotated(Math.PI).getTheta() - heading.getTheta());

        double turnDirection, drivePower, turnPower;
        if(Math.abs(deltaTheta) < Math.toRadians(90)){
            turnDirection = Math.signum(deltaTheta);
            drivePower = driveVector.getMagnitude()/((Math.abs(deltaTheta) * driveDecay) + 1);
            turnPower = (driveVector.getMagnitude() - drivePower) * turnDirection;
        }
        else{
            turnDirection = Math.signum(oppositeDeltaTheta);
            drivePower = -driveVector.getMagnitude()/((Math.abs(oppositeDeltaTheta) * driveDecay) + 1);
            turnPower  = (driveVector.getMagnitude() + drivePower) * turnDirection;
        }

        driveComponents[0] = deltaTheta;
        driveComponents[1] = turnDirection;
        driveComponents[2] = drivePower;
        driveComponents[3] = turnPower;
        driveComponents[4] = oppositeDeltaTheta;

        return driveComponents;
    }
}

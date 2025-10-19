package org.firstinspires.ftc.teamcode.Decode.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.Vector;
import org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 */
public class swervePod {
    private HardwareMap hardwareMap;

    private DcMotorEx motor0, motor1;
    private Vector heading = new Vector();
    private double radiansPerTick = 1692;

    //Tangent to robot center in direction of positive radian turning
    public Vector turnVector = new Vector(1, Math.toRadians(90));
    public double startingHeading = Math.toRadians(90);
    public double driveDamping = 2.5;

    /**
     *
     * @param motor0 assignment for motor0, probably counterclockwise
     * @param motor1 assignment for motor1, probably clockwise
     * @return Returns the swervePod object
     */
    public swervePod assignMotors(DcMotorEx motor0, DcMotorEx motor1){
        this.motor0 = motor0;
        this.motor1 = motor1;
        return this;
    }

    public swervePod assignMotors(HardwareMap hardwareMap, String name0, String name1){
        motor0 = hardwareMap.get(DcMotorEx.class, name0);
        motor1 = hardwareMap.get(DcMotorEx.class, name1);
        return this;
    }

    public void resetEncoding(){
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reverseMotors(){
        motor0.setDirection(motor0.getDirection().inverted());
        motor1.setDirection(motor1.getDirection().inverted());
    }

    private void updateHeading(){
        heading.setComponents(1,((motor0.getCurrentPosition() - motor1.getCurrentPosition())/radiansPerTick) + startingHeading);
    }

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
    public void turnAndDrive(Vector driveVector){
        double[] drivePowers = getDrivePowers(driveVector);

        motor0.setPower(drivePowers[0]);
        motor1.setPower(drivePowers[1]);
    }

    /**
     * Returns the necessary motor powers for the swerve drive to apporach and drive a given vector<br>
     * - Note that turning logic needs to be implemented in the drivetrain class
     * @param driveVector the desired vector
     * @return
     */
    public double[] getDrivePowers(Vector driveVector){
        updateHeading();

        double[] drivePowers = new double[2];
        double deltaTheta         = mathFuncs.normalizeToPi(driveVector.getTheta()                       - heading.getTheta());
        double oppositeDeltaTheta = mathFuncs.normalizeToPi(driveVector.rotateVector(Math.PI).getTheta() - heading.getTheta());

        double turnDirection, drivePower, turnPower;
        if(Math.abs(deltaTheta) < Math.toRadians(90)){
            turnDirection = Math.signum(deltaTheta);
            drivePower = driveVector.getMagnitude()/(Math.abs(deltaTheta) * driveDamping);
            turnPower = (driveVector.getMagnitude() - drivePower) * turnDirection;
        }
        else{
            turnDirection = Math.signum(oppositeDeltaTheta);
            drivePower = -driveVector.getMagnitude()/(Math.abs(oppositeDeltaTheta) * driveDamping);
            turnPower  = (driveVector.getMagnitude() + drivePower) * turnDirection;
        }

        drivePowers[0] = drivePower + turnPower;
        drivePowers[1] = drivePower - turnPower;

        return drivePowers;
    }

    //Used for debugging
    public double[] getDriveComponents(Vector driveVector){
        double[] driveComponents = new double[5];
        double deltaTheta         = mathFuncs.normalizeToPi(driveVector.getTheta()                       - heading.getTheta());
        double oppositeDeltaTheta = mathFuncs.normalizeToPi(driveVector.rotateVector(Math.PI).getTheta() - heading.getTheta());

        double turnDirection, drivePower, turnPower;
        if(Math.abs(deltaTheta) < Math.toRadians(90)){
            turnDirection = Math.signum(deltaTheta);
            drivePower = driveVector.getMagnitude()/(Math.abs(deltaTheta) * driveDamping);
            turnPower = (driveVector.getMagnitude() - drivePower) * turnDirection;
        }
        else{
            turnDirection = Math.signum(oppositeDeltaTheta);
            drivePower = -driveVector.getMagnitude()/(Math.abs(oppositeDeltaTheta) * driveDamping);
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

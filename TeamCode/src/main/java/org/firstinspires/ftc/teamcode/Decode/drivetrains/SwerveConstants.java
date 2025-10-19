package org.firstinspires.ftc.teamcode.Decode.drivetrains;

import org.firstinspires.ftc.teamcode.Decode.drivetrains.Swerve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveConstants {
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public  double xVelocity = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public  double yVelocity = 65.43028;

    private  double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     *  Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
    public  double maxPower = 1;
    public  String leftMotor0 = "left0";
    public  String leftMotor1 = "left1";
    public  String rightMotor0 = "right0";
    public  String rightMotor1 = "right1";
    public  DcMotorSimple.Direction leftMotor0Direction = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction leftMotor1Direction = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightMotor0Direction = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightMotor1Direction = DcMotorSimple.Direction.FORWARD;
    public  double motorCachingThreshold = 0.01;
    public  boolean useBrakeModeInTeleOp = false;
    public  boolean useVoltageCompensation = false;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;

    public SwerveConstants() {
        defaults();
    }

    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public SwerveConstants leftMotor0Name(String leftMotor0) {
        this.leftMotor0 = leftMotor0;
        return this;
    }

    public SwerveConstants leftMotor1Name(String leftMotor1) {
        this.leftMotor1 = leftMotor1;
        return this;
    }

    public SwerveConstants rightMotor0Name(String rightMotor0) {
        this.rightMotor0 = rightMotor0;
        return this;
    }

    public SwerveConstants rightMotor1Name(String rightMotor1) {
        this.rightMotor1 = rightMotor1;
        return this;
    }

    public SwerveConstants leftMotor0Direction(DcMotorSimple.Direction leftMotor0Direction) {
        this.leftMotor0Direction = leftMotor0Direction;
        return this;
    }

    public SwerveConstants leftMotor1Direction(DcMotorSimple.Direction leftMotor1Direction) {
        this.leftMotor1Direction = leftMotor1Direction;
        return this;
    }

    public SwerveConstants rightMotor0Direction(DcMotorSimple.Direction rightMotor0Direction) {
        this.rightMotor0Direction = rightMotor0Direction;
        return this;
    }

    public SwerveConstants rightMotor1Direction(DcMotorSimple.Direction rightMotor1Direction) {
        this.rightMotor1Direction = rightMotor1Direction;
        return this;
    }

    public SwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public SwerveConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public SwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public SwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftMotor0;
    }

    public void setLeftFrontMotorName(String leftMotor0) {
        this.leftMotor0 = leftMotor0;
    }

    public String getLeftRearMotorName() {
        return leftMotor1;
    }

    public void setLeftRearMotorName(String leftMotor1) {
        this.leftMotor1 = leftMotor1;
    }

    public String getRightFrontMotorName() {
        return rightMotor0;
    }

    public void setRightFrontMotorName(String rightMotor0) {
        this.rightMotor0 = rightMotor0;
    }

    public String getRightRearMotorName() {
        return rightMotor1;
    }

    public void setRightRearMotorName(String rightMotor1) {
        this.rightMotor1 = rightMotor1;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftMotor0Direction;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftMotor0Direction) {
        this.leftMotor0Direction = leftMotor0Direction;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftMotor1Direction;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftMotor1Direction) {
        this.leftMotor1Direction = leftMotor1Direction;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightMotor0Direction;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightMotor0Direction) {
        this.rightMotor0Direction = rightMotor0Direction;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightMotor1Direction;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightMotor1Direction) {
        this.rightMotor1Direction = rightMotor1Direction;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * This method sets the default values for the SwerveConstants class.
     * It is called in the constructor of the SwerveConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        leftMotor0 = "left0";
        leftMotor1 = "left1";
        rightMotor0 = "right0";
        rightMotor1 = "right1";
        leftMotor0Direction = DcMotorSimple.Direction.FORWARD;
        leftMotor1Direction = DcMotorSimple.Direction.FORWARD;
        rightMotor0Direction = DcMotorSimple.Direction.FORWARD;
        rightMotor1Direction = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }
}

package org.firstinspires.ftc.teamcode.Decode.drivetrains;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Decode.MathUtils.vector;
import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.*;
import org.firstinspires.ftc.teamcode.Decode.drivetrains.swervePod;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

/**
 * This is the Mecanum class, a child class of Drivetrain. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers.
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 4/30/2025
 */
public class Swerve extends Drivetrain {
    public SwerveConstants constants;
    private final DcMotorEx left0, left1, right0, right1;
    private final swervePod leftPod = new swervePod(),
                            rightPod = new swervePod();
    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    /**
     * This creates a new Mecanum, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param swerveConstants this is the SwerveConstants object that contains the names of the motors and directions etc.
     */
    public Swerve(HardwareMap hardwareMap, SwerveConstants swerveConstants) {
        constants = swerveConstants;

        this.maxPowerScaling = swerveConstants.maxPower;
        this.motorCachingThreshold = swerveConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = swerveConstants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        left0 = hardwareMap.get(DcMotorEx.class, swerveConstants.leftMotor0);
        left1 = hardwareMap.get(DcMotorEx.class, swerveConstants.leftMotor1);
        right1 = hardwareMap.get(DcMotorEx.class, swerveConstants.rightMotor0);
        right0 = hardwareMap.get(DcMotorEx.class, swerveConstants.rightMotor1);
        
        leftPod.assignMotors(left0,left1);
        rightPod.assignMotors(right0,right1);

        motors = Arrays.asList(left0, left1, right0, right1);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();
    }

    public void updateConstants() {
        left0.setDirection(constants.leftMotor0Direction);
        left1.setDirection(constants.leftMotor1Direction);
        right0.setDirection(constants.rightMotor0Direction);
        right1.setDirection(constants.rightMotor1Direction);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     * <p>
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower    this Vector points in the direction of the robot's current heading, and
     *                        the magnitude tells the robot how much it should turn and in which
     *                        direction.
     * @param pathingPower    this Vector points in the direction the robot needs to go to continue along
     *                        the Path.
     * @param robotHeading    this is the current heading of the robot, which is used to calculate how
     *                        much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double[] motorPowers = new double[4];
        double scalingFactor;
        vector leftVector = new vector(correctivePower),
               rightVector = new vector(correctivePower),
               leftTurn = constants.leftPodTurn.times(headingPower.getMagnitude()),
               rightTurn = constants.rightPodTurn.times(headingPower.getMagnitude());

        //This is mostly copy paste once for each Vector parameter, the order determines what is prioritized

        //Makes our version of a vector out of correctivePower
        vector newCorrectivePower = new vector(correctivePower);
        //NormalizingScaling is the factor applied to the variable vector such when it is summed with the static vector the resultant vector's magnitude is 1
        //We do this so that no vector exceeds a magnitude of 1
        //We multiply both tacked-on vectors by same factor so that movement is proportional and things don't break
        scalingFactor = Math.min(Math.min(
                        findNormalizingScaling(leftVector,  newCorrectivePower, maxPowerScaling),
                        findNormalizingScaling(rightVector, newCorrectivePower, maxPowerScaling)),
                        1);
        leftVector  = leftVector .plus(leftTurn .times(scalingFactor));
        rightVector = rightVector.plus(rightTurn.times(scalingFactor));

        //Adds turning power (headingPower)
        scalingFactor = Math.min(Math.min(
                findNormalizingScaling(leftVector,  leftTurn,  maxPowerScaling),
                findNormalizingScaling(rightVector, rightTurn, maxPowerScaling)),
                1);
        leftVector  = leftVector .plus(leftTurn .times(scalingFactor));
        rightVector = rightVector.plus(rightTurn.times(scalingFactor));

        //Adds pathing power (pathingPower)
        vector newPathingPower = new vector(pathingPower);
        scalingFactor = Math.min(Math.min(
                findNormalizingScaling(leftVector,  newPathingPower, maxPowerScaling),
                findNormalizingScaling(rightVector, newPathingPower, maxPowerScaling)),
                1);
        leftVector  = leftVector .plus(newPathingPower.times(scalingFactor));
        rightVector = rightVector.plus(newPathingPower.times(scalingFactor));

        //Now we assign motor powers using the return of swervePod.getDrivePowers()
        double[] leftPowers = leftPod.getDrivePowers(leftVector);
        double[] rightPowers = rightPod.getDrivePowers(rightVector);

        motorPowers[0] = leftPowers[0];
        motorPowers[1] = leftPowers[1];
        motorPowers[2] = rightPowers[0];
        motorPowers[3] = rightPowers[1];

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] *= voltageNormalized;
            }
        }

        //Failsafe if any power is greater than max power, in theory it shouldn't happen
        double wheelPowerMax = Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));

        if (wheelPowerMax > maxPowerScaling) {
            motorPowers[0] = (motorPowers[0] / wheelPowerMax) * maxPowerScaling;
            motorPowers[1] = (motorPowers[1] / wheelPowerMax) * maxPowerScaling;
            motorPowers[2] = (motorPowers[2] / wheelPowerMax) * maxPowerScaling;
            motorPowers[3] = (motorPowers[3] / wheelPowerMax) * maxPowerScaling;
        }

        return motorPowers;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        return constants.yVelocity;
    }

    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }
    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public String debugString() {
        return "Mecanum{" +
                " left0=" + left0 +
                ", left1=" + left1 +
                ", right0=" + right0 +
                ", right1=" + right1 +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }
}
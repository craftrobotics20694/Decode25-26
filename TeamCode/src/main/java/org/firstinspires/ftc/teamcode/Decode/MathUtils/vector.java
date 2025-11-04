package org.firstinspires.ftc.teamcode.Decode.MathUtils;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import static org.firstinspires.ftc.teamcode.Decode.MathUtils.mathFuncs.*;
public class vector {
    /**
     * The magnitude (length) of the vector.
     */
    private double magnitude;
    /**
     * The direction (angle in radians) of the vector.
     */
    private double theta;
    /**
     * The x component of the vector in Cartesian coordinates.
     */
    private double xComponent;
    /**
     * The y component of the vector in Cartesian coordinates.
     */
    private double yComponent;

    /**
     * Constructs a new Vector with zero magnitude and direction.
     */
    public vector() {
        setComponents(0, 0);
    }

    /**
     * Constructs a new Vector from a given Pose's x and y coordinates.
     *
     * @param pose the Pose object to extract x and y from
     */
    public vector(Pose pose) {
        setOrthogonalComponents(pose.getX(), pose.getY());
    }

    /**
     * Constructs a new Vector with a specified magnitude and direction.
     *
     * @param magnitude the magnitude (length) of the vector
     * @param theta     the direction (angle in radians) of the vector
     */
    public vector(double magnitude, double theta) {
        setComponents(magnitude, theta);
    }

    public vector(Vector vector){setComponents(vector.getMagnitude(), vector.getTheta());}
    /**
     * Sets the vector's magnitude and direction (polar coordinates).
     * Updates the Cartesian components accordingly.
     *
     * @param magnitude the magnitude to set
     * @param theta     the direction (angle in radians) to set
     */
    public void setComponents(double magnitude, double theta) {
        if (magnitude < 0) {
            this.magnitude = -magnitude;
            this.theta = mod(theta + Math.PI,  2*Math.PI);
        } else {
            this.magnitude = magnitude;
            this.theta = mod(theta, 2*Math.PI);
        }
        xComponent = magnitude * Math.cos(this.theta);
        yComponent = magnitude * Math.sin(this.theta);
    }

    /**
     * Sets vector magnitude, theta is static
     *
     * @param magnitude the new magnitude
     */
    public void setMagnitude(double magnitude) {
        setComponents(magnitude, theta);
    }

    /**
     * Sets vector theta, magnitude is static
     *
     * @param theta the new direction (radians)
     */
    public void setTheta(double theta) {
        setComponents(magnitude, theta);
    }

    /**
     * Rotates the vector by a given angle
     *
     * @param theta2 the angle to add to the current direction
     */
    public void rotateVector(double theta2) {
        setTheta(theta + theta2);
    }
    
    public vector rotated(double theta2){
        return new vector(magnitude, theta + theta2);
    }

    /**
     * Sets the vector's Cartesian components (x, y).
     * Updates the polar representation accordingly.
     *
     * @param x the x component to set
     * @param y the y component to set
     */
    public void setOrthogonalComponents(double x, double y) {
        xComponent = x;
        yComponent = y;
        magnitude = Math.sqrt((x*x)+(y*y));
        theta = mod(Math.atan2(y,x), 2 * Math.PI);
    }

    /**
     * This multiplies the current Vector by a scalar and returns the result as a Vector.
     *
     * @param scalar the scalar multiplying into the Vector.
     * @return returns the scaled Vector.
     */
    public vector times(double scalar) {
        return new vector(getMagnitude() * scalar, getTheta());
    }

    /**
     * This normalizes this Vector to be of magnitude 1, unless this Vector is the zero Vector.
     * In that case, it just returns back the zero Vector but with a different memory location.
     *
     * @return returns the normalized (or zero) Vector.
     */
    public vector normalize() {
        if (getMagnitude() == 0) {
            return new vector(0.0, getTheta());
        } else {
            return new vector(getMagnitude() / Math.abs(getMagnitude()), getTheta());
        }
    }

    /**
     * Returns the sum of two vectors without modifying either
     * @param other the adder
     * @return returns the sum of this vector + param
     */
    public vector plus(vector other) {
        vector returnVector = new vector();
        returnVector.setOrthogonalComponents(getXComponent() + other.getXComponent(), getYComponent() + other.getYComponent());
        return returnVector;
    }

    /**
     * Returns the difference of two vectors without modifying either
     * @param other the subtractor
     * @return the difference of this vector - param
     */
    public vector minus(vector other) {
        vector returnVector = new vector();
        returnVector = this.plus(other.rotated(Math.PI));
        return returnVector;
    }

    /**
     * @return magnitude
     */
    public double getMagnitude() {
        return magnitude;
    }

    /**
     * @return theta (radians)
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x component of the vector.
     *
     * @return the x component
     */
    public double getXComponent() {
        return xComponent;
    }

    /**
     * Returns the y component of the vector.
     *
     * @return the y component
     */
    public double getYComponent() {
        return yComponent;
    }

    /**
     * Returns a copy of this vector.
     *
     * @return a new Vector with the same magnitude and direction
     */
    public vector copy() {
        return new vector(this.magnitude, this.theta);
    }

    /**
     * Returns a string representation of the vector, including magnitude, theta, x, and y components.
     *
     * @return a string describing the vector
     */
    @Override
    public String toString() {
        return "Vector{" +
                "magnitude=" + magnitude +
                ", theta=" + theta +
                ", xComponent=" + xComponent +
                ", yComponent=" + yComponent +
                '}';
    }
}
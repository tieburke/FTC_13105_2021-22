package org.firstinspires.ftc.robotcontroller.RobotClasses.Subsytems;

import org.firstinspires.ftc.robotcontroller.RobotClasses.Misc.Vector2D;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class TankDrive {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    public double x;
    private double maxSpeedCap;
    private double speedMultiplier;
    private Gyro gyro;

    public void TankDrive(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Gyro gyro) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;

        this.gyro = gyro;

        maxSpeedCap = 1;
        speedMultiplier = 1;
    }

    /**
     * Control the mecanum drivetrain using forward, strafe, and turn parameters
     *
     * @param forward move forward (positive value) or backward (negative value)
     * @param //strafe  move right (positive) or left (negative)
     * @param turn    turn clock-wise (positive) or counter-clockwise (negative)
     */

    public void drive(double forward, double turn, boolean isFieldOriented) {
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        double y = -forward;
        //double x = strafe;
        double rx = turn;

        // Field oriented math
        if (isFieldOriented) {
            Vector2D input = new Vector2D(x, y);
            input.rotate(-gyro.getAngle());

            frontLeftPower = input.getX() + input.getY() + rx;
            frontRightPower = -input.getX() + input.getY() - rx;
            backLeftPower = -input.getX() + input.getY() + rx;
            backRightPower = input.getX() + input.getY() - rx;
        } else {

            y = -forward; // this is reversed
            //x = strafe;
            rx = turn;

            //frontLeftPower = y + x - rx;
            //backLeftPower = y - x + rx;
            //frontRightPower = y - x - rx;
            //backRightPower = y + x - rx;

            // Set motor power
            frontLeftPower = y - rx;
            backLeftPower = y + rx;
            frontRightPower = y - rx;
            backRightPower = y - rx;

        }

        // if one of the powers is over 1 (or maxSpeedCap), divide them by the max so that all motor powers stay the same ratio
        // (so that they're not over 1 or the maxSpeedCap)
        if (Math.abs(frontLeftPower) > maxSpeedCap || Math.abs(backLeftPower) > maxSpeedCap ||
                Math.abs(frontRightPower) > maxSpeedCap || Math.abs(backRightPower) > maxSpeedCap) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);
    }
/*
    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, true);
    }
*/
    /**
     * Cap the power given to the drivetrain
     *
     * @param cap a double from 0 to 1
     */
    public void setSpeedCap(double cap) {
        if (cap > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed cap over 1. Cap has been automatically set to 1");
        }
        maxSpeedCap = Range.clip(Math.abs(cap), 0, 1);
    }

    /**
     * Multiply the speed of all the motors (this is applied after the speed cap is applied)
     *
     * @param multiplier a double from 0 to 1
     */
    public void setSpeedMultiplier(double multiplier) {
        if (multiplier > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed multiplier over 1. Multiplier has been automatically set to 1");
        }
        speedMultiplier = Range.clip(Math.abs(multiplier), 0, 1);
    }
/*
    public void driveStraight(double forward, double turn) {
        double setpoint = gyro.getAngle();
        double offset = setpoint - (gyro.getAngle() * .08);

        drive(forward, turn, offset);
}
*/
}
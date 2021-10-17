package org.firstinspires.ftc.robotcontroller.RobotClasses.Subsytems;

import org.firstinspires.ftc.robotcontroller.RobotClasses.Misc.Vector2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.RobotClasses.Standard_Bot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class TankDrive {
    private static DcMotor frontRight;
    private static DcMotor frontLeft;
    private static DcMotor backRight;
    private static DcMotor backLeft;

    public double x;
    private Gyro gyro;

    public void tankDrive(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Gyro gyro) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;

        this.gyro = gyro;

    }



    public static void drive(double straight, double power) {
        int rightTarget;
        int leftTarget;

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        leftTarget = frontLeft.getCurrentPosition() + (int) (straight * 33);
        rightTarget = frontRight.getCurrentPosition() + (int) (straight * 33);

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public static void turn(double turn, double power) {
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        int rightTarget = 0;
        int leftTarget = 0;

        if (turn > 0){
            frontLeftPower = power;
            backLeftPower = power;
            frontRightPower = -power;
            backRightPower = -power;
        }
        else if (turn < 0){
            frontLeftPower = -power;
            backLeftPower = -power;
            frontRightPower = power;
            backRightPower = power;
        }
        else if (turn == 0){
            frontLeftPower = power;
            backLeftPower = power;
            frontRightPower = power;
            backRightPower = power;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        leftTarget = frontLeft.getCurrentPosition() + (int) (turn * 33);
        rightTarget = frontRight.getCurrentPosition() + (int) (turn * 33);

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
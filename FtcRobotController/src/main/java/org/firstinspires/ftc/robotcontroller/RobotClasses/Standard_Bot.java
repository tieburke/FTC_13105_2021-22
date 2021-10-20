package org.firstinspires.ftc.robotcontroller.RobotClasses;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Standard_Bot {

    /* Public OpMode members. */
    public DcMotor  StdFrontRight   = null;
    public DcMotor  StdBackRight  = null;
    public DcMotor  StdFrontLeft   = null;
    public DcMotor  StdBackLeft = null;
    public DcMotor  StdIntakeMotor  = null;
    //public Servo    StdArmServo    = null;
    //public CRServo  StdShooterServo  = null;
    //public DistanceSensor       StdDistanceSensor = null;
    //public Rev2mDistanceSensor  StdRevDistanceSensor = null;

    //public static final double MID_SERVO       =  0.5 ;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Standard_Bot(){

    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        StdFrontLeft  =     hwMap.get(DcMotor.class, "FrontLeft");
        StdFrontRight =     hwMap.get(DcMotor.class, "RightFront");
        StdBackLeft =       hwMap.get(DcMotor.class, "LeftRear");
        StdBackRight =      hwMap.get(DcMotor.class, "RightRear");
        StdIntakeMotor =    hwMap.get(DcMotor.class, "IntakeMotor");
        //StdDistanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        //StdRevDistanceSensor = (Rev2mDistanceSensor)StdDistanceSensor;

        // Define and initialize ALL installed servos.

        //StdCartridgeServo = hwMap.get(Servo.class, "CartridgeServo");
        //StdCartridgeServo.setPosition(CARTRIDGE_START);

        StdFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        StdFrontRight.setDirection(DcMotor.Direction.FORWARD);
        StdBackLeft.setDirection(DcMotor.Direction.FORWARD);
        StdBackRight.setDirection(DcMotor.Direction.REVERSE);
        StdIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        StdFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        StdFrontLeft.setPower(0);
        StdBackLeft.setPower(0);
        StdFrontRight.setPower(0);
        StdBackRight.setPower(0);
        StdIntakeMotor.setPower(0);

        // Set all motors to run without encoders.
        // May use RUN_WITHOUT_ENCODER or RUN_USING_ENCODERS

        StdFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


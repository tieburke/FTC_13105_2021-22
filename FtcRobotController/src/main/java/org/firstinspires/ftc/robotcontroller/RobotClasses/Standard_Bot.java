package org.firstinspires.ftc.robotcontroller.RobotClasses;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Standard_Bot {

    public static DcMotor StdFrontRight   = null;
    public static DcMotor StdBackRight  = null;
    public static DcMotor StdFrontLeft   = null;
    public static DcMotor StdBackLeft = null;
    public static DcMotor StdIntakeMotor  = null;
    public static DcMotor StdCapperMotor = null;
    public static DcMotor StdCarouselMotor = null;
    public static DcMotor StdOuttakeMotor = null;
    public static Servo StdRotateCapperServo = null;
    public static Servo StdOuttakeServo = null;
    //public DistanceSensor       StdDistanceSensor = null;
    //public Rev2mDistanceSensor  StdRevDistanceSensor = null;

    public static final double Capper_Start= 0;
    public static final double Outtake_Servo= 0;

    static HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    public Standard_Bot(){

    }
    public static void InitHardware(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        StdFrontLeft  = hwMap.get(DcMotor.class, "FrontLeft");
        StdFrontRight = hwMap.get(DcMotor.class, "RightFront");
        StdBackLeft = hwMap.get(DcMotor.class, "LeftRear");
        StdBackRight = hwMap.get(DcMotor.class, "RightRear");
        StdIntakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");
        StdCapperMotor = hwMap.get(DcMotor.class, "CapperMotor");
        StdCarouselMotor = hwMap.get(DcMotor.class, "CarouselMotor");
        StdOuttakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");
        StdRotateCapperServo = hwMap.get(Servo.class, "CapperServo");
        StdOuttakeServo = hwMap.get(Servo.class, "OuttakeServo");

        //StdDistanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        //StdRevDistanceSensor = (Rev2mDistanceSensor)StdDistanceSensor;

        StdFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        StdFrontRight.setDirection(DcMotor.Direction.FORWARD);
        StdBackLeft.setDirection(DcMotor.Direction.FORWARD);
        StdBackRight.setDirection(DcMotor.Direction.REVERSE);
        StdIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        StdCapperMotor.setDirection(DcMotor.Direction.FORWARD);
        StdCarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        StdOuttakeMotor.setDirection(DcMotor.Direction.FORWARD);

        StdFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdCapperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdCarouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        StdRotateCapperServo.setPosition(Capper_Start);
        StdOuttakeServo.setPosition(Outtake_Servo);

        // Set all motors to zero power
        StdFrontLeft.setPower(0);
        StdBackLeft.setPower(0);
        StdFrontRight.setPower(0);
        StdBackRight.setPower(0);
        StdIntakeMotor.setPower(0);
        StdCapperMotor.setPower(0);
        StdOuttakeMotor.setPower(0);
        StdCarouselMotor.setPower(0);

        // Set all motors to run without encoders.
        // May use RUN_WITHOUT_ENCODER or RUN_USING_ENCODERS

        StdFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdCapperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdOuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdCarouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}


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
    public DcMotor  StdRightFront   = null;
    public DcMotor  StdRightRear  = null;
    public DcMotor  StdLeftFront   = null;
    public DcMotor  StdLeftRear = null;
    public DcMotor  StdArmMotor  = null;
    public DcMotor  StdIntakeMotor  = null;
    public DcMotor  StdShooterMotor  = null;
    public DcMotor  StdCartridgeMotor = null;
    //    public DcMotor  transportMotor = null;
    public Servo    StdArmServo    = null;
    public Servo    StdIntakeServo    = null;
    public Servo    StdCartridgeServo  = null;
    public CRServo  StdShooterServo  = null;
    //    public CRServo shooterServo = null;
    public DistanceSensor       StdDistanceSensor = null;
    public Rev2mDistanceSensor  StdRevDistanceSensor = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double INTAKE_START    =  0.62 ;
    public static final double CARTRIDGE_START =  0.0;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double SHOOTER_START   = 0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Standard_Bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        StdLeftFront  =     hwMap.get(DcMotor.class, "LeftFront");
        StdRightFront =     hwMap.get(DcMotor.class, "RightFront");
        StdLeftRear =       hwMap.get(DcMotor.class, "LeftRear");
        StdRightRear =      hwMap.get(DcMotor.class, "RightRear");
        StdArmMotor =       hwMap.get(DcMotor.class, "ArmMotor");
        StdIntakeMotor =    hwMap.get(DcMotor.class, "IntakeMotor");
        StdCartridgeMotor = hwMap.get(DcMotor.class, "CartridgeMotor");
        StdShooterMotor =   hwMap.get(DcMotor.class, "ShooterMotor");
        StdDistanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        StdRevDistanceSensor = (Rev2mDistanceSensor)StdDistanceSensor;

        // Define and initialize ALL installed servos.

        StdShooterServo = hwMap.get(CRServo.class, "ShooterServo");
        StdArmServo = hwMap.get(Servo.class, "ArmServo");
        StdArmServo.setPosition(MID_SERVO);
        StdIntakeServo = hwMap.get(Servo.class, "IntakeServo");
        StdIntakeServo.setPosition(INTAKE_START);
        StdCartridgeServo = hwMap.get(Servo.class, "CartridgeServo");
        StdCartridgeServo.setPosition(CARTRIDGE_START);

        StdLeftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        StdRightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        StdLeftRear.setDirection(DcMotor.Direction.FORWARD);
        StdRightRear.setDirection(DcMotor.Direction.REVERSE);
        StdArmMotor.setDirection(DcMotor.Direction.REVERSE);
        StdIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        StdShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        StdCartridgeMotor.setDirection(DcMotor.Direction.REVERSE);

        StdRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StdArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      StdIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        StdLeftFront.setPower(0);
        StdLeftRear.setPower(0);
        StdRightFront.setPower(0);
        StdRightRear.setPower(0);
        StdArmMotor.setPower(0);
        StdIntakeMotor.setPower(0);
        StdShooterMotor.setPower(0);
        StdCartridgeMotor.setPower(0);


        // Set all motors to run without encoders.
        // May use RUN_WITHOUT_ENCODER or RUN_USING_ENCODERS

        StdLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StdIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StdCartridgeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


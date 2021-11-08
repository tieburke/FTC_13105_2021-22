package org.firstinspires.ftc.robotcontroller.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop", group="Teleop")
@Disabled
public class Teleop extends LinearOpMode {

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
    public static final double Capper_Start= 0;
    public static final double Outtake_Servo= 0;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    private DcMotor carouselMotor = null;
    private DcMotor capperMotor = null;

    private Servo outtakeServo = null;
    private Servo capperServo = null;

    static HardwareMap hwMap =  null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

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

    while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

        if (gamepad2.a){
            intakeMotor.setPower(0.75);
        }
        if (gamepad2.b){
            intakeMotor.setPower(-0.75);
        }
        if (gamepad2.y){
            outtakeMotor.setPower(0.25);
        }
        if (gamepad2.x){
            outtakeMotor.setPower(-0.25);
        }
        if (gamepad2.left_bumper){
            capperMotor.setPower(0.5);
        }
        if (gamepad2.right_bumper){
            capperMotor.setPower(-0.5);
        }
        if (gamepad1.a){
            carouselMotor.setPower(0.75);
        }
        if (gamepad2.dpad_up){
            outtakeServo.setPosition(95);
        }
        if (gamepad2.dpad_down){
            outtakeServo.setPosition(0);
        }
        if (gamepad2.dpad_left){
            capperServo.setPosition(180);
        }
        if (gamepad2.dpad_right){
            capperServo.setPosition(0);
        }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

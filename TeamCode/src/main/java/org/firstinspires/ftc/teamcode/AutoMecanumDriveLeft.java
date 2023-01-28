package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoMecanumDriveLeft", group="Movebot")
public class AutoMecanumDriveLeft extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.5;
    static final double     TURN_SPEED    = 0.5;

    public static DcMotor leftBackDrive = null;
    public static DcMotor leftFrontDrive = null;
    public static DcMotor rightBackDrive = null;
    public static DcMotor rightFrontDrive = null;

    private DcMotor linearSlide = null;
    private DcMotor armJoint = null;
    private Servo clawServo1 = null;
    private Servo clawServo2 = null;
    private Servo armServo = null;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_drive_back");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "right_drive_front");

        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        armJoint = hardwareMap.get(DcMotor.class, "arm");
        clawServo1 = hardwareMap.get(Servo.class, "servo1");
        clawServo2 = hardwareMap.get(Servo.class, "servo2");
        armServo = hardwareMap.get(Servo.class, "servo3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        DriveMethods dm = new DriveMethods(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);

        dm.driveLeft(0.5);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1.0)) {
            telemetry.addData("Path", "Leg 0: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        /*// Step 1:  Drive left for 3 seconds

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 0.6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        linearSlide.setPower(0);

        // Step 2:  Drive forward for 1 second
        dm.driveForward(0.5);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 0.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Turn right for 2 seconds
        dm.turnLeft(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 0.25)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Drive backward for 1 second
        dm.driveBackward(0.5);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1.0)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        linearSlide.setPower(0.5);
        armJoint.setPower(1);
        armServo.setPosition(1);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1.0)) {
            telemetry.addData("Path", "Leg 5: %2.5f S elapsed", runtime.seconds());
            telemetry.update();
        }

        linearSlide.setPower(0);
        clawServo1.setPosition(0.45);
        clawServo2.setPosition(0.65);

        dm.driveForward(0.5);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1.0)) {
            telemetry.addData("Path", "Leg 6: %2.5f S elapsed", runtime.seconds());
            telemetry.update();
        }*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

//        // Step 5:  Turn Carousel for 3 seconds
//        dm.turnCarouselLeft(0.5);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() <= 3.0)) {
//            telemetry.addData("Path", "Leg 5: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 6:  Drive left diagonal for 2 seconds
//        dm.driveLeftDiagonal(0.5);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() <= 2.0)) {
//            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
    }


}
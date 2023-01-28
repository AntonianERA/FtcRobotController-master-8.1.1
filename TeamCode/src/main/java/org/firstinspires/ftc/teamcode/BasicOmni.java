/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;



import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Iterative Opmode")
public class BasicOmni extends OpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private DcMotor armJoint = null;
    private Servo clawServo1 = null;
    private Servo clawServo2 = null;
    private Servo armServo = null;


    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    int speedIndex = 0;
    double[] speeds = { 0.25, 0.125, 0.5};
    boolean _bPressed = false;

    boolean _aPressed = false;
    boolean servoEngaged = false;

    boolean _yPressed = false;

    boolean _xPressed = false;
    int armJointVal = 0;
    double linearSlideVal = 0;
    int monkeyID = 0;
    int vineBoomId = 0;
    int goofyID = 0;
    int soundID = 0;
    double armServoVal = 0.5;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_drive_back");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        armJoint = hardwareMap.get(DcMotor.class, "arm");
        clawServo1 = hardwareMap.get(Servo.class, "servo1");
        clawServo2 = hardwareMap.get(Servo.class, "servo2");
        armServo = hardwareMap.get(Servo.class, "servo3");
        //limitSwitch = LimitSwitch(self, "limit");


        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armJoint.setDirection(DcMotor.Direction.FORWARD);

        armJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setTargetPosition(armJointVal);
        armJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        monkeyID = hardwareMap.appContext.getResources().getIdentifier("monkeysound", "raw", hardwareMap.appContext.getPackageName());
        goofyID = hardwareMap.appContext.getResources().getIdentifier("goofy", "raw", hardwareMap.appContext.getPackageName());
        vineBoomId = hardwareMap.appContext.getResources().getIdentifier("vineboom", "raw", hardwareMap.appContext.getPackageName());

        soundID = monkeyID;


        // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop(){ }

    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {


        calculatePower();

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

        /*
        leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
        */


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        double linearSlideIncrement = (gamepad1.right_trigger - gamepad1.left_trigger)/1.5;
        linearSlideVal += linearSlideIncrement;
        if (linearSlideVal < 0 || linearSlideVal > 162.5) {
            linearSlideVal -= linearSlideIncrement;
            linearSlide.setPower(0);
        } else {
            linearSlide.setPower(linearSlideIncrement);
        }

        armJoint.setPower(gamepad1.right_bumper ? -1.0 : gamepad1.left_bumper ? 1.0 : 0.0);

        // Adjust relative to LEFT value
        clawServo1.setPosition(servoEngaged ? 0.60 : 0.45); // Left value is closed; right is open
        // Adjust relative to RIGHT value
        clawServo2.setPosition(servoEngaged ? 0.50 : 0.65); // ''

        if (gamepad1.dpad_left && armServoVal < 1) {
            armServoVal += 0.005;
        } else if (gamepad1.dpad_right && armServoVal > 0) {
            armServoVal -= 0.005;
        }
        armServo.setPosition(armServoVal);

        // Show the elapsed game time and wheel power.

        telemetry.addData("Servo value", armServoVal);
        telemetry.addData("Joint value", armJointVal);
        telemetry.addData("True joint value", armJoint.getCurrentPosition());
        telemetry.addData("Linear slide value", linearSlideVal);
        telemetry.update();
    }

    @Override
    public void stop(){
    }


    void calculatePower(){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double DEADZONE = 0.2;

        if (Math.abs(axial) < DEADZONE) {
            axial = 0;
        }
        if (Math.abs(lateral) < DEADZONE) {
            lateral = 0;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (this._bPressed != gamepad1.b && gamepad1.b) {
            // Selected speed increments by one or goes back to 0 if it's gone past the length of the speed array
            speedIndex = (speedIndex + 1) % speeds.length;
            int[] soundsIDs = { monkeyID, goofyID, vineBoomId };
            soundID = soundsIDs[speedIndex];
        }

        this._bPressed = gamepad1.b;

        leftFrontPower *= speeds[speedIndex];
        rightFrontPower *= speeds[speedIndex];
        leftBackPower *= speeds[speedIndex];
        rightBackPower *= speeds[speedIndex];

        if (gamepad1.a != this._aPressed && gamepad1.a) {
            this.servoEngaged = !this.servoEngaged;
        }

        this._aPressed = gamepad1.a;

        if (gamepad1.y != this._yPressed && gamepad1.y) {
            if (armServoVal < 0.5) {
                armServoVal = 1;
            } else {
                armServoVal = 0;
            }
        }

        this._yPressed = gamepad1.y;


        if (gamepad1.x != this._xPressed && gamepad1.x) {
            // SoundPlayer.getInstance().stopPlayingAll();
            // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

            linearSlideVal = 0;
        }

        this._xPressed = gamepad1.x;

        if (gamepad1.left_bumper) {
            armJointVal -= 25;
        } else if (gamepad1.right_bumper) {
            armJointVal += 25;
        }
    }
}
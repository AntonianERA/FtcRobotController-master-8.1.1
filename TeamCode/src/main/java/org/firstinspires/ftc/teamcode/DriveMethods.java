package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveMethods {

    DcMotor leftDriveFront = null;
    DcMotor rightDriveFront = null;
    DcMotor leftDriveBack = null;
    DcMotor rightDriveBack = null;
    DcMotor carousel = null;

    public DriveMethods(DcMotor pLeftDriveFront, DcMotor pRightDriveFront, DcMotor pLeftDriveBack, DcMotor pRightDriveBack) {
        leftDriveFront = pLeftDriveFront;
        rightDriveFront = pRightDriveFront;
        leftDriveBack = pLeftDriveBack;
        rightDriveBack = pRightDriveBack;
    }
    public void driveForward(double power) {
        leftDriveFront.setPower(-power);
        rightDriveFront.setPower(power);
        leftDriveBack.setPower(-power);
        rightDriveBack.setPower(power);
    }

    public void driveBackward(double power) {
        leftDriveFront.setPower(power);
        rightDriveFront.setPower(-power);
        leftDriveBack.setPower(power);
        rightDriveBack.setPower(-power);
    }

    public void driveRight(double power) {
        leftDriveFront.setPower(power);
        rightDriveFront.setPower(power);
        leftDriveBack.setPower(-power);
        rightDriveBack.setPower(-power);
    }

    public void driveLeft(double power) {
        leftDriveFront.setPower(power);
        rightDriveFront.setPower(-power);
        leftDriveBack.setPower(-power);
        rightDriveBack.setPower(power);
    }

    public void driveLeftDiagonal(double power) {
        rightDriveFront.setPower(power);
        leftDriveBack.setPower(power);
    }

    public void driveRightDiagonal(double power) {
        leftDriveFront.setPower(power);
        rightDriveBack.setPower(power);
    }

    public void turnLeft(double power) {
        leftDriveFront.setPower(-power);
        rightDriveFront.setPower(power);
        leftDriveBack.setPower(-power);
        rightDriveBack.setPower(power);
    }

    public void turnRight(double power) {
        leftDriveFront.setPower(power);
        rightDriveFront.setPower(-power);
        leftDriveBack.setPower(power);
        rightDriveBack.setPower(-power);
    }

    public void turnCarouselLeft(double power) {
        carousel.setPower(power);
    }

    public void turnCarouselRight(double power) {
        carousel.setPower(-power);
    }

    public void clearMotorPower(){
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
    }
}

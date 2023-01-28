package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimitSwitch {
    private DigitalChannel digIn;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public LimitSwitch(OpMode robot, String name) {
        try {
            telemetry = robot.telemetry;
            hardwareMap = robot.hardwareMap;
            digIn = hardwareMap.get(DigitalChannel.class, name);
            telemetry.addData("<", "Fully initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Limit switch initialization error");
        }
    }

    public void loop() {
        try {
            telemetry.addData("pressed", isPressed());
        } catch (Exception e) {
            telemetry.addData("Error", "Limit loop error");
        }
    }

    public boolean isPressed() {
        return digIn.getState();
    }
}

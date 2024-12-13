package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

public class DragonsLights {
    public boolean isValid = true;
    public RevBlinkinLedDriver lights;

    public DragonsLights (LinearOpMode opmode) {
        try {
            opmode.telemetry.addLine("Configuring lights...");
            opmode.telemetry.update();

            lights = opmode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");

            opmode.telemetry.addLine("Lights configured!");
            opmode.telemetry.update();
        } catch (IllegalArgumentException ex) {
            Global.exceptions.append("Configuration Error: ").append("lights").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
    }
}

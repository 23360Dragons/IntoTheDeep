package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Global;

public class DragonsLights {
    public boolean isValid = true;
    public RevBlinkinLedDriver lights;

    public DragonsLights (LinearOpMode opmode) {
        try {
            opmode.telemetry.addLine("Configuring lights...");
            opmode.telemetry.update();

            lights = opmode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            opmode.telemetry.addLine("Lights configured!");
            opmode.telemetry.update();
        } catch (Exception ex) {
            Global.exceptions.append("lights\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
    }
}

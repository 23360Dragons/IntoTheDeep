package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

public class DragonsLights {
    public static boolean isValid;

    public static void initialize (HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            telemetry.addLine("Configuring lights...");
            telemetry.update();

            Global.light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            isValid = true;

            telemetry.addLine("Lights configured!");
            telemetry.update();
        } catch (IllegalArgumentException ex) {
            Global.exceptions.append("Configuration Error: ").append("lights").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public static void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        Global.light.setPattern(pattern);
    }
}

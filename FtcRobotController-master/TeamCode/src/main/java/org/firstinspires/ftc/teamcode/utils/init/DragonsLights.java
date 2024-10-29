package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DragonsLights {
    public static boolean isValid;

    public static void initialize (HardwareMap hardwareMap) {
        try {
            InitInfo.light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            isValid = true;
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("Configuration Error: ").append("lights").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
        }
    }

    public static void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        InitInfo.light.setPattern(pattern);
    }
}

package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class DragonsLights {
    public static RevBlinkinLedDriver light;
    public static boolean isValid;

    public static RevBlinkinLedDriver initialize (HardwareMap hardwareMap) {
        try {
            light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            isValid = true;
            return light;
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("Configuration Error: ").append("lights").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
            return null;
        }
    }

    public static void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        light.setPattern(pattern);
    }
}

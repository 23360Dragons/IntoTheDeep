package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class DragonsLights extends RevBlinkinLedDriver {
    public RevBlinkinLedDriver light;
    public boolean isValid = false;

    /**
     * RevBlinkinLedDriver
     *
     * @param controller A REV servo controller
     * @param port       the port that the driver is connected to
     */
    public DragonsLights(ServoControllerEx controller, int port) {
        super(controller, port);
    }

    public void initialize (HardwareMap hardwareMap) {
        try {
            this.light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            isValid = true;
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("Configuration Error: ").append("lights").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
        }
    }

    public void setPattern (RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.light.setPattern(pattern);
    }
}

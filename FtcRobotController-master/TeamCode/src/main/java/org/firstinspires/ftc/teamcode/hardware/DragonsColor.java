package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.utils.Global;

public class DragonsColor {
    public boolean isValid = true;
    public ColorRangeSensor colorSensor;

    public DragonsColor (LinearOpMode opmode) {
        try {
            opmode.telemetry.addLine("Configuring Color Sensor...");
            opmode.telemetry.update();

            colorSensor = opmode.hardwareMap.get(ColorRangeSensor.class, "colorSensor");
            isValid = true;

            opmode.telemetry.addLine("Color Sensor configured!");
            opmode.telemetry.update();
        } catch (Exception ex) {
            Global.exceptions.append("imu\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }
}

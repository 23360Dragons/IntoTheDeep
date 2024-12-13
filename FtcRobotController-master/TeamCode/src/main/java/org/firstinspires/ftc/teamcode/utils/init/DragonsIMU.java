package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

import static org.firstinspires.ftc.teamcode.utils.Global.logoFacingDirection;
import static org.firstinspires.ftc.teamcode.utils.Global.usbFacingDirection;

public class DragonsIMU {
    public boolean isValid = true;
    public IMU imu;

    public DragonsIMU (LinearOpMode opmode) {
        try {
            opmode.telemetry.addLine("Configuring IMU...");
            opmode.telemetry.update();

            imu = opmode.hardwareMap.get(IMU.class, "imu");

            isValid = true;

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));

            imu.initialize(parameters);

            imu.resetYaw();

            opmode.telemetry.addLine("IMU configured!");
            opmode.telemetry.update();
        } catch (IllegalArgumentException ex) {
            Global.exceptions.append("CRITICAL Configuration Error: ").append("imu").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }
}

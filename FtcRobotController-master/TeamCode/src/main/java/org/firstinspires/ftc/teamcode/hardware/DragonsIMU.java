package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

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

            imu.resetDeviceConfigurationForOpMode();
            isValid = true;

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));

            imu.initialize(parameters);

            imu.resetYaw();

            opmode.telemetry.addLine("IMU configured!");
            opmode.telemetry.update();
        } catch (Exception ex) {
            Global.exceptions.append("imu\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }
}

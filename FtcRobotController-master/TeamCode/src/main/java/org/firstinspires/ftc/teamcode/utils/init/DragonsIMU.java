package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DragonsIMU {
    public static boolean isValid = false;

    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            telemetry.addLine("Configuring IMU...");
            telemetry.update();

            Consts.imu = hardwareMap.get(IMU.class, "imu");

            isValid = true;

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));

            Consts.imu.initialize(parameters);

            telemetry.addLine("IMU configured!");
            telemetry.update();
        } catch (IllegalArgumentException ex) {
            Consts.exceptions.append("CRITICAL Configuration Error: ").append("imu").append(" does not exist").append("\n");
            Consts.exceptionOccurred = true;
            isValid = false;
        }
    }
}

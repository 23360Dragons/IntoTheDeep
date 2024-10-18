package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DragonsIMU {
    public static IMU imu;
    public static boolean isValid = false;

    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static IMU initialize(HardwareMap hardwareMap) {
        try {
            imu = hardwareMap.get(IMU.class, "imu");

            isValid = true;

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));
            imu.initialize(parameters);
            return imu;
        } catch (IllegalArgumentException ex) {
            InitInfo.exceptions.append("CRITICAL Configuration Error: ").append("imu").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
            isValid = false;
            return null;
        }
    }
}

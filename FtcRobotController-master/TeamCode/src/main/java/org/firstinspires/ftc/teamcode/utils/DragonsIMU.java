package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DragonsIMU {
    public static IMU initialize (HardwareMap hardwareMap,
                                  RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection,
                                  RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection) throws ConfigurationException
    {
        try {
            IMU imu = hardwareMap.get(IMU.class, "imu");

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection));
            imu.initialize(parameters);

            return imu;
        } catch (IllegalArgumentException ex)
        {
            throw new ConfigurationException("imu does not exist!");
        }
    }
}

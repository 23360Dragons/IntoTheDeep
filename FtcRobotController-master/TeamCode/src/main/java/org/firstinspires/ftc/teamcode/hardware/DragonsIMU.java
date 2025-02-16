package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

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

//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    logoFacingDirection, // forward, up
//                    usbFacingDirection));

            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                                new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.XZY,
                                AngleUnit.DEGREES,
                                -90,
                                0,
                                // this allows field centric to be correct after auto, i think
                                AutoRobotPos.getRotation(),
                                0
                            )
                    )
            );

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
package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.SuperStructure;

public class Global {
    public static StringBuilder exceptions = new StringBuilder("The following exceptions occurred:\n");
    public static boolean exceptionOccurred = false;

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection  usbFacingDirection   = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static final int BLUE   = 0;
    public static final int RED    = 1;
    public static final int YELLOW = 2;

    public static final int LEFT  = 0;
    public static final int RIGHT = 1;

    public static DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public static IMU imu;
    public static Limelight3A limelight;
    public static RevBlinkinLedDriver light;
    public static SparkFunOTOS sparkFunOTOS;
    public static SuperStructure superStructure;
    public static Arm arm;
}
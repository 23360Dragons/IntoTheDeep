package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.SuperStructure.SuperStructure;

public class Global {
    public static StringBuilder exceptions;
    public static boolean exceptionOccurred;

    public static final int BLUE   = 0;
    public static final int RED    = 1;
    public static final int YELLOW = 2;

    public static final int LEFT  = 0;
    public static final int RIGHT = 1;

    public static DcMotor leftFront, rightFront, leftBack, rightBack;
    public static IMU imu;
    public static Limelight3A limelight;
    public static RevBlinkinLedDriver light;
    public static SparkFunOTOS sparkFunOTOS;
    public static SuperStructure superStructure;

    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection   = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
}
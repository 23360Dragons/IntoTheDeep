package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.HashMap;

public class InitInfo {
    public static StringBuilder exceptions;
    public static boolean exceptionOccurred;

    public static final int BluePipeline = 0;
    public static final int RedPipeline = 1;
    public static DcMotor leftFront, rightFront, leftBack, rightBack;
    public static DcMotor[] motors;
    public static IMU imu;
    public static Limelight3A limelight;
    public static RevBlinkinLedDriver light;
}
package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class Consts {
    public static StringBuilder exceptions;
    public static boolean exceptionOccurred;

    public static final int BluePipeline = 0;
    public static final int RedPipeline = 1;
    public static final int yellowPipeline = 2;
    public static DcMotor leftFront, rightFront, leftBack, rightBack;
    public static IMU imu;
    public static Limelight3A limelight;
    public static RevBlinkinLedDriver light;
    public static SparkFunOTOS sparkFunOTOS;
}
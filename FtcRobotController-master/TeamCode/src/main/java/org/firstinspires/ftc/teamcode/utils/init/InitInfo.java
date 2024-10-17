package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public class InitInfo {
    public static StringBuilder exceptions;
    public static boolean exceptionOccurred;

    public static final int BluePipeline = 0;
    public static final int RedPipeline = 1;

    //public motors such that they don't have to be individually initialized
    public static DcMotor leftFront;
    public static DcMotor rightFront;
    public static DcMotor leftBack;
    public static DcMotor rightBack;
    public static DcMotor[] driveMotors;
    public static String[] driveMotorNames = {"leftFront", "rightFront", "leftBack", "rightBack"};


}

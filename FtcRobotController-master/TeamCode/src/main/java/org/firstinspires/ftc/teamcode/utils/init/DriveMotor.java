package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveMotor {
    public static boolean isValid;

    public static DcMotor[] initialize (HardwareMap hardwareMap) {
        DcMotor leftFront = null, rightFront = null, leftBack = null, rightBack = null;
        DcMotor[] motors = {leftFront, leftBack, rightFront, rightBack};
        try
        {
                //left front
                try{
                    leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                    motors[0] = leftFront;
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("leftFront").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;}
                //right front
                try
                {
                    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motors[1] = rightFront;

                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("rightFront").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;
                }
                //left back
                try
                {
                    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
                    leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftBack.setDirection(DcMotor.Direction.REVERSE);
                    motors[2] = leftBack;
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("leftBack").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;}
                //right back
                try
                {
                    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
                    rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motors[3] = rightBack;
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;
                }
        }
        catch(Exception ex)
        {
            InitInfo.exceptions.append("Configuration Error: ").append("General Motor Config").append(ex.getMessage()).append("\n");
            InitInfo.exceptionOccurred = true;
            return null;
        }
        return motors;
    }
}


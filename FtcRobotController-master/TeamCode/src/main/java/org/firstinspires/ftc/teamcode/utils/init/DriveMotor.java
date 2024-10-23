package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveMotor {
    public static boolean isValid;
    public static void initialize (HardwareMap hardwareMap) {
        try
        {
                //left front
                try{
                    InitInfo.leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
                    InitInfo.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    InitInfo.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    InitInfo.leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("leftFront").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;
                }
                //right front
                try
                {
                    InitInfo.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                    InitInfo.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    InitInfo.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("rightFront").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;
                }
                //left back
                try
                {
                    InitInfo.leftBack = hardwareMap.get(DcMotor.class, "leftBack");
                    InitInfo.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    InitInfo.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    InitInfo.leftBack.setDirection(DcMotor.Direction.REVERSE);
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("leftBack").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;}
                //right back
                try
                {
                    InitInfo.rightBack = hardwareMap.get(DcMotor.class, "rightBack");
                    InitInfo.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    InitInfo.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        }
    }
}


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
                    Consts.leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
                    Consts.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Consts.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Consts.leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                }
                catch(IllegalArgumentException e)
                {
                    Consts.exceptions.append("Configuration Error: ").append("leftFront").append(" does not exist").append("\n");
                    Consts.exceptionOccurred = true;
                }
                //right front
                try
                {
                    Consts.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                    Consts.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Consts.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
                catch(IllegalArgumentException e)
                {
                    Consts.exceptions.append("Configuration Error: ").append("rightFront").append(" does not exist").append("\n");
                    Consts.exceptionOccurred = true;
                }
                //left back
                try
                {
                    Consts.leftBack = hardwareMap.get(DcMotor.class, "leftBack");
                    Consts.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Consts.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Consts.leftBack.setDirection(DcMotor.Direction.REVERSE);
                }
                catch(IllegalArgumentException e)
                {
                    Consts.exceptions.append("Configuration Error: ").append("leftBack").append(" does not exist").append("\n");
                    Consts.exceptionOccurred = true;}
                //right back
                try
                {
                    Consts.rightBack = hardwareMap.get(DcMotor.class, "rightBack");
                    Consts.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Consts.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                catch(IllegalArgumentException e)
                {
                    Consts.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
                    Consts.exceptionOccurred = true;
                }
        }
        catch(Exception ex)
        {
            Consts.exceptions.append("Configuration Error: ").append("General Motor Config").append(ex.getMessage()).append("\n");
            Consts.exceptionOccurred = true;
        }
    }
}


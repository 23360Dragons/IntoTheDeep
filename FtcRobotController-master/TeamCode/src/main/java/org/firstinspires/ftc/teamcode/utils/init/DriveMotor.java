package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.driveMotors;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.driveMotors;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftBack;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightBack;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.driveMotorNames;




public class DriveMotor {
    public static boolean isValid;

    public static void initialize (HardwareMap hardwareMap) {
        for (int i = 0; i < driveMotors.length - 1; i++)
        {
            try {
                driveMotors[i] = hardwareMap.get(DcMotor.class, driveMotorNames[i]);
                driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (i % 2 == 0)
                    driveMotors[i].setDirection(DcMotor.Direction.REVERSE);

                switch (i) {
                    case 0: {
                        leftFront = driveMotors[i];
                    }
                    case 1: {
                        rightFront = driveMotors[i];
                    }
                    case 2: {
                        leftBack = driveMotors[i];
                    }
                    case 3: {
                        driveMotors[3] = driveMotors[i];
                    }
                }
            } catch (IllegalArgumentException e) {
                InitInfo.exceptions.append("Configuration Error: ").append(driveMotorNames[i]).append(" does not exist").append("\n");
                InitInfo.exceptionOccurred = true;
                isValid = false;
            }
        /*
        int i=0;
        for (DcMotor m : driveMotors) {
            try {
                m = hardwareMap.get(DcMotor.class, driveMotorNames[i]);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (i % 2 == 0)
                    m.setDirection(DcMotor.Direction.REVERSE);

                switch (i) {
                    case 0: {
                        leftFront = m;
                        driveMotors[0] = m;
                    }
                    case 1: {
                        rightFront = m;
                        driveMotors[1] = m;
                    }
                    case 2: {
                        leftBack = m;
                        driveMotors[2] = m;
                    }
                    case 3: {
                        rightBack = m;
                        driveMotors[3] = m;
                    }
                }
            } catch (IllegalArgumentException e) {
                InitInfo.exceptions.append("Configuration Error: ").append(driveMotorNames[i]).append(" does not exist").append("\n");
                InitInfo.exceptionOccurred = true;
                isValid = false;
            } finally {
                i++;
            }*/
        }
    }

    public static void initialize2 (HardwareMap hardwareMap) {
        DcMotor[] motors = {leftFront, leftBack, rightFront, rightBack};
        try
        {
            for (DcMotor m : motors)
            {
                //left front
                try{
                    leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
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
                }
                catch(IllegalArgumentException e)
                {
                    InitInfo.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
                    InitInfo.exceptionOccurred = true;
                }
            }

        }
        catch(Exception ex)
        {
            InitInfo.exceptions.append("Configuration Error: ").append("General Motor Config").append(ex.getMessage()).append("\n");
            InitInfo.exceptionOccurred = true;
        }
    }
}


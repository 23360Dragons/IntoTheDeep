package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveMotor {
    public static boolean isValid;
    public static void initialize (HardwareMap hardwareMap, Telemetry telemetry) {
        try
        {
            telemetry.addLine("Configuring drive motors...");
            telemetry.update();
                //left front
                try{
                    Consts.leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
                    Consts.leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                    configure(Consts.leftFront);
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
                    configure(Consts.rightFront);

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
                    configure(Consts.leftBack);
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
                    configure(Consts.rightBack);
                }
                catch(IllegalArgumentException e)
                {
                    Consts.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
                    Consts.exceptionOccurred = true;
                }
                telemetry.addLine("Drive motors configured!");
                telemetry.update();
        }
        catch(Exception ex)
        {
            Consts.exceptions.append("Configuration Error: ").append("General Motor Config").append(ex.getMessage()).append("\n");
            Consts.exceptionOccurred = true;
        }
    }

    private static void configure(DcMotor m) {
        //java passes in the actual object rather than a reference, so this actually changes the passed in variable
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


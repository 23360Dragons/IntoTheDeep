package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;

public class DriveMotor {
    public static boolean isValid = true;
    public static void initialize (HardwareMap hardwareMap, Telemetry telemetry) {
        try
        {
            telemetry.addLine("Configuring drive motors...");
            telemetry.update();

                //left front
                try{
                    Global.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront"); // gets a dcMotor object of the name "name"
                    Global.leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                    configure(Global.leftFront);
                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("Configuration Error: ").append("leftFront").append(" does not exist").append("\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                //right front
                try
                {
                    Global.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
                    configure(Global.rightFront);

                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("Configuration Error: ").append("rightFront").append(" does not exist").append("\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                //left back
                try
                {
                    Global.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
                    configure(Global.leftBack);
                    Global.leftBack.setDirection(DcMotor.Direction.REVERSE);
                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("Configuration Error: ").append("leftBack").append(" does not exist").append("\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }
                
                //right back
                try
                {
                    Global.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
                    configure(Global.rightBack);
                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                telemetry.addLine("Drive motors configured!");
                telemetry.update();
        }
        catch(Exception ex)
        {
            Global.exceptions.append("Configuration Error: ").append("General Motor Config").append(ex.getMessage()).append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    private static void configure(DcMotorEx m) {
        //java passes in the actual object rather than a reference, so this actually changes the passed in variable
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


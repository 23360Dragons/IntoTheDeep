package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Global;

public class DriveMotors {
    public boolean isValid = true;
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public DriveMotors (LinearOpMode opmode) {
        try
        {
            opmode.telemetry.addLine("Configuring drive motors...");
            opmode.telemetry.update();

                //left front
                try
                {
                    leftFront = opmode.hardwareMap.get(DcMotorEx.class, "leftFront"); // gets a dcMotor object of the name "name"
                    leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15
                    configure(leftFront);
                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("leftFront\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                //right front
                try
                {
                    rightFront = opmode.hardwareMap.get(DcMotorEx.class, "rightFront");
                    configure(rightFront);

                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("rightFront\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                //left back
                try
                {
                    leftBack = opmode.hardwareMap.get(DcMotorEx.class, "leftBack");
                    leftBack.setDirection(DcMotor.Direction.REVERSE);
                    configure(leftBack);
                }
                catch(IllegalArgumentException e)
                {
                    Global.exceptions.append("leftBack\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }
                
                //right back
                try
                {
                    rightBack = opmode.hardwareMap.get(DcMotorEx.class, "rightBack");
                    configure(rightBack);
                }
                catch(Exception e)
                {
                    Global.exceptions.append("rightBack\n");
                    Global.exceptionOccurred = true;
                    isValid = false;
                }

                opmode.telemetry.addLine("Drive motors configured!");
                opmode.telemetry.update();
        }
        catch(Exception ex)
        {
            Global.exceptions.append("General Motor Config: ").append(ex.getMessage()).append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    private void configure(DcMotorEx m) {
        //java passes in the actual object rather than a reference, so this actually changes the passed in variable
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower (double[] drivePowers) {
        leftFront.setPower(drivePowers[0]);
        rightFront.setPower(drivePowers[1]);
        leftBack.setPower(drivePowers[2]);
        rightBack.setPower(drivePowers[3]);
    }

    public double[] getPower () {
        return new double[]{
            leftFront.getPower(),
            rightFront.getPower(),
            leftBack.getPower(),
            rightBack.getPower()
        };
    }
}


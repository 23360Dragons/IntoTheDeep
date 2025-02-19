package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Global;

public class Drivetrain {
    public boolean isValid = true;
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public Drivetrain(LinearOpMode opmode) {
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

    //robot-centric move function
    public void RC (double x, double y, double rightX, double speed) {
        //rightX is yaw

        x*=1.1; //counteract imperfect strafing
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rightX), 1); //normalize the inputs

        double leftFrontPower  = (((y + x + rightX) ) / denominator)* speed;
        double rightFrontPower = (((y - x - rightX) ) / denominator)* speed;
        double leftBackPower   = (((y - x + rightX) ) / denominator)* speed;
        double rightBackPower  = (((y + x - rightX) ) / denominator)* speed;
        setPower(new double[] {leftFrontPower, rightFrontPower, leftBackPower, rightBackPower});
    }

    //field-centric move function
    public void FC (double botHeading, double x, double y, double rightX, double speed)
    {

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading)  * 1.1; // to counteract imperfect strafing
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        /*
        these are the rotated x and y (i.e. the vector relative to the field rather than to the robot)

        x * Math.cos and Math.sin are using trigonometry to find the values of the distances between
        the robot-centric vector and the field-centric vector (the target vector). rotY does the same

        in essence, x and y are the values of the initial vector, and rotX and rotY are the values
        of the field-centric vector (the vector after it has been transformed by the botHeading)

        see the Mecanum Drive Tutorial on gm0.org for more info
         */

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1); //makes sure values don't get scaled up by dividing by a decimal
        //takes the sum of all inputs so the total motor power is within the range -1 to 1

        double leftFrontPower  = (((rotY + rotX + rightX) * speed) / denominator);
        double rightFrontPower = (((rotY - rotX - rightX) * speed) / denominator);
        double leftBackPower   = (((rotY - rotX + rightX) * speed) / denominator);
        double rightBackPower  = (((rotY + rotX - rightX) * speed) / denominator);

        setPower(new double[] {leftFrontPower, rightFrontPower, leftBackPower, rightBackPower});
    }

    // input the angle at which you want to move and it will go that way, relative to the robot
    public void moveRobotAngle (double angle, double speed) {
        double x = Math.abs(-Math.abs(((double) 1 /90) * angle) + 2)-1;
        double y = Math.abs(-Math.abs(((double) 1 /90) * angle - 1) + 2)-1; // math works out.

        double leftFrontPower  = (y + x) * speed;
        double rightFrontPower = (y - x) * speed;
        double leftBackPower   = (y - x) * speed;
        double rightBackPower  = (y + x) * speed;
        setPower(new double[] {leftFrontPower, rightFrontPower, leftBackPower, rightBackPower});
    }
}


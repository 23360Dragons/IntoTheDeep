package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveRobot
{
    //robot-centric move function
    public static double[] moveRobotRC (double x, double y, double rightX, double speed)
    {
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rightX), 1);

        double leftFrontPower  = (((y + x + rightX) * speed) / denominator);
        double rightFrontPower = (((y - x - rightX) * speed) / denominator);
        double leftBackPower   = (((y - x + rightX) * speed) / denominator);
        double rightBackPower  = (((y + x - rightX) * speed) / denominator);

        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }

    //field-centric move function
    public static double[] moveRobotFC(double botHeading, double x, double y, double rightX, double speed)
    {

        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1; // Counteract imperfect strafing
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
        //TODO: possibly swap speed and denominator - test

        double leftFrontPower  = (((rotY + rotX + rightX) * speed) / denominator);
        double rightFrontPower = (((rotY - rotX - rightX) * speed) / denominator);
        double leftBackPower   = (((rotY - rotX + rightX) * speed) / denominator);
        double rightBackPower  = (((rotY + rotX - rightX) * speed) / denominator);

        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }
}

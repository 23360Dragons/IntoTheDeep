package org.firstinspires.ftc.teamcode.TeleOp.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveRobot
{

    public void moveRobotRC (double x, double y, double rightX, double speed,
                             DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack)
    {
        // speed == -0.5

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rightX), 1);

        leftFront.setPower (((y + x + rightX) * speed) / denominator);
        leftBack.setPower  (((y - x + rightX) * speed) / denominator);
        rightFront.setPower(((y - x - rightX) * speed) / denominator);
        rightBack.setPower (((y + x - rightX) * speed) / denominator);
    }
}

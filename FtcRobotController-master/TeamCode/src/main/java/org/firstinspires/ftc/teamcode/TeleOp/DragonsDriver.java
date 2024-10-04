package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DragonsDriver extends OpMode{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    private Limelight3A limelight;

    @Override
    public void init() {

        leftFront = new DriveMotor(hardwareMap, "leftFront").dcMotor;
        rightFront = new DriveMotor(hardwareMap, "rightFront").dcMotor;
        leftBack = new DriveMotor(hardwareMap, "leftBack").dcMotor;
        rightBack = new DriveMotor(hardwareMap, "rightBack").dcMotor;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    @Override
    public void loop() {

        double y, x, rightX;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        moveRobot(x,y, rightX);




    }

    private void moveRobot(double x, double y, double rightX)
    {
        //we have to initialize the variable to control speed percentage
        //because y on the stick is negative, speed must be negative
        double speed = -0.5;

        leftFront.setPower(((y + x) + rightX)*speed);
        leftBack.setPower(((y - x) + rightX)*speed);
        rightFront.setPower(((y - x) - rightX)*speed);
        rightBack.setPower(((y + x) - rightX)*speed);
    }
}

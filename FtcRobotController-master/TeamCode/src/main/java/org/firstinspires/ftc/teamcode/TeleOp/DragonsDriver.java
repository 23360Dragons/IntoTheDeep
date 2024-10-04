package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DragonsDriver extends OpMode{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


    }

    @Override
    public void loop() {
        
        double y;
        double x;
        double rightX;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        //we have to initialize the variable to control speed percentage
        double speed = -0.5;

        leftFront.setPower(((y + x) + rightX)*speed);
        leftBack.setPower(((y - x) + rightX)*speed);
        rightFront.setPower(((y - x) - rightX)*speed);
        rightBack.setPower(((y + x) - rightX)*speed);


    }
}

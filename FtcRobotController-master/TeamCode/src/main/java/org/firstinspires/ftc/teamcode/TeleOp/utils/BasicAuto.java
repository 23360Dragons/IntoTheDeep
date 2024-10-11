package org.firstinspires.ftc.teamcode.TeleOp.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class BasicAuto extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = DriveMotor.newMotor(hardwareMap, "leftFront");  // returns DcMotor - editable in utils\DriveMotor.java
        rightFront = DriveMotor.newMotor(hardwareMap, "rightFront");
        leftBack   = DriveMotor.newMotor(hardwareMap, "leftBack");
        rightBack  = DriveMotor.newMotor(hardwareMap, "rightBack");

        //reverse the right motors due to the direction they rotate being flipped on the right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
    }
}

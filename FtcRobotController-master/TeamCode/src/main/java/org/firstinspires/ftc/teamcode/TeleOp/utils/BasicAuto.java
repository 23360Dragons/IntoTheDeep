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
        try
        {
            leftFront  = DriveMotor.newMotor(hardwareMap, "leftFront");  // returns DcMotor - editable in utils\DriveMotor.java
            rightFront = DriveMotor.newMotor(hardwareMap, "rightFront");
            leftBack   = DriveMotor.newMotor(hardwareMap, "leftBack");
            rightBack  = DriveMotor.newMotor(hardwareMap, "rightBack");
        }
        catch(IllegalArgumentException ex) {
            telemetry.addData("Configuration issue", "One or more motors not available");
            telemetry.addLine();
            telemetry.update();

            Thread.sleep(5000);

            terminateOpModeNow();
            return;
        }


        //reverse the right motors due to the direction they rotate being flipped on the right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();

        moveRobot(-1, 0, 0, 1,1);

    }

    public void moveRobot(double x, double y, double yaw, double straif, double straif1) {
        // Calculate wheel powers.
        //changed power calculations by adding *0.7 to run at 70% of what it ran at beforehand
        double leftFrontPower    =  (x -y -yaw) *0.7*straif;
        double rightFrontPower   =  (x +y +yaw) *0.7*straif1;
        double leftBackPower     =  (x +y -yaw) *0.7*straif1;
        double rightBackPower    =  (x -y +yaw) *0.7*straif;

        //straif makes robot go right straif1 makes robot go left
        //always set straifing variables to 1, set one of them negitive to go left or right as instructed above

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}

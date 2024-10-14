package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.init.InitInfo;

@Autonomous
public class BasicAuto extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    IMU imu;

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        InitInfo.exceptions = new StringBuilder("The following exceptions occurred: \n");
        InitInfo.exceptionOccurred = false;
        InitInfo.movementExceptionOccurred = false;

        DcMotor[] driveMotors = DriveMotor.initialize(hardwareMap);

        //assigns the motors to the corresponding motor from the array
        if (driveMotors != null) {
            leftFront = driveMotors[0];
            rightFront = driveMotors[1];
            leftBack = driveMotors[2];
            rightBack = driveMotors[3];
        }

        // reverse the right motors due to the direction they rotate being flipped on the right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = DragonsIMU.initialize(hardwareMap);

        limelight = DragonsLimelight.initialize(hardwareMap, 0);

        //check for configuration issues
        if (InitInfo.exceptionOccurred) {
            telemetry.addLine(InitInfo.exceptions.toString());

            Thread.sleep(5000);
            if (InitInfo.movementExceptionOccurred) {
                requestOpModeStop();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        moveRobot(-1, 0, 0, 1, 1);
        //why not this?
        //moveRobotRC(0, 1, 0, 0.5);

    }

    public void moveRobot(double x, double y, double yaw, double strafe, double strafe1) {
        // Calculate wheel powers.
        //changed power calculations by adding *0.7 to run at 70% of what it ran at beforehand
        double leftFrontPower = (x - y - yaw) * 0.7 * strafe;
        double rightFrontPower = (x + y + yaw) * 0.7 * strafe1;
        double leftBackPower = (x + y - yaw) * 0.7 * strafe1;
        double rightBackPower = (x - y + yaw) * 0.7 * strafe;

        //strafe makes robot go right strafe1 makes robot go left
        //always set strafing variables to 1, set one of them negative to go left or right as instructed above

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

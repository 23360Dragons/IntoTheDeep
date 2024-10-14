package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.ConfigurationException;
import org.firstinspires.ftc.teamcode.utils.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.InitInfo;

@Autonomous
public class BasicAuto extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    IMU imu;
    Limelight3A limelight;

    // TODO: change these values based on robot construction
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;

    @Override
    public void runOpMode() throws InterruptedException {
        StringBuilder exceptions = new StringBuilder("The following exceptions occurred: \n");
        boolean exceptionOccurred = false;

        DcMotor[] driveMotors = {leftFront, leftBack, rightFront, rightBack};
        String[] driveMotorNames = {"leftFront", "leftBack", "rightFront", "rightBack"};

        //tests each motor
        for (int i = 0; i < driveMotors.length - 1; i++)
        {
            try
            {
                driveMotors[i] = DriveMotor.newMotor(hardwareMap, driveMotorNames[i]);  // returns DcMotor - editable in utils\DriveMotor.java
            } catch (ConfigurationException ex)
            {
                exceptions.append(ex.getMessage()).append("\n");
                exceptionOccurred = true;
            }
        }

        //assigns the motors to the corresponding motor from the array
        leftFront = driveMotors[0];
        rightFront = driveMotors[1];
        leftBack = driveMotors[2];
        rightBack = driveMotors[3];

        // reverse the right motors due to the direction they rotate being flipped on the right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        logoFacingDirection = InitInfo.logoFacingDirection;
        usbFacingDirection = InitInfo.usbFacingDirection;

        //initializes the imu
        try
        {
            imu = DragonsIMU.initialize(hardwareMap, usbFacingDirection, logoFacingDirection);
        } catch (ConfigurationException ex)
        {
            exceptions.append(ex.getMessage()).append("\n");
            exceptionOccurred = true;
        }

        //initializes the limelight
        try
        {
            limelight = DragonsLimelight.initialize(hardwareMap, 0);
        } catch (ConfigurationException ex)
        {
            exceptions.append(ex.getMessage()).append("\n");
            exceptionOccurred = true;
        }

        if (exceptionOccurred)
        {
            telemetry.addLine(exceptions.toString());

            Thread.sleep(5000);

            requestOpModeStop();
            return;
        }

        waitForStart();

        if(isStopRequested()) return;

        moveRobot(-1, 0, 0, 1,1);
        //why not this?
        //moveRobotRC(0, 1, 0, 0.5, leftFront, leftBack, rightFront, rightBack);

    }

    public void moveRobot(double x, double y, double yaw, double strafe, double strafe1) {
        // Calculate wheel powers.
        //changed power calculations by adding *0.7 to run at 70% of what it ran at beforehand
        double leftFrontPower  = (x - y - yaw) * 0.7 * strafe;
        double rightFrontPower = (x + y + yaw) * 0.7 * strafe1;
        double leftBackPower   = (x + y - yaw) * 0.7 * strafe1;
        double rightBackPower  = (x - y + yaw) * 0.7 * strafe;

        //strafe makes robot go right strafe1 makes robot go left
        //always set strafing variables to 1, set one of them negative to go left or right as instructed above

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}

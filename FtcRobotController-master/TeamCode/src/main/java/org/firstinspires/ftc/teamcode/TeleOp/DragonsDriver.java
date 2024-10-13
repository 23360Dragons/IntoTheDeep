package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ConfigurationException;
import org.firstinspires.ftc.teamcode.utils.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;

@TeleOp(name = "Dragons Driver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode
{
    //drive motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    //imu - for orientation
    IMU imu;

    //initialization parameters for the imu
    // TODO: change these values based on robot construction
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    //limelight camera
    private Limelight3A limelight;

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

        while(opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y, x, rightX;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            MoveRobot.moveRobotFC(botHeading, x, y, rightX, 0.5, leftFront, leftBack, rightFront, rightBack); // x, y, and rightX are the gamepad inputs
            //speed should be negative because gamepad y is negative

            //telemetry placeholder code
            {
                int i = 0;
                for (DcMotor motor : driveMotors) {
                    telemetry.addLine().addData(driveMotorNames[i] + " power", motor.getPower());
                    i++;
                }
                telemetry.addLine().addData("botHeading", botHeading);
            }

            DragonsLimelight.update(limelight, telemetry);

            telemetry.update();
        }
    }
}

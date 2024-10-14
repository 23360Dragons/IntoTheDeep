package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.init.InitInfo;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;

@TeleOp(name = "Dragons Driver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    //drive motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    double[] drivePowers;

    //imu - for orientation
    IMU imu;

    //limelight camera
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

            if (InitInfo.movementExceptionOccurred) {
                requestOpModeStop();
            }
            Thread.sleep(5000);
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y, x, rightX;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 0.5); // x, y, and rightX are the gamepad inputs

            leftFront.setPower(drivePowers[0]);
            rightFront.setPower(drivePowers[1]);
            leftBack.setPower(drivePowers[2]);
            rightBack.setPower(drivePowers[3]);

            if (!InitInfo.movementExceptionOccurred) {
                DragonsLimelight.update(limelight, telemetry);

                //put all non-movement code here
            }

            //telemetry placeholder code

            int i = 0;
            for (DcMotor motor : driveMotors) {
                telemetry.addLine().addData(DriveMotor.getDriveMotorNames()[i] + " power", motor.getPower());
                i++;
            }
            telemetry.addLine().addData("botHeading", botHeading);

            telemetry.update();
        }
    }
}

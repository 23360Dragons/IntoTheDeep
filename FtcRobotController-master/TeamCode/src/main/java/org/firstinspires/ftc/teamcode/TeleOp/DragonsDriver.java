package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.BluePipeline;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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



        DcMotor[] driveMotors = DriveMotor.initialize(hardwareMap);

        //assigns the motors to the corresponding motor from the array
        if (driveMotors != null) {
            leftFront = driveMotors[0];
            rightFront = driveMotors[1];
            leftBack = driveMotors[2];
            rightBack = driveMotors[3];
        }

        imu = DragonsIMU.initialize(hardwareMap);

        limelight = DragonsLimelight.initialize(hardwareMap, BluePipeline);



        //check for configuration issues
        if (InitInfo.exceptionOccurred) {
            telemetry.addLine(InitInfo.exceptions.toString());

            Thread.sleep(5000);
        if (!DragonsIMU.isValid && !DriveMotor.isValid) {
                requestOpModeStop();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw(); // provides a way to recalibrate the imu
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu

            //gets input
            double y, x, rightX;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            //creates the powers array
            drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 0.5); // x, y, and rightX are the gamepad inputs

            //sets the motors to their corresponding power
            leftFront.setPower(drivePowers[0]);
            rightFront.setPower(drivePowers[1]);
            leftBack.setPower(drivePowers[2]);
            rightBack.setPower(drivePowers[3]);

            if(DragonsLimelight.isValid)
            {
                DragonsLimelight.update(limelight, telemetry);

            }

            //telemetry placeholder code

            int i = 0;
            assert driveMotors != null;
            for (DcMotor motor : driveMotors) {
                telemetry.addLine().addData(DriveMotor.getDriveMotorNames()[i] + " power", motor.getPower());
                i++;
            }
            telemetry.addLine().addData("botHeading", botHeading);

            telemetry.update();
        }
    }
}

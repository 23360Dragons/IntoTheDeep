package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.exceptionOccurred;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.exceptions;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;

public class DragonsDriver {
    //imu - for orientation
    IMU imu;

    //motors
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor[] motors;

    //limelight camera
    Limelight3A limelight;

    public void init (HardwareMap hardwareMap, Telemetry telemetry, int pipeline) throws Exception {
        exceptions = new StringBuilder("The following exceptions occurred: \n");
        exceptionOccurred = false;

        motors = DriveMotor.initialize(hardwareMap);
        //driveMotors is an array of the motors for telemetry, maybe other things too

        imu = DragonsIMU.initialize(hardwareMap);

        limelight = DragonsLimelight.initialize(hardwareMap, pipeline);

        //check for configuration issues
        if (exceptionOccurred) {
            telemetry.addLine(exceptions.toString());
            telemetry.update();

            Thread.sleep(5000);

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                throw new Exception();
            }
        }
    }

    public void update (Telemetry telemetry) throws InterruptedException {

        if (DragonsLimelight.isValid) {
            DragonsLimelight.update(limelight, telemetry,0);
        }

        while (gamepad1.b) {
            telemetry.addLine("B is pressed!");
            sleep(50);
        }

        if (gamepad1.a) { //provides a way to recalibrate the imu
            telemetry.addLine("reset imu yaw");
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
        telemetry.addData("IMU heading", botHeading);

        //gets input
        double y = -gamepad1.left_stick_y,
                x = gamepad1.left_stick_x,
                rightX = gamepad1.right_stick_x;

        // calls for movement
        double[] drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs

        //sets the motors to their corresponding power
        leftFront.setPower(drivePowers[0]);
        rightFront.setPower(drivePowers[1]);
        leftBack.setPower(drivePowers[2]);
        rightBack.setPower(drivePowers[3]);

        //telemetry
        telemetry.addLine();
        telemetry.addData("leftFront power",  String.valueOf(Math.round(leftFront.getPower()  * 10)/10));
        telemetry.addData("rightFront power", String.valueOf(Math.round(rightFront.getPower() * 10)/10));
        telemetry.addData("leftBack power",   String.valueOf(Math.round(leftBack.getPower()   * 10)/10));
        telemetry.addData("rightBack power",  String.valueOf(Math.round(rightBack.getPower()  * 10)/10));
        telemetry.update();
    }

    static void sleep (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}


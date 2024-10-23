package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.exceptionOccurred;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.exceptions;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.teamcode.utils.init.InitInfo;

public class DragonsDriver {
    public static void init (HardwareMap hardwareMap, Telemetry telemetry, int pipeline) throws Exception {
        exceptions = new StringBuilder("The following exceptions occurred: \n");
        exceptionOccurred = false;

        DriveMotor.initialize(hardwareMap);
        //driveMotors is an array of the motors for telemetry, maybe other things too

        DragonsIMU.initialize(hardwareMap);

        DragonsLimelight.initialize(hardwareMap, pipeline);

        DragonsLights.initialize(hardwareMap);
        InitInfo.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

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

    public static void update (Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {

        if (DragonsLimelight.isValid && DragonsLights.isValid) {
            DragonsLimelight.update(telemetry,0, InitInfo.light);
        }

        while (gamepad1.b) {
            telemetry.addLine("B is pressed!");
            sleep(50);
        }

        if (gamepad1.a) { //provides a way to recalibrate the imu
            telemetry.addLine("reset imu yaw");
            InitInfo.imu.resetYaw();
        }

        double botHeading = InitInfo.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
        telemetry.addData("IMU heading", botHeading);

        //gets input
        double y = -gamepad1.left_stick_y,
                x = gamepad1.left_stick_x,
                rightX = gamepad1.right_stick_x;

        // calls for movement
        double[] drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs

        //sets the motors to their corresponding power
        InitInfo.leftFront.setPower(drivePowers[0]);
        InitInfo.rightFront.setPower(drivePowers[1]);
        InitInfo.leftBack.setPower(drivePowers[2]);
        InitInfo.rightBack.setPower(drivePowers[3]);

        //telemetry
        telemetry.addLine();
        telemetry.addData("leftFront power",  String.valueOf(Math.round(InitInfo.leftFront.getPower()  * 10)/10));
        telemetry.addData("rightFront power", String.valueOf(Math.round(InitInfo.rightFront.getPower() * 10)/10));
        telemetry.addData("leftBack power",   String.valueOf(Math.round(InitInfo.leftBack.getPower()   * 10)/10));
        telemetry.addData("rightBack power",  String.valueOf(Math.round(InitInfo.rightBack.getPower()  * 10)/10));
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


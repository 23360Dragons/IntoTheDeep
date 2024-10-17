package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.BluePipeline;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.RedPipeline;

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

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftBack;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightBack;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.driveMotors;


@TeleOp(name = "Dragons Driver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    double[] drivePowers;

    //imu - for orientation
    IMU imu;

    //limelight camera
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        InitInfo.exceptions = new StringBuilder("The following exceptions occurred: \n");
        InitInfo.exceptionOccurred = false;

        /*DcMotor[] driveMotors = DriveMotor.initialize(hardwareMap);

        //assigns the motors to the corresponding motor from the array
        if (driveMotors != null) {
            leftFront = driveMotors[0];
            rightFront = driveMotors[1];
            leftBack = driveMotors[2];
            rightBack = driveMotors[3];
        } else {
            telemetry.addLine("driveMotors = null");
        }*/

        DriveMotor.initialize(hardwareMap);
        driveMotors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack}; // for telemetry, maybe other things too

        imu = DragonsIMU.initialize(hardwareMap);

        limelight = DragonsLimelight.initialize(hardwareMap, BluePipeline);


        //check for configuration issues
        if (InitInfo.exceptionOccurred) {
            telemetry.addLine(InitInfo.exceptions.toString());
            telemetry.update();

            Thread.sleep(5000);


            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                requestOpModeStop();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (DragonsLimelight.isValid) {
                DragonsLimelight.update(limelight, telemetry);
            }

            if (gamepad1.a) { //provides a way to recalibrate the imu
                telemetry.addLine("reset imu yaw");
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
            telemetry.addData("IMU heading", botHeading);

            //gets input
            double y, x, rightX;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            // calls for movement
            drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs

            //sets the motors to their corresponding power
            leftFront.setPower(drivePowers[0]);
            rightFront.setPower(drivePowers[1]);
            leftBack.setPower(drivePowers[2]);
            rightBack.setPower(drivePowers[3]);

            //telemetry

            telemetry.addLine();
            telemetry.addData("leftFront power", String.valueOf((leftFront.getPower())).substring(0, 5));
            telemetry.addData("rightFront power",String.valueOf((rightFront.getPower())).substring(0, 5));
            telemetry.addData("leftBack power",String.valueOf((leftBack.getPower())).substring(0, 5));
            telemetry.addData("rightBack power",String.valueOf((rightBack.getPower())).substring(0, 5));

            telemetry.update();
        }
    }
}

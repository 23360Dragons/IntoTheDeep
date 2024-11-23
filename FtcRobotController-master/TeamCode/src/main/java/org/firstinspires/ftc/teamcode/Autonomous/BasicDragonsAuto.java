package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.Global;

import static org.firstinspires.ftc.teamcode.utils.Global.leftFront;
import static org.firstinspires.ftc.teamcode.utils.Global.rightFront;
import static org.firstinspires.ftc.teamcode.utils.Global.leftBack;
import static org.firstinspires.ftc.teamcode.utils.Global.rightBack;
import static org.firstinspires.ftc.teamcode.utils.Global.superStructure;
//TODO: UPDATE

@Disabled
@Autonomous
public class BasicDragonsAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        DriveMotor.initialize(hardwareMap, telemetry);

        DragonsIMU.initialize(hardwareMap, telemetry);
        DragonsLimelight.initialize(hardwareMap, telemetry);

        DragonsLights.initialize(hardwareMap, telemetry);

        if (DragonsLights.isValid)
            Global.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        superStructure = new SuperStructure(hardwareMap, telemetry);

        DragonsOTOS.initialize(hardwareMap, telemetry);
        //</editor-fold>

        // --------------------- Configuration Error Handing ---------------------
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep (3000);

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! The IMU, Motors, and all movement code will not work.");
                telemetry.update();
                sleep(2000);
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        moveRobot(-1, 0, 0, 1, 1);

        double[] movement = MoveRobot.RC(0, 1, 0, 0.7);

        // Send powers to the wheels.
        leftFront.setPower (movement[0]);
        rightFront.setPower(movement[1]);
        leftBack.setPower  (movement[2]);
        rightBack.setPower (movement[3]);
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
    }
}

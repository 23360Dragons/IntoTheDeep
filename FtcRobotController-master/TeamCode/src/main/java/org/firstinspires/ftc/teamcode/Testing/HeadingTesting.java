package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.leftBack;
import static org.firstinspires.ftc.teamcode.utils.Global.leftFront;
import static org.firstinspires.ftc.teamcode.utils.Global.rightBack;
import static org.firstinspires.ftc.teamcode.utils.Global.rightFront;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;

@Config
@TeleOp(name="Limelight Heading Test")
public class HeadingTesting extends LinearOpMode {
    PIDController controller;

    public static double p = 0.01,
                         i = 0,
                         d = 0,
                         speed = 0.5,
                         move = 0,
                         targetDiff = 0.1;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveMotor.initialize(hardwareMap, telemetry);
        DragonsLimelight.initialize(hardwareMap, telemetry);

        if (DragonsLimelight.isValid)
            DragonsLimelight.setPipeline(BLUE);

        DragonsIMU.initialize(hardwareMap, telemetry);
        DragonsLights.initialize(hardwareMap, telemetry);

        if (DragonsLights.isValid)
            Global.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        // --------------------- Configuration Error Handing ---------------------
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep (5000);

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! The IMU, Motors, and all movement code will not work.");
                telemetry.update();
                sleep(2000);
            }

            if (!DragonsLimelight.isValid) {
                telemetry.addLine("Limelight is invalid! Exiting...");
                telemetry.update();
                sleep (2000);
                requestOpModeStop();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //get new results
            DragonsLimelight.update(telemetry);

            //update the pid controller
            controller.setPID(p, i, d);

            //update the process variable and the difference
            double  tx = DragonsLimelight.getTx(),
                    pid = controller.calculate(tx, targetDiff);

            //sets movement to the difference
            move += pid;

            // ---- Movement ----
            double[] drivePowers = MoveRobot.RC(0, 0, move, speed);

            leftFront.setPower (drivePowers[0]);
            rightFront.setPower(drivePowers[1]);
            leftBack.setPower  (drivePowers[2]);
            rightBack.setPower (drivePowers[3]);

            // telemetry for debugging
            telemetry.addData("tx", tx);

            telemetry.addLine();
            telemetry.addData("leftFront power",  Math.round(Global.leftFront.getPower()));
            telemetry.addData("rightFront power", Math.round(Global.rightFront.getPower()));
            telemetry.addData("leftBack power",   Math.round(Global.leftBack.getPower()));
            telemetry.addData("rightBack power",  Math.round(Global.rightBack.getPower()));

            telemetry.update();
        }
    }
}

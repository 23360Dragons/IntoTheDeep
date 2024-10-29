package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.BluePipeline;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.RedPipeline;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp(name = "TeleOpRed", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        try {
            DragonsDriver.init(hardwareMap, telemetry, RedPipeline);
        } catch (Exception e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();

            sleep(10000);
            requestOpModeStop();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            try {
                DragonsDriver.update(telemetry, gamepad1, gamepad2, RedPipeline); // todo: finishing touches, maybe add extra params
            } catch (InterruptedException e) {
                stop();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.BluePipeline;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.RedPipeline;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        try {
            DragonsDriver.init(hardwareMap, telemetry, BluePipeline);
        } catch (Exception e) {
            requestOpModeStop();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            try {
                DragonsDriver.update(telemetry, gamepad1, gamepad2); // todo: finishing touches, maybe add extra params
            } catch (InterruptedException e) {
                stop();
            }
        }
    }
}

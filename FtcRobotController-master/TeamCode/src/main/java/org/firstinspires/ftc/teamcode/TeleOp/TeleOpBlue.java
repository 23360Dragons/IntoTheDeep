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
    public void runOpMode() throws InterruptedException {
        HardwareDevice[] devices = new HardwareDevice[5];
        try {
            devices = DragonsDriver.init(hardwareMap, telemetry, BluePipeline);
        } catch (Exception e) {
            requestOpModeStop();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            DragonsDriver.update(devices, telemetry); // todo: finishing touches, maybe add extra params
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.BluePipeline;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.RedPipeline;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp(name = "TeleOpRed", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {
    //needs to be linear, otherwise an exception would be thrown in loop due to the time it takes to call update()
    @Override
    public void runOpMode() {
        DragonsDriver dragonsDriver = new DragonsDriver();
        try {
            dragonsDriver.init(hardwareMap, telemetry, RedPipeline);
        } catch (Exception e) {
            requestOpModeStop();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            try {
                dragonsDriver.update(telemetry); // todo: finishing touches, maybe add extra params
            } catch (InterruptedException e) {
                requestOpModeStop();
            }
        }
    }
}

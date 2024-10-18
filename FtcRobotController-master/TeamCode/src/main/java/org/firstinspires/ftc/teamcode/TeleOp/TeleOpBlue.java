package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.ArrayList;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareDevice[] devices = new HardwareDevice[5];

        try {
            devices = DragonsDriver.init(hardwareMap, telemetry);
        } catch (Exception e) {
            requestOpModeStop();
        }

        waitForStart();

        while (opModeIsActive())
        {
            DragonsDriver.update(devices, telemetry); // todo: finish this, and delete errors
        }
    }
}

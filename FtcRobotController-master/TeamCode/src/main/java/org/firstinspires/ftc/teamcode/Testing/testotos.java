package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DragonsOTOS;

@Config
@TeleOp
public class testotos extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DragonsOTOS otos = new DragonsOTOS(this);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            telemetry.addData("sf x pos", otos.sparkFunOTOS.getPosition().x);
            telemetry.addData("sf y pos", otos.sparkFunOTOS.getPosition().y);
            telemetry.addData("sf h pos", otos.sparkFunOTOS.getPosition().h);
            telemetry.update();
        }
    }
}

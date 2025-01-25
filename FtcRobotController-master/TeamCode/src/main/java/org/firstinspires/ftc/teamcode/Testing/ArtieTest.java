package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SuperStructure;

@Config
@TeleOp
public class ArtieTest extends LinearOpMode {
    public static double p = 0,
            i = 0,
            d = 0,
            target = 0;

    PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SuperStructure superStructure = new SuperStructure(this);
        controller = new PIDController(p, i, d);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controller.setPID(p, i, d);

            double armPos = superStructure.arm.getPosition().avg,
                    pid = controller.calculate(armPos, target);

            superStructure.arm.setPower(pid);

            telemetry.addData("Artie pos", armPos);
            telemetry.addData("Power", pid);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}

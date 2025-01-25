package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;

@Config
@TeleOp(name="Extension PIDF Test")
public class ExtensionTest extends LinearOpMode {
    public static double p = 0,
                         i = 0,
                         d = 0;

    public static int target;
    SuperStructure superStructure;
    PIDController controller;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        superStructure = new SuperStructure(this);

        controller.setPID(p, i, d);

        if (!superStructure.isValid) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();
            sleep (5000);
            telemetry.addLine("Super Structure is invalid. Exiting...");
            telemetry.update();
            sleep (2000);
            requestOpModeStop();
        }

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            controller.setPID(p, i, d);

            double extPos = superStructure.extension.getPosition().right,
                    pid = controller.calculate(extPos, target),
                    armPos = superStructure.arm.getPosition().right;

            superStructure.extension.setPower(pid);

            telemetry.addData("extension power", pid);
            telemetry.addData("Arm pos", armPos);
            telemetry.addData("extension pos", extPos);
            telemetry.update();
        }
    }
}

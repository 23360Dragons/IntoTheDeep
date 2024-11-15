package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.Global;

@Config
@TeleOp(name="Articulation Test")
public class ArticulationTesting extends LinearOpMode {
    public static double p = 0,
                         i = 0,
                         d = 0,
                         f = 0;

    public static int target = 0; // in ticks

    static double ticks_per_degree;

    PIDController controller;
    SuperStructure superStructure;

    @Override
    public void runOpMode() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        superStructure = new SuperStructure(hardwareMap, telemetry);

        if (!superStructure.isValid) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();
            sleep (3000);
            telemetry.addLine("Super Structure is invalid. Exiting...");
            telemetry.update();
            sleep (2000);
            requestOpModeStop();
        }

        ticks_per_degree = superStructure.articulation.getTPD(); //todo: make this final based on reading

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p,i,d);

            double  armPosition = superStructure.articulation.getPosition().right,
                    pid = controller.calculate(armPosition, target),
                    ff = Math.cos(Math.toRadians(superStructure.articulation.getPosition().right / ticks_per_degree)) * f,
                    power = pid + ff;

            superStructure.articulation.setPower(power);

            telemetry.addData("Position", armPosition);
            telemetry.addData("Target", target);
            telemetry.addData("ticks per degree", superStructure.articulation.getTPD());
            telemetry.addLine();

            // TODO: tune this using      192.168.43.1:8080/dash
        }
    }
}

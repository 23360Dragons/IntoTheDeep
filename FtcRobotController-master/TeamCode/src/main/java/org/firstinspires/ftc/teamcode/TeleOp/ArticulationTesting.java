package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SuperStructure.SuperStructure;

@Config
@TeleOp(name="Articulation Test")
public class ArticulationTesting extends LinearOpMode {
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    double ticks_per_degree;

    PIDController controller;
    SuperStructure superStructure;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        superStructure = new SuperStructure(hardwareMap);
        ticks_per_degree = superStructure.getTPR();

        while (opModeIsActive()) {
            controller.setPID(p,i,d);
            int armPosition = superStructure.articulation.getPosition().right;
            double pid = controller.calculate(armPosition, target);
            double ff = Math.cos(Math.toRadians(superStructure.articulation.getPosition().right / ticks_per_degree)) * f;

            double power = Math.max(-1, Math.min(1, pid + ff));

            superStructure.articulation.setPower(power);

            telemetry.addData("Position", armPosition);
            telemetry.addData("Target", target);

            // TODO: tune this using      192.168.43.1:8080/dash
        }
    }
}

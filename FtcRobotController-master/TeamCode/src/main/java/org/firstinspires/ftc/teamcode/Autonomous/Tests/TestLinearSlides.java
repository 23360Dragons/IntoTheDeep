package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.SuperStructure;


@Autonomous(name = "LinearSlidestoHang")
public class TestLinearSlides extends LinearOpMode {
    public static final int hangTicks = SuperStructure.Extension.hangTicks,
                               fullTicks = SuperStructure.Extension.fullTicks,
                               downTicks = SuperStructure.Extension.downTicks;

    public static double kP = 0,
                         kI = 0,
                         kD = 0,
                         speed = 0.5;

    public static int target = downTicks;

    SuperStructure superStructure;

    @Override
    public void runOpMode () {
        superStructure = new SuperStructure(this);

        superStructure.extension.resetEncoders();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            superStructure.extension.setTarget(target);

            superStructure.extension.setVeloCoefficients(kP, kI, kD);

            superStructure.extension.updatePosition(speed);

            telemetry.addData("Target", target);
            telemetry.addData("Current Position", superStructure.extension.getPosition().right);
        }
    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MiniStructure;

@Autonomous(name = "test claw w/ ministructure init", group = "Tests", preselectTeleOp = "DragonsDriver")
public class TestClawMinistructure extends LinearOpMode {
    MiniStructure m;
    @Override
    public void runOpMode() throws InterruptedException {
        m = new MiniStructure(this);

        waitForStart();
        if (!m.claw.isValid) return;

        m.claw.open();
        sleep(2000);
        m.claw.close();
    }
}

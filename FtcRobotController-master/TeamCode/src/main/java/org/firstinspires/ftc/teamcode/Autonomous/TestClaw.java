package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="ClawOpenClose", group="Tests")
public class TestClaw extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
// Initialize the Claw object
        Servo clawServo = hardwareMap.get(Servo.class, "claw"); // From MiniStructure.java
        // From MiniStructure.java
        clawServo.scaleRange(0, 1);
        Claw claw = new Claw(clawServo);

        waitForStart();

        // Open the claw
        claw.OpenClaw();

        // Wait for 2 seconds
        sleep(2000);

        // Close the claw
        claw.CloseClaw();

    }
}

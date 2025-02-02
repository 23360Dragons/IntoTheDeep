package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    public Claw(Servo c) {
        claw = c;
    }

    public void CloseClaw()  {
        double closedRotation = 0.45;
        claw.setPosition(closedRotation);
    }

    public void OpenClaw() {
        //from MiniStructure.java
        double openRotation = 0.7;
        claw.setPosition(openRotation);
    }
}

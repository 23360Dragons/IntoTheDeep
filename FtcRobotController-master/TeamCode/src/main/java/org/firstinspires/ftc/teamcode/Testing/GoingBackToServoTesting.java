package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "testServo", group = "TeleOp")
public class GoingBackToServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftArtie, rightArtie, claw, tilt, twist;
        leftArtie = hardwareMap.get(Servo.class, "leftArtie");
        rightArtie = hardwareMap.get(Servo.class, "rightArtie");
        rightArtie.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        tilt = hardwareMap.get(Servo.class, "tilt");
        twist = hardwareMap.get(Servo.class, "twist");
        waitForStart();
        double lA = leftArtie.getPosition();
        double rA = rightArtie.getPosition();
        double aA = (lA+rA)/2;
        double cL = claw.getPosition();
        double tI = tilt.getPosition();
        double tW = twist.getPosition();
        telemetry.addData("left Artie Pos:", lA);
        telemetry.addData("right Artie Pos:", rA);
        telemetry.addData("average Artie Pos:", aA);
        telemetry.addData("claw Pos:", cL);
        telemetry.addData("tilt Pos:", tI);
        telemetry.addData("twist Pos:", tW);
    }

}

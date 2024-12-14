package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class testCRSERVO extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo;

        servo = hardwareMap.get(CRServo.class, "twist");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.b) {
                servo.setPower(0.5);
            }
            if (gamepad2.a) {
                servo.setPower(0);
            }
            if (gamepad2.x) {
                servo.setPower(1);
            }
            if (gamepad2.y) {
                servo.setPower(-1);
            }

            telemetry.addData("Servo power", servo.getPower());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SuperStructure;

//@TeleOp(name = "test the hang")
public class TestHang extends LinearOpMode {

    public boolean isCanceled = false;
    public boolean cancelHangPressed = false;

    public static boolean hanging = false;
    public static boolean hangButtonPressed = false;
    public static double hangButtonStartTime = 0;
    public static double hangButtonHoldTime = 1000; // milliseconds
    public static double hangDropTime = 5;
    public static double hangTime = 8; // the amount of time to hang for, in seconds

    SuperStructure superStructure;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime hangTimer = new ElapsedTime();

        superStructure = new SuperStructure(this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            boolean hangButton = gamepad2.a;

            if (hangButton) {
                // runs the first time and only the first time every time this button is pressed
                if (!hangButtonPressed) {
                    hangButtonPressed = true;
                    hangButtonStartTime = hangTimer.milliseconds();
                }

                // if the time passed since the button was first pressed is greater than hangButtonHoldTime, do this
                if (hangTimer.milliseconds() >= (hangButtonStartTime + hangButtonHoldTime)) {
                    // do hang
                    telemetry.clearAll();
                    telemetry.addLine("HANGING BEGUN!");
                    telemetry.update();
                    hanging = true;
                }
            } else {
                hangButtonPressed = false;
            }

            telemetry.addData("hanging?", hanging);

            if (hanging) {
                hanging = hangSequence(hangButton, superStructure.extension, hangTime, hangDropTime, this);
            }

            telemetry.update();
        }
    }

    private boolean hangSequence (boolean cancelHang, SuperStructure.Extension superstructureExtension, double hangTime, double hangDropTime, LinearOpMode opmode) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime cancelTimer = new ElapsedTime();
        double power = -1;

        if (cancelHang) {
            if (!cancelHangPressed) {
                cancelHangPressed = true;
                cancelTimer.reset();
            }

            if (cancelTimer.milliseconds() > 300) {
                isCanceled = true;
            }
        } else {
            cancelHangPressed = false;
        }

        // after time is up, fade out the power
        if (timer.seconds() > hangTime || isCanceled) {
            power = Math.min(0, (power + (timer.seconds() - (timer.startTime() + 6)) * (1 / hangDropTime)));
        }

//        superstructureExtension.setPower(power);

        //todo verify these values are expected, then implement it
        opmode.telemetry.clearAll();
        opmode.telemetry.addData("Superstructure Hang Power", power);
        opmode.telemetry.update();

        // if the power is faded out, stop hanging
        return power != 0;
    }
}

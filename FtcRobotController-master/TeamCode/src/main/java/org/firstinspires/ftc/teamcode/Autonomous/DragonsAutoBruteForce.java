package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Autonomous
public class DragonsAutoBruteForce extends LinearOpMode {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        RobotMovement robotMovement = new RobotMovement(leftFront, rightFront, leftBack, rightBack);

        double tiles = 24;
        int starty = 0;
        while (opModeInInit()) {
            if (gamepad1.x) {
                starty = 1;
            } else if (gamepad1.b) {
                starty = 2;
            } else if (gamepad1.a) {
                starty = 3;
            } else if (gamepad1.y) {
                starty = 4;
            }
            switch (starty) {
                case 1:
                    //Actions.runBlocking(littleLarryLime.LarryLimeYellow());
                    telemetry.addLine("Starting Position Set To Blue, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 2:
                    //Actions.runBlocking(littleLarryLime.LarryLimeYellow());
                    telemetry.addLine("Starting Position Set To Red, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 3:
                    //Actions.runBlocking(littleLarryLime.LarryLimeBlues());
                    telemetry.addLine("Starting Position Set To Blue, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 4:
                    //Actions.runBlocking(littleLarryLime.LarryLimeRedTV());
                    telemetry.addLine("Starting Position Set To Red, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                default:
                    telemetry.addLine("Please select starting position! If not selected, the robot will not run during Auto.");
                    telemetry.update();
                    break;
            }
        }

        waitForStart();
        switch (starty) {
            case 1:
                robotMovement.strafe((3.5 * tiles), true);
            case 2:
                robotMovement.strafe((3.5 * tiles), true);
        }
    }

}

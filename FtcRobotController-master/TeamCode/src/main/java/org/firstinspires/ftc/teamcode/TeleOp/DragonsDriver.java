package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.LEFT;
import static org.firstinspires.ftc.teamcode.utils.Global.RED;
import static org.firstinspires.ftc.teamcode.utils.Global.RIGHT;
import static org.firstinspires.ftc.teamcode.utils.Global.yellowPipeline;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.Global;

@TeleOp(name = "DragonsDriver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    static int runPipeline;

    static String startingPos = "None";
    static int startingColor; // 0 is blue, 1 is red
    static int startingSide;  // 0 is left, 1 is right

    @Override
    public void runOpMode () throws InterruptedException {
        //<editor-fold desc="Initialize Robot Hardware">
        Global.exceptions = new StringBuilder("The following exceptions occurred: \n");
        Global.exceptionOccurred = false;

        currentGamepad1  = new Gamepad();
        currentGamepad2  = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        DriveMotor.initialize(hardwareMap, telemetry);

        DragonsIMU.initialize(hardwareMap, telemetry);
        DragonsLimelight.initialize(hardwareMap, telemetry);

        DragonsLights.initialize(hardwareMap, telemetry);
        Global.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        DragonsOTOS.initialize(hardwareMap, telemetry);
        //</editor-fold>

        //check for configuration issues
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! Exiting...");
                telemetry.update();
                sleep(5000);

                requestOpModeStop();
            }

            sleep(5000);
        }

        // init loop
        while (opModeInInit()) {
            telemetry.addLine("Press A for Blue Left");
            telemetry.addLine("Press B for Blue Right");
            telemetry.addLine("Press X for Red Left");
            telemetry.addLine("Press Y for Red Right");

            if (gamepad1.a) {
                startingPos = ("Blue Left");
                startingColor = BLUE;
                startingSide = LEFT;
            } else if (gamepad1.b) {
                startingPos = ("Blue Right");
                startingColor = BLUE;
                startingSide = RIGHT;
            } else if (gamepad1.x) {
                startingPos = ("Red Left");
                startingColor = RED;
                startingSide = LEFT;
            } else if (gamepad1.y) {
                startingPos = ("Red Right");
                startingColor = RED;
                startingSide = RIGHT;
            }

            telemetry.addLine();
            telemetry.addData("Current starting position", startingPos);

            telemetry.update();
        }

        //set limelight pipeline after the field starting position has been determined
        DragonsLimelight.setPipeline(startingColor);
        runPipeline = DragonsLimelight.getPipeline();

        if (isStopRequested()) return;

        // loop
        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop, which
            // does the same thing as copying them at the end. In the first loop
            // through, it will make it a new gamepad.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (DragonsLimelight.isValid && DragonsLights.isValid) {
                DragonsLimelight.update(telemetry);
            }

            if (currentGamepad1.b && !previousGamepad1.b) { //rising edge
                DragonsLimelight.setPipeline(yellowPipeline);
            } else if (!currentGamepad1.b && previousGamepad1.b) { //falling edge
                DragonsLimelight.setPipeline(runPipeline);
            }

            if (DragonsOTOS.isValid) {
                telemetry.addData("Sparkfun velocity along x axis", Global.sparkFunOTOS.getVelocity().x);
                telemetry.addData("Sparkfun velocity along y axis", Global.sparkFunOTOS.getVelocity().y);
                telemetry.addLine();
                telemetry.addData("x", Global.sparkFunOTOS.getPosition().x);
                telemetry.addData("y", Global.sparkFunOTOS.getPosition().y);
                telemetry.addData("heading", Global.sparkFunOTOS.getPosition().h);
            }


            if (currentGamepad1.y) { //provides a way to recalibrate the imu
                telemetry.addLine("reset imu yaw");
                Global.imu.resetYaw();
            }

            double botHeading = Global.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
            telemetry.addData("IMU heading", botHeading);

            //gets input
            double y = -gamepad1.left_stick_y,
                    x = gamepad1.left_stick_x,
                    rightX = gamepad1.right_stick_x;

            // calls for movement
            double[] drivePowers = MoveRobot.FC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs
            //sets the motors to their corresponding power
            Global.leftFront.setPower(drivePowers[0]);
            Global.rightFront.setPower(drivePowers[1]);
            Global.leftBack.setPower(drivePowers[2]);
            Global.rightBack.setPower(drivePowers[3]);

            //telemetry
            telemetry.addLine();
            telemetry.addData("leftFront power",  String.valueOf(Math.round(Global.leftFront.getPower()  * 10)/10));
            telemetry.addData("rightFront power", String.valueOf(Math.round(Global.rightFront.getPower() * 10)/10));
            telemetry.addData("leftBack power",   String.valueOf(Math.round(Global.leftBack.getPower()   * 10)/10));
            telemetry.addData("rightBack power",  String.valueOf(Math.round(Global.rightBack.getPower()  * 10)/10));
            telemetry.update();
        }
    }
}


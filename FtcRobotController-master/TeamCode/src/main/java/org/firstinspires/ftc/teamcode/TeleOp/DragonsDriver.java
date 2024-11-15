package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.LEFT;
import static org.firstinspires.ftc.teamcode.utils.Global.RED;
import static org.firstinspires.ftc.teamcode.utils.Global.RIGHT;
import static org.firstinspires.ftc.teamcode.utils.Global.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Arm.Arm;
import org.firstinspires.ftc.teamcode.SuperStructure.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.Global;

@TeleOp(name = "DragonsDriver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    //<editor-fold desc="--------------------- Local Constants ---------------------">
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    static int runPipeline;

    static String startingPos = "None";
    static int startingColor; // 0 is blue, 1 is red
    static boolean colorIsSet = false;
    static int startingSide;  // 0 is left, 1 is right
    static boolean sideIsSet = false;
    //</editor-fold>

    @Override
    public void runOpMode () throws InterruptedException {
        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        currentGamepad1  = new Gamepad();
        currentGamepad2  = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        DriveMotor.initialize(hardwareMap, telemetry);

        DragonsIMU.initialize(hardwareMap, telemetry);
        DragonsLimelight.initialize(hardwareMap, telemetry);

        DragonsLights.initialize(hardwareMap, telemetry);

        if (DragonsLights.isValid)
            Global.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        DragonsOTOS.initialize(hardwareMap, telemetry);

        Global.superStructure = new SuperStructure(hardwareMap, telemetry);

        Global.arm = new Arm(hardwareMap, telemetry);
        //</editor-fold>

        // --------------------- Configuration Error Handing ---------------------
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep (3000);

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! The IMU, Motors, and all movement code will not work.");
                telemetry.update();
                sleep(2000);
            }
        }

        // --------------------- Choose Starting Position ---------------------
        while (opModeInInit()) {
            telemetry.addLine("Press A for Blue Left");
            telemetry.addLine("Press B for Blue Right");
            telemetry.addLine("Press X for Red Left");
            telemetry.addLine("Press Y for Red Right");
            telemetry.update();

            if (gamepad1.a) {
                startingPos = ("Blue Left");
                startingColor = BLUE;
                startingSide = LEFT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad1.b) {
                startingPos = ("Blue Right");
                startingColor = BLUE;
                startingSide = RIGHT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad1.x) {
                startingPos = ("Red Left");
                startingColor = RED;
                startingSide = LEFT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad1.y) {
                startingPos = ("Red Right");
                startingColor = RED;
                startingSide = RIGHT;

                colorIsSet = true;
                sideIsSet = true;
            }

            telemetry.addLine();
            telemetry.addData("Current starting position", startingPos);
        }

        //<editor-fold desc="--------------------- Set Limelight Pipeline ---------------------">
        if (sideIsSet && colorIsSet && DragonsLimelight.isValid && DragonsLights.isValid) {
            DragonsLimelight.setPipeline(startingColor);
            runPipeline = DragonsLimelight.getPipeline();
        } else {
            if (!DragonsLimelight.isValid) {
                telemetry.addLine("Limelight is not valid. It and the lights will not work.");
            }

            if (!DragonsLights.isValid) {
                telemetry.addLine("Lights are not valid. They will not work.");
            }

            if (!sideIsSet || !colorIsSet) {
                telemetry.addLine("The starting position is not set. Exiting OpMode...");
                telemetry.update();

                sleep(2000);
                requestOpModeStop();
            }

            telemetry.update();
            sleep(2000);
        }
        //</editor-fold>

        if (isStopRequested()) return;

        // --------------------- Main Loop ---------------------
        while (opModeIsActive()) {

            //<editor-fold desc=" --------------------- Input ---------------------">

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

            double  y = -currentGamepad1.left_stick_y,
                    x = currentGamepad1.left_stick_x,
                    rightX = currentGamepad1.right_stick_x;

            //</editor-fold>

            if (Global.superStructure.isValid) {
                double SSspeed = 0.5,
                        articulationPower,
                        extensionPower;

                boolean dpadUp = currentGamepad1.dpad_up,
                        dpadDown = currentGamepad1.dpad_down,
                        rightBumper = gamepad1.right_bumper, // rotates clockwise
                        leftBumper  = gamepad1.left_bumper;  // rotates counterclockwise

                // if dpadUp, 1, else if down, -1, else 0
                extensionPower = dpadUp ? 1 : dpadDown ? -1 : 0;

                articulationPower = rightBumper ? 1 : leftBumper ? -1 : 0;

                if (currentGamepad1.x) {
                    SSspeed = 1;
                }

                Global.superStructure.articulation.setPower(articulationPower * SSspeed);
                Global.superStructure.extension.setPower(extensionPower * SSspeed);

                telemetry.addData("Super Structure extension power",    Global.superStructure.extension.getPower());
                telemetry.addData("Super Structure articulation power", Global.superStructure.articulation.getPower());
                telemetry.addData("Super Structure speed", SSspeed);
            }

            // --------------------- Limelight ---------------------
            if (DragonsLimelight.isValid) {
                // --------------------- Pipeline Switching ---------------------
                if (currentGamepad1.b && !previousGamepad1.b) { //rising edge
                    DragonsLimelight.setPipeline(YELLOW);
                } else if (!currentGamepad1.b && previousGamepad1.b) { //falling edge
                    DragonsLimelight.setPipeline(runPipeline);
                }

                DragonsLimelight.update(telemetry);
            }

            // --------------------- SparkFun OTOS ---------------------
            if (DragonsOTOS.isValid) {
                telemetry.addData("Sparkfun velocity along x axis", Global.sparkFunOTOS.getVelocity().x);
                telemetry.addData("Sparkfun velocity along y axis", Global.sparkFunOTOS.getVelocity().y);
                telemetry.addLine();
                telemetry.addData("x",       Global.sparkFunOTOS.getPosition().x);
                telemetry.addData("y",       Global.sparkFunOTOS.getPosition().y);
                telemetry.addData("heading", Global.sparkFunOTOS.getPosition().h);
            }

            // --------------------- Movement ---------------------
            if (DragonsIMU.isValid && DriveMotor.isValid) {
                if (currentGamepad1.y) { //provides a way to recalibrate the imu
                    telemetry.addLine("reset imu yaw");
                    Global.imu.resetYaw();
                }

                double botHeading = Global.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
                telemetry.addData("IMU heading", botHeading);

                // calls for movement
                double[] drivePowers = MoveRobot.FC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs
                //sets the motors to their corresponding power
                Global.leftFront.setPower(drivePowers[0]);
                Global.rightFront.setPower(drivePowers[1]);
                Global.leftBack.setPower(drivePowers[2]);
                Global.rightBack.setPower(drivePowers[3]);

                //telemetry
                telemetry.addLine();
                telemetry.addData("leftFront power",  String.valueOf(Math.round(Global.leftFront.getPower()  * 10) / 10));
                telemetry.addData("rightFront power", String.valueOf(Math.round(Global.rightFront.getPower() * 10) / 10));
                telemetry.addData("leftBack power",   String.valueOf(Math.round(Global.leftBack.getPower()   * 10) / 10));
                telemetry.addData("rightBack power",  String.valueOf(Math.round(Global.rightBack.getPower()  * 10) / 10));
            }

            telemetry.update();
        }
    }
}
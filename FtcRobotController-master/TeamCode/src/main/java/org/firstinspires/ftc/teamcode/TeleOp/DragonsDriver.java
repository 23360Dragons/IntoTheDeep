package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.LEFT;
import static org.firstinspires.ftc.teamcode.utils.Global.RED;
import static org.firstinspires.ftc.teamcode.utils.Global.RIGHT;
import static org.firstinspires.ftc.teamcode.utils.Global.YELLOW;
import static org.firstinspires.ftc.teamcode.utils.Global.exceptions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotors;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.Global;

@Config
@TeleOp(name = "DragonsDriver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    //<editor-fold desc="--------------------- Local Constants ---------------------">
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    public static double SSspeed;
    public static double extSpeed;

    public static double LLAlignAngle = 0;

    static int runPipeline;

    static String startingPos = "None";
    static int startingColor; // 0 is blue, 1 is red
    static boolean colorIsSet = false;
    static int startingSide;  // 0 is left, 1 is right
    static boolean sideIsSet = false;

    public DriveMotors driveMotors;
    public DragonsIMU dragonsIMU;
    public DragonsLimelight dragonsLimelight;
    public DragonsLights dragonsLights;
    public DragonsOTOS dragonsOTOS;
    public SuperStructure superStructure;
    public Arm arm;
    //</editor-fold>

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Global.exceptions.delete(0, exceptions.capacity()).append("The following were not found:\n");
        Global.exceptionOccurred = false;
        currentGamepad1  = new Gamepad();
        currentGamepad2  = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        driveMotors = new DriveMotors(this);

        dragonsIMU = new DragonsIMU(this);
        dragonsLimelight = new DragonsLimelight(this);

        dragonsLights = new DragonsLights(this);

        if (dragonsLights.isValid)
            dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        dragonsOTOS = new DragonsOTOS(this);

        superStructure = new SuperStructure(this);

        arm = new Arm(this);
        //</editor-fold>

        // --------------------- Configuration Error Handing ---------------------
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep(3000);

            if (!dragonsIMU.isValid || !driveMotors.isValid) {
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

            if (gamepad2.a) {
                startingPos = ("Blue Left");
                startingColor = BLUE;
                startingSide = LEFT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad2.b) {
                startingPos = ("Blue Right");
                startingColor = BLUE;
                startingSide = RIGHT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad2.x) {
                startingPos = ("Red Left");
                startingColor = RED;
                startingSide = LEFT;

                colorIsSet = true;
                sideIsSet = true;
            } else if (gamepad2.y) {
                startingPos = ("Red Right");
                startingColor = RED;
                startingSide = RIGHT;

                colorIsSet = true;
                sideIsSet = true;
            }

            telemetry.addData("Current starting position", startingPos);
        }

        //<editor-fold desc="--------------------- Set Limelight Pipeline ---------------------">
        if (sideIsSet && colorIsSet && dragonsLimelight.isValid) {
            dragonsLimelight.setPipeline(startingColor);
            runPipeline = dragonsLimelight.getPipeline();
        } else {
            if (!dragonsLimelight.isValid) {
                telemetry.addLine("Limelight is not valid. It and the lights will not work.");
            }

            if (!dragonsLights.isValid) {
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

            //gamepad 1 (DRIVER)

            double  y      = -currentGamepad1.left_stick_y,
                    x      = currentGamepad1.left_stick_x,
                    rightX = currentGamepad1.right_stick_x,
                    creepSpeed = currentGamepad1.left_trigger;

            boolean recalibrateIMU = currentGamepad1.y;

            // gamepad 2 (MANIPULATOR)

            double  articulationPower = -currentGamepad2.left_stick_y,
                    extensionPower    = -currentGamepad2.right_stick_y;

            boolean openClaw          = currentGamepad2.right_bumper,
                    closeClaw         = currentGamepad2.left_bumper,
//                    SSFullSpeed       = currentGamepad2.x,
                    // set limelight pipeline
                    currentB2         = currentGamepad2.b, previousB2 = previousGamepad2.b,
                    armDown           = currentGamepad2.a,
//                    armBack           = currentGamepad2.x,
                    armUp             = currentGamepad2.y,
                    twistLeft         = currentGamepad2.dpad_left,
                    twistRight        = currentGamepad2.dpad_right,
                    tiltUp            = currentGamepad2.dpad_up,
                    tiltDown          = currentGamepad2.dpad_down;

            //</editor-fold>

            // <editor-fold desc="--------------------- SuperStructure ---------------------">

            if (superStructure.articulation.isValid) {
                /*if (SSFullSpeed) {
                    if (SSspeed != 1)
                        SSspeed = 1;
                    telemetry.addLine("SS Full Speed!");
                } else*/ if (SSspeed != 0.5 && extSpeed != 0.5) {
                    SSspeed  = 0.5;
                    extSpeed = 0.5;
                }

                superStructure.articulation.setPower(articulationPower * SSspeed);

                telemetry.addData("Super Structure articulation power             ", superStructure.articulation.getPower());
                telemetry.addData("Super Structure right arm position in degrees",   superStructure.articulation.getPosition().right);
                telemetry.addData("Super Structure left arm position in degrees",    superStructure.articulation.getPosition().left);
            }

            if (superStructure.extension.isValid) {
                /*if (SSFullSpeed) {
                    if (extSpeed != 1)
                        extSpeed = 1;
                    telemetry.addLine("Ext Full Speed!");
                } else*/ if (extSpeed != 0.5) {
                    extSpeed = 0.5;
                }

                superStructure.extension.setPower(extensionPower * extSpeed);

                telemetry.addData("Super Structure extension power",     superStructure.extension.getPower());
                telemetry.addData("Super Structure extension position",  superStructure.extension.getPosition().right);
            }

            //</editor-fold>

            // --------------------- Limelight ---------------------
//            if (dragonsLimelight.isValid) {
//                // --------------------- Pipeline Switching ---------------------
//                if (currentB2 && !previousB2) { //rising edge
//                    dragonsLimelight.setPipeline(YELLOW);
//                } else if (!currentB2 && previousB2) { //falling edge
//                    dragonsLimelight.setPipeline(runPipeline);
//                }
//
//                LLAlignAngle = Math.min(Math.abs(dragonsLimelight.update(this, dragonsLights)), 180);
//                telemetry.addData("LLalignTarget", LLAlignAngle);
//            }

            // <editor-fold desc=" --------------------- Arm ---------------------">
            if (arm.claw.isValid) {
//                if (openClaw) {
//                    arm.claw.open();
//                    telemetry.addLine("Open claw");
//                }
//                if (closeClaw) {
//                    arm.claw.close();
//                    telemetry.addLine("Close claw");
//                }
//
//                telemetry.addData("claw Position", arm.claw.getPosition());

                double power = openClaw ? 1 : closeClaw ? -1 : 0;
                arm.claw.setPower(power);
                telemetry.addData("Arm claw power", power);
            }

            if (arm.twist.isValid) {
//                arm.twist.setRotation(LLAlignAngle / 270);
                double power = twistLeft ? -1 : twistRight ? 1 : 0;
                arm.twist.setPower(power);
                telemetry.addData("Arm twist power", power);
            }

            if (arm.tilt.isValid) {
//                if (tiltUp) {
//                    arm.tilt.up();
//                    telemetry.addLine("moving tilt up");
//                }
//                else if (tiltDown) {
//                    arm.tilt.down();
//                    telemetry.addLine("moving tilt down");
//                }

                double power = tiltUp ? 1 : tiltDown ? -1 : 0;
                arm.tilt.setPower(power);
                telemetry.addData("Arm tilt power", power);
            }


            if (arm.artie.isValid) {
//                if (armUp)
//                    arm.artie.setPosition(Arm.Artie.ArtiePos.UP);
//                if (armDown)
//                    arm.artie.setPosition(Arm.Artie.ArtiePos.DOWN);
//                if (armBack)
//                    arm.artie.setPosition(Arm.Artie.ArtiePos.BACK);
//
//                arm.artie.updatePosition();
//                telemetry.addData("artie pos", arm.artie.getPosition().name());
                double power = armUp ? 1 : armDown ? -1 : 0;

                arm.artie.setPower(power);
                telemetry.addData ("Arm artie power", power);
            }
            //</editor-fold>

            // --------------------- SparkFun OTOS ---------------------
//            if (dragonsOTOS.isValid) {
//                telemetry.addData("Sparkfun velocity along x axis", Math.round(dragonsOTOS.sparkFunOTOS.getVelocity().x));
//                telemetry.addData("Sparkfun velocity along y axis", Math.round(dragonsOTOS.sparkFunOTOS.getVelocity().y));
//                telemetry.addLine();
//                telemetry.addData("sparkfun x", Math.round(dragonsOTOS.sparkFunOTOS.getPosition().x));
//                telemetry.addData("sparkfun y", Math.round(dragonsOTOS.sparkFunOTOS.getPosition().y));
//                telemetry.addData("sparkfun heading", Math.round(dragonsOTOS.sparkFunOTOS.getPosition().h));
//            }

            // --------------------- Movement ---------------------
            if (dragonsIMU.isValid && driveMotors.isValid) {
                double driveSpeed = 1;

                if (creepSpeed > 0.1) {
                    driveSpeed *= 0.3;
                }

                if (recalibrateIMU) {
                    telemetry.addLine("reset imu yaw");
                    dragonsIMU.imu.resetYaw();
                }



                double botHeading = dragonsIMU.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
                telemetry.addData("IMU heading", Math.toDegrees(botHeading));

                // calls for movement
                double[] drivePowers = MoveRobot.FC(botHeading, x, y, rightX, driveSpeed); // x, y, and rightX are the gamepad inputs
                //sets the motors to their corresponding power
                driveMotors.setPower(drivePowers);

                //telemetry
                telemetry.addLine();
                telemetry.addData("leftFront power",  String.valueOf(Math.round(driveMotors.getPower()[0])));
                telemetry.addData("rightFront power", String.valueOf(Math.round(driveMotors.getPower()[1])));
                telemetry.addData("leftBack power",   String.valueOf(Math.round(driveMotors.getPower()[2])));
                telemetry.addData("rightBack power",  String.valueOf(Math.round(driveMotors.getPower()[3])));
            }

            telemetry.update();
        }
    }
}
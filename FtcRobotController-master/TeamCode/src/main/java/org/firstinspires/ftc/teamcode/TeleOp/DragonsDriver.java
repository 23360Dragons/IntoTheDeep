package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.RED;
import static org.firstinspires.ftc.teamcode.utils.Global.YELLOW;
import static org.firstinspires.ftc.teamcode.utils.Global.exceptions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.DragonsLights;
import org.firstinspires.ftc.teamcode.hardware.DragonsLimelight;
import org.firstinspires.ftc.teamcode.hardware.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.Global;

import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "DragonsDriver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    //<editor-fold desc="--------------------- Local Constants ---------------------">
    public Drivetrain       drivetrain;
    public DragonsIMU       dragonsIMU;
    public DragonsLimelight dragonsLimelight;
    public DragonsLights    dragonsLights;
    public DragonsOTOS      dragonsOTOS;
    public SuperStructure   superStructure;
    public MiniStructure    miniStructure;

    public static double LLAlignAngle = 0;

    //<editor-fold desc="--------------------- Part Speeds ---------------------">
    public static double SSSpeed      = 0.5;
    public static double SSFullSpeed  = 0.8;
    public static double extSpeed     = 0.7;
    public static double extFullSpeed = 1;
    public static double twistSpeed   = 40;
    public static double tiltSpeed    = 15;
    public static double armSpeed     = 5;
    //</editor-fold>

    //</editor-fold>

    @Override
    public void runOpMode() throws InterruptedException {
        //<editor-fold desc="--------------------- Housekeeping ---------------------">
        boolean debugMode = false;
        telemetry.clearAll();
        telemetry.update();
        // clear exceptions, then re add stuff
        Global.exceptions.delete(0, exceptions.capacity()).append("The following were not found:\n");
        Global.exceptionOccurred = false;
        telemetry        = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Gamepad currentGamepad1  = new Gamepad();
        Gamepad currentGamepad2  = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime time = new ElapsedTime();
        //</editor-fold>

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        drivetrain = new Drivetrain(this);
        dragonsIMU       = new DragonsIMU(this);
        dragonsLimelight = new DragonsLimelight(this);
        dragonsLights    = new DragonsLights(this);
        dragonsOTOS      = new DragonsOTOS(this);
        superStructure   = new SuperStructure(this);
        miniStructure    = new MiniStructure(this);
        //</editor-fold>

        //<editor-fold desc="--------------------- Configuration Error Handing ---------------------">
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep(3000);

            if (!dragonsIMU.isValid || !drivetrain.isValid) {
                telemetry.addLine("Critical Error Occurred! The IMU, Motors, and all movement code will not work.");
                telemetry.update();
                sleep(2000);
            }
        }
        //</editor-fold>

        //<editor-fold desc="--------------------- Wait For Start ---------------------">
        waitForStart();

        if (isStopRequested()) return;
        telemetry.clearAll();

        time.reset();
        time.startTime();
        //</editor-fold>

        //<editor-fold desc="--------------------- Set Ministructure Default Pos ---------------------">
        miniStructure.twist.setPosition(1);
        miniStructure.arm.setPosition(0.5);
        miniStructure.tilt.setPosition(0.2);
        //</editor-fold>

        //<editor-fold desc="--------------------- Main Loop ---------------------">
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

            double  y                 = -currentGamepad1.left_stick_y,
                    x                 = currentGamepad1.left_stick_x,
                    rightX            = currentGamepad1.right_stick_x,

                    articulationDown  = -currentGamepad1.left_trigger;

            boolean recalibrateIMU    = currentGamepad1.a,
                    creepSpeed        = currentGamepad1.right_bumper,
                    SSFullPower       = currentGamepad1.dpad_right,
                    articulationUp    = currentGamepad1.left_bumper,

                    lDebugMode        = currentGamepad1.left_stick_button,
                    rDebugMode        = currentGamepad1.right_stick_button,

                    slidesUp          = currentGamepad1.dpad_up,
                    slidesDown        = currentGamepad1.dpad_down;

            // gamepad 2 (MANIPULATOR)

            //TO DO move articulation to the Drivers gamepad
            //TO DO either LB and LT, or LT and RT
            //TO DO what makes the most sense?

            double  /*articulationPower = -currentGamepad2.left_stick_y,*/
//                    extensionPower    = -currentGamepad2.right_stick_y,
                    armPower          = currentGamepad2.left_stick_y,
                    twistPower        = -currentGamepad2.right_stick_x,
                    tiltPower         = currentGamepad2.right_stick_y;

            boolean openClaw          = currentGamepad2.right_bumper,
                    closeClaw         = currentGamepad2.left_bumper,
                    bluePipeline      = currentGamepad2.x,
                    yellowPipeline    = currentGamepad2.y,
                    redPipeline       = currentGamepad2.b;

            //</editor-fold>

            //<editor-fold desc="--------------------- Debug Mode ---------------------">
            if (lDebugMode && rDebugMode) {
                debugMode = !debugMode;
            }

            telemetry.addData("Debug Mode", debugMode);
            //</editor-fold>

            // <editor-fold desc="--------------------- SuperStructure ---------------------">
            if (superStructure.articulation.isValid || superStructure.extension.isValid)
                telemetry.addLine("-----Super Structure-----");

            if (superStructure.articulation.isValid) {
                double articulationPower = ((articulationUp ? 1 : 0) - (articulationDown > 0.1 ? 1 : 0));

                double velocity = superStructure.articulation.getVelocity().avg;
                double velLimitPwr = articulationPower;
                double speed;

                if (superStructure.articulation.getState() == SuperStructure.ARTICULATION_POS.UP) {
                    speed = SSFullSpeed;
                } else {
                    speed = SSSpeed;
                }

//                if (SSFullPower) {
//                    speed = SSFullSpeed;
//                    telemetry.addLine("SS Full Speed!");
//                } else {
//                    speed = SSSpeed;
//                }

                // handles articulation state for limiting extension
                if (superStructure.articulation.getPosition().avg <= -300) {
                    superStructure.articulation.setState(SuperStructure.ARTICULATION_POS.DOWN);
                } else {
                    superStructure.articulation.setState(SuperStructure.ARTICULATION_POS.UP);
                }

                if (Math.abs(velocity) > 25) {
                    velLimitPwr += velocity / -25;
                } // this is breaking things.

                if (superStructure.extension.isValid
                        && superStructure.articulation.getState() == SuperStructure.ARTICULATION_POS.UP
                        && superStructure.extension.getPosition().avg > superStructure.extension.maxDownExtension
                        && articulationPower < 0
                ) {
                    telemetry.addLine("Articulation cannot go down, as extension is too extended!");
                    //TO //DO flash the lights white or orange (orange might be too close to yellow, test it)
                    dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    currentGamepad1.rumble(1);
                } else {
                    superStructure.articulation.setPower(articulationPower * speed);
                }

                if (debugMode) {
                    telemetry.addData("Super Structure right artie position", superStructure.articulation.getPosition().right);
                    telemetry.addData("Super Structure  left artie position", superStructure.articulation.getPosition().left);
                }

                telemetry.addData("Super structure      vel limit power", velLimitPwr); //todo look at velocity stuff
                telemetry.addData("Super Structure   articulation power", superStructure.articulation.getPower());
                telemetry.addData("Super Structure        enum position", superStructure.articulation.getState());
            }

            if (superStructure.extension.isValid) {
                double extensionPower = ((slidesUp ? 1 : 0) - (slidesDown ? -1 : 0));
                double speed;

                if (SSFullPower) {
                    speed = extFullSpeed;
                    telemetry.addLine("Ext Full Speed!");
                } else {
                    speed = extSpeed;
                }

                if (superStructure.articulation.isValid
                        && superStructure.articulation.getState() == SuperStructure.ARTICULATION_POS.DOWN
                        && extensionPower > 0
                        && superStructure.extension.getPosition().avg > superStructure.extension.maxDownExtension
                ) {
                    telemetry.addLine("Extension cannot extend more, as the arms are down!");
                    //TO //DO flash the lights white or orange (orange might be too close to yellow, test it)
                    dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    currentGamepad1.rumble(1);
                    //todo study rumble
                } else {
                    superStructure.extension.setPower(extensionPower * speed);
                }

                if(debugMode) {
                    telemetry.addData("Super Structure extension power", superStructure.extension.getPower());
                    telemetry.addData("Super Structure extension position", superStructure.extension.getPosition().right);
                    telemetry.addData("Super Structure Extension Draw", superStructure.extension.getCurrent().avg);
                    //todo look at draw
                }
            }

            telemetry.addLine();
            //</editor-fold>

            //<editor-fold desc="--------------------- Limelight ---------------------">
            telemetry.addLine("-----Limelight-----");

            if (dragonsLimelight.isValid) {
                // --------------------- Pipeline Switching ---------------------
//                if (currentB2 && !previousB2) { //rising edge
//                    dragonsLimelight.setPipeline(YELLOW);
//                } else if (!currentB2 && previousB2) { //falling edge
//                    dragonsLimelight.setPipeline();
//                }

                     if (bluePipeline   && dragonsLimelight.getPipeline().num != BLUE)
                    dragonsLimelight.setPipeline(BLUE);
                else if (redPipeline    && dragonsLimelight.getPipeline().num != RED)
                    dragonsLimelight.setPipeline(RED);
                else if (yellowPipeline && dragonsLimelight.getPipeline().num != YELLOW)
                    dragonsLimelight.setPipeline(YELLOW);

                telemetry.addData("Limelight Pipeline", dragonsLimelight.getPipeline().getName());

                LLAlignAngle = Math.min(Math.abs(dragonsLimelight.update(this)), 180);

                if (debugMode) {
                telemetry.addData("LLAlignAngle", LLAlignAngle);
                }
            }

            telemetry.addLine();
            //</editor-fold>

            // <editor-fold desc=" --------------------- MiniStructure ---------------------">
            telemetry.addLine("-----Mini Structure-----");

            if (miniStructure.claw.isValid) {
                if (openClaw) {
                    miniStructure.claw.open();
                    telemetry.addLine("Open claw");
                }

                if (closeClaw) {
                    miniStructure.claw.close();
                    telemetry.addLine("Close claw");
                }

                if (debugMode)
                    telemetry.addData("claw Position", miniStructure.claw.getPosition());

//                double power = openClaw ? 1 : closeClaw ? -1 : 0;
//                arm.claw.setPower(power);
//                telemetry.addData("MiniStructure claw power", power);
            }

            if (miniStructure.twist.isValid) {
//                arm.twist.setRotation(LLAlignAngle / 270);
//                double power = twistLeft ? 1 : twistRight ? -1 : 0;

                double power = (twistPower * 0.001) * twistSpeed;
                double targetPosition = miniStructure.twist.getPosition() + power;

                miniStructure.twist.setPosition(targetPosition);


                telemetry.addData("MiniStructure twist power", power);
                telemetry.addData("MiniStructure twist position", miniStructure.twist.getPosition());
            }

            if (miniStructure.tilt.isValid) {
//                if (tiltUp) {
//                    arm.tilt.up();
//                    telemetry.addLine("moving tilt up");
//                }
//                else if (tiltDown) {
//                    arm.tilt.down();
//                    telemetry.addLine("moving tilt down");
//                }

                double power = (tiltPower * 0.001) * tiltSpeed;
                double targetPosition = miniStructure.tilt.getPosition() + power;

                miniStructure.tilt.setPosition(targetPosition);


                telemetry.addData("MiniStructure tilt power", power);
                telemetry.addData("MiniStructure tilt position", miniStructure.tilt.getPosition());
            }


            if (miniStructure.arm.isValid) {
//                if (armUp)
//                    arm.artie.setPosition(MiniStructure.Arm.ArtiePos.UP);
//                if (armDown)
//                    arm.artie.setPosition(MiniStructure.Arm.ArtiePos.DOWN);
//                if (armBack)
//                    arm.artie.setPosition(MiniStructure.Arm.ArtiePos.BACK);
//
//                arm.artie.updatePosition();
//                telemetry.addData("artie pos", arm.artie.getPosition().name());
//                double power = armUp ? 1 : armDown ? -1 : 0;

                double targetPosition;

                double power = armPower * (0.001 * armSpeed);
                if (superStructure.articulation.isValid) {
                    if (superStructure.articulation.getState() == SuperStructure.ARTICULATION_POS.DOWN && superStructure.articulation.getLastState() != SuperStructure.ARTICULATION_POS.DOWN)
                        miniStructure.arm.setPosition(0.7);
                }
                targetPosition = miniStructure.arm.getPosition().avg + power;

                miniStructure.arm.setPosition(targetPosition);


                telemetry.addData("MiniStructure arm power", power);
                telemetry.addData("MiniStructure arm position", miniStructure.arm.getPosition().avg);
            }

            telemetry.addLine();
            //</editor-fold>

            //<editor-fold desc="--------------------- SparkFun OTOS ---------------------">
            telemetry.addLine("-----Sparkfun OTOS-----");
            DecimalFormat sparkfunDF = new DecimalFormat("#.###");
            //todo: make sure this works

            if (dragonsOTOS.isValid) {
                telemetry.addData("sparkfun x velocity", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getVelocity().x)));
                telemetry.addData("sparkfun y velocity", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getVelocity().y)));
                telemetry.addData("sparkfun x position", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().x)));
                telemetry.addData("sparkfun y position", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().y)));
                telemetry.addData("sparkfun    heading", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().h)));
            }

            telemetry.addLine();
            //</editor-fold>

            //<editor-fold desc="--------------------- Movement ---------------------">
            if (dragonsIMU.isValid && drivetrain.isValid) {
                telemetry.addLine("-----Drivetrain-----");

                double driveSpeed = 1;

                if (creepSpeed) {
                    driveSpeed *= 0.7;
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
                drivetrain.setPower(drivePowers);

                //telemetry
                if (debugMode) {
                    telemetry.addLine();
                    telemetry.addData("leftFront power", String.valueOf(Math.round(drivetrain.getPower()[0])));
                    telemetry.addData("rightFront power", String.valueOf(Math.round(drivetrain.getPower()[1])));
                    telemetry.addData("leftBack power", String.valueOf(Math.round(drivetrain.getPower()[2])));
                    telemetry.addData("rightBack power", String.valueOf(Math.round(drivetrain.getPower()[3])));
                }

                telemetry.addLine();
            }
            //</editor-fold>

            telemetry.addData("Time seconds", time.seconds());
            telemetry.addData("Time toString", time.toString());
            telemetry.addData("Time starttime", time.startTime());
            telemetry.addData("Time seconds - starttime", time.seconds() - time.startTime());
            telemetry.addData("Time time", time.time(TimeUnit.SECONDS));

            telemetry.update();
        }
        //</editor-fold>
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.BLUE;
import static org.firstinspires.ftc.teamcode.utils.Global.RED;
import static org.firstinspires.ftc.teamcode.utils.Global.YELLOW;
import static org.firstinspires.ftc.teamcode.utils.Global.exceptions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.MoveRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.DriveMotors;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.DragonsLights;
import org.firstinspires.ftc.teamcode.hardware.DragonsLimelight;
import org.firstinspires.ftc.teamcode.hardware.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.Global;

@Config
@TeleOp(name = "DragonsDriver", group = "TeleOp")
public class DragonsDriver extends LinearOpMode {
    //<editor-fold desc="--------------------- Local Constants ---------------------">
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    public DriveMotors driveMotors;
    public DragonsIMU dragonsIMU;
    public DragonsLimelight dragonsLimelight;
    public DragonsLights dragonsLights;
    public DragonsOTOS dragonsOTOS;
    public SuperStructure superStructure;
    public Arm arm;

    public static double LLAlignAngle = 0;
    //</editor-fold>

    @Override
    public void runOpMode() throws InterruptedException {
        //<editor-fold desc="--------------------- Housekeeping ---------------------">
        telemetry.clearAll();
        telemetry.update();
        Global.exceptions.delete(0, exceptions.capacity()).append("The following were not found:\n");
        Global.exceptionOccurred = false;
        telemetry        = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentGamepad1  = new Gamepad();
        currentGamepad2  = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        //</editor-fold>

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        driveMotors      = new DriveMotors(this);
        dragonsIMU       = new DragonsIMU(this);
        dragonsLimelight = new DragonsLimelight(this);
        dragonsLights    = new DragonsLights(this);
        dragonsOTOS      = new DragonsOTOS(this);
        superStructure   = new SuperStructure(this);
        arm              = new Arm(this);
        //</editor-fold>

        //<editor-fold desc="--------------------- Part Speeds ---------------------">
        double SSSpeed = 0.5;
        double SSFullSpeed = 0.8;

        double extSpeed = 0.7;
        double extFullSpeed = 1;

        double twistSpeed = 5;
        //</editor-fold>

        //<editor-fold desc="--------------------- Configuration Error Handing ---------------------">
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
        //</editor-fold>

        //<editor-fold desc="--------------------- Wait For Start ---------------------">
        waitForStart();

        if (isStopRequested()) return;
        telemetry.clearAll();
        //</editor-fold>

        //<editor-fold desc="--------------------- Set Twist Default Pos ---------------------">
        arm.twist.setPosition(0.5);
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
                    creepSpeed        = currentGamepad1.left_trigger;

            boolean recalibrateIMU    = currentGamepad1.a,
                    bluePipeline      = currentGamepad1.x,
                    yellowPipeline    = currentGamepad1.y,
                    redPipeline       = currentGamepad1.b;

            // gamepad 2 (MANIPULATOR)

            double  articulationPower = -currentGamepad2.left_stick_y, previousArticulationPower = -previousGamepad2.left_stick_y,
                    extensionPower    = -currentGamepad2.right_stick_y,
                    armUp             = currentGamepad2.right_trigger,
                    armDown           = currentGamepad2.left_trigger;

            boolean openClaw          = currentGamepad2.right_bumper,
                    closeClaw         = currentGamepad2.left_bumper,
                    SSFullPower       = currentGamepad2.x,
//                    armDown           = currentGamepad2.a,
//                    armBack           = currentGamepad2.x,
//                    armUp             = currentGamepad2.y,
                    twistLeft         = currentGamepad2.dpad_left,
                    twistRight        = currentGamepad2.dpad_right,
                    tiltUp            = currentGamepad2.dpad_up,
                    tiltDown          = currentGamepad2.dpad_down;

            //</editor-fold>

            // <editor-fold desc="--------------------- SuperStructure ---------------------">
            if (superStructure.articulation.isValid) {
                double speed;

                if (SSFullPower) {
                    speed = SSFullSpeed;
                    telemetry.addLine("SS Full Speed!");
                } else {
                    speed = SSSpeed;
                }

                superStructure.articulation.setPower(articulationPower * speed);

                if (superStructure.articulation.getPosition().right <= -300) {
                    superStructure.articulation.setState(SuperStructure.ARTICULATION_POS.DOWN);
                } else {
                    superStructure.articulation.setState(SuperStructure.ARTICULATION_POS.UP);
                }

//                if (articulationPower >= 0.3 && previousArticulationPower < 0.3) {
//                    superStructure.articulation.moveUp();
//                    telemetry.addLine("Articulation Up");
//                }
//
//                if (articulationPower <= -0.3 && previousArticulationPower > -0.3) {
//                    superStructure.articulation.moveDown();
//                    telemetry.addLine("Articulation Down");
//                }
//
//                superStructure.articulation.updatePosition();
//
                telemetry.addData("Super Structure articulation power", superStructure.articulation.getPower());
                telemetry.addData("Super Structure right arm position in degrees",   superStructure.articulation.getPosition().right);
                telemetry.addData("Super Structure left arm position in degrees",    superStructure.articulation.getPosition().left);
            }

            if (superStructure.extension.isValid) {
                double speed;

                if (SSFullPower) {
                    speed = extFullSpeed;
                    telemetry.addLine("Ext Full Speed!");
                } else {
                    speed = extSpeed;
                }

                if (superStructure.articulation.getState() == SuperStructure.ARTICULATION_POS.DOWN) {
                    if (extensionPower > 0) {
                        if (superStructure.extension.getPosition().avg > superStructure.extension.maxDownExtension) {
                            telemetry.addLine("Extension cannot extend more, as the arms are down!");
                        } else
                            superStructure.extension.setPower(extensionPower * speed);
                    } else
                        superStructure.extension.setPower(extensionPower * speed);
                } else
                    superStructure.extension.setPower(extensionPower * speed);



                superStructure.extension.setPower(extensionPower * speed);

                telemetry.addData("Super Structure extension power",     superStructure.extension.getPower());
                telemetry.addData("Super Structure extension position",  superStructure.extension.getPosition().right);
            }
            //</editor-fold>

            //<editor-fold desc="--------------------- Limelight ---------------------">
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
                telemetry.addData("LLAlignAngle", LLAlignAngle);
            }
            //</editor-fold>

            // <editor-fold desc=" --------------------- Arm ---------------------">
            if (arm.claw.isValid) {
                if (openClaw) {
                    arm.claw.open();
                    telemetry.addLine("Open claw");
                }
                if (closeClaw) {
                    arm.claw.close();
                    telemetry.addLine("Close claw");
                }

                telemetry.addData("claw Position", arm.claw.getPosition());

//                double power = openClaw ? 1 : closeClaw ? -1 : 0;
//                arm.claw.setPower(power);
//                telemetry.addData("Arm claw power", power);
            }

            if (arm.twist.isValid) {
//                arm.twist.setRotation(LLAlignAngle / 270);

//                double power = twistLeft ? 1 : twistRight ? -1 : 0;

                double power = twistLeft ? 0.001 * twistSpeed : twistRight ? -0.001 * twistSpeed : 0;
                double targetPosition;
//                arm.twist.setPower(power);
                telemetry.addData("Arm twist power", power * 100);


                if (power != 0) {
                    targetPosition = arm.twist.getPosition() + power;

                    arm.twist.setPosition(targetPosition);
                }

                telemetry.addData("twist pos", arm.twist.getPosition());
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
//                double power = armUp ? 1 : armDown ? -1 : 0;

                double power = armUp - armDown;

                arm.artie.setPower(power);
                telemetry.addData ("Arm artie power", power);
            }
            //</editor-fold>

            //<editor-fold desc="--------------------- SparkFun OTOS ---------------------">
            if (dragonsOTOS.isValid) {
                telemetry.addData("Sparkfun velocity along x axis", (dragonsOTOS.sparkFunOTOS.getVelocity().x));
                telemetry.addData("Sparkfun velocity along y axis", (dragonsOTOS.sparkFunOTOS.getVelocity().y));
                telemetry.addLine();
                telemetry.addData("sparkfun x", (dragonsOTOS.sparkFunOTOS.getPosition().x));
                telemetry.addData("sparkfun y", (dragonsOTOS.sparkFunOTOS.getPosition().y));
                telemetry.addData("sparkfun heading", (dragonsOTOS.sparkFunOTOS.getPosition().h));
            }
            //</editor-fold>

            //<editor-fold desc="--------------------- Movement ---------------------">
            if (dragonsIMU.isValid && driveMotors.isValid) {
                double driveSpeed = 1;

                if (creepSpeed > 0.1) {
                    driveSpeed *= 0.5;
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
            //</editor-fold>

            telemetry.update();
        }
        //</editor-fold>
    }
}
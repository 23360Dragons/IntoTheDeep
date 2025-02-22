package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.utils.Global.SSCreepSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.SSSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.extSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.twistSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.tiltSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.armSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.normalDriveSpeed;
import static org.firstinspires.ftc.teamcode.utils.Global.alternateDriveSpeed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DragonsColor;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.DragonsLights;
import org.firstinspires.ftc.teamcode.hardware.DragonsLimelight;
import org.firstinspires.ftc.teamcode.hardware.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

import java.io.PipedInputStream;
import java.text.DecimalFormat;
import java.util.List;

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
    public DragonsColor     dragonsColor;


//    // These enums are for Finite State Machines (see gm0)
//    public enum ScoringState {
//        // slides down, artie down
//        INTAKE,
//        // artie up, slides move up
//        LIFTING,
//        // slides up, artie up, wait a sec then open claw
//        SCORING,
//        //artie up, tilt up (?), slides down
//        LOWERING
//    }
//
//    public enum HangState {
//        // anything when you're not hanging
//        DEFAULT,
//        // ministructure down, extension up
//        EXTENDING,
//        // superstructure down
//        TILTING,
//        // extension down
//        HANGING
//    }
//
//    // set the current state of both to default
//    private HangState    hangState = HangState.DEFAULT;
//    private ScoringState scoringState = ScoringState.INTAKE;

    //</editor-fold>

    @Override
    public void runOpMode() throws InterruptedException {
        //<editor-fold desc="--------------------- Housekeeping ---------------------">
        telemetry.clearAll();
        telemetry.update();
        // clear exceptions, then re add stuff
        Global.exceptions.delete(0, Global.exceptions.capacity()).append("The following were not found:\n");
        Global.exceptionOccurred = false;

        // Puts the control state in manual. This means that slides, ministructure, superstructure are all manually controlled
        Global.switchToManual();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // creates previous gamepads for making holding buttons down only send one input (on down)
        Gamepad currentGamepad1  = new Gamepad();
        Gamepad currentGamepad2  = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // create timers for the FSMs
        ElapsedTime scoringTimer = new ElapsedTime();
        scoringTimer.startTime();
        ElapsedTime hangingTimer = new ElapsedTime();
        hangingTimer.startTime();
        ElapsedTime colorTimer = new ElapsedTime();
        colorTimer.startTime();
        //</editor-fold>

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        drivetrain       = new Drivetrain(this);
        dragonsIMU       = new DragonsIMU(this);
        dragonsLimelight = new DragonsLimelight(this);
        dragonsLights    = new DragonsLights(this, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        dragonsOTOS      = new DragonsOTOS(this);

        //                                               if an auto was run, don't reset the encoders
        superStructure   = new SuperStructure(this, !AutoRobotPos.autoRun);
        miniStructure    = new MiniStructure(this);

        // ministructure and superstructure targets are set in init (make sure to reset them at end of auto)
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
        // for bulk reads (reduces loop time)
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        telemetry.addLine("> Robot ready. Press play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        telemetry.clearAll();
        //</editor-fold>

        //<editor-fold desc="--------------------- Reset timers ---------------------">
        scoringTimer.reset();
        hangingTimer.reset();
        colorTimer.reset();
        //</editor-fold>

        //<editor-fold desc="--------------------- Main Loop ---------------------">
        while (opModeIsActive()) {

            //<editor-fold desc=" --------------------- Input ---------------------">
            // bulk reads reset
            hubs.forEach(LynxModule::clearBulkCache);

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

                    armUp             = currentGamepad1.left_trigger,
                    armDown           = currentGamepad1.right_trigger;

            boolean recalibrateIMU    = currentGamepad1.a,
                    creepSpeed1       = currentGamepad1.right_bumper,

                    SSFull = currentGamepad1.y, prevSSFull = previousGamepad1.y,
                    SSHang = currentGamepad1.x, prevSSHang = previousGamepad1.x,
                    SSDown = currentGamepad1.b, prevSSDown = previousGamepad1.b;

            // gamepad 2 (MANIPULATOR)

            double  artiePower  = -currentGamepad2.right_stick_y,

                    slidesDown  = currentGamepad2.left_trigger,
                    slidesUp    = currentGamepad2.right_trigger;

            boolean closeClaw     = currentGamepad2.left_bumper,
                    openClaw      = currentGamepad2.right_bumper,

                    twistLeft      = currentGamepad2.dpad_left,
                    twistRight     = currentGamepad2.dpad_right,
                    tiltUp         = currentGamepad2.dpad_up,
                    tiltDown       = currentGamepad2.dpad_down,

                    // linear slide stuff
                    controlToggle       = currentGamepad2.back,  prevControlToggle       = previousGamepad2.back,

                    hang = currentGamepad2.x, prevHang = previousGamepad2.x,
                    basketScore = currentGamepad2.y, prevBasketScore = previousGamepad2.y,
                    intake = currentGamepad2.a, prevIntake = previousGamepad2.a,
                    scoreSpecimen = currentGamepad2.b, prevScoreSpecimen = previousGamepad2.b;

            if (controlToggle && !prevControlToggle) {
                Global.toggleControlState();
            }

            telemetry.addData("Control state", Global.controlState.name());
            //</editor-fold>

            if (colorTimer.seconds() > 90) {
                dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                if (colorTimer.seconds() > 110) {
                    dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
            }

            // <editor-fold desc="--------------------- SuperStructure ---------------------">
            if (superStructure.arm.isValid || superStructure.extension.isValid)
                telemetry.addLine("-----Super Structure-----");
            
            // Superstructure arm
            if (superStructure.arm.isValid){
                //                         left trigger  right trigger
                double articulationPower = (armUp - (armDown));

                // handles arm state for limiting extension
                if (superStructure.arm.getPosition().avg <= -190) {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.DOWN);
                } else if (superStructure.arm.getPosition().avg <= -50) {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.HANG);
                } else {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.UP);
                }
                
                double extensionPos = Math.min(superStructure.extension.getPosition().right, SuperStructure.Extension.maxDownExtension)
                        / SuperStructure.Extension.maxDownExtension;
                double speed = (extensionPos * (SSCreepSpeed - SSSpeed)) + SSSpeed;
                
//                switch (Global.controlState) {
                    
//                    case MANUAL:
                        
                        // get extension on a scale of 0 to 1
                        // meaning 0 = 0 ticks, 1 = hang extension or more
                        
                        superStructure.arm.switchToManual();
                        
                        if (superStructure.arm.getState() != SuperStructure.ARTICULATION_POS.DOWN
                                && superStructure.extension.getPosition().left >= SuperStructure.Extension.maxDownExtension
                                && articulationPower < 0) {
                            telemetry.addLine("Arm cannot go down, as extension is too extended!");
                        } else {
                            superStructure.arm.setPower(articulationPower * speed);  // 0.8 if down, 0.5 if up
                        }
                        
                        telemetry.addData("superstructure is being set to", articulationPower * speed);
                    
//                    case AUTO:
//
//                        if (SSFull && !prevSSFull) {
//                            superStructure.arm.full();
//                        } else if (SSHang && !prevSSHang) {
//                            superStructure.arm.hang();
//                            miniStructure.down();
//                        } else if (SSDown && !prevSSDown) {
//                            superStructure.arm.down();
//                            miniStructure.basket();
//                        }
//
//                        if (superStructure.arm.getState() != SuperStructure.ARTICULATION_POS.DOWN
//                                && superStructure.extension.getPosition().left >= SuperStructure.Extension.maxDownExtension) {
//                            superStructure.arm.full();
//                        }
//
//                        superStructure.arm.switchToAuto();
//
//                        if (superStructure.arm.getTarget() == SuperStructure.Arm.SSdownTicks) {
//                            superStructure.arm.setPower((3/4) * speed);
//                        } else if (superStructure.arm.getTarget() == SuperStructure.Arm.SShangTicks) {
//                            superStructure.arm.setPower((3/4) * speed);
//                        } else if (superStructure.arm.getTarget() == SuperStructure.Arm.SSfullTicks) {
//                            superStructure.arm.setPower((2/3) * speed);
//                        }
//                        break;
//                }

                telemetry.addData("Super Structure right artie position", superStructure.arm.getPosition().right);
                telemetry.addData("Super Structure  left artie position", superStructure.arm.getPosition().left);
                telemetry.addData("Super Structure        enum position", superStructure.arm.getState());
            }

            // SUperstructure extension
            if (superStructure.extension.isValid) {
                //                right trigger left trigger
                double slidesPower = (slidesUp - slidesDown);
                double speed = extSpeed;
                
                switch (Global.controlState) {
                    case MANUAL:
                        
                        superStructure.extension.switchToManual();
                        
                        if ((superStructure.arm.getState() != SuperStructure.ARTICULATION_POS.UP)
                                && superStructure.extension.getPosition().left >= SuperStructure.Extension.maxDownExtension) {
                                superStructure.extension.setPower(-(slidesPower * speed));
                                telemetry.addLine("Should just go back");
//                            }
                            
                            telemetry.addLine("Extension cannot extend more, as the arms are down!");
                        } else {
                            if (slidesPower > 0
                                    && superStructure.extension.getPosition().left >= SuperStructure.Extension.maxDownExtension
                                    && superStructure.arm.getState() == SuperStructure.ARTICULATION_POS.DOWN)
                            {
                                telemetry.addLine("slides power > 0, but no allowed (illegal)");
                            } else {
                                superStructure.extension.setPower(slidesPower * speed);
                            }
                        }
                        
                        telemetry.addData("Extension  is being set to", slidesPower * speed);
                        break;
                    
                    case AUTO:
                        
                        if (basketScore && !prevBasketScore) {
                            basketScore(superStructure, miniStructure, telemetry);
                        } else if (hang && !prevHang) {
                            superStructure.extension.actuallyHangOnRung();
                            miniStructure.down();
                        } else if (intake && !prevIntake) {
                            intake(superStructure, miniStructure, telemetry);
                        } else if (scoreSpecimen && !prevScoreSpecimen) {
                            chamberScore(superStructure, miniStructure, telemetry);
                        }
                        
                        if ((superStructure.arm.getState() != SuperStructure.ARTICULATION_POS.UP)
                                && superStructure.extension.getPosition().left > SuperStructure.Extension.maxDownExtension) {
                            if (superStructure.extension.getTarget() > SuperStructure.Extension.maxDownExtension) {
                                superStructure.extension.setTarget(SuperStructure.Extension.maxDownExtension);
                            }
                        }
                        
                        superStructure.extension.switchToAuto();
                        
                        superStructure.extension.setPower(speed);
                        break;
                }

                telemetry.addData("Extension L position", superStructure.extension.getPosition().left);
                telemetry.addData("Extension R position (used for control)", superStructure.extension.getPosition().right);
            }

            telemetry.addLine();
            //</editor-fold>

            // <editor-fold desc=" --------------------- MiniStructure ---------------------">
            telemetry.addLine("-----Mini Structure-----");
            
            if(currentGamepad2.left_stick_button) {
                miniStructure.tilt.setPosition(0.8);
                miniStructure.artie.setPosition(miniStructure.artie.intakeSpec);
            }

            if (miniStructure.claw.isValid) {
                //  right bumper
                if (openClaw) {
                    miniStructure.claw.open();
                }

                //  left bumper
                if (closeClaw) {
                    miniStructure.claw.close();
                }
            }

            if (miniStructure.twist.isValid) {
//                             dpad left                        dpad right
                double power = twistLeft ? 0.001 * twistSpeed : twistRight ? -0.001 * twistSpeed : 0;
                double targetPosition = miniStructure.twist.getPosition() + power;

                miniStructure.twist.setPosition(targetPosition);

                telemetry.addData("MiniStructure twist position", miniStructure.twist.getPosition());
            }

            if (miniStructure.tilt.isValid) {
                double power = tiltUp ? 0.001 * tiltSpeed : tiltDown ? -0.001 * tiltSpeed : 0;
                double targetPosition = miniStructure.tilt.getPosition() + power;
                
                miniStructure.tilt.setPosition(targetPosition);
                telemetry.addData("MiniStructure tilt power", power);
                telemetry.addData("MiniStructure tilt position", miniStructure.tilt.getPosition());
            }

            if (miniStructure.artie.isValid) {
                double power = artiePower * (0.005 * armSpeed);
                double targetPosition = (miniStructure.artie.getPosition().avg + power);
                
                miniStructure.artie.setPosition(targetPosition);
                telemetry.addData("MiniStructure artie power", power);
                telemetry.addData("Ministructure Target Position", targetPosition);
                telemetry.addData("MiniStructure artie L position", miniStructure.artie.getPosition().left);
                telemetry.addData("MiniStructure artie R position", miniStructure.artie.getPosition().right);
            }

            telemetry.addLine();
            //</editor-fold>

            // <editor-fold desc="--------------------- Movement ---------------------">
            if (dragonsIMU.isValid && drivetrain.isValid) {
                telemetry.addLine("-----Drivetrain-----");

                double driveSpeed = normalDriveSpeed;

                if (creepSpeed1) {
                    driveSpeed = alternateDriveSpeed;
                }

                if (recalibrateIMU) {
                    telemetry.addLine("reset imu yaw");
                    dragonsIMU.imu.resetYaw();

                    if (dragonsOTOS.isValid) {
                        dragonsOTOS.sparkFunOTOS.calibrateImu();
                    }
                }

                double botHeading = dragonsIMU.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
                telemetry.addData("IMU heading", Math.toDegrees(botHeading));

                // calls for movement
                drivetrain.RC(x, y, rightX, driveSpeed); // x, y, and rightX are the gamepad inputs

//                telemetry.addData("leftFront power",  String.valueOf(Math.round(drivetrain.getPower()[0])));
//                telemetry.addData("rightFront power", String.valueOf(Math.round(drivetrain.getPower()[1])));
//                telemetry.addData("leftBack power",   String.valueOf(Math.round(drivetrain.getPower()[2])));
//                telemetry.addData("rightBack power",  String.valueOf(Math.round(drivetrain.getPower()[3])));

                telemetry.addLine();
            }

            //</editor-fold>

            telemetry.update();
        }
        //</editor-fold>

        AutoRobotPos.reset();
    }

    private void basketScore (SuperStructure superStructure, MiniStructure miniStructure, Telemetry telemetry) {
        superStructure.extension.full();
        miniStructure.basket();

        telemetry.addLine("Raising extension and artie!");
    }

    private void chamberScore (SuperStructure superStructure, MiniStructure miniStructure, Telemetry telemetry) {
        superStructure.extension.hang();
        miniStructure.chamber();

        telemetry.addLine("Chamber extension and artie!");
    }

    private void intake (SuperStructure superStructure, MiniStructure miniStructure, Telemetry telemetry) {
        superStructure.extension.down();
        miniStructure.down();

        telemetry.addLine("Lowering extension and artie!");
    }
}
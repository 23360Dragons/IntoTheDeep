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

    public boolean isCanceled = false;
    public boolean cancelHangPressed = false;

    //<editor-fold desc="--------------------- Part Speeds ---------------------">
    public static double SSSpeed      = 1;
    public static double SSCreepSpeed = 0.5;
    public static double extSpeed     = 1;
    public static double extCreepSpeed = 0.75;
    public static double twistSpeed   = 50;
    public static double tiltSpeed    = 30;
    public static double armSpeed     = 5;
    public static double armCreepSpeed = 3;
    //</editor-fold>

    //</editor-fold>

    @Override
    public void runOpMode() throws InterruptedException {
        //<editor-fold desc="--------------------- Housekeeping ---------------------">
        telemetry.clearAll();
        telemetry.update();
        // clear exceptions, then re add stuff
        Global.exceptions.delete(0, exceptions.capacity()).append("The following were not found:\n");
        Global.exceptionOccurred = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Gamepad currentGamepad1  = new Gamepad();
        Gamepad currentGamepad2  = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean hanging = false;
        boolean hangButtonPressed = false;
        double hangButtonStartTime = 0;
        double hangButtonHoldTime = 300; // milliseconds
        double hangTime = 6; // the amount of time to hang for, in seconds
        ElapsedTime hangTimer = new ElapsedTime();
        //</editor-fold>

        //<editor-fold desc="--------------------- Initialize Robot Hardware ---------------------">
        drivetrain       = new Drivetrain(this);
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

        hangTimer.reset();
        hangTimer.startTime();
        //</editor-fold>

        //<editor-fold desc="--------------------- Set Ministructure Default Pos ---------------------">
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

                    armUp             = currentGamepad1.left_trigger,
                    armDown           = currentGamepad1.right_trigger;

            boolean recalibrateIMU    = currentGamepad1.a,
                    creepSpeed1       = currentGamepad1.right_bumper;

            // gamepad 2 (MANIPULATOR)

            double  artiePower  = -currentGamepad2.right_stick_y,

                    slidesPower = -currentGamepad2.left_stick_y;

            boolean fullSpeed2     = currentGamepad2.right_bumper,
                    toggleClaw     = currentGamepad2.left_bumper,

                    //reset extension encoder positions
                    leftStickButton  = currentGamepad2.left_stick_button,
                    rightStickButton = currentGamepad2.right_stick_button,

                    twistLeft      = currentGamepad2.dpad_left,
                    twistRight     = currentGamepad2.dpad_right,
                    tiltUp         = currentGamepad2.dpad_up,
                    tiltDown       = currentGamepad2.dpad_down,

                    bluePipeline   = currentGamepad2.x,
                    yellowPipeline = currentGamepad2.y,
                    redPipeline    = currentGamepad2.b,

                    hangButton     = currentGamepad2.a;

            //</editor-fold>

            // <editor-fold desc="--------------------- SuperStructure ---------------------">
            if (superStructure.arm.isValid || superStructure.extension.isValid)
                telemetry.addLine("-----Super Structure-----");

            if (superStructure.arm.isValid){
                //                         left trigger       right trigger
                double articulationPower = (armUp - (armDown));
                telemetry.addData("Plain arm", (armUp - armDown));

                double velocity = superStructure.arm.getVelocity().avg;
                double velLimitPwr = articulationPower;
                double speed;

                if (creepSpeed1) {
                    speed = SSCreepSpeed;
                } else {
                    speed = SSSpeed;
                }

                // handles arm state for limiting extension
                if (superStructure.arm.getPosition().avg <= -200) {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.DOWN);
                } else if (superStructure.arm.getPosition().avg <= -100) {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.HANG);
                } else {
                    superStructure.arm.setState(SuperStructure.ARTICULATION_POS.UP);
                }

                if (Math.abs(velocity) > 25) {
                    velLimitPwr += velocity / -25;
                } // this is breaking things.

                if (superStructure.extension.isValid
                        && superStructure.arm.getState() != SuperStructure.ARTICULATION_POS.DOWN
                        && superStructure.extension.getPosition().avg >= superStructure.extension.maxDownExtension
//                        && articulationPower < 0
                ) {
                    telemetry.addLine("Arm cannot go down, as extension is too extended!");
                    //TO //DO flash the lights white or orange (orange might be too close to yellow, test it)
                    dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    currentGamepad1.rumble(1);
                }
                else {
                    superStructure.arm.setPower(articulationPower * speed);
                }

                telemetry.addData("Super Structure right artie position", superStructure.arm.getPosition().right);
                telemetry.addData("Super Structure  left artie position", superStructure.arm.getPosition().left);
                telemetry.addData("Super structure      vel limit power", velLimitPwr); //todo look at velocity stuff
                telemetry.addData("Super Structure            arm power", superStructure.arm.getPower());
                telemetry.addData("Super Structure        enum position", superStructure.arm.getState());
            }

            if (superStructure.extension.isValid) {
                                        //dpad up              dpad down
//                double extensionPower = ((slidesUp ? 1 : 0) - (slidesDown ? 1 : 0));
                double speed;

                if (fullSpeed2) {
                    speed = extSpeed;
                } else {
                    speed = extCreepSpeed;
                }

                if (leftStickButton && rightStickButton) {
                    superStructure.extension.resetEncoders();
                }

                if (superStructure.arm.isValid
                        && superStructure.arm.getState() == SuperStructure.ARTICULATION_POS.DOWN
                        && slidesPower > 0
                        && superStructure.extension.getPosition().avg >= superStructure.extension.maxDownExtension
                ) {
                    telemetry.addLine("Extension cannot extend more, as the arms are down!");
                    //TO //DO flash the lights white or orange (orange might be too close to yellow, test it)
                    dragonsLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    currentGamepad1.rumble(1);
                }
                else {
                    if (!hanging)
                        superStructure.extension.setPower(slidesPower * speed);
                }

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

                if (hanging) {
                    hanging = hangSequence(hangButton, superStructure.extension, hangTime, this);
                }

                telemetry.addData("Hanging value", hanging);

                telemetry.addData("Super Structure extension power", superStructure.extension.getPower());
                telemetry.addData("Super Structure extension L position", superStructure.extension.getPosition().left);
                telemetry.addData("Super Structure extension R position", superStructure.extension.getPosition().right);
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

                //       x
                     if (bluePipeline   && dragonsLimelight.getPipeline().num != BLUE)
                    dragonsLimelight.setPipeline(BLUE);//
                //       b
                else if (redPipeline    && dragonsLimelight.getPipeline().num != RED)
                    dragonsLimelight.setPipeline(RED);
                //       y
                else if (yellowPipeline && dragonsLimelight.getPipeline().num != YELLOW)
                    dragonsLimelight.setPipeline(YELLOW);

                telemetry.addData("Limelight Pipeline", dragonsLimelight.getPipeline().getName());

//                LLAlignAngle = Math.min(Math.abs(dragonsLimelight.update(this)), 180);
                dragonsLimelight.update(this);
            }

            telemetry.addLine();
            //</editor-fold>

            // <editor-fold desc=" --------------------- MiniStructure ---------------------">
            telemetry.addLine("-----Mini Structure-----");

            if (miniStructure.claw.isValid) {
                //  left bumper
                if (toggleClaw) {
                    miniStructure.claw.toggle();
                }

                telemetry.addData("claw Position", miniStructure.claw.getPosition());
            }

            if (miniStructure.twist.isValid) {
//                             dpad left                        dpad right
//                double power = twistLeft ? 0.001 * twistSpeed : twistRight ? -0.001 * twistSpeed : 0;
//                double targetPosition = miniStructure.twist.getPosition() + power;

                if (twistLeft || twistRight) {
                    miniStructure.twist.togglePos();
                }

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
                double targetPosition;
                double speed;

                if (fullSpeed2) {
                    speed = armSpeed;
                } else {
                    speed = armCreepSpeed;
                }

                //             right stick y
                double power = artiePower * (0.005 * armSpeed);

                targetPosition = miniStructure.artie.getPosition().avg + power;

                miniStructure.artie.setPosition(targetPosition);

                telemetry.addData("MiniStructure artie power", power);
                telemetry.addData("Ministructure Target Position", targetPosition);
                telemetry.addData("MiniStructure artie L position", miniStructure.artie.getPosition().left);
                telemetry.addData("MiniStructure artie R position", miniStructure.artie.getPosition().right);
            }

            telemetry.addLine();
            //</editor-fold>

            //<editor-fold desc="--------------------- SparkFun OTOS ---------------------">
            telemetry.addLine("-----Sparkfun OTOS-----");
            DecimalFormat sparkfunDF = new DecimalFormat("#.###");

            if (dragonsOTOS.isValid) {
                telemetry.addData("sparkfun x velocity", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getVelocity().x)));
                telemetry.addData("sparkfun y velocity", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getVelocity().y)));
                telemetry.addData("sparkfun x position", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().x)));
                telemetry.addData("sparkfun y position", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().y)));
                telemetry.addData("sparkfun    heading", (sparkfunDF.format(dragonsOTOS.sparkFunOTOS.getPosition().h)));
            }

            telemetry.addLine();
            //</editor-fold>

            // <editor-fold desc="--------------------- Movement ---------------------">
            if (dragonsIMU.isValid && drivetrain.isValid) {
                telemetry.addLine("-----Drivetrain-----");

                double driveSpeed = 1;

                if (creepSpeed1) {
                    driveSpeed *= 0.6;
                }

                if (recalibrateIMU) {
                    telemetry.addLine("reset imu yaw");
                    dragonsIMU.imu.resetYaw();
                }

                double botHeading = dragonsIMU.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
                telemetry.addData("IMU heading", Math.toDegrees(botHeading));

                // calls for movement
                double[] drivePowers = MoveRobot.RC(x, y, rightX, driveSpeed); // x, y, and rightX are the gamepad inputs
                //sets the motors to their corresponding power
                drivetrain.setPower(drivePowers);
//                telemetry.addData("leftFront power", String.valueOf(Math.round(drivetrain.getPower()[0])));
//                telemetry.addData("rightFront power", String.valueOf(Math.round(drivetrain.getPower()[1])));
//                telemetry.addData("leftBack power", String.valueOf(Math.round(drivetrain.getPower()[2])));
//                telemetry.addData("rightBack power", String.valueOf(Math.round(drivetrain.getPower()[3])));
                telemetry.addLine();
            }

            //</editor-fold>

            telemetry.update();
        }
        //</editor-fold>
    }

    private boolean hangSequence (boolean cancelHang, SuperStructure.Extension superstructureExtension, double hangTime, LinearOpMode opmode) {
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
            power = Math.min(0, (power + (timer.seconds() - (timer.startTime() + 6)) * 0.2));
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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.init.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLights;
import org.firstinspires.ftc.teamcode.utils.init.DragonsLimelight;
import org.firstinspires.ftc.teamcode.utils.init.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.init.DriveMotor;
import org.firstinspires.ftc.teamcode.utils.init.Consts;

public class DragonsDriver {
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    static int currentPipeline;
    static int runPipeline;

    static String startingPos = "None";

    public static void init (HardwareMap hardwareMap, Telemetry telemetry, int pipeline) throws InterruptedException {

        Consts.exceptions = new StringBuilder("The following exceptions occurred: \n");
        Consts.exceptionOccurred = false;

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        DriveMotor.initialize(hardwareMap, telemetry);

        DragonsIMU.initialize(hardwareMap, telemetry);
        DragonsLimelight.initialize(hardwareMap, telemetry);
        DragonsLimelight.setPipeline(pipeline);
        currentPipeline = pipeline;
        runPipeline = pipeline;

        DragonsLights.initialize(hardwareMap, telemetry);
        Consts.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        DragonsOTOS.initialize(hardwareMap, telemetry);

        //check for configuration issues
        if (Consts.exceptionOccurred) {
            telemetry.addLine(Consts.exceptions.toString());
            telemetry.update();

            sleep(5000);

            if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! Exiting...");
                telemetry.update();
                sleep(5000);

                throw new InterruptedException();
            }
        }
    }

    public static void init_loop (Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        telemetry.addLine("Press A for Blue Left");
        telemetry.addLine("Press B for Blue Right");
        telemetry.addLine("Press X for Red Left");
        telemetry.addLine("Press Y for Red Right");

        if (gamepad1.a) {
            startingPos = ("Blue Left");
        }
        else if (gamepad1.b) {
            startingPos = ("Blue Right");
        }
        else if (gamepad1.x) {
            startingPos = ("Red Left");
        }
        else if (gamepad1.y) {
            startingPos = ("Red Right");
        }

        telemetry.addLine();
        telemetry.addData("Current starting position", startingPos);

        telemetry.update();
    }

    public static void update (Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
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
            DragonsLimelight.setPipeline(Consts.yellowPipeline);
        } else if (!currentGamepad1.b && previousGamepad1.b) { //falling edge
            DragonsLimelight.setPipeline(runPipeline);
        }

        if (DragonsOTOS.isValid) {
            telemetry.addData("Sparkfun angular scalar", Consts.sparkFunOTOS.getAngularScalar());
            telemetry.addData("Sparkfun acceleration", Consts.sparkFunOTOS.getAcceleration());
            telemetry.addData("Sparkfun velocity", Consts.sparkFunOTOS.getVelocity());
            telemetry.addLine();
            telemetry.addData("x", Consts.sparkFunOTOS.getPosition().x);
            telemetry.addData("y", Consts.sparkFunOTOS.getPosition().y);
            telemetry.addData("heading", Consts.sparkFunOTOS.getPosition().h);
        }


        /*if (currentGamepad1.y) { //provides a way to recalibrate the imu
            telemetry.addLine("reset imu yaw");
            Consts.imu.resetYaw();
        }*/

        /*double botHeading = Consts.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
        telemetry.addData("IMU heading", botHeading);

        //gets input
        double y = -gamepad1.left_stick_y,
                x = gamepad1.left_stick_x,
                rightX = gamepad1.right_stick_x;

        // calls for movement
        double[] drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs
        //sets the motors to their corresponding power
        Consts.leftFront.setPower(drivePowers[0]);
        Consts.rightFront.setPower(drivePowers[1]);
        Consts.leftBack.setPower(drivePowers[2]);
        Consts.rightBack.setPower(drivePowers[3]);

        //telemetry
        telemetry.addLine();
        telemetry.addData("leftFront power",  String.valueOf(Math.round(Consts.leftFront.getPower()  * 10)/10));
        telemetry.addData("rightFront power", String.valueOf(Math.round(Consts.rightFront.getPower() * 10)/10));
        telemetry.addData("leftBack power",   String.valueOf(Math.round(Consts.leftBack.getPower()   * 10)/10));
        telemetry.addData("rightBack power",  String.valueOf(Math.round(Consts.rightBack.getPower()  * 10)/10));*/
        telemetry.update();
    }

    static void sleep (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}


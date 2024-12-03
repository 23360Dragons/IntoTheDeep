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
import org.firstinspires.ftc.teamcode.utils.Global;

public class DragonsDriver {
    static Gamepad currentGamepad1;
    static Gamepad currentGamepad2;
    static Gamepad previousGamepad1;
    static Gamepad previousGamepad2;

    static int currentPipeline;
    static int runPipeline;

    static String startingPos = "None";
    static int count = 0;
    static int colorPipeline;

    public static void init (HardwareMap hardwareMap, Telemetry telemetry, int pipeline) throws InterruptedException {
//
        Global.exceptions = new StringBuilder("The following exceptions occurred: \n");
        Global.exceptionOccurred = false;

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

//        DriveMotor.initialize(hardwareMap, telemetry);
//        DragonsIMU.initialize(hardwareMap, telemetry);
//
        DragonsLimelight.initialize(hardwareMap, telemetry);
        DragonsLimelight.setPipeline(pipeline);
        currentPipeline = pipeline;
        runPipeline = pipeline;

        DragonsLights.initialize(hardwareMap, telemetry);
        Global.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

//        DragonsOTOS.initialize(hardwareMap, telemetry);

        //check for configuration issues
        if (Global.exceptionOccurred) {
            telemetry.addLine(Global.exceptions.toString());
            telemetry.update();

            sleep(5000);

           /* if (!DragonsIMU.isValid || !DriveMotor.isValid) {
                telemetry.addLine("Critical Error Occurred! Exiting...");
                telemetry.update();
                sleep(3000);

                throw new InterruptedException();
            }*/
        }
    }

    /*public static void init_loop (Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        telemetry.addLine("Press A for Blue Left");
        telemetry.addLine("Press B for Blue Right");
        telemetry.addLine("Press X for Red Left");
        telemetry.addLine("Press Y for Red Right");

        if (gamepad1.a) {
            startingPos = ("Blue Left");
            runPipeline = Global.BluePipeline;
        }
        else if (gamepad1.b) {
            startingPos = ("Blue Right");
            runPipeline = Global.BluePipeline;
        }
        else if (gamepad1.x) {
            startingPos = ("Red Left");
            runPipeline = Global.RedPipeline;
        }
        else if (gamepad1.y) {
            startingPos = ("Red Right");
            runPipeline = Global.RedPipeline;
        }

        telemetry.addLine();
        telemetry.addData("Current starting position", startingPos);

        telemetry.update();
    }*/

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

        boolean color = false;
        count++;

        if(count > 0 && count < 50)
        {
            color=true;
        }
        else if (count> 50 && count < 100)
        {
            color =false;
        }
        else if(count>100)
        {
            count=0;
        }


        telemetry.addData("count", count);
        if(color)
        {
            telemetry.addData("color","looking for blue" );
        }
        else{
            telemetry.addData("color", "looking for yellow");
        }


        if (currentGamepad1.b && !previousGamepad1.b) { //rising edge
            DragonsLimelight.setPipeline(Global.YELLOW);
        } else if (!currentGamepad1.b && previousGamepad1.b) { //falling edge
            DragonsLimelight.setPipeline(runPipeline);
        }

        if (DragonsLimelight.isValid && DragonsLights.isValid) {
            DragonsLimelight.update(telemetry);
        }

        if (!color) { //rising edge
            DragonsLimelight.setPipeline(Global.YELLOW);
        } else { //falling edge
            DragonsLimelight.setPipeline(colorPipeline);
        }


        /*if (DragonsOTOS.isValid) {
            telemetry.addData("Sparkfun angular scalar", Global.sparkFunOTOS.getAngularScalar());
            telemetry.addData("Sparkfun acceleration", Global.sparkFunOTOS.getAcceleration());
            telemetry.addData("Sparkfun velocity", Global.sparkFunOTOS.getVelocity());
            telemetry.addLine();
            telemetry.addData("x", Global.sparkFunOTOS.getPosition().x);
            telemetry.addData("y", Global.sparkFunOTOS.getPosition().y);
            telemetry.addData("heading", Global.sparkFunOTOS.getPosition().h);
        }
*/

        /*if (currentGamepad1.y) { //provides a way to recalibrate the imu
            telemetry.addLine("reset imu yaw");
            Global.imu.resetYaw();
        }*/

        /*double botHeading = Global.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //updates the imu
        telemetry.addData("IMU heading", botHeading);

        //gets input
        double y = -gamepad1.left_stick_y,
                x = gamepad1.left_stick_x,
                rightX = gamepad1.right_stick_x;

        // calls for movement
        double[] drivePowers = MoveRobot.moveRobotFC(botHeading, x, y, rightX, 1); // x, y, and rightX are the gamepad inputs
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
        telemetry.addData("rightBack power",  String.valueOf(Math.round(Global.rightBack.getPower()  * 10)/10));*/
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


//package org.firstinspires.ftc.teamcode.Autonomous;
package org.firstinspires.ftc.teamcode.Autonomous;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Autonomous
public class FirstAuto extends LinearOpMode
//
// //do we need extends LinearOpMode above or should we use without below
// public class MoreAuto
        //original code example below for Blue2

//public class Blue2 extends LinearOpMode
{
    IMU _orient;
 //   DcMotor _intake;

    DcMotor _leftFront;//Port 0
    DcMotor _leftBack;//Port 1
    DcMotor _rightFront;//Port2
    DcMotor _rightBack;//Port3
    @Override
    public void runOpMode() throws InterruptedException {

       /*
            AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

            VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcamera"))
                .setCameraResolution(new Size(640,480))
                .build();
        */

       // _intake = hardwareMap.get(DcMotor.class, "Intake");
        _orient = hardwareMap.get(IMU.class, "imu");
        _leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //assumption - right side reversed
        _leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        //For encoders
        int lF = _leftFront.getCurrentPosition();
        int rF = _rightFront.getCurrentPosition();
        int lB = _leftBack.getCurrentPosition();
        int rB = _rightBack.getCurrentPosition();

        YawPitchRollAngles robotOrientation;
        robotOrientation = _orient.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);


        waitForStart();
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        /*

        Thread.sleep(5000);
        _leftFront.setPower(-0.5);
        _leftBack.setPower(-0.5);
        _rightFront.setPower(-0.5);
        _rightBack.setPower(-0.5);
        Thread.sleep(1650);
        _leftFront.setPower(0.75);
        _leftBack.setPower(-0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(0.75);
        Thread.sleep(2900);
        _leftFront.setPower(0.5);
        _leftBack.setPower(0.5);
        _rightFront.setPower(0.5);
        _rightBack.setPower(0.5);
        Thread.sleep(200);
        _intake.setPower(0.3);
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(50);
        _intake.setPower(-0.3);
        Thread.sleep(360);
        _intake.setPower(0);;
        Thread.sleep(30000);
        */
        /*
//go forward 5 tiles at .25
        _leftFront.setPower(0.25);
        _leftBack.setPower(0.25);
        _rightFront.setPower(0.25);
        _rightBack.setPower(0.25);
        Thread.sleep(7300);
        //wait
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(1000);
        //go backwards 5 tiles at 0.75
        _leftFront.setPower(-0.75);
        _leftBack.setPower(-0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(1700);

         */
        // turn right
        _leftFront.setPower(0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(440);
        //strafe right four tiles
        _leftFront.setPower(-0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(1750);
        //turn right
        _leftFront.setPower(0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(480);

        //strafe right four tiles
        _leftFront.setPower(-0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(1770);
        // turn right
        _leftFront.setPower(0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(500);
        //strafe right
        _leftFront.setPower(-0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(1710);
        // turn right
        _leftFront.setPower(0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(470);
        //strafe right
        _leftFront.setPower(-0.75);
        _leftBack.setPower(0.75);
        _rightFront.setPower(0.75);
        _rightBack.setPower(-0.75);
        Thread.sleep(1610);
        /*
        //strafe right
        _leftFront.setPower(0.25);
        _leftBack.setPower(-0.25);
        _rightFront.setPower(-0.25);
        _rightBack.setPower(0.25);
        Thread.sleep(1000);
        //strafe left
        _leftFront.setPower(-0.25);
        _leftBack.setPower(0.25);
        _rightFront.setPower(0.25);
        _rightBack.setPower(-0.25);
        Thread.sleep(1000);
        //wait
        _leftFront.setPower(0);
        _leftBack.setPower( 0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(1000);
        //turn right
        _leftFront.setPower(0.50);
        _leftBack.setPower(0.50);
        _rightFront.setPower(-0.50);
        _rightBack.setPower(-0.50);
        Thread.sleep(4000);
        //turn left
        _leftFront.setPower(-0.25);
        _leftBack.setPower(-0.25);
        _rightFront.setPower(0.25);
        _rightBack.setPower(0.25);
        Thread.sleep(8000);
        //wait
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(1000);
        //go forward
        _leftFront.setPower(0.25);
        _leftBack.setPower(0.25);
        _rightFront.setPower(0.25);
        _rightBack.setPower(0.25);
        Thread.sleep(1000);

        //wait
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(1000);

        //go backwards
        _leftFront.setPower(-0.25);
        _leftBack.setPower(-0.25);
        _rightFront.setPower(-0.25);
        _rightBack.setPower(-0.25);
        Thread.sleep(1000);

        //wait
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);
        Thread.sleep(1000);

         */
    }
}

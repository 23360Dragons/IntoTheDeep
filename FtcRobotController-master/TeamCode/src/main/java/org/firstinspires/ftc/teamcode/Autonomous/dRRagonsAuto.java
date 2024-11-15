package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "dRRagonsAuto", group = "Autonomous")
public class dRRagonsAuto extends LinearOpMode {
   /* public class Linearz {
        private DcMotorEx rightLinear, leftLinear;
        public Linearz(HardwareMap hardwareMap){
            leftLinear = hardwareMap.get(DcMotorEx.class, "leftLinear");
            leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLinear.setDirection(DcMotorSimple.Direction.FORWARD);

            rightLinear = hardwareMap.get(DcMotorEx.class, "leftLinear");
            rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLinear.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
   /* public class Armz {
        private DcMotorEx leftArm, rightArm;
        public Armz (HardwareMap hardwareMap){
            leftArm = hardwareMap.get(DcMotorEx.class, "leftLinear");
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftArm.setDirection(DcMotorSimple.Direction.FORWARD);

            rightArm = hardwareMap.get(DcMotorEx.class, "rightLinear");
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    public class Artie {
        private Servo leftArtie, rightArtie;
        public Artie(HardwareMap hardwareMap){
            leftArtie = hardwareMap.get(Servo.class,"leftArtie");
            rightArtie = hardwareMap.get(Servo.class, "rightArtie");
        }
    }
    public class Limitz {
        private Servo leftLimit, rightLimit;
        public Limitz (HardwareMap hardwareMap){
            leftLimit = hardwareMap.get(Servo.class, "leftLimit");
            rightLimit = hardwareMap.get(Servo.class, "rightLimit");
        }
    }
    public class Clawz {
        private Servo claw;
        public Clawz (HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
        }
    }

    public class Seesaw {
        private Servo tilt;
        public Seesaw (HardwareMap hardwareMap){
            tilt = hardwareMap.get(Servo.class, "tilt")
        }
    }


    public class TwistNTurn {
        private Servo twist;
        public TwistNTurn (HardwareMap hardwareMap){
            twist = hardwareMap.get(Servo.class, "twist");
        }
    }
    public class ThisLittleLight {
        private RevBlinkinLedDriver light;
        public ThisLittleLight (HardwareMap hardwareMap){
            light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        }
    }
    public class GreenLight {
        private Limelight3A limelight;
        public GreenLight (HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        }
    }*/
    public class FriendlyFire {
        private SparkFunOTOS sensor_otos;
        public FriendlyFire (HardwareMap hardwareMap){
            sensor_otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        int blueFace, redFace;
        blueFace = 270;
        redFace = 90;
        Pose2d blueStartBasket, blueStartObserve, redStartBasket, redStartObserve;
        Vector2d  blueSpecimen, redSpecimen, blueBasket, redBasket, redAscent, blueAscent, blueObserve, redObserve;
        blueStartBasket = new Pose2d(35,62,Math.toRadians(blueFace));
        blueStartObserve = new Pose2d(-12,62,Math.toRadians(blueFace));
        redStartBasket = new Pose2d(12,60,Math.toRadians(redFace));
        redStartObserve = new Pose2d(-35,60,Math.toRadians(redFace));
        blueSpecimen = new Vector2d(0,34);
        redSpecimen = new Vector2d(0,-33);
        blueBasket = new Vector2d(53,53);
        redBasket = new Vector2d(-52,-52);
        redAscent = new Vector2d(-25,0);
        blueAscent = new Vector2d(25, 0);
        blueObserve = new Vector2d(-57,60);
        redObserve = new Vector2d(57,-58);
        Pose2d startPose = null;
        Pose2d notSelected = new Pose2d(0,0,0);
        int starty = 0;
        while (opModeInInit()){

            if(gamepad1.x){
                starty=1;
            } else if (gamepad1.b) {
                starty=2;
            } else if (gamepad2.x) {
                starty=3;
            } else if (gamepad2.b) {
                starty=4;
            }

            switch (starty) {
                case 1:
                    startPose = blueStartBasket;
                    telemetry.addLine("Starting Position Set To Blue, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 2:
                    startPose = redStartBasket;
                    telemetry.addLine("Starting Position Set To Red, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 3:
                    startPose = blueStartObserve;
                    telemetry.addLine("Starting Position Set To Blue, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 4:
                    startPose = redStartObserve;
                    telemetry.addLine("Starting Position Set To Red, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                default:
                    startPose = notSelected;
                    telemetry.addLine("Please select starting position!");
                    break;
            }
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FriendlyFire friendlyFire = new FriendlyFire(hardwareMap);
        TrajectoryActionBuilder waterPool = drive.actionBuilder(startPose)
                .lineToX(blueSpecimen.x)
                .lineToY(blueSpecimen.y)
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(blueObserve,Math.toRadians(90)), Math.PI/2);

        waitForStart();

        /*Action autonomousAnonymous = null;
        if (starty==3){
            autonomousAnonymous = waterPool.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        autonomousAnonymous
                )
        );*/
    }
}

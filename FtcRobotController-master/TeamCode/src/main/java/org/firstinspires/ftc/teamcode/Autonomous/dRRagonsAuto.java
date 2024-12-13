package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Global;

import java.util.List;

//import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "dRRagonsAuto", group = "Autonomous")
public class dRRagonsAuto extends LinearOpMode {
    public enum ARM_POS {
        UP,
        DOWN,
    }
    ARM_POS armPos;
   public class Linearz {
        private DcMotorEx rightLinear, leftLinear;
        public Linearz(HardwareMap hardwareMap){
            leftLinear = hardwareMap.get(DcMotorEx.class, "leftLinear");
            leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLinear.setDirection(DcMotorSimple.Direction.FORWARD);

            rightLinear = hardwareMap.get(DcMotorEx.class, "rightLinear");
            rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLinear.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ElevatorUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftLinear.setTargetPosition(0);

                return false;
            }
        }
        public Action LiftUp() {
            return new ElevatorUp();
        }
   }
   public class Armz {
       private DcMotorEx leftArm, rightArm;
       public Armz (HardwareMap hardwareMap){
           leftArm = hardwareMap.get(DcMotorEx.class, "leftLinear");
           leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
           leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           rightArm = hardwareMap.get(DcMotorEx.class, "rightLinear");
           rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
           rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
       public class ArmToBasket implements Action {

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               leftArm.setTargetPosition(90);
               rightArm.setTargetPosition(90);
               armPos = ARM_POS.UP;
               return false;
           }
       }
       public Action ToBasket () {
           return new ArmToBasket();
       }
       

   }
   public class Artie {
       private Servo leftArtie, rightArtie;
       public Artie(HardwareMap hardwareMap){
           leftArtie = hardwareMap.get(Servo.class,"leftArtie");
           rightArtie = hardwareMap.get(Servo.class, "rightArtie");
       }
   }
   public class Clawz {
       private Servo claw;
       public Clawz (HardwareMap hardwareMap){
           claw = hardwareMap.get(Servo.class, "claw");
       }
       public class CloseClawz implements Action {

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               claw.setPosition(0.5);
               return false;
           }
       }
       public Action CloseClaw(){
           return new CloseClawz();
       }
       public class OpenClawz implements Action {

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               claw.setPosition(1);
               return false;
           }
       }
       public Action OpenClaw(){
           return new OpenClawz();
       }
   }
   public class Seesaw {
       private Servo tilt;
       public Seesaw (HardwareMap hardwareMap){
           tilt = hardwareMap.get(Servo.class, "tilt");
       }
   }
   public class TwistNTurn {
       private Servo twist;
       public TwistNTurn (HardwareMap hardwareMap){
           twist = hardwareMap.get(Servo.class, "twist");
       }
       public class GoingHome implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {
               twist.setPosition(0.49);
               return false;
           }
       }
       public Action Resetting() {
           return new GoingHome();
       }

       public class Postitioning implements Action {

           @Override
           public boolean run(@NonNull TelemetryPacket telemetryPacket) {

               return false;
           }
       }
   }
   public class LarryLime {
       public Limelight3A littleLarryLime;
       public LarryLime(HardwareMap hardwareMap) {
           littleLarryLime = hardwareMap.get(Limelight3A.class, "limelight");
           littleLarryLime.start();
       }
       public class LarryLimeBluest implements Action {

           @Override
           public boolean run(@NonNull TelemetryPacket packet) {
               littleLarryLime.pipelineSwitch(Global.BLUE);
               return false;
           }
       }
       public Action LarryLimeBlues() {
           return new LarryLimeBluest();
       }
       public class LarryLimeRed implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet){
               littleLarryLime.pipelineSwitch(Global.RED);
               return false;
           }
       }
       public Action LarryLimeRedTV() {
           return new LarryLimeRed();
       }
       public class LarryLimeYeller implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet){
               littleLarryLime.pipelineSwitch(Global.YELLOW);
               return false;
           }
       }
       public Action LarryLimeYellow() {
           return new LarryLimeYeller();
       }
       public double rotateClaw (List<List<Double>> cr) {
           List<Double> tl = cr.get(0),
                   tr = cr.get(1);

           double rise  = Math.abs(tl.get(0) - tr.get(0)),
                   run  = Math.abs(tl.get(1) - tr.get(1));

           return Math.toDegrees(Math.tan(Math.toRadians(rise/run))) + 30;
       }
       public double CalcAngle () {
               LLResult larrysJudgement = littleLarryLime.getLatestResult();
               List<LLResultTypes.ColorResult> colorResults = larrysJudgement.getColorResults();
               double myAngle = 0;

               if (!colorResults.isEmpty()) {
                   telemetry.addLine("true");

                   for (LLResultTypes.ColorResult cr : colorResults) {
                       List<List<Double>> la = cr.getTargetCorners(); // should return {{0,0}, {1,0}, {1,1}, {0,1}} or something like that

                       double angle = rotateClaw(la);
                       telemetry.addData("CR target corners", la.get(0).toString());
                       telemetry.addData("CR target corners", la.get(1).toString());
                       telemetry.addData("Target Angle", angle);
                       myAngle = angle;
                   }
               }
               return myAngle;
       }
   }
   public class ThisLittleLight {
       private RevBlinkinLedDriver lilLight;

       public ThisLittleLight(HardwareMap hardwareMap) {
           lilLight = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
       }
       public class Bushels implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet) {
               lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
               return false;
           }
       }
       public Action Bushel (){
           return new Bushels();
       }
       public class YellowGlow implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet){
               lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
               return false;
           }
       }
       public Action LetTheYellowShine (){
           return new YellowGlow();
       }
       public class BlueGlow implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet){
               lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
               return false;
           }
       }
       public Action LetTheBlueShine (){
           return new BlueGlow();
       }
       public class RedGlow implements Action {
           @Override
           public boolean run(@NonNull TelemetryPacket packet){
               lilLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
               return false;
           }
       }
       public Action LetTheRedShine (){
           return new RedGlow();
       }
   }
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
        Vector2d  blueSpecimen, redSpecimen, blueBasket, redBasket, redAscent, blueAscent, blueObserve, redObserve, redYellow1, redYellow2, redYellow3, blueYellow1, blueYellow2, blueYellow3;
        blueStartBasket  = new Pose2d(35,62,Math.toRadians(blueFace));
        blueStartObserve = new Pose2d(-12,62,Math.toRadians(blueFace));
        redStartBasket   = new Pose2d(12,60,Math.toRadians(redFace));
        redStartObserve  = new Pose2d(-35,60,Math.toRadians(redFace));
        blueSpecimen     = new Vector2d(0,34);
        redSpecimen      = new Vector2d(0,-33);
        blueBasket       = new Vector2d(53,53);
        redBasket        = new Vector2d(-52,-52);
        redAscent        = new Vector2d(-25,0);
        blueAscent       = new Vector2d(25, 0);
        blueObserve      = new Vector2d(-57,60);
        redObserve       = new Vector2d(57,-58);
        redYellow1       = new Vector2d(-48,-42);
        redYellow2       = new Vector2d(-58,-42);
        redYellow3       = new Vector2d(-55,-25);
        blueYellow1      = new Vector2d(48,42);
        blueYellow2      = new Vector2d(58,42);
        blueYellow3      = new Vector2d(55,25);
        Pose2d startPose = null;
        Pose2d notSelected= new Pose2d(0,0,0);
        int starty = 0;
        Linearz linearSlidez = new Linearz(hardwareMap);
        Armz ourArmz = new Armz(hardwareMap);
        Artie littleArtie = new Artie(hardwareMap);
        Clawz clawz = new Clawz(hardwareMap);
        Seesaw seesaw = new Seesaw(hardwareMap);
        TwistNTurn twistyturny = new TwistNTurn(hardwareMap);
        LarryLime littleLarryLime = new LarryLime(hardwareMap);
        ThisLittleLight littleLight = new ThisLittleLight(hardwareMap);


        while (opModeInInit()){

            if (gamepad1.x){
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
                    Actions.runBlocking(littleLarryLime.LarryLimeYellow());
                    telemetry.addLine("Starting Position Set To Blue, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 2:
                    startPose = redStartBasket;
                    Actions.runBlocking(littleLarryLime.LarryLimeYellow());
                    telemetry.addLine("Starting Position Set To Red, Basket Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 3:
                    startPose = blueStartObserve;
                    Actions.runBlocking(littleLarryLime.LarryLimeBlues());
                    telemetry.addLine("Starting Position Set To Blue, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                case 4:
                    startPose = redStartObserve;
                    Actions.runBlocking(littleLarryLime.LarryLimeRedTV());
                    telemetry.addLine("Starting Position Set To Red, Observation Zone Side. If inncorrect, please reselect");
                    telemetry.update();
                    break;
                default:
                    startPose = notSelected;
                    telemetry.addLine("Please select starting position! If not selected, the robot will not run during Auto.");
                    break;
            }
        }



        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FriendlyFire friendlyFire = new FriendlyFire(hardwareMap);

        TrajectoryActionBuilder waterPool = drive.actionBuilder(startPose)
                .strafeTo(blueSpecimen)
                .waitSeconds(3)
                .strafeToSplineHeading(blueObserve, 270)
                .waitSeconds(3)
                .turnTo(Math.toRadians(blueFace));

        TrajectoryActionBuilder firePit = drive.actionBuilder(startPose)
                .strafeTo(redSpecimen)
                .waitSeconds(3)
                .strafeToSplineHeading(redObserve,Math.toRadians(90))
                .waitSeconds(3)
                .turnTo(Math.toRadians(redFace));

        TrajectoryActionBuilder tomato = drive.actionBuilder(startPose)
                .waitSeconds(3)
                .strafeTo(redYellow1)
                .waitSeconds(3)
                .strafeToSplineHeading(redBasket, Math.toRadians(225))
                .waitSeconds(3)
                .strafeToSplineHeading(redYellow2, Math.toRadians(90))
                .waitSeconds(3)
                .strafeToSplineHeading(redBasket, Math.toRadians(225))
                .waitSeconds(3)
                .strafeToSplineHeading(redYellow3, Math.toRadians(125))
                .waitSeconds(3)
                .strafeToSplineHeading(redBasket, Math.toRadians(225))
                .waitSeconds(3)
                .turnTo(Math.toRadians(redFace));

        TrajectoryActionBuilder blueberry = drive.actionBuilder(startPose)
                .waitSeconds(3)
                .strafeTo(blueYellow1)
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket, Math.toRadians(45))
                .waitSeconds(3)
                .strafeToSplineHeading(blueYellow2, Math.toRadians(270))
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket, Math.toRadians(45))
                .waitSeconds(3)
                .strafeToSplineHeading(blueYellow3, Math.toRadians(0))
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket, Math.toRadians(45))
                .waitSeconds(3)
                .turnTo(Math.toRadians(blueFace));

        TrajectoryActionBuilder stopping = drive.actionBuilder(startPose)
                .waitSeconds(30);

        waitForStart();

        Action autonomousAnonymous = null;
        switch (starty) {
            case 1:
                autonomousAnonymous = blueberry.build();
                break;
            case 2:
                autonomousAnonymous = tomato.build();
                break;
            case 3:
                autonomousAnonymous = waterPool.build();
                break;
            case 4:
                autonomousAnonymous = firePit.build();
                break;
            default:
                autonomousAnonymous = stopping.build();
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        autonomousAnonymous

                )
        );
    }
}
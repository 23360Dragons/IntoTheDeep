package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
//import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "dRRagonsAutoTest", group = "Autonomous")
public class dRRagonsAuto extends LinearOpMode {
    public class Linearz {
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

            rightArm = hardwareMap.get(DcMotorEx.class, "leftLinear");
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }*/
    public class Artie {
        private Servo leftArtie, rightArtie;
        public Artie(HardwareMap hardwareMap){
            leftArtie = hardwareMap.get(Servo.class,"leftArtie");
            rightArtie = hardwareMap.get(Servo.class, "rightArtie");
        }
    }
    /*public class Limitz {
        private Servo leftLimit, rightLimit;
        public Limitz (HardwareMap hardwareMap){
            leftLimit = hardwareMap.get(Servo.class, "leftLimit");
            rightLimit = hardwareMap.get(Servo.class, "rightLimit");
        }
    }
    public class Rotary {
        private Servo rotaryPickup;
        public Rotary (HardwareMap hardwareMap){
            rotaryPickup = hardwareMap.get(Servo.class, "rotaryPickup");
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
        Pose2d blueStartBasket, blueStartObserve, redStartBasket, redStartObserve, blueSpecimen, redSpecimen, blueBasket, redBasket, redAscent, blueAscent, blueObserve, redObserve;
        blueStartBasket = new Pose2d(35,62,Math.toRadians(blueFace));
        blueStartObserve = new Pose2d(-12,62,Math.toRadians(blueFace));
        redStartBasket = new Pose2d(12,60,Math.toRadians(redFace));
        redStartObserve = new Pose2d(-35,60,Math.toRadians(redFace));
        blueSpecimen = new Pose2d(0,34, Math.toRadians(blueFace));
        redSpecimen = new Pose2d(0,-33,Math.toRadians(redFace));
        blueBasket = new Pose2d(53,53,Math.toRadians(45));
        redBasket = new Pose2d(-52,-52,Math.toRadians(225));
        redAscent = new Pose2d(-25,0,Math.toRadians(0));
        blueAscent = new Pose2d(25, 0, Math.toRadians(180));
        blueObserve = new Pose2d(-57,60,Math.toRadians(90));
        redObserve = new Pose2d(57,-58,Math.toRadians(270));
    }
}

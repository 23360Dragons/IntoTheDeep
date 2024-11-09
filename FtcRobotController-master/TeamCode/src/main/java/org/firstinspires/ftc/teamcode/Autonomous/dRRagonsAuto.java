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
//        Pose2d initialPose = new Pose2d();
    }
}

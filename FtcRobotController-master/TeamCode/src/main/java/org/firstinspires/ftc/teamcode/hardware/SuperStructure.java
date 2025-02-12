package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

import java.util.List;

public class SuperStructure {
    public boolean isValid;
    public Arm arm;
    public Extension extension;

    public enum ARTICULATION_POS {
        UP,
        DOWN,
        HANG
    }

    private ARTICULATION_POS articulationPos;

    public SuperStructure (LinearOpMode opmode) {
        opmode.telemetry.addLine("Configuring Superstructure Arm!");
        opmode.telemetry.update();

        this.arm = new Arm(opmode);

        opmode.telemetry.addData("Superstructure Arm configured", arm.isValid);
        opmode.telemetry.addLine("Configuring Superstructure Extension!");
        opmode.telemetry.update();

        this.extension    = new Extension(opmode);

        opmode.telemetry.addData("Superstructure Extension configured", extension.isValid);
        opmode.telemetry.update();

        this.isValid = arm.isValid && extension.isValid;
    }

    public class Arm {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid = true;

        public  int hangTicks = -50;
        public  int downTicks = -300;
        public  int fullTicks = -0;

        Arm (LinearOpMode opmode) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftArtie");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                Global.exceptions.append("leftArm\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                rightMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightArtie");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                Global.exceptions.append("rightArm\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            articulationPos = ARTICULATION_POS.UP;
        }

        public void setState (ARTICULATION_POS pos) {
            articulationPos = pos;
        }

        public void hang () {
            setTarget(hangTicks);
        }
        public void full () {
            setTarget(fullTicks);
        }
        public void down () {
            setTarget(downTicks);
        }

        public ARTICULATION_POS getState () {
            return articulationPos;
        }

        public void setTarget (int target) {
            leftMotor.setTargetPosition(target);
            rightMotor.setTargetPosition(target);
        }

        public void setPower (double power) {
            leftMotor.setPower (power);
            rightMotor.setPower(power);
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

        public void switchToManual () {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void switchToAuto () {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public boolean atTargetPosition () {
            return !rightMotor.isBusy();
        }
    }

    public static class Extension {
        public boolean isValid = true;
//
        public  static int maxDownExtension = 1350;
        private static final int tolerance  = 100;
        public  static int hangTicks        = maxDownExtension - tolerance;
        public  static int downTicks        = -5;
        public  static int fullTicks        = 3000;

        private DcMotorEx leftMotor, rightMotor;
        Extension (LinearOpMode opmode) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftLinear");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e) {
                Global.exceptions.append("leftLinear\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            try {
                rightMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightLinear");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                Global.exceptions.append("rightLinear\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }
        }

        public void setPower(double power) {
            leftMotor.setPower (power);
            rightMotor.setPower(power);
        }

        public void setTarget (int target) {
            leftMotor.setTargetPosition(target);
            rightMotor.setTargetPosition(target);
        }

        public void resetEncoders() {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void switchToManual () {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void switchToAuto () {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void hang () {
            setTarget(hangTicks);
        }
        public void full () {
            setTarget(fullTicks);
        }
        public void down () {
            setTarget(downTicks);
        }

        public int getTarget () {
            return rightMotor.getTargetPosition();
        }

        public boolean atTargetPosition () {
            return !rightMotor.isBusy();
        }

        public double getTPD () {
            return this.rightMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

//        public Positions getCurrent () {
//            return new Positions(leftMotor.getCurrent(CurrentUnit.AMPS), rightMotor.getCurrent(CurrentUnit.AMPS));
//        }
    }
}
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

    public SuperStructure (LinearOpMode opmode, boolean resetEncoders) {
        opmode.telemetry.addLine("Configuring Superstructure Arm!");
        opmode.telemetry.update();

        this.arm = new Arm(opmode, resetEncoders);

        opmode.telemetry.addData("Superstructure Arm configured", arm.isValid);
        opmode.telemetry.addLine("Configuring Superstructure Extension!");
        opmode.telemetry.update();

        this.extension = new Extension(opmode, resetEncoders);

        opmode.telemetry.addData("Superstructure Extension configured", extension.isValid);
        opmode.telemetry.update();

        this.isValid = arm.isValid && extension.isValid;
    }

    public class Arm {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid = true;

        public static final int SShangTicks = -50;
        public static final int SSdownTicks = -300;
        public static final int SSfullTicks = -0;
        private static final int tolerance = 10;

        Arm (LinearOpMode opmode, boolean resetEncoders) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftArtie");

                if (resetEncoders)
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

                if (resetEncoders)
                    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                Global.exceptions.append("rightArm\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            leftMotor.setTargetPositionTolerance(tolerance);
            rightMotor.setTargetPositionTolerance(tolerance);
            articulationPos = ARTICULATION_POS.UP;
            setTarget(0);
        }

        public void setState (ARTICULATION_POS pos) {
            articulationPos = pos;
        }

        public void hang () {
            setTarget(SShangTicks);
        }
        public void full () {
            setTarget(SSfullTicks);
        }
        public void down () {
            setTarget(SSdownTicks);
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
        private static final int tolerance  = 50;
        public static final int maxDownExtension = 1350;
        public static final int hangTicks        = maxDownExtension - tolerance;
        public static final int downTicks        = -5;
        public static final int fullTicks        = 3000;
        public static final int chamberTicks = 350;

        private DcMotorEx leftMotor, rightMotor;
        Extension (LinearOpMode opmode, boolean resetEncoders) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftLinear");

                if (resetEncoders)
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

                if (resetEncoders)
                    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                Global.exceptions.append("rightLinear\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            leftMotor.setTargetPositionTolerance(tolerance);
            rightMotor.setTargetPositionTolerance(tolerance);
            setTarget(0);
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

        public void chamber () {
            setTarget(chamberTicks);
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
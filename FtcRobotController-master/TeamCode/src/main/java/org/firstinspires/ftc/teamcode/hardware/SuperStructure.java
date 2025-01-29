package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

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
    private ARTICULATION_POS lastArticulationPos;

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
        private double power;

        Arm(LinearOpMode opmode) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftArtie");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // STOP AND Reset CANNOT BE AFTER RUN WITHOUT ENCODER - NOTHING WORKS IF YOU DO
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
            setLastState(articulationPos);
            articulationPos = pos;
        }

        private void setLastState (ARTICULATION_POS pos) {
            lastArticulationPos = pos;
        }

        public ARTICULATION_POS getState () {
            return articulationPos;
        }

        public ARTICULATION_POS getLastState () {
            return lastArticulationPos;
        }

        public void setPower (double power) {
            leftMotor.setPower (power);
            rightMotor.setPower(power);
            this.power = power;
        }

        public double getPower() {
            return this.power;
        }

        public double getTPD () {
            return this.leftMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

        public Positions getVelocity () {
            return new Positions(leftMotor.getVelocity(AngleUnit.DEGREES), rightMotor.getVelocity(AngleUnit.DEGREES));
        }
    }

    public static class Extension {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public double maxDownExtension = 1350;

        public boolean isValid = true;
        private double power;

        Extension (LinearOpMode opmode) {
            try {
                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftLinear");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

            this.power = power;
        }
        
        public void setLeftPower (double power) {
            leftMotor.setPower(power);
        }

        public void setRightPower (double power) {
            rightMotor.setPower(power);
        }

        public double getPower() {
            return this.power;
        }

        public double getTPD () {
            return this.rightMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

        public Positions getCurrent () {
            return new Positions(leftMotor.getCurrent(CurrentUnit.AMPS), rightMotor.getCurrent(CurrentUnit.AMPS));
        }
    }
}
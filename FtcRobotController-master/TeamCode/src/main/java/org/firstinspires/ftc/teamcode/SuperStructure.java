package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

public class SuperStructure {
    public boolean isValid;
    public Articulation articulation;
    public Extension extension;

    public enum ARTICULATION_POS {
        UP,
        DOWN,
    }

    ARTICULATION_POS articulationPos;

    public SuperStructure (HardwareMap hardwareMap, Telemetry telemetry) {
        telemetry.addLine("Configuring Superstructure Articulation!");
        telemetry.update();

        this.articulation = new Articulation(hardwareMap);

        telemetry.addData("Superstructure Articulation configured", articulation.isValid);
        telemetry.addLine("Configuring Superstructure Extension!");
        telemetry.update();

        this.extension    = new Extension(hardwareMap);

        telemetry.addData("Superstructure Extension configured", extension.isValid);
        telemetry.update();

        this.isValid = articulation.isValid && extension.isValid;
    }

    public class Articulation {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid = true;
        private double power;

        Articulation (HardwareMap hardwareMap) {
            try {
                leftMotor = hardwareMap.get(DcMotorEx.class, "leftArm");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // STOP AND Reset CANNOT BE AFTER RUN WITHOUT ENCODER - NOTHING WORKS IF YOU DO
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("leftArm").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                rightMotor = hardwareMap.get(DcMotorEx.class, "rightArm");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightArm").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            articulationPos = ARTICULATION_POS.UP;
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

        public void moveUp() {
            setPosition(90);
            articulationPos = ARTICULATION_POS.UP;
        }

        public void moveDown() {
            setPosition(0);
            articulationPos = ARTICULATION_POS.DOWN;
        }

        private void setPosition (double tickPos) {
            //todo: put the values from articulation testing into here, make this a pidloop
        }
    }

    public static class Extension {
        private DcMotor leftMotor;
        private DcMotor rightMotor;

        public boolean isValid = true;
        private double power;

        Extension (HardwareMap hardwareMap) {
            try {
                leftMotor = hardwareMap.get(DcMotor.class, "leftLinear");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("leftLinear").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            try {
                rightMotor = hardwareMap.get(DcMotor.class, "rightLinear");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightLinear").append(" does not exist").append("\n");
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
    }
}
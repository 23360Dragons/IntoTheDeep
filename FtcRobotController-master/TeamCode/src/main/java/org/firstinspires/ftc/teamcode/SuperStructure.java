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
        DOWN;
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
                leftMotor = hardwareMap.get(DcMotorEx.class, "leftArticulationMotor");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("leftArticulationMotor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                rightMotor = hardwareMap.get(DcMotorEx.class, "rightArticulationMotor");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightArticulationMotor").append(" does not exist").append("\n");
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

    public class Extension {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid;
        private double power;

        Extension (HardwareMap hardwareMap) {
            try {
                leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtensionMotor");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("left extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }

            try {
                rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtensionMotor");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("right extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                this.isValid = false;
            }
        }

        public void setPower(double power) {
            leftMotor.setPower (power);
            rightMotor.setPower(power);

            this.power = power;
        }

        public double getPower() {
            return this.power;
        }

        public double getTPD () {
            return this.rightMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition() / getTPD(), rightMotor.getCurrentPosition() / getTPD());
        }
    }
}
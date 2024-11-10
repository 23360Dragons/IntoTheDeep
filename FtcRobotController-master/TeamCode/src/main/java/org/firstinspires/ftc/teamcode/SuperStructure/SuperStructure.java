package org.firstinspires.ftc.teamcode.SuperStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.Positions;

public class SuperStructure {
    public boolean isValid;
    public Articulation articulation;
    public Extension extension;

    public SuperStructure (HardwareMap hardwareMap) {
        this.articulation = new Articulation(hardwareMap);
        this.extension = new Extension(hardwareMap);

        this.isValid = articulation.isValid && extension.isValid;
    }

    public static class Articulation {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid = true;
        private double power;

        public Articulation (HardwareMap hardwareMap) {
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
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightArticulationMotor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
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

        public void setPosition (double tickPos) {
            //todo: put the values from articulation testing into here, make this a pidloop
        }
    }

    public static class Extension {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid;
        private double power;

        public Extension (HardwareMap hardwareMap) {
            try {
                leftMotor = hardwareMap.get(DcMotorEx.class, "leftSSmotor");
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("left extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                rightMotor = hardwareMap.get(DcMotorEx.class, "rightSSmotor");
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("right extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
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
            return this.leftMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition() / getTPD(), rightMotor.getCurrentPosition() / getTPD());
        }
    }
}
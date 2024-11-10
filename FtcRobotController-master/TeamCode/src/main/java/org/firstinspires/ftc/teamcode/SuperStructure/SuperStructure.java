package org.firstinspires.ftc.teamcode.SuperStructure;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Global;

public class SuperStructure {
    PIDController pidController;
    public Articulation articulation;
    public Extension extension;

    public SuperStructure (HardwareMap hardwareMap) {
        this.articulation = new Articulation(hardwareMap);
        this.extension = new Extension(hardwareMap);
    }

    public static class Articulation {
        private DcMotor rightMotor;
        private DcMotor leftMotor;

        public boolean isValid = true;
        private double power;
        private double targetPos;

        public Articulation (HardwareMap hardwareMap) {
            try {
                rightMotor = hardwareMap.get(DcMotor.class, "rightArticulationMotor");
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("rightArticulationMotor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                leftMotor = hardwareMap.get(DcMotor.class, "leftArticulationMotor");
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("leftArticulationMotor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public void setPower (double power) {
            rightMotor.setPower(power);
            leftMotor.setPower (power);
            this.power = power;
        }

        public double getPower() {
            return this.power;
        }
    }

    public static class Extension {
        private DcMotor rightMotor;
        private DcMotor leftMotor;

        public boolean isValid;
        private double power;
        private double targetPos;

        public Extension (HardwareMap hardwareMap) {
            try {
                rightMotor = hardwareMap.get(DcMotor.class, "rightSSmotor");
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightSSmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("right extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }

            try {
                leftMotor = hardwareMap.get(DcMotor.class, "leftSSmotor");
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftSSmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            } catch (IllegalArgumentException e) {
                Global.exceptions.append("Configuration Error: ").append("left extension motor").append(" does not exist").append("\n");
                Global.exceptionOccurred = true;
                isValid = false;
            }
        }

        public void setPower(double power) {
            rightMotor.setPower(power);
            leftMotor.setPower (power);

            this.power = power;
        }

        public double getPower() {
            return this.power;
        }
    }
}

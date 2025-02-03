package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        private MotorEx leftMotor;
        private MotorEx rightMotor;
        private MotorGroup motors;

        public boolean isValid = true;

        private int tolerance = 25;
        public  int hangTicks = -50;
        public  int downTicks = -330;
        public  int fullTicks = -0;

        public int currentTarget = fullTicks;

        public Arm (LinearOpMode opmode) {
            rightMotor = new MotorEx(opmode.hardwareMap, "leftArtie");
            leftMotor  = new MotorEx(opmode.hardwareMap, "rightArtie");
            motors     = new MotorGroup(
                    rightMotor,
                    leftMotor
            );

            motors.resetEncoder();
            motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motors.setPositionTolerance(tolerance);

            switchToAuto();
        }

        public void updatePosition (double speed) {
            if (!motors.atTargetPosition()) {
                motors.set(speed);
            } else {
                motors.stopMotor();
            }
        }

        public void setTarget(int target) {
            currentTarget = target;
            motors.setTargetPosition(target);
        }

        public void setFeedbackCoeffs (double kP, double kI, double kD) {
            motors.setVeloCoefficients(kP, kI, kD);
        }

        public void setFeedforwardCoeffs (double kV, double kS) {
            motors.setFeedforwardCoefficients(kV, kS);
        }

        public void switchToManual () {
            motors.setRunMode(Motor.RunMode.RawPower);
        }

        public void switchToAuto () {
            motors.setRunMode(Motor.RunMode.VelocityControl);
        }

        public void setState (ARTICULATION_POS pos) {
            articulationPos = pos;
        }

        public ARTICULATION_POS getState () {
            return articulationPos;
        }

        public void setPower (double power) {
            leftMotor.set (power);
            rightMotor.set(power);
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

        /*private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;

        public boolean isValid = true;

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
        }

        public double getPower() {
            return leftMotor.getPower();
        }

        public double getTPD () {
            return this.leftMotor.getMotorType().getTicksPerRev() / 360;
        }

        public Positions getPosition() {
            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        }

        public Positions getVelocity () {
            return new Positions(leftMotor.getVelocity(AngleUnit.DEGREES), rightMotor.getVelocity(AngleUnit.DEGREES));
        }*/
    }

    public static class Extension {
        private MotorEx leftMotor;
        private MotorEx rightMotor;
        private MotorGroup motors;

        public boolean isValid = true;

        public  int maxDownExtension = 1350;
        private int tolerance        = 15;
        public  int hangTicks        = maxDownExtension - tolerance;
        public  int downTicks        = 0;
        public  int fullTicks        = 2110;

        public int currentTarget = downTicks;

        public Extension(LinearOpMode opmode) {
            rightMotor = new MotorEx(opmode.hardwareMap, "rightLinear");
            leftMotor  = new MotorEx(opmode.hardwareMap, "leftLinear");
            motors     = new MotorGroup(
                    rightMotor,
                    leftMotor
            );

            leftMotor.setInverted(true);

            motors.resetEncoder();
            motors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motors.setPositionTolerance(tolerance);

            switchToAuto();
        }

        public void switchToManual () {
            motors.setRunMode(Motor.RunMode.RawPower);
        }

        public void switchToAuto () {
            motors.setRunMode(Motor.RunMode.PositionControl);
        }

        public void toggleControlMode () {
            if (Global.controlState == Global.ControlState.MANUAL)
                switchToAuto(); // todo might need to re add coeffs and tolerance?
            else
                switchToManual();
        }

        // --------------- Manual Methods ---------------

        public void setPower (double power) {
            motors.set(power);
        }

        // --------------- Auto Methods ---------------

        public void setPositionCoefficient (double kP) {
            motors.setPositionCoefficient(kP);
        }

        public void setTarget (int target) {
            motors.setTargetPosition(target);
            currentTarget = target;
        }

        public void updatePosition (double speed) {
            if (!motors.atTargetPosition()) {
                motors.set(speed);
            } else {
                motors.stopMotor();
            }
        }

        public void resetEncoders () {
            motors.resetEncoder();
        }

        public Positions getPosition () {
            List<Double> position = motors.getPositions();

            // left then right, so since right is added first, it needs this order
            return new Positions(position.get(1), position.get(0));
        }

//        this is for dcMotorsEx
//        Extension (LinearOpMode opmode) {
//            /*try {
//                leftMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftLinear");
//                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            } catch (Exception e) {
//                Global.exceptions.append("leftLinear\n");
//                Global.exceptionOccurred = true;
//                this.isValid = false;
//            }
//
//            try {
//                rightMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightLinear");
//                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            } catch (Exception e) {
//                Global.exceptions.append("rightLinear\n");
//                Global.exceptionOccurred = true;
//                this.isValid = false;
//            }*/
//        }
//
//        public void setPower(double power) {
//            leftMotor.setPower (power);
//            rightMotor.setPower(power);
//
//            this.power = power;
//        }
//
//        public void resetEncoders() {
//            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        public void setLeftPower (double power) {
//            leftMotor.setPower(power);
//        }
//
//        public void setRightPower (double power) {
//            rightMotor.setPower(power);
//        }
//
//        public double getPower() {
//            return this.power;
//        }
//
//        public double getTPD () {
//            return this.rightMotor.getMotorType().getTicksPerRev() / 360;
//        }
//
//        public Positions getPosition() {
//            return new Positions(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
//        }
//
//        public Positions getCurrent () {
//            return new Positions(leftMotor.getCurrent(CurrentUnit.AMPS), rightMotor.getCurrent(CurrentUnit.AMPS));
//        }
    }
}
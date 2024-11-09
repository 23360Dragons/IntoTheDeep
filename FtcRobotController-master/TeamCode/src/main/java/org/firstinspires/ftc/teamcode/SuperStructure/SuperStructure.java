package org.firstinspires.ftc.teamcode.SuperStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Global;

public class SuperStructure {
    private DcMotor rightSSmotor;
    private DcMotor leftSSmotor;

    private DcMotor rightArticulationMotor;
    private DcMotor leftArticulationMotor;

    private double position;
    public boolean isValid;
    private double power;

    public SuperStructure (HardwareMap hardwareMap) {
        try {
            rightSSmotor = hardwareMap.get(DcMotor.class, "rightSSmotor");
            rightSSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightSSmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSSmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("rightSSmotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }

        try {
            leftSSmotor = hardwareMap.get(DcMotor.class, "leftSSmotor");
            leftSSmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftSSmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSSmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSSmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("leftSSmotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }

        try {
            rightArticulationMotor = hardwareMap.get(DcMotor.class, "rightArticulationMotor");
            rightArticulationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArticulationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("rightArticulationMotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }

        try {
            leftArticulationMotor = hardwareMap.get(DcMotor.class, "leftArticulationMotor");
            leftArticulationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArticulationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("leftArticulationMotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }
//
//    public void setPositionDown () {
//        rightSSmotor.setTargetPosition(0);
//        leftSSmotor.setTargetPosition (0);
//        position = 0;
//    }
//
//    public void setPositionUp () {
//        rightSSmotor.setTargetPosition(90);
//        leftSSmotor.setTargetPosition(90);
//        position = 90;
//    }
//
//    public double getPosition () {
//        return position;
//    }

    public void setSSPower(double power) {
        rightSSmotor.setPower(power);
        leftSSmotor.setPower (power);

        this.power = power;
    }

    public double getSSPower() {
        return this.power;
    }

    public void setArticulationPower (double power) {
        rightArticulationMotor.setPower(power);
        leftArticulationMotor.setPower(power);
    }
}

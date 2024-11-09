package org.firstinspires.ftc.teamcode.SuperStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Global;

public class SuperStructure {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private int position;
    public boolean isValid;

    public SuperStructure (HardwareMap hardwareMap) {
        try {
            rightMotor = hardwareMap.get(DcMotor.class, "rightSSmotor");
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("rightSSmotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }

        try {
            leftMotor = hardwareMap.get(DcMotor.class, "leftSSmotor");
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            Global.exceptions.append("Configuration Error: ").append("leftSSmotor").append(" does not exist").append("\n");
            Global.exceptionOccurred = true;
            isValid = false;
        }
    }

    public void setPositionDown () {
        rightMotor.setTargetPosition(0);
        leftMotor.setTargetPosition (0);
        position = 0;
    }

    public void setPositionUp () {
        rightMotor.setTargetPosition(90);
        leftMotor.setTargetPosition(90);
        position = 90;
    }

    public int getPosition () {
        return position;
    }

    public void setPower (int power) {
        rightMotor.setPower(power);
        leftMotor.setPower (power);
    }
}

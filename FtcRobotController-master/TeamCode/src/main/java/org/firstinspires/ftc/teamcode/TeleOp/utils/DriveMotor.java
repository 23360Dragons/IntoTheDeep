package org.firstinspires.ftc.teamcode.TeleOp.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveMotor {
    public static DcMotorEx createNewMotor (HardwareMap hardwareMap, String name) {

        DcMotorEx dcMotor;

        dcMotor = hardwareMap.get(DcMotorEx.class, name);
        dcMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return dcMotor;
    }
}

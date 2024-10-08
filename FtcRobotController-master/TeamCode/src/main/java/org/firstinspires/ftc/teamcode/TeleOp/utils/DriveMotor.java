package org.firstinspires.ftc.teamcode.TeleOp.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveMotor {
    public static DcMotorEx createNewMotor (HardwareMap hardwareMap, String name) {
        DcMotorEx dcMotor;

        dcMotor = hardwareMap.get(DcMotorEx.class, name); // gets a dcMotorEx object of the name "name"
        dcMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        return dcMotor; // returns a DcMotorEx object
    }
}

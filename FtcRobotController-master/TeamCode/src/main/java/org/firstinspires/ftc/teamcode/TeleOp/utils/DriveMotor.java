package org.firstinspires.ftc.teamcode.TeleOp.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveMotor {
    public static DcMotor newMotor(HardwareMap hardwareMap, String name) {
        DcMotor dcMotor;

        dcMotor = hardwareMap.get(DcMotor.class, name); // gets a dcMotorEx object of the name "name"
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return dcMotor; // returns a DcMotorEx object
    }
}

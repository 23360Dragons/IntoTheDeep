package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveMotor {
    public static DcMotor newMotor(HardwareMap hardwareMap, String name) throws ConfigurationException {

        DcMotor dcMotor;
        try
        {
            dcMotor = hardwareMap.get(DcMotor.class, name); // gets a dcMotor object of the name "name"
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return dcMotor; // returns a DcMotor object
        } catch (IllegalArgumentException ex)
        {
            throw new ConfigurationException(name + " does not exist"); //"throws" said exception back to the script that called newMotor()
        }

    }
}


package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveMotor {
    private static final DcMotor[] driveMotors = new DcMotor[4];
    private static final String[] driveMotorNames = {"leftFront", "leftBack", "rightFront", "rightBack"};

    public static String[] getDriveMotorNames () {
        return driveMotorNames;
    }

    public static DcMotor[] initialize (HardwareMap hardwareMap) {
        for (int i = 0; i < driveMotors.length - 1; i++) {
            try {
                driveMotors[i] = hardwareMap.get(DcMotor.class, driveMotorNames[i]); // gets a dcMotor object of the name "name"
                driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException ex) {
                InitInfo.exceptions.append("CRITICAL Configuration Error: ").append(driveMotorNames[i]).append(" does not exist").append("\n");
                InitInfo.exceptionOccurred = true;
                InitInfo.movementExceptionOccurred = true;
            }
        }

        if (InitInfo.exceptionOccurred) {
            return null;
        } else {
            return driveMotors;
        }
    }
}


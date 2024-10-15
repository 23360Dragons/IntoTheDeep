package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveMotor {
    private static final DcMotor[] driveMotors = new DcMotor[4];
    private static final String[] driveMotorNames = {"leftFront", "leftBack", "rightFront", "rightBack"};
    public static boolean isValid;

    public static String[] getDriveMotorNames () {
        return driveMotorNames;
    }

    public static DcMotor[] initialize (HardwareMap hardwareMap) {
        for (int i = 0; i < driveMotors.length - 1; i++) {
            try {
                driveMotors[i] = hardwareMap.get(DcMotor.class, driveMotorNames[i]); // gets a dcMotor object of the name "name"
                driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                if (i % 2 != 0) { // if the index is odd (1 or 3), set direction to reverse. (this flips the right motors)
                    driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
                }

                isValid = true;

            } catch (IllegalArgumentException ex) {
                InitInfo.exceptions.append("CRITICAL Configuration Error: ").append(driveMotorNames[i]).append(" does not exist").append("\n");
                InitInfo.exceptionOccurred = true;
                isValid = false;
            }
        }

        if (InitInfo.exceptionOccurred) {
            return null;
        } else {
            return driveMotors;
        }
    }
}


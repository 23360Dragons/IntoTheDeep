package org.firstinspires.ftc.teamcode.utils.init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightFront;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.leftBack;
import static org.firstinspires.ftc.teamcode.utils.init.InitInfo.rightBack;



public class DriveMotor {
    public static boolean isValid;

    public static void initialize (HardwareMap hardwareMap) {
        try {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // gets a dcMotor object of the name "name"
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setDirection(DcMotor.Direction.REVERSE); // fix faulty drive behavior 10/15

        } catch (IllegalArgumentException e) {
            InitInfo.exceptions.append("Configuration Error: ").append("leftFront").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
        }

        try {
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            InitInfo.exceptions.append("Configuration Error: ").append("rightFront").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
        }

        try {
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (IllegalArgumentException e) {
            InitInfo.exceptions.append("Configuration Error: ").append("leftBack").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
        }

        try {
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException e) {
            InitInfo.exceptions.append("Configuration Error: ").append("rightBack").append(" does not exist").append("\n");
            InitInfo.exceptionOccurred = true;
        }
    }
}


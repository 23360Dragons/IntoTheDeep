package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.DragonsOTOS;
import org.firstinspires.ftc.teamcode.utils.Global;

@Disabled
@Autonomous(name="TestStrafe12InchesRightOTOS", group="Tests")
public class TestStrafeOTOS extends LinearOpMode {
    // Declare motors
    //motor code from CombinedMaster
    // are those ports correct?
    DcMotor _leftFront;//Port 0
    DcMotor _leftBack;//Port 1
    DcMotor _rightFront;//Port2
    DcMotor _rightBack;//Port3
    
    // Declare OTOS sensor
    private DragonsOTOS otos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors - copied from CombinedMaster
        _leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        _leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        //assumption - right side reversed - copied from CombinedMaster
        //that was the comment in CombinedMaster. Is this right?
        _leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Initialize OTOS sensor 
        otos = new DragonsOTOS(this);

        // Wait for start
        waitForStart();

         // Create an instance of AutoRobotMovement
        RobotMovementOTOS robotMovementOTOS = new RobotMovementOTOS(_leftFront, _rightFront, _leftBack, _rightBack, otos);

        // Strafe right 12 inches
        robotMovementOTOS.strafe(12.0, Global.Right);

        // Stop motors (optional, as subroutine already stops them)
        _leftFront.setPower(0);
        _rightFront.setPower(0);
        _leftBack.setPower(0);
        _rightBack.setPower(0);

        // Set motors to run without encoder
        _leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

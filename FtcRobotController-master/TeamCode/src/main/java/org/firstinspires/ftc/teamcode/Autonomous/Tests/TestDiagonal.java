package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

@Disabled
@Autonomous(name="TestDiagonal12Inches45", group="Tests")
public class TestDiagonal extends AutonomousOpMode {
    // Declare motors
    //motor code from CombinedMaster
    // are those ports correct?
    DcMotor _leftFront;//Port 0
    DcMotor _leftBack;//Port 1
    DcMotor _rightFront;//Port2
    DcMotor _rightBack;//Port3
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        // Wait for start
        waitForStart();

        // Move diagonally 12 inches at 45 degrees
//        autoRobotMovement.moveDiagonallyRight(12.0, 45.0);

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

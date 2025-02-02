package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;

//Brute Force Auto Phase 1
//Parking Only
//Starting from the Blue Basket (Tile A5 on the diagram)
//Move to the Observation Zone (Tile A1 on the diagram)
//Suggest path is A5 to B4 to B2 to A1
@Autonomous
public class BlueBasketBFP1 extends LinearOpMode{
    //motor code from CombinedMaster
    // are those ports correct?
    DcMotor _leftFront;//Port 0
    DcMotor _leftBack;//Port 1
    DcMotor _rightFront;//Port2
    DcMotor _rightBack;//Port3

    @Override
    public void runOpMode() throws InterruptedException {
// Initialize motors - copied from CombinedMaster
        _leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        //assumption - right side reversed - copied from CombinedMaster
        _leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for start
        waitForStart();

        // Create an instance of AutoRobotMovement
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(_leftFront, _rightFront, _leftBack, _rightBack);

        //Step 1
        ////Suggest path is A5 to B4
        //Since each tile is a square with 24-inch sides, the distance between the centers of adjacent tiles horizontally or vertically is also 24 inches.
        //Calculate the diagonal distance across one tile:
        //Use the Pythagorean theorem:
        //Diagonal  = side  + side
        //Diagonal  = 24  + 24
        //Diagonal  = 576 + 576
        //Diagonal  = 1152
        //Diagonal = v1152
        //Diagonal   33.94 inches


        // Call the RobotMovemenet class to move diagonally 33.94 in at a heading of 45 degress
        autoRobotMovement.moveDiagonallyRight(12.0, 45.0);

        //Step 2
        //Suggest path is B4 to B2
        autoRobotMovement.strafe(12.0, true);

        //Step 3
        //Suggest path B2 to A1
        autoRobotMovement.moveDiagonallyRight(33.94, 135); //Moves diagonally 33.94 in at a heading of 135 degress.

        // Set motors to run without encoder
        _leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }




}

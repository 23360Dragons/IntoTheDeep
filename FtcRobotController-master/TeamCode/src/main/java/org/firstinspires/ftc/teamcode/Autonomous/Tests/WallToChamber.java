package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.Global;

import static org.firstinspires.ftc.teamcode.utils.Global.Right;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Backward;


@Disabled
@Autonomous(name="WallToChamber", group="Tests")
public class WallToChamber extends LinearOpMode {
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors - copied from CombinedMaster
        drivetrain = new Drivetrain(this);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain.leftFront, drivetrain.rightFront, drivetrain.leftBack, drivetrain.rightBack);

        waitForStart();

        // drive to submersible
        autoRobotMovement.moveForward(24, Forward);
        // hangSpecimen();
        //go to push samples into obs zone
        autoRobotMovement.strafe(24, Right);
        autoRobotMovement.moveForward(26, Forward);

        pushSampleToObs(autoRobotMovement);
        autoRobotMovement.moveForward(48, Forward);
        pushSampleToObs(autoRobotMovement);
        autoRobotMovement.moveForward(48, Forward);
        pushSampleToObs(autoRobotMovement);

        // Stop motors (optional, as subroutine already stops them)
        drivetrain.setPower(new double[]{0, 0, 0, 0});
    }

    private void pushSampleToObs (AutoRobotMovement autoRobotMovement) {
        autoRobotMovement.strafe(8, Right);
        autoRobotMovement.moveForward(48, Backward);
    }
}

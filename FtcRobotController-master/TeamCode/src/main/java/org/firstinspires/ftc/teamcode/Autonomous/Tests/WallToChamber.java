package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.Global;

import static org.firstinspires.ftc.teamcode.utils.Global.Right;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Backward;
import static org.firstinspires.ftc.teamcode.utils.Global.Clockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.CounterClockwise;

@Config
@Autonomous(name="WallToChamber", group="Tests")
public class WallToChamber extends LinearOpMode {
    Drivetrain drivetrain;
    MiniStructure miniStructure;

    public static double dist = 0,
    strafeDist = 0,
    dist2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        miniStructure = new MiniStructure(this);
        // Initialize motors - copied from CombinedMaster
        drivetrain = new Drivetrain(this);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain.leftFront, drivetrain.rightFront, drivetrain.leftBack, drivetrain.rightBack);

        waitForStart();

        // drive to submersible
        autoRobotMovement.moveForward(dist, Forward);
        autoRobotMovement.strafe(strafeDist, Left);
        autoRobotMovement.moveForward(dist2, Forward);

        // hangSpecimen();
        //go to push samples into obs zone
//        autoRobotMovement.strafe(24, Right);
//        autoRobotMovement.moveForward(26, Forward);
//
//        pushSampleToObs(autoRobotMovement);
//        autoRobotMovement.moveForward(48, Forward);
//        pushSampleToObs(autoRobotMovement);
//        autoRobotMovement.moveForward(48, Forward);
//        pushSampleToObs(autoRobotMovement);
//        autoRobotMovement.moveForward(18, Forward);
//        autoRobotMovement.rotate(180, Clockwise);
//        //arm/ministructure up here
//        autoRobotMovement.moveForward(6, Forward);
//        Thread.sleep(3000);
//        autoRobotMovement.moveForward(6, Forward);
//        //close claw
//        // arm up all the way
//        autoRobotMovement.strafe(48, Right);
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.moveForward(12, Forward);
//        // hang specimen
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.strafe(48, Left);
//        autoRobotMovement.moveForward(6, Forward);
//        Thread.sleep(3000);
//        autoRobotMovement.moveForward(6, Forward);
//        //close claw
//        // arm up all the way
//        autoRobotMovement.strafe(48, Right);
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.moveForward(12, Forward);
//        // hang specimen
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.strafe(48, Left);
//        autoRobotMovement.moveForward(6, Forward);
//        Thread.sleep(3000);
//        autoRobotMovement.moveForward(6, Forward);
//        //close claw
//        // arm up all the way
//        autoRobotMovement.strafe(48, Right);
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.moveForward(12, Forward);
//        // hang specimen
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.strafe(48, Left);autoRobotMovement.moveForward(6, Forward);
//        Thread.sleep(3000);
//        autoRobotMovement.moveForward(6, Forward);
//        //close claw
//        // arm up all the way
//        autoRobotMovement.strafe(48, Right);
//        autoRobotMovement.rotate(180, Clockwise);
//        autoRobotMovement.moveForward(12, Forward);
//        // hang specimen
//
//        // Stop motors (optional, as subroutine already stops them)
//        drivetrain.setPower(new double[]{0, 0, 0, 0});
    }

    private void pushSampleToObs (AutoRobotMovement autoRobotMovement) {
        autoRobotMovement.strafe(8, Right);
        autoRobotMovement.moveForward(48, Backward);
    }
}

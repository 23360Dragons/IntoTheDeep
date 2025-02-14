package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
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
    SuperStructure superStructure;

    public static double dist = 20,
    strafeDist = 12,
    dist2 = 20;

    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        timer.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        miniStructure = new MiniStructure(this);
        miniStructure.artie.up();
        drivetrain = new Drivetrain(this);
        superStructure = new SuperStructure(this);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain.leftFront, drivetrain.rightFront, drivetrain.leftBack, drivetrain.rightBack);

        Global.switchToAuto();
        superStructure.arm.switchToAuto();
        superStructure.extension.switchToAuto();
        waitForStart();

        // drive to submersible
        miniStructure.artie.up();
        miniStructure.tilt.down();
        superStructure.extension.chamber();
        superStructure.extension.switchToAuto();
        autoRobotMovement.moveForward( dist, Forward);
        autoRobotMovement.strafe(strafeDist, Left);
        autoRobotMovement.moveForward(dist2, Forward);
        superStructure.extension.down();
        superStructure.extension.setPower(1);
        timer.reset();
        while(timer.milliseconds() < 1500) {}
        miniStructure.claw.open();
        timer.reset();
        while(timer.milliseconds() < 3000) {}

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

}

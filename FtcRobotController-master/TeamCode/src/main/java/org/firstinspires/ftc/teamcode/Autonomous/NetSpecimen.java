package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

import static org.firstinspires.ftc.teamcode.utils.Global.Backward;
import static org.firstinspires.ftc.teamcode.utils.Global.Clockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Right;

@Config
@Autonomous(name="NetSpecimen", group="Auto", preselectTeleOp = "DragonsDriver")
public class NetSpecimen extends AutonomousOpMode {
    public static double dist = 20,
            strafeDist = 12,
            dist2 = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        timer.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        miniStructure = new MiniStructure(this);

        AutoRobotPos.reset(); // this removes the imu orientation offset from any previous autos
        imu = new DragonsIMU(this);

        drivetrain = new Drivetrain(this);
        superStructure = new SuperStructure(this, true);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain.leftFront, drivetrain.rightFront, drivetrain.leftBack, drivetrain.rightBack);

        Global.switchToAuto();
        superStructure.arm.switchToAuto();

        waitForStart();

        if (isStopRequested()) return;

        miniStructure.artie.up();
        miniStructure.tilt.down();

        // start
        superStructure.extension.hang();
        superStructure.extension.switchToAuto();
        superStructure.extension.setPower(0.8);
//        sleep(500);

        // drive to submersible
        miniStructure.artie.up();
        miniStructure.tilt.down();
        autoRobotMovement.moveForward(dist, Forward, 0.5);
        autoRobotMovement.strafe(strafeDist, Right, 0.5);
        autoRobotMovement.moveForward(dist2, Forward, 0.2);

        timerSleep(200);

        // lower linear slides
        superStructure.extension.chamber();

        timerSleep(300);

        // push arm forward to clip it
        miniStructure.artie.chamberRelPos();

        timerSleep(500);

        // release claw
        miniStructure.claw.open();

        timerSleep(500);

        // move stuff back for driving
        miniStructure.tilt.up();
        miniStructure.artie.up();
        superStructure.extension.down();
        miniStructure.claw.close();

        // park at ascent
        autoRobotMovement.moveForward(24, Backward, 0.5);
        autoRobotMovement.strafe(48, Left, 0.5);
        autoRobotMovement.moveForward(56, Forward, 0.6);
        autoRobotMovement.rotate(195, Clockwise);
        miniStructure.artie.chamberRelPos();
        miniStructure.tilt.hang();
        superStructure.extension.chamber();
        autoRobotMovement.moveForward(35, Forward, 0.2);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());
    }
}
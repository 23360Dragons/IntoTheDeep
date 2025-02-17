package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="OldNetSpecimen", group="Auto", preselectTeleOp = "DragonsDriver")
public class OldNetSpecimen extends LinearOpMode {
    Drivetrain drivetrain;
    DragonsIMU imu;
    MiniStructure miniStructure;
    SuperStructure superStructure;

    public static double dist = 20,
            strafeDist = 12,
            dist2 = 24;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoRobotPos.reset();
        timer = new ElapsedTime();
        timer.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        miniStructure = new MiniStructure(this);
        imu = new DragonsIMU(this);

        drivetrain = new Drivetrain(this);
        superStructure = new SuperStructure(this, true);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain);

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
        sleep(200);
        superStructure.extension.chamber();
        timer.reset();
        while (timer.milliseconds() < 300) {
        }
        miniStructure.artie.chamberRelPos();
        while (timer.milliseconds() < 800) ;
        miniStructure.claw.open();
        timer.reset();
        while (timer.milliseconds() < 500) {
        }
        miniStructure.tilt.up();
        miniStructure.artie.up();
        superStructure.extension.down();
        miniStructure.claw.close();


        autoRobotMovement.strafe(48, Left, 0.5);
        autoRobotMovement.moveForward(56, Forward, 0.6);
        autoRobotMovement.rotate(90, Clockwise, 0.5);
        miniStructure.artie.chamberRelPos();
        miniStructure.tilt.hang();
        superStructure.extension.chamber();
        autoRobotMovement.moveForward(35, Forward, 0.2);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());
    }
}
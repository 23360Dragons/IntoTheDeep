package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.Global;

import static org.firstinspires.ftc.teamcode.utils.Global.Backward;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Right;

@Config
@Autonomous(name="OldObsSpecimen", group="OldAuto", preselectTeleOp = "DragonsDriver")
public class OldObsSpecimen extends LinearOpMode {
    Drivetrain drivetrain;
    MiniStructure miniStructure;
    SuperStructure superStructure;
    DragonsIMU imu;

    public static double dist = 28,
            strafeDist = 12,
            dist2 = 16;

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
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain, this);

        Global.switchToAuto();
        superStructure.arm.switchToAuto();
//        superStructure.extension.switchToAuto();
        waitForStart();

        if (isStopRequested()) return;
        
        timer.reset();
//        while (timer.seconds() < 13);

        miniStructure.chamber();

        // start

        superStructure.extension.hang();
        superStructure.extension.switchToAuto();
        superStructure.extension.setPower(0.8);
//        sleep(500);

        // drive to submersible
        miniStructure.chamber();
        autoRobotMovement.moveForward(dist, Forward, 0.5);
        autoRobotMovement.strafe(strafeDist, Left, 0.5);
        autoRobotMovement.moveForward(dist2, Forward, 0.2);
        sleep(200);
        superStructure.extension.chamber();
        timer.reset();
        while (timer.milliseconds() < 300 && opModeIsActive()) {
        }
        miniStructure.artie.chamberRelPos();
        while (timer.milliseconds() < 800 && opModeIsActive()) ;
        miniStructure.claw.open();
        timer.reset();
        while (timer.milliseconds() < 500 && opModeIsActive()) {
        }
        miniStructure.tilt.up();
        miniStructure.artie.up();
        superStructure.extension.down();
        miniStructure.claw.close();

        // park in observation zone

        autoRobotMovement.moveForward(39, Backward, 0.4);
        autoRobotMovement.strafe(40, Right, 0.5);
        autoRobotMovement.moveForward(12, Backward, 0.2);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }
}
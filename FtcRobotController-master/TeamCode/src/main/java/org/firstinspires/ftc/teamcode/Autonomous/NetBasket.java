package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

import static org.firstinspires.ftc.teamcode.utils.Global.Backward;
import static org.firstinspires.ftc.teamcode.utils.Global.Clockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.CounterClockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Right;

import java.time.Clock;

@Config
@Autonomous(name="NetBasket", group="Auto", preselectTeleOp = "DragonsDriver")
public class NetBasket extends AutonomousOpMode {
    public static double dist = 20,
            strafeDist = 12,
            dist2 = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        if (isStopRequested()) return;

        autoRobotMovement.moveForward(2, Forward, 0.3);
        autoRobotMovement.strafe(24, Left, 0.5);
        superStructure.extension.full();
        superStructure.extension.setPower(0.8);
        miniStructure.basket();
        autoRobotMovement.rotate(135, CounterClockwise, 0.5);
        autoRobotMovement.moveForward(6 , Forward, 0.2);
        while (!superStructure.extension.atTargetPosition()) {}
        miniStructure.claw.open();
        timerSleep(1000);
        miniStructure.claw.close();
        timerSleep(500);
        autoRobotMovement.moveForward(6, Backward, 0.3);
        superStructure.extension.down();
        superStructure.extension.setPower(0.6);
        miniStructure.down();
        timerSleep(500);
        autoRobotMovement.rotate(135, Clockwise, 0.5);
        autoRobotMovement.strafe(12, Right, 0.5);
        autoRobotMovement.moveForward(56, Forward, 0.6);

        autoRobotMovement.rotate(90, Clockwise, 0.5);
        miniStructure.artie.chamberRelPos();
        miniStructure.tilt.hang();
        superStructure.extension.chamber();
        superStructure.extension.setPower(0.8);
        timerSleep(300);
        autoRobotMovement.moveForward(48, Forward, 0.2);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());
    }
}
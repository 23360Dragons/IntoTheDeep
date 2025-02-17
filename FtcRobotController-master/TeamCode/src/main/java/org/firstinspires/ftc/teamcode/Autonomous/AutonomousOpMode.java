package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.Global;

public abstract class AutonomousOpMode extends LinearOpMode {
    public SuperStructure superStructure;
    public DragonsIMU imu;
    public MiniStructure miniStructure;
    public Drivetrain drivetrain;

    public ElapsedTime timer;
    public Pose2D robotPose;
    public AutoRobotMovement autoRobotMovement;

    public int blueFace        = 180 ;
    public Pose2d blueOne      = new Pose2d(-12, 62, 0);
    public Pose2d blueTwo      = new Pose2d(35, 62, 0);
    public Vector2d toBlueSide = new Vector2d(0, 34);
    public double redFace      = 0;
    public Vector2d redOne     = new Vector2d(-35, -63);
    public Vector2d redTwo     = new Vector2d(12, -63);
    public Vector2d toRedSide  = new Vector2d(0, -33);
    public Pose2d blueBasket   = new Pose2d(53, 53, Math.toRadians(45));
    public Vector2d run        = new Vector2d(10, 43);
    public Vector2d backup     = new Vector2d(0, 43);
    public Pose2d redBasket    = new Pose2d(-57, -57, Math.toRadians(225));
    public Pose2d redAscent    = new Pose2d(-25, 0, Math.toRadians(0));
    public Pose2d blueAscent   = new Pose2d(25, 0, Math.toRadians(180));
    public Pose2d blueObserve  = new Pose2d(-57, 60, Math.toRadians(90));
    public Pose2d redObserve   = new Pose2d(57, -58, Math.toRadians(270));

    public void timerSleep (int milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds) {
            if (isStopRequested()) return;
        }
    }

    public void initRobot () {
        AutoRobotPos.reset(); // this removes the imu orientation offset from any previous autos

        timer = new ElapsedTime();
        timer.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = new DragonsIMU(this);
        drivetrain = new Drivetrain(this);
        superStructure = new SuperStructure(this, true);
        miniStructure = new MiniStructure(this);
        autoRobotMovement = new AutoRobotMovement(drivetrain);

        Global.switchToAuto();
        superStructure.extension.switchToAuto();
        superStructure.arm.switchToAuto();
    }

}

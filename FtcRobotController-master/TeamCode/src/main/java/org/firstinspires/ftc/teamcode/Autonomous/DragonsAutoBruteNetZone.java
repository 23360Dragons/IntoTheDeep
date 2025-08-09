package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

@Autonomous (preselectTeleOp = "DragonsDriver")
public class DragonsAutoBruteNetZone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoRobotPos.reset();
        
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MiniStructure miniStructure = new MiniStructure(this);
        DragonsIMU imu = new DragonsIMU(this);
        
        Drivetrain drivetrain = new Drivetrain(this);
        SuperStructure superStructure = new SuperStructure(this, true);
        AutoRobotMovement autoRobotMovement = new AutoRobotMovement(drivetrain, this);
        
        Global.switchToAuto();
        superStructure.arm.switchToAuto();

        waitForStart();

        autoRobotMovement.strafe(     14, Global.Left, 0.5);

        sleep (500);

        autoRobotMovement.strafe(     100, Global.Right, 0.5);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());
    }
}
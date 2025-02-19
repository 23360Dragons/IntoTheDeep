package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.utils.Global;

@Autonomous
public class TestingAuto extends AutonomousOpMode {
    Drivetrain drivetrain;
    DragonsIMU imu;
    MiniStructure miniStructure;
    SuperStructure superStructure;
    
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
        
        waitForStart();
        
        autoRobotMovement.rotate(45, true, 0.3);
        
        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}

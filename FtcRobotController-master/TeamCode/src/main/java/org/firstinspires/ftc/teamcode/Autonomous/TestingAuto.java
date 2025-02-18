package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;

public class TestingAuto extends AutonomousOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoRobotPos.reset();
        initRobot();

        waitForStart();
        
        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}

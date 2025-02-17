package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;

@Autonomous (preselectTeleOp = "DragonsDriver")
public class DragonsAutoBruteObZone extends AutonomousOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        if (isStopRequested()) return;

        autoRobotMovement.strafe(44, true, 0.5);

        //autoRobotMovement.strafe(200, true);
        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());
    }
}

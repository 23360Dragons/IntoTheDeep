package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.utils.Global;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

@Autonomous (preselectTeleOp = "DragonsDriver")
public class DragonsAutoBruteNetZone extends AutonomousOpMode {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public DragonsIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        autoRobotMovement.strafe(     14, Global.Left, 0.5);

        sleep (500);

        autoRobotMovement.strafe(     100, Global.Right, 0.5);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw());

    }
}
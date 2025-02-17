package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.AutoRobotMovement;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;
import org.firstinspires.ftc.teamcode.utils.Global;

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

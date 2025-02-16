package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DragonsIMU;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.MiniStructure;
import org.firstinspires.ftc.teamcode.hardware.SuperStructure;
import org.firstinspires.ftc.teamcode.utils.StoreAutoRobotPos;

public abstract class AutonomousOpMode extends LinearOpMode {
    public SuperStructure superStructure;
    public DragonsIMU imu;
    public MiniStructure miniStructure;
    public Drivetrain drivetrain;
    public ElapsedTime timer;

    public void resetIMUOrientation () {
        StoreAutoRobotPos.reset();
    }

    public void timerSleep (int milliseconds) {
        timer.reset();
        while (timer.milliseconds() < milliseconds) {
            if (isStopRequested()) return;
        }
    }

}

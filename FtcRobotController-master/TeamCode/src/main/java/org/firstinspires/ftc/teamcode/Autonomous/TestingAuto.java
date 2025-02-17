package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TestingAuto extends AutonomousOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        robotPose = new Pose2D(DistanceUnit.INCH, redOne.x, redOne.y, AngleUnit.DEGREES, redFace);

        waitForStart();
    }
}

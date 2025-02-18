package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.utils.AutoRobotPos;

import static org.firstinspires.ftc.teamcode.utils.Global.Backward;
import static org.firstinspires.ftc.teamcode.utils.Global.Clockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.CounterClockwise;
import static org.firstinspires.ftc.teamcode.utils.Global.Left;
import static org.firstinspires.ftc.teamcode.utils.Global.Forward;
import static org.firstinspires.ftc.teamcode.utils.Global.Right;

@Config
@Autonomous(name="NetSpecimen", group="Auto", preselectTeleOp = "DragonsDriver")
public class NetSpecimen extends AutonomousOpMode {
    public static double moveAwayFromChamberdist = 12,
    moveLeftToSampledist         = 50,
    moveTowardsSampledist        = 12,
    moveBackwardToBasketdist     = 15,
    rotateToFaceBasketdist       = 135,
    moveTowardsBasketdist        = 14,
    moveAwayFromBasketdist       = 18,
    rotateAwayFromBasketdist     = 135,
    moveForwardTowardsAscentdist = 52,
    moveTowardsRungForAscentdist = 35;
    
    public static boolean doBasket = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();

        if (isStopRequested()) return;

        miniStructure.artie.up();
        miniStructure.tilt.down();

        // start
        superStructure.extension.hang();
        superStructure.extension.switchToAuto();
        superStructure.extension.setPower(0.8);
//        sleep(500);

        // drive to submersible
        miniStructure.artie.up();
        miniStructure.tilt.down();
        autoRobotMovement.moveForward(20, Forward, 0.5);
        autoRobotMovement.strafe(12, Right, 0.5);
        autoRobotMovement.moveForward(24, Forward, 0.2);

        timerSleep(100);

        // lower linear slides
        superStructure.extension.chamber();

        timerSleep(300);

        // push arm forward to clip it
        miniStructure.artie.chamberRelPos();

        timerSleep(500);

        // release claw
        miniStructure.claw.open();

        timerSleep(200);

        // move stuff back for driving
        miniStructure.tilt.up();
        miniStructure.artie.up();
        superStructure.extension.down();
        miniStructure.claw.close();

        // park at ascent
        autoRobotMovement.moveForward(moveAwayFromChamberdist, Backward, 0.5);
        
        // NEW FOR BASKET STUFF
        if (doBasket) {
            miniStructure.down();
            miniStructure.claw.open();
            
            autoRobotMovement.strafe(moveLeftToSampledist, Left, 0.5);
            autoRobotMovement.moveForward(moveTowardsSampledist, Forward, 0.3);
            
            miniStructure.claw.close();
            timerSleep(300);
            miniStructure.basket();
            superStructure.extension.full();
            superStructure.extension.setPower(0.8);
            
            autoRobotMovement.moveForward(moveBackwardToBasketdist, Backward, 0.3);
            autoRobotMovement.rotate(rotateToFaceBasketdist, CounterClockwise, 0.3);
            autoRobotMovement.moveForward(moveTowardsBasketdist, Forward, 0.3);
            
            miniStructure.tilt.down();
            timerSleep(100);
            miniStructure.claw.open();
            timerSleep(500);
            miniStructure.tilt.up();
            autoRobotMovement.moveForward(moveAwayFromBasketdist, Backward, 0.3);
            superStructure.extension.down();
            superStructure.extension.setPower(0.6);
            autoRobotMovement.rotate(rotateAwayFromBasketdist, Clockwise, 0.3);
            autoRobotMovement.moveForward(moveForwardTowardsAscentdist, Forward, 0.6);
            
            // END NEW BASKET STUFF
        } else {
            autoRobotMovement.strafe(48, Left, 0.5);
            autoRobotMovement.moveForward(44, Forward, 0.6);
        }
        
        autoRobotMovement.rotate(90, Clockwise, 0.5);
        miniStructure.artie.chamberRelPos();
        miniStructure.tilt.hang();
        superStructure.extension.chamber();
        autoRobotMovement.moveForward(moveTowardsRungForAscentdist, Forward, 0.2);

        AutoRobotPos.store(imu.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
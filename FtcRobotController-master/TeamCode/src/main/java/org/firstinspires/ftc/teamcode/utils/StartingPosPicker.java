package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class StartingPosPicker {
    public static Object pickStart;
    int starty;

    public void pickStart(){
        if (gamepad1.x) {
            starty = 1;
        } else if (gamepad1.b) {
            starty = 2;
        } else if (gamepad1.a) {
            starty = 3;
        } else if (gamepad1.y) {
            starty = 4;
        }
    }
//        switch (starty) {
//            case 1:
//                startPose = blueStartBasket;
//                Actions.runBlocking(littleLarryLime.LarryLimeYellow());
//                telemetry.addLine("Starting Position Set To Blue, Basket Side. If inncorrect, please reselect");
//                telemetry.update();
//                break;
//            case 2:
//                startPose = redStartBasket;
//                Actions.runBlocking(littleLarryLime.LarryLimeYellow());
//                telemetry.addLine("Starting Position Set To Red, Basket Side. If inncorrect, please reselect");
//                telemetry.update();
//                break;
//            case 3:
//                startPose = blueStartObserve;
//                Actions.runBlocking(littleLarryLime.LarryLimeBlues());
//                telemetry.addLine("Starting Position Set To Blue, Observation Zone Side. If inncorrect, please reselect");
//                telemetry.update();
//                break;
//            case 4:
//                startPose = redStartObserve;
//                Actions.runBlocking(littleLarryLime.LarryLimeRedTV());
//                telemetry.addLine("Starting Position Set To Red, Observation Zone Side. If inncorrect, please reselect");
//                telemetry.update();
//                break;
//            default:
//                startPose = notSelected;
//                telemetry.addLine("Please select starting position! If not selected, the robot will not run during Auto.");
//                break;
//        }

}

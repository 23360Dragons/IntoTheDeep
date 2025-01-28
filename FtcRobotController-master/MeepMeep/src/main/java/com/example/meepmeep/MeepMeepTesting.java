package com.example.meepmeep;

import static com.example.meepmeep.MeepMeepTesting.myBot;
import static com.example.meepmeep.MeepMeepTesting.redFace;
import static com.example.meepmeep.MeepMeepTesting.redTwo;
import static com.example.meepmeep.MeepMeepTesting.toRedSide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;

import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static int blueFace = 270;
    public static Vector2d blueOne    = new Vector2d(-12, 62);
    public static Vector2d blueTwo    = new Vector2d(35, 62);
    public static Vector2d toBlueSide = new Vector2d(0, 34);
    public static int redFace = 90;
    public static Vector2d redOne     = new Vector2d(-35, -60);
    public static Vector2d redTwo     = new Vector2d(12, -60);
    public static Vector2d toRedSide  = new Vector2d(0, -33);
    public static Pose2d blueBasket   = new Pose2d(53, 53, Math.toRadians(45));
    public static Vector2d run        = new Vector2d(10, 43);
    public static Vector2d backup     = new Vector2d(0, 43);
    public static Pose2d redBasket    = new Pose2d(-52, -52, Math.toRadians(225));
    public static Pose2d redAscent    = new Pose2d(-25, 0, Math.toRadians(0));
    public static Pose2d blueAscent   = new Pose2d(25, 0, Math.toRadians(180));
    public static Pose2d blueObserve  = new Pose2d(-57, 60, Math.toRadians(90));
    public static Pose2d redObserve   = new Pose2d(57, -58, Math.toRadians(270));
    public static RoadRunnerBotEntity myBot;
    public static void main(String[] args) {
        MeepMeep meepMeep   = new MeepMeep(760);


        myBot = new DefaultBotBuilder(meepMeep)
//                 Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(redTwo, Math.toRadians(redFace)))
//                    .waitSeconds(3)
                .strafeTo(toRedSide)
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(redBasket.position.x - 5, redBasket.position.y - 5))

//                    .splineTo(toRedSide, redFace)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

class Observe {
    public static class moveToObserve implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(redTwo, Math.toRadians(redFace)))
//                    .waitSeconds(3)
                                    .lineToX(15)
                                    .lineToY(15)
                                    .turnTo(90)
//                    .splineTo(toRedSide, redFace)
                    .build());

            return false;
        }
    }

    public Action toObserve () {
        return new moveToObserve();
    }
}
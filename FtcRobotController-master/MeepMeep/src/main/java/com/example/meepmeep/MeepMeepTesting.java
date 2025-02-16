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

import java.util.Set;

public class MeepMeepTesting {
    public static int blueFace = 270;
    public static Pose2d blueOne    = new Pose2d(-12, 62, 0);
    public static Pose2d blueTwo    = new Pose2d(35, 62, 0
    );
    public static Vector2d toBlueSide = new Vector2d(0, 34);
    public static double redFace = Math.toRadians(90);
    public static Vector2d redOne     = new Vector2d(-35, -60);
    public static Vector2d redTwo     = new Vector2d(12, -60);
    public static Vector2d toRedSide  = new Vector2d(0, -33);
    public static Pose2d blueBasket   = new Pose2d(53, 53, Math.toRadians(45));
    public static Vector2d run        = new Vector2d(10, 43);
    public static Vector2d backup     = new Vector2d(0, 43);
    public static Pose2d redBasket    = new Pose2d(-57, -57, Math.toRadians(225));
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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(redOne, redFace))
                .strafeTo(toRedSide)
                .waitSeconds(4)
                        .strafeTo(new Vector2d(-48, -40))
                                .strafeTo(new Vector2d(-48, -30))
                .waitSeconds(1)
                .strafeToSplineHeading(redBasket.position, redBasket.heading)
                .waitSeconds(2)
                
                .strafeToLinearHeading(new Vector2d(-58, -30), redFace)
                .waitSeconds(1)
                
                .strafeToSplineHeading(redBasket.position, redBasket.heading)
                .waitSeconds(2)
                
                .strafeToLinearHeading(new Vector2d(-58, -27), Math.toRadians(180))
                        .strafeTo(new Vector2d(-60, -25))
                .waitSeconds(2)
                
                .strafeToSplineHeading(redBasket.position, redBasket.heading)
                .waitSeconds(2)
                
                        .strafeTo(new Vector2d(-50, -50))
                        .splineToSplineHeading(redAscent, redAscent.heading)
                
                
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

class Observe {
    public class moveToObserve implements Action {
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
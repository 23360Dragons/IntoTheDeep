package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DragonsMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(760);
        int blueFace = 270;
        Vector2d blueOne = new Vector2d(-12,62);
        Vector2d blueTwo = new Vector2d(35,62);
        Vector2d toBlueSide = new Vector2d(0,34);
        int redFace = 90;
        Vector2d redOne = new Vector2d(-35,-60);
        Vector2d redTwo = new Vector2d(12,-60);
        Vector2d toRedSide = new Vector2d(0, -33);
        Pose2d blueBasket = new Pose2d(53,53,Math.toRadians(45));
        Vector2d run = new Vector2d(10,43);
        Vector2d backup = new Vector2d(0,43);
        Pose2d redBasket = new Pose2d(-52, -52,Math.toRadians(225));
        Pose2d redAscent = new Pose2d (-25,0,Math.toRadians(0));
        Pose2d blueAscent = new Pose2d(25,0, Math.toRadians(180));
        Vector2d blueObserve = new Vector2d(-57,60);
        Pose2d redObserve = new Pose2d(57,-58,Math.toRadians(270));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(blueOne, Math.toRadians(blueFace)))
                .waitSeconds(3)
                .strafeTo(new Vector2d(48,42))
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket.component1(), Math.toRadians(45))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(58,42),Math.toRadians(270))
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket.component1(), Math.toRadians(45))
                .waitSeconds(3)
                .strafeToSplineHeading(new Vector2d(55,25),Math.toRadians(0))
                .waitSeconds(3)
                .strafeToSplineHeading(blueBasket.component1(), Math.toRadians(45))
                .waitSeconds(3)
                .turnTo(Math.toRadians(blueFace))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
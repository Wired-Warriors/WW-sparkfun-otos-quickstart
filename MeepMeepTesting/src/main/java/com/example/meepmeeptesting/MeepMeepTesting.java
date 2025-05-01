package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.173)
                .build();


        //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16.5, -62.69, 90*Math.PI/180))
                    //.splineTo(new Vector2d(40,-8), Math.toRadians(90))
                    //.strafeTo(new Vector2d(44,-6))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-23, -59, 90*Math.PI/180))
                    //.strafeTo(new Vector2d(54,-51))
                    .turnTo(Math.toRadians(157.5))
                    .strafeTo(new Vector2d(-52.7,-51.9))
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(90))
                    .strafeTo(new Vector2d(-48.2,-38.6))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-52.7,-51.9))
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    .turnTo(Math.toRadians(90))
                    .strafeTo(new Vector2d(-57,-38.6))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-52.7,-51.9))
                    .turnTo(Math.toRadians(225))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-40,-35.5))
                    .splineToLinearHeading(new Pose2d(-61,-6,Math.toRadians(270)),Math.toRadians(225))
                    .strafeTo(new Vector2d (-61,-51.5))
                //.strafeTo(new Vector2d (-24,-46))
                //.splineToLinearHeading(new Pose2d(-49,-36,Math.toRadians(0)),Math.toRadians(90))



//                .splineTo(new Vector2d(48,-25), Math.toRadians())
//                .splineTo(new Vector2d(56,47), Math.toRadians(90))
//                .strafeTo(new Vector2d(-38,47))
//                .strafeTo(new Vector2d(-47,-47))
//                .strafeTo(new Vector2d(23,-47))
//

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
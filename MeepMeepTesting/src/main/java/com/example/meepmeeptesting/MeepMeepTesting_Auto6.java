package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Auto6 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.173)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16.5, -62.69, 90*Math.PI/180))
                .lineToY(-55)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(36,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46,-7),Math.toRadians(90))
                .setReversed(true)
                .lineToY(-61)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(47,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54,-7),Math.toRadians(90))
                .setReversed(true)
                .lineToY(-59)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(55,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62,-7),Math.toRadians(90))
                .lineToY(-50)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.75f)
                .addEntity(myBot)
                .start();
    }
}
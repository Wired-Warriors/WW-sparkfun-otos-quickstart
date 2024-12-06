package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Auto1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.173)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16.5, -62.69, 90*Math.PI/180))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                //.splineTo(new Vector2d(63.75, 63.75), Math.PI/4)
                .splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
                                .waitSeconds(2)
                        //`.turnTo(Math.toRadians(-180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-51,-42, 90*Math.PI/180),90*Math.PI/180)
                //.splineTo(new Vector2d(-51, -42), 90*Math.PI/180)
                                .waitSeconds(2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54, -54,-135*Math.PI/180), -135*Math.PI/180)
                .waitSeconds(2)
                //.splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
                                .waitSeconds(2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-60.5, -42, 90*Math.PI/180), 90*Math.PI/180)
                                .waitSeconds(2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54, -54,-135*Math.PI/180), -135*Math.PI/180)
                                .waitSeconds(2)
                                .splineTo(new Vector2d(-51,-42),90*Math.PI/180)
                .splineTo(new Vector2d(-60.5,-7),90*Math.PI/180)
                //.setTangent(0)
                //.splineToConstantHeading(new Vector2d(-60.5, -55),90*Math.PI/180)
                .lineToY(-55)
                        .setReversed(false)
                .splineTo(new Vector2d(-21,-11),0*Math.PI/180)
                                //.lineToX(-21)
                //.splineToSplineHeading(new Pose2d(-21,-11,0),180*Math.PI/180)


                       // .lineToX(46.75)
                        //.splineToConstantHeading(new Vector2d(-63.75,63.75 ), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
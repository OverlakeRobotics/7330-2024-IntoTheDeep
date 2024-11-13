package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d initialPose = new Pose2d(-37, -62, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(25, 30, Math.toRadians(90), Math.toRadians(90), 14.978770126140358)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .strafeTo(new Vector2d(-37, -55))
                .strafeToLinearHeading(new Vector2d(-48.5, -53.5), Math.toRadians(-133))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(90))
                .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
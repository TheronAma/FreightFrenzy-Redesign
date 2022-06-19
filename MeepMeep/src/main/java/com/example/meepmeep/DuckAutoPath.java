package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DuckAutoPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d START_POSE = new Pose2d(-36, 63, Math.PI/2);
        Vector2d HUB_POS = new Vector2d(-12, 24);
        Pose2d DUCK_POSE = new Pose2d(-64, 64, 0);

        double SCORE_DISTANCE = 25;
        double SCORE_ANGLE = Math.toRadians(120);

        Pose2d SCORE_POSE = new Pose2d(
                HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
                HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
                SCORE_ANGLE);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START_POSE)
                                .lineToLinearHeading(SCORE_POSE)
                                .waitSeconds(0.4)
                                .splineTo(DUCK_POSE.minus(new Pose2d(0,10,0)).vec(), DUCK_POSE.getHeading())
                                .lineToConstantHeading(DUCK_POSE.vec())
                        .build()
                );
    }
}

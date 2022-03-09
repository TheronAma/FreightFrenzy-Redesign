package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class WarehouseAutoPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d START_POSE = new Pose2d(12, 63, Math.PI/2);
        Vector2d HUB_POS = new Vector2d(-12, 24);
        Pose2d BLUE_WAREHOUSE_POSE = new Pose2d(48, 66, 0);

        double SCORE_DISTANCE = 35;
        double SCORE_ANGLE = Math.toRadians(60);


        Pose2d SCORE_POSE = new Pose2d(
                HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
                HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
                SCORE_ANGLE);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START_POSE)
                                .lineToLinearHeading(SCORE_POSE)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                                .setReversed(true)
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                                .setReversed(true)
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

//                                .setReversed(false)
//                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
//                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
//                                .setReversed(true)
//                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
//                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)
//
//                                .setReversed(false)
//                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
//                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
//                                .setReversed(true)
//                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
//                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                                .setReversed(true)
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                                .setReversed(true)
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                                .setReversed(true)
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().minus(new Vector2d(20,0)),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineToSplineHeading(SCORE_POSE,SCORE_POSE.getHeading() + Math.PI)

                                .setReversed(false)
                                .splineToSplineHeading(BLUE_WAREHOUSE_POSE.minus(new Pose2d(20,0,0)),BLUE_WAREHOUSE_POSE.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
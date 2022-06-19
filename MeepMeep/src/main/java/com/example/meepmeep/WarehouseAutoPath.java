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

        Pose2d START_POSE = new Pose2d(12, 65.5, Math.PI/2);
        Vector2d HUB_POS = new Vector2d(-13.5, 24);
        Pose2d BLUE_WAREHOUSE_POSE = new Pose2d(45, 70, 0);
        Pose2d BLUE_WARE_HOUSE_TRANSITION_POSE_TWO = new Pose2d(36, 70, 0);
        Pose2d BLUE_WAREHOUSE_TRANSITION_POSE = new Pose2d(13, 70, 0);

        double SCORE_DISTANCE = 29;
        double SCORE_ANGLE = Math.toRadians(65);

        double SECOND_SCORE_DISTANCE = 29;

        double TRANSITION_DISTANCE = 12;

        Pose2d SCORE_POSE = new Pose2d(
                HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
                HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
                SCORE_ANGLE - Math.toRadians(8));

        Pose2d SECOND_SCORE_POSE = new Pose2d(
                HUB_POS.getX() + SECOND_SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
                HUB_POS.getY() + SECOND_SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
                SCORE_ANGLE - Math.toRadians(8));

        Pose2d SECOND_SCORE_TRANSITION_POSE = new Pose2d(
                SCORE_POSE.getX() + TRANSITION_DISTANCE * Math.cos(SCORE_ANGLE),
                SCORE_POSE.getY() + TRANSITION_DISTANCE * Math.sin(SCORE_ANGLE),
                0
        );


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BLUE_WAREHOUSE_POSE)
                                //traj 1
//                                .lineToLinearHeading(SCORE_POSE)
//                                .waitSeconds(0.4)
//                                .lineToLinearHeading(BLUE_WAREHOUSE_TRANSITION_POSE.plus(new Pose2d(0,6,0)))
//                                .splineToConstantHeading(BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.plus(new Pose2d(0,6,0)).vec(), BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.getHeading())
//                                .splineToLinearHeading(BLUE_WAREHOUSE_POSE.plus(new Pose2d(8,6,0)), BLUE_WAREHOUSE_POSE.getHeading())

                                //2
                                .setReversed(true)
                                .splineToConstantHeading(BLUE_WAREHOUSE_TRANSITION_POSE.vec(),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                                .splineTo(SECOND_SCORE_POSE.vec(),SECOND_SCORE_POSE.getHeading() + Math.PI)
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(SECOND_SCORE_TRANSITION_POSE, SCORE_ANGLE)
                                .splineToConstantHeading(BLUE_WAREHOUSE_TRANSITION_POSE.plus(new Pose2d(0,6,0)).vec(), 0)
                                .splineToConstantHeading(BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.plus(new Pose2d(0,6,0)).vec(), BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.getHeading())
                                .splineTo(BLUE_WAREHOUSE_POSE.vec().plus(new Vector2d(6,6)), BLUE_WAREHOUSE_POSE.getHeading())



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous
public class RedWarehouseAuto extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(12, 63, Math.PI/2);
    Vector2d HUB_POS = new Vector2d(-12, 24);
    Pose2d BLUE_WAREHOUSE_POSE = new Pose2d(48, 67.5, 0);
    Pose2d BLUE_WAREHOUSE_TRANSITION_POSE = new Pose2d(24, 67.5, 0);


    double SCORE_DISTANCE = 35;
    double SCORE_ANGLE = Math.toRadians(50);

    double SECOND_SCORE_DISTANCE = 35;

    Robot robot;

    Pose2d SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE - Math.toRadians(10));

    Pose2d SECOND_SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SECOND_SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SECOND_SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE);

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);

        //lift
        robot.lift.setTargetHeight(1.5);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_BACK_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);

        //intake
        robot.intake.setPower(0);

        TrajectorySequence preloadCycle = robot.drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(SCORE_POSE)
                .addTemporalMarker(0.25, ()->robot.lift.setHorizontalPos(0.52))
                .addTemporalMarker(0.01,()->{
                    robot.lift.setTargetHeight(16);
                })
                .addTemporalMarker(0.5,()->robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS))
                .addTemporalMarker(0.8, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_BACK_POS))
                .addTemporalMarker(1.0, ()->{
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                })
                .addTemporalMarker(1.5, ()->{
                    robot.lift.setTargetHeight(1.5);
                    robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
                })
                .setReversed(false)
                .addTemporalMarker(1.8, ()->{
                    robot.lift.setTargetHeight(0);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                    robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    robot.intake.setPower(0.7);
                })
                .splineToSplineHeading(BLUE_WAREHOUSE_TRANSITION_POSE,BLUE_WAREHOUSE_POSE.getHeading())
                .splineToConstantHeading(BLUE_WAREHOUSE_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                .build();

        TrajectorySequence regularCycle = robot.drive.trajectorySequenceBuilder(BLUE_WAREHOUSE_POSE)
                .setReversed(true)
                .splineToConstantHeading(BLUE_WAREHOUSE_TRANSITION_POSE.vec(),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
//                .addTemporalMarker(0.1,()->robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS))
//                .addTemporalMarker(0.5,()->{
//                    robot.lift.setTargetHeight(1.5);
//                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
//                    robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
//                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_POS);
//                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
//                    robot.intake.setPower(-0.5);
//                })
                .splineToSplineHeading(SECOND_SCORE_POSE,SECOND_SCORE_POSE.getHeading() + Math.PI)
//                .addTemporalMarker(1.5, ()->robot.lift.setHorizontalPos(0.52))
//                .addTemporalMarker(1.2,()->{
//                    robot.lift.setTargetHeight(16);
//                })
//                .addTemporalMarker(1.9,()->robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS))
//                .addTemporalMarker(2.4, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_POS))
//                .addTemporalMarker(2.9, ()->{
//                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
//                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
//                    robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
//                })
//                .addTemporalMarker(3.2, ()->{
//                    robot.lift.setTargetHeight(1.5);
//                })
                .build();

        robot.drive.setPoseEstimate(START_POSE);
        while(!isStopRequested() && !isStarted()) {
            robot.update();
        }

        waitForStart();

        robot.drive.followTrajectorySequenceAsync(preloadCycle);
        while(!isStopRequested() && robot.drive.isBusy()) {
            robot.update();
        }

        robot.drive.followTrajectorySequenceAsync(regularCycle);
        while(!isStopRequested() && robot.drive.isBusy()) {
            robot.update();
        }

        while(!isStopRequested()) {
            robot.update();
        }
    }
}

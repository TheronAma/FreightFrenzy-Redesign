package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.DuckPipeline;
import org.firstinspires.ftc.teamcode.subsystem.RedCapDetector;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous()
public class BlueCarouselAuto2 extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(-39, 63, Math.PI/2);
    Vector2d HUB_POS = new Vector2d(-12, 27);
    Pose2d DUCK_POSE = new Pose2d(-74, 50, Math.PI/2);
    Pose2d DEPOT_POSE = new Pose2d(-72, 27);

    double SCORE_DISTANCE = 27;
    double SCORE_ANGLE = Math.toRadians(120);

    double SECOND_SCORE_ANGLE = Math.toRadians(180);

    ElapsedTime matchTime;

    Pose2d SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE);

    Pose2d SECOND_SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SECOND_SCORE_ANGLE),
            HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SECOND_SCORE_ANGLE),
            SCORE_ANGLE
    );

    RedCapDetector detector;
    int position = 3;

    boolean foundDuck = false;

    Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        robot.lift.setTargetHeight(0);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
        robot.cap.setPosition(0.95);

        detector = new RedCapDetector(hardwareMap, "Webcam 1");
        detector.init();

        robot.drive.setPoseEstimate(START_POSE);

        TrajectorySequence carouselSequence = robot.drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(SCORE_POSE)
                .waitSeconds(0.4)
                .lineToLinearHeading(DUCK_POSE.minus(new Pose2d(0,24,0)))
                .setAccelConstraint(robot.drive.getAccelerationConstraint(30))
                .lineToConstantHeading(DUCK_POSE.vec())
                .addTemporalMarker(0.5, ()->robot.lift.setHorizontalPos(0.55))
                .addTemporalMarker(0.3,()->{
                    switch(position){
                        case 3:
                            robot.lift.setTargetHeight(15);
                            break;
                        case 2:
                            robot.lift.setTargetHeight(LiftConstants.MID_HUB_HEIGHT);
                            break;
                        case 1:
                            robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                            break;

                    }
                })
                .addTemporalMarker(0.5,()-> {
                    if(position != 1)
                        robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                    else
                        robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_LOW_POS);
                })
                .addTemporalMarker(0.9, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_BACK_POS))
                .addTemporalMarker(1.3, ()->{
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                })
                .addTemporalMarker(1.3, ()->{
                    robot.lift.setTargetHeight(1.5);
                    robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
                })
                .build();

        TrajectorySequence pickUpDuckSequence = robot.drive.trajectorySequenceBuilder(DUCK_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.PI, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(DUCK_POSE.plus(new Pose2d(6,-30,Math.PI)))
                .lineToLinearHeading(DUCK_POSE.plus(new Pose2d(54, -30, Math.PI)))
                .build();

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(DUCK_POSE)
                .setReversed(true)
                .lineToConstantHeading(DEPOT_POSE.vec())
                .build();


        while(!isStarted() && !isStopRequested()){
            if(detector.getX() < 120){
                position = 2;
            } else if(detector.getX() > 119 && detector.getX() < 1000){
                position = 3;
            } else {
                position = 1;
            }
            telemetry.addData("position",position);
            telemetry.update();
        }



        waitForStart();
        matchTime = new ElapsedTime();
        matchTime.reset();
        robot.cap.setPosition(0.65);

        robot.drive.followTrajectorySequenceAsync(carouselSequence);
        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){
            robot.update();
        }

        timer = new ElapsedTime();
        while(timer.seconds() < 6 && opModeIsActive()) {
            robot.carousel.setPower(-0.2);
        }

        detector.setPipeline(new DuckPipeline());
        timer.reset();
        robot.drive.followTrajectorySequenceAsync(pickUpDuckSequence);
        while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()){
            if(Math.abs(detector.getX() - 160) < 10 && timer.milliseconds() > 4000) {
                robot.drive.stopTrajectory();
                foundDuck = true;
            }
            robot.update();
        }

        detector.close();

        if(foundDuck) {
            //run dropoff
            robot.drive.turnAsync(Math.toRadians(180));
            while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()) {
                robot.drive.update();
            }

            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .forward(30)
                    .addTemporalMarker(0.1, ()->robot.intake.setPower(0.9))
                    .build());

            while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()) {
                robot.drive.update();
            }

            robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(SECOND_SCORE_POSE)
                    .build());

            while(opModeIsActive() && !isStopRequested() && robot.drive.isBusy()) {
                robot.drive.update();
            }

        } else {
            //park

        }

        detector.close();
        robot.drive.turnAsync(Math.PI);
        while(!isStopRequested() && opModeIsActive() && robot.drive.isBusy()) {
            robot.drive.update();
        }



    }
}

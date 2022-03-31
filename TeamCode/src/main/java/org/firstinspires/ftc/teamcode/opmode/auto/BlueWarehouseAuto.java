package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous
public class BlueWarehouseAuto extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(12, 63, Math.PI/2);
    Vector2d HUB_POS = new Vector2d(-15, 24);
    Pose2d BLUE_WAREHOUSE_POSE = new Pose2d(45, 67.5, 0);
    Pose2d BLUE_WARE_HOUSE_TRANSITION_POSE_TWO = new Pose2d(24, 68, 0);
    Pose2d BLUE_WAREHOUSE_TRANSITION_POSE = new Pose2d(18, 68.5, Math.toRadians(10));

    Vector2d leftDistancePos = new Vector2d(3, 8);
    Vector2d frontDistancePos = new Vector2d(10, -6);
    private AnalogInput frontSensor, leftSensor, rightSensor;

    double SCORE_DISTANCE = 31.5;
    double SCORE_ANGLE = Math.toRadians(65);

    double SECOND_SCORE_DISTANCE = 31.5;

    Robot robot;

    Pose2d SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE - Math.toRadians(8));

    Pose2d SECOND_SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SECOND_SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SECOND_SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE - Math.toRadians(8));

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        frontSensor = hardwareMap.get(AnalogInput.class, "frontSensor");
        leftSensor = hardwareMap.get(AnalogInput.class, "leftSensor");

        //lift
        robot.lift.setTargetHeight(0);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_BACK_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);

        //intake
        robot.intake.setPower(0);

        TrajectorySequence preloadCycle = robot.drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(SCORE_POSE)
                .waitSeconds(0.3)
                .addTemporalMarker(0.7, ()->robot.lift.setHorizontalPos(0.55))
                .addTemporalMarker(0.5,()->{
                    robot.lift.setTargetHeight(16);
                })
                .addTemporalMarker(0.9,()->robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS))
                .addTemporalMarker(1.3, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_BACK_POS))
                .addTemporalMarker(1.5, ()->{
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                })
                .addTemporalMarker(1.5, ()->{
                    robot.lift.setTargetHeight(1.5);
                    robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
                })
                .setReversed(false)
                .addTemporalMarker(2.6, ()->{
                    robot.lift.setTargetHeight(0);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                    robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    robot.intake.setPower(-1);
                })
                .addTemporalMarker(2.5, ()->{
                    robot.intake.setPower(0.8);
                })
                .splineTo(BLUE_WAREHOUSE_TRANSITION_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                .splineToLinearHeading(BLUE_WAREHOUSE_POSE.plus(new Pose2d(4,0,0)), BLUE_WAREHOUSE_POSE.getHeading())
                .build();

        TrajectorySequence regularCycle = robot.drive.trajectorySequenceBuilder(BLUE_WAREHOUSE_POSE)
                .setReversed(true)
                .splineToConstantHeading(BLUE_WAREHOUSE_TRANSITION_POSE.vec(),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
                .addTemporalMarker(0.5,()->{
                    robot.lift.setTargetHeight(0.5);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_BACK_POS);
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.intake.setPower(0.8);;
                })
                .addTemporalMarker(0.8, ()->{
                    robot.intake.setPower(-0.7);
                })
                .addTemporalMarker(2.1, ()->{
                    robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_TELE_OP_EXTEND_POS);
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                })
                .addTemporalMarker(2.6, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_BACK_POS))
                .addTemporalMarker(3.0, ()->{
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                })
                .addTemporalMarker(3.2, ()->{
                    robot.lift.setTargetHeight(1.5);
                    robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
                })
                .splineToSplineHeading(SECOND_SCORE_POSE,SECOND_SCORE_POSE.getHeading() + Math.PI)
                .waitSeconds(0.5)
                .setReversed(false)
                .addTemporalMarker(4.3, ()->{
                    robot.lift.setTargetHeight(0);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                    robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    robot.intake.setPower(-1);
                })
                .addTemporalMarker(4.5, ()->{
                    robot.intake.setPower(0.8);
                })
                .splineTo(BLUE_WAREHOUSE_TRANSITION_POSE.vec(), BLUE_WAREHOUSE_POSE.getHeading())
                .splineToConstantHeading(BLUE_WAREHOUSE_POSE.vec().plus(new Vector2d(6,0)), BLUE_WAREHOUSE_POSE.getHeading())
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

        for(int i = 0; i < 4; i++) {
            timer.reset();
            while(timer.milliseconds() < 300) {
                relocalize();
            }

            robot.drive.followTrajectorySequenceAsync(regularCycle);
            while(!isStopRequested() && robot.drive.isBusy()) {
                robot.update();
            }
        }


//        while(!isStopRequested()) {
//            robot.update();
//        }
    }

    public void relocalize() {
        double leftDist = voltageToDistance(leftSensor.getVoltage());
        double frontDist = voltageToDistance(frontSensor.getVoltage());
        double heading = robot.drive.getPoseEstimate().getHeading();

        double yPos = 72 - leftDist * Math.cos(heading) - leftDistancePos.rotated(heading).getY();
        double xPos = 72 - frontDist * Math.cos(heading) - frontDistancePos.rotated(heading).getX();
        robot.drive.setPoseEstimate(new Pose2d(xPos, yPos, heading));

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.update();
    }

    public double voltageToDistance(double voltage) {
        return (voltage - 0.142) * 12 / 0.138;
    }
}

package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.BlueCapDetector;
import org.firstinspires.ftc.teamcode.subsystem.RedCapDetector;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous
public class RedWarehouseAuto extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(12, -65.5, -Math.PI/2);
    Vector2d HUB_POS = new Vector2d(-10, -24);
    Pose2d BLUE_WAREHOUSE_POSE = new Pose2d(45, -66.5, 0);
    Pose2d BLUE_WARE_HOUSE_TRANSITION_POSE_TWO = new Pose2d(36, -66.5, 0);
    Pose2d BLUE_WAREHOUSE_TRANSITION_POSE = new Pose2d(13, -66.5, 0);

    ElapsedTime intakeTimer;
    int intakeState = 0;
    private BlueCapDetector detector;
    int position = 3;

    Pose2d[] BLUE_WAREHOUSE_POSES = {
            new Pose2d(43,-66.5,0),
            new Pose2d(45, -66.5, 0),
            new Pose2d(47, -66.5, 0),
            new Pose2d(49, -66.5, 0),
            new Pose2d(40, -66.5, 0)
    };

    Vector2d rightDistancePos = new Vector2d(3, -8);
    Vector2d frontDistancePos = new Vector2d(8, -6);
    private AnalogInput frontSensor, rightSensor;

    double SCORE_DISTANCE = 30.5;
    double SCORE_ANGLE = -Math.toRadians(65);

    double SECOND_SCORE_DISTANCE = 28;

    Robot robot;

    Pose2d SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SCORE_DISTANCE * Math.cos(SCORE_ANGLE),
            HUB_POS.getY() + SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE + Math.toRadians(5));

    Pose2d SECOND_SCORE_POSE = new Pose2d(
            HUB_POS.getX() + SECOND_SCORE_DISTANCE * Math.cos(SCORE_ANGLE) - 4.5,
            HUB_POS.getY() + SECOND_SCORE_DISTANCE * Math.sin(SCORE_ANGLE),
            SCORE_ANGLE);

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        detector = new BlueCapDetector(hardwareMap, "Webcam 1");
        detector.init();

        frontSensor = hardwareMap.get(AnalogInput.class, "frontSensor");
        rightSensor = hardwareMap.get(AnalogInput.class, "rightSensor");

        //lift
        robot.lift.setTargetHeight(0);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
        robot.cap.setPosition(0.95);

        //intake
        robot.intake.setPower(0);

        TrajectorySequence preloadCycle = robot.drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(SCORE_POSE)
                .waitSeconds(0.4)
                .addTemporalMarker(0.5, ()->robot.lift.setHorizontalPos(0.54))
                .addTemporalMarker(0.3,()->{
                    switch(position){
                        case 3:
                            robot.lift.setTargetHeight(15);
                            robot.lift.setHorizontalPos(0.58);
                            break;
                        case 2:
                            robot.lift.setTargetHeight(LiftConstants.MID_HUB_HEIGHT);
                            break;
                        case 1:
                            robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                            robot.lift.setHorizontalPos(0.57);
                            break;

                    }

                })
                .addTemporalMarker(0.5,()->{
                    if(position != 3)
                        robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_LOW_POS);
                    else
                        robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
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
                .setReversed(false)
                .addTemporalMarker(2.8, ()->{
                    robot.lift.setTargetHeight(0);
                    robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                    robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
                    robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                    robot.intake.setPower(-1);
                })
                .addTemporalMarker(3.3, ()->{
                    robot.intake.setPower(0.9);
                    robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                })
                .lineToLinearHeading(BLUE_WAREHOUSE_TRANSITION_POSE.plus(new Pose2d(0,-8,0)))
                .splineToConstantHeading(BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.plus(new Pose2d(0,-8,0)).vec(), BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.getHeading())
                .splineToLinearHeading(BLUE_WAREHOUSE_POSES[0].plus(new Pose2d(8,-8,0)), BLUE_WAREHOUSE_POSES[0].getHeading())
                .build();

        TrajectorySequence[] regularCycles = new TrajectorySequence[4];
        for(int i = 0; i < 4; i++) {

            regularCycles[i] = robot.drive.trajectorySequenceBuilder(BLUE_WAREHOUSE_POSES[i])
                    .setReversed(true)
                    .splineToConstantHeading(BLUE_WAREHOUSE_TRANSITION_POSE.vec(),BLUE_WAREHOUSE_POSE.getHeading()+Math.PI)
//                    .addTemporalMarker(1.2, ()->{intakeState = 0;})
                    .addTemporalMarker(0.2,()->{
                        robot.lift.setTargetHeight(0.5);
                        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                        robot.intake.setPower(-0.7);;
                    })
                    .addTemporalMarker(0.35, ()->{
                        robot.intake.setPower(-0.7);
                    })
                    .addTemporalMarker(1.3, ()->{
                        robot.intake.setPower(-0.7);
                    })
                    .addTemporalMarker(1.5, ()->{
                        robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    })
                    .addTemporalMarker(1.8, ()->{
                        robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_TELE_OP_EXTEND_POS);
                    })
                    .addTemporalMarker(2.15, ()->robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_BACK_POS))
                    .addTemporalMarker(2.55, ()->{
                        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                    })
                    .addTemporalMarker(2.6, ()->{
                        robot.lift.setTargetHeight(1.5);
                        robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
                    })
                    .splineTo(SECOND_SCORE_POSE.vec(),SECOND_SCORE_POSE.getHeading() + Math.PI)
                    .waitSeconds(0.4)
                    .setReversed(false)
                    .addTemporalMarker(3.5, ()->{
                        robot.lift.setTargetHeight(0);
                        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                        robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
                        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                        robot.intake.setPower(-1);
                    })
                    .addTemporalMarker(4.0, ()->{
                        robot.intake.setPower(0.9);
                        robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    })
                    .lineToLinearHeading(BLUE_WAREHOUSE_TRANSITION_POSE.plus(new Pose2d(0,-4,0)))
                    .splineToConstantHeading(BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.plus(new Pose2d(0,-4,0)).vec(), BLUE_WARE_HOUSE_TRANSITION_POSE_TWO.getHeading())
                    .splineTo(BLUE_WAREHOUSE_POSES[i+1].vec().plus(new Vector2d(6,-4)), BLUE_WAREHOUSE_POSES[i].getHeading())
                    .build();
        }

        robot.drive.setPoseEstimate(START_POSE);

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

        detector.close();

        intakeTimer = new ElapsedTime();
        robot.cap.setPosition(0.65);

        intakeState = 0;
        robot.drive.followTrajectorySequenceAsync(preloadCycle);
        while(!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
            robot.update();
            if(robot.lift.getDistance() < 1.2 && robot.drive.getPoseEstimate().getX() > 30) {
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                robot.intake.setPower(0.0);
            }
        }

        for(int i = 0; i < 4; i++) {
            timer.reset();
            while(timer.milliseconds() < 500 && opModeIsActive()) {
                relocalize();
                robot.drive.setDrivePower(new Pose2d(0.1,0,0));
                if(robot.lift.getDistance() < 1.2 && robot.drive.getPoseEstimate().getX() > 30) {
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.intake.setPower(-0.5);
                }
                robot.update();
            }

            robot.drive.followTrajectorySequenceAsync(regularCycles[i]);
            while(!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
                robot.update();
                if(robot.lift.getDistance() < 1.2 && robot.drive.getPoseEstimate().getX() > 30 && timer.milliseconds() > 2000) {
                    robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    robot.intake.setPower(-0.2);
                }
            }
        }


//        while(!isStopRequested()) {
//            robot.update();
//        }
    }

    public void relocalize() {
        double rightDist = voltageToDistance(rightSensor.getVoltage());
        double frontDist = frontSensor.getVoltage()/0.0032;
        double heading = robot.drive.getPoseEstimate().getHeading();

        if(rightDist > 30) {
            rightDist = 0;
        }

        if(!(frontDist > 50 || frontDist < 5)) {
            double yPos = -72 + rightDist * Math.cos(heading) - rightDistancePos.rotated(heading).getY();
            double xPos = 72 - frontDist * Math.cos(heading) - frontDistancePos.rotated(heading).getX();
            robot.drive.setPoseEstimate(new Pose2d(xPos, yPos, heading));
        } else {
            double yPos = -72 + rightDist * Math.cos(heading) - rightDistancePos.rotated(heading).getY();
            double xPos = 72 - frontDist * Math.cos(heading) - frontDistancePos.rotated(heading).getX();
            robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), yPos, heading));
        }
    }

    public double voltageToDistance(double voltage) {
        return (voltage - 0.142) * 12 / 0.138;
    }
}

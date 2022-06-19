package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous
public class ParkAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        waitForStart();
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build());
    }
}

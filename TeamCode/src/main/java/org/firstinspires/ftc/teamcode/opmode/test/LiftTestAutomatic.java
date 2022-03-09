package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

@TeleOp
@Config
public class LiftTestAutomatic extends LinearOpMode {
    private Lift lift;

    public static double targetHeight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            lift.setTargetHeight(targetHeight);
            lift.update();
            telemetry.addData("Height",lift.getHeight());
            telemetry.update();
        }
    }
}
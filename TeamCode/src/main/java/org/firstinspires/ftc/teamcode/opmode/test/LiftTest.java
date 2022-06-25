package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

@TeleOp
@Config
public class LiftTest extends LinearOpMode {
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            lift.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Height",lift.getHeight());
            telemetry.update();
        }
    }
}

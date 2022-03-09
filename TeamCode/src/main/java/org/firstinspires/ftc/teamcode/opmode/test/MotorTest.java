package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class MotorTest extends LinearOpMode {
    public static String name = "intake";

    private DcMotorEx motor;
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, name);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            motor.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}

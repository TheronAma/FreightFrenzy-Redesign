package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SingleServoTest extends LinearOpMode {
    public static String SERVO_NAME = "door";
    public static double position = 0.5;

    private Servo servo;

    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            servo.setPosition(position);
        }
    }
}

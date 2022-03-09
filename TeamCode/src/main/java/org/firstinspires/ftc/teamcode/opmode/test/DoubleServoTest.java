package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

@Config
@TeleOp
public class DoubleServoTest extends LinearOpMode {
    public static String SERVO_NAME1 = "botLinkage";
    public static double position1 = 0.5;

    public static String SERVO_NAME2 = "topLinkage";
//    public static double position2 = 0.5;

    private Servo servo1;
    private Servo servo2;

    public static double height = 5;

    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, SERVO_NAME1);
        servo2 = hardwareMap.get(Servo.class, SERVO_NAME2);
//        Lift lift = new Lift(hardwareMap, telemetry);
//        lift.setArmPos(LiftConstants.ARM_UP_POS);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
//        lift.setTargetHeight(height);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            servo1.setPosition(position1);
            servo2.setPosition(position1);
//            lift.setTargetHeight(height);
//            lift.update();
        }
    }
}

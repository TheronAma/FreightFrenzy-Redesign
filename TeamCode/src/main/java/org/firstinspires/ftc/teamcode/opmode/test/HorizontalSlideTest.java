package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;
@TeleOp
@Config
public class HorizontalSlideTest extends LinearOpMode {
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gamepadEx1;
        lift = new Lift(hardwareMap, telemetry);

        gamepadEx1 = new GamepadEx(gamepad1);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Horizontal1", lift.getHorizontalPosition1());
            telemetry.addData("Horizontal2", lift.getHorizontalPosition2());
            telemetry.addData("slidey", -gamepad1.left_stick_y);
            telemetry.update();
            lift.setHorizontalPos(-gamepad1.left_stick_y);
                if(gamepadEx1.wasJustPressed(GamepadKeys.Button.A)){
                    lift.setHorizontalPos(0.6);
                }
                if(gamepadEx1.wasJustPressed(GamepadKeys.Button.B)){
                    lift.setHorizontalPos(0.2);
                }

        }
    }
}

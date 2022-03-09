package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

public abstract class BaseOpMode extends LinearOpMode {
    public Robot robot;

    public void initializeHardware() {
        robot.init();
    }

    public void waitTime(int millis) {
        ElapsedTime timer = new ElapsedTime();
        while(!isStopRequested() && opModeIsActive() && timer.milliseconds() < millis) {
            update();
        }
    }

    public void update() {
        robot.update();
    }
}

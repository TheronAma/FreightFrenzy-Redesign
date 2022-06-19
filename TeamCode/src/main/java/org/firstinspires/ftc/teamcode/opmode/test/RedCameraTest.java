package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.RedCapDetector;

@TeleOp
public class RedCameraTest extends LinearOpMode {

    RedCapDetector detector;

    public void runOpMode() throws InterruptedException {
        detector = new RedCapDetector(hardwareMap, "Webcam 1");
        detector.init();
        while(!isStarted() && !isStopRequested()){
            telemetry.addData("Rings",detector.getX());
            telemetry.update();
        }
        detector.close();


    }
}

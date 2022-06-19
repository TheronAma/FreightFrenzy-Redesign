package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.BlueCapDetector;
import org.firstinspires.ftc.teamcode.subsystem.DuckPipeline;

@TeleOp
public class BlueCameraTest extends LinearOpMode {

    BlueCapDetector detector;

    public void runOpMode() throws InterruptedException {
        detector = new BlueCapDetector(hardwareMap, "Webcam 1");
        detector.init();
        detector.setPipeline(new DuckPipeline());
        while(!isStarted() && !isStopRequested()){
            telemetry.addData("Rings",detector.getX());
            telemetry.update();
        }
        detector.close();


    }
}

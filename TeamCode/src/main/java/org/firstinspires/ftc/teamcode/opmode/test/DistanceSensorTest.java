package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;

@TeleOp()
public class DistanceSensorTest extends LinearOpMode {
    private Drivetrain drive;
    private AnalogInput frontSensor, leftSensor, rightSensor;

    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);

        frontSensor = hardwareMap.get(AnalogInput.class, "frontSensor");
        leftSensor = hardwareMap.get(AnalogInput.class, "leftSensor");
        rightSensor = hardwareMap.get(AnalogInput.class, "rightSensor");

        waitForStart();



        while(!isStopRequested() && opModeIsActive()) {
            double frontVoltage = frontSensor.getVoltage();
            double leftVoltage = leftSensor.getVoltage();
            double rightVoltage = rightSensor.getVoltage();

            telemetry.addData("Front Voltage", frontVoltage);
            telemetry.addData("Left Voltage", leftVoltage);
            telemetry.addData("Right Voltage", rightVoltage);

            telemetry.addData("Front Distance" , voltageToDistance(frontVoltage));
            telemetry.addData("Left Distance", voltageToDistance(leftVoltage));
            telemetry.addData("Right Voltage", voltageToDistance(rightVoltage));



            telemetry.update();
        }
    }

    public static double voltageToDistance(double voltage) {
        return (voltage - 0.142) * 12 / 0.138;
    }
}

package org.firstinspires.ftc.teamcode.subsystem.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Lift implements Subsystem {

    public static double kG = 0.3;


    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo turretServo, horizontalServo1, horizontalServo2;

    private AnalogSensor weightSensor;
    private ColorRangeSensor colorRangeSensor;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        motorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight");

        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        horizontalServo1 = hardwareMap.get(Servo.class,"horizontalServo1");
        horizontalServo2 = hardwareMap.get(Servo.class, "horizontalServo2");

        turretServo = hardwareMap.get(Servo.class, "turret");

        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "freightColorSensor");
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void update(TelemetryPacket packet) {

    }


}

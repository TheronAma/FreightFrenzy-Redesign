package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Carousel implements Subsystem {

    CRServo servo1, servo2;

    public Carousel(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "carousel1");
        servo2 = hardwareMap.get(CRServo.class, "carousel2");
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

    public void setPower(double power) {
        servo1.setPower(power);
        servo2.setPower(power);
    }
}

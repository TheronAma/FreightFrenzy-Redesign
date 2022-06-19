package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake implements Subsystem {

    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        //maybe reverse
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getPower() {
        return motor.getPower();
    }

    @Override
    public void init() {}

    @Override
    public void update() {}

    @Override
    public void update(TelemetryPacket packet) {}

    public void setPower(double power){
        motor.setPower(power);
    }
}

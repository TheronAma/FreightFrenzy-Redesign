package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Lift lift;
    public Drivetrain drive;

    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        subsystems = new ArrayList<>();
        subsystems.add((Subsystem) drive);
        subsystems.add((Subsystem) lift);
//        subsystems.add((Subsystem) drive);
//        subsystems.add((Subsystem) drive);
    }

    public void update() {
        for(Subsystem system : subsystems) {
            system.update();
        }
    }
}
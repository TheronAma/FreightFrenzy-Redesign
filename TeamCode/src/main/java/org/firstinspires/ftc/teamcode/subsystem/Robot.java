package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.lift.Lift;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Lift lift;
    public Drivetrain drive;
    public Carousel carousel;
    public Intake intake;
    public Servo cap;

    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        drive = new Drivetrain(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        carousel = new Carousel(hardwareMap);
        intake = new Intake(hardwareMap);
        cap = hardwareMap.get(Servo.class, "cap");

        subsystems = new ArrayList<>();
        subsystems.add(drive);
        subsystems.add(lift);
        subsystems.add(intake);
        subsystems.add(carousel);
    }


    //Initializes hardware - NOTE: moves servos
    public void init() {
        for(Subsystem system : subsystems) {
            system.init();
        }
        cap.setPosition(0.95);
    }

    public void update() {
        for(Subsystem system : subsystems) {
            system.update();
        }
        telemetry.update();
    }
}
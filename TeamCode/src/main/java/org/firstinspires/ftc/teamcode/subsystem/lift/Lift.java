package org.firstinspires.ftc.teamcode.subsystem.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Lift implements Subsystem {

    public static double kG = 0.3;

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private ServoImplEx armServo1;
    private ServoImplEx armServo2;
    private Servo turretServo, horizontalServo1, horizontalServo2;
    private Servo doorServo;

    private AnalogSensor weightSensor;
    private RevColorSensorV3 colorRangeSensor;

    //constants in inches and RPM, everything else unitless
    private static double UP_WINCH_RADIUS = 40./25.4;
    private static double MOTOR_GEAR_RATIO =  1 + 46./11.;
    private static double MOTOR_BASE_RPM = 5960;
    private static double TICKS_PER_REVOLUTION = 28;

    private double MAX_POWER = 1;

    public static double kP = 0.6, kI = 0, kD = 0.01;

    private PIDController pid;

    private int bottomOffset = 0;

    private double targetHeight = 0;
    private double currentHeight = 0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reverse correctly
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        armServo1 = hardwareMap.get(ServoImplEx.class, "botArm");
        armServo2 = hardwareMap.get(ServoImplEx.class, "topArm");
        armServo1.setPwmRange(new PwmControl.PwmRange(500,2500));
        armServo2.setPwmRange(new PwmControl.PwmRange(500,2500));
        armServo2.setDirection(Servo.Direction.REVERSE);



        horizontalServo1 = hardwareMap.get(Servo.class,"botLinkage");
        horizontalServo2 = hardwareMap.get(Servo.class, "topLinkage");
        horizontalServo2.setDirection(Servo.Direction.REVERSE);

        doorServo = hardwareMap.get(Servo.class, "door");

        turretServo = hardwareMap.get(Servo.class, "turret");

        colorRangeSensor = hardwareMap.get(RevColorSensorV3.class, "freightColorSensor");


        bottomOffset = motor2.getCurrentPosition();

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void init() {
        setTargetHeight(0);
        setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        setArmPos(LiftConstants.ARM_READY_POS);
        setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        setDoorPos(LiftConstants.DOOR_READY_POS);
    }

    @Override
    public void update() {
        currentHeight = extensionLengthToHeight(encoderTicksToInches(getCurrentPosition()));
        double power;

        power = 0.2;
        if(targetHeight == 0 && currentHeight < 0.8) {
            setPower(0);
        } else if(motor1.getPower() > 0.5 && motor1.getCurrent(CurrentUnit.AMPS) > 10) {
            setPower(-1);
        } else {
            power += pid.calculate(currentHeight);
            setPower(power);
        }
    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public double getArmPosition() {
        return armServo1.getPosition();
    }

    public double getTurretPosition() {
        return turretServo.getPosition();
    }

    public void open() { }

    public void close() { }

    public double getDistance() {
        return colorRangeSensor.getDistance(DistanceUnit.INCH);
    }

    public void setPower(double power) {
        power = Range.clip(power, -0.3,MAX_POWER);
        motor2.setPower(power);
        motor1.setPower(power);
    }

    public double getHeight() {
        return currentHeight = extensionLengthToHeight(encoderTicksToInches(getCurrentPosition()));
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public int getCurrentPosition() {
        return motor2.getCurrentPosition() - bottomOffset;
    }

    public void setTurretPosition(double turretPosition) {
        turretServo.setPosition(turretPosition);
    }

    //Sets the angle of the turret, starts at 0, CCW is positive and CW is negative, in degrees
    public void setTurretAngle(double angle){
        turretServo.setPosition(TURRET_CENTER_POS + (angle / 180.));
    }

    public void setArmPos(double position){
        armServo1.setPosition(position);
        armServo2.setPosition(position);
    }

    public void setHorizontalPos(double position){
        horizontalServo1.setPosition(position);
        horizontalServo2.setPosition(position);
    }

    public void setDoorPos(double position){
        doorServo.setPosition(position);
    }


    public void setTargetHeight(double height) {
        if(Math.abs(targetHeight - height) < 0.5) {
            return;
        }
        targetHeight = height;
        pid.setSetPoint(targetHeight);
    }

    public static double inchesToEncoderTicks(double inches) {
        return inches / (UP_WINCH_RADIUS * Math.PI) * (TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO);
    }

    public static double encoderTicksToInches(double ticks) {
        return ticks / (TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO) * UP_WINCH_RADIUS * Math.PI;
    }

    public static double extensionLengthToHeight(double length) {
        return length * Math.sin(Math.toRadians(70));
    }

    public static double heightToExtensionLength(double height) {
        return height / Math.sin(Math.toRadians(70));
    }

}

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.genetics.ElitisticListPopulation;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class IntakeTesting extends LinearOpMode {

    private Robot robot;
    private ScoreMode mode;
    private State state;
    private ElapsedTime timer;

    private GamepadEx gamepadEx2, gamepadEx1;

    private boolean lastAPress=false;
    private boolean lastBPress=false;

    private boolean mineralDetected=false;
    CarouselState carouselState;

    private Servo cap;
    private int capState = 0;

    private double multi = 1;
    private double turretMulti = 0.01;
    private double armMulti = 0.01;

    public enum State {
        IDLE,
        INTAKING,
        READY_TO_EXTEND,
        VERTICAL_AND_ROTATE_TURRET,
        EXTEND_ARM,
        OPEN_DOOR,
        RETRACT_ARM,
        RETRACT_VERTICAL_AND_TURRET,
        CAPPING
    }

    public enum ScoreMode {
        HIGH,
        LEFT,
        RIGHT,
        LOW,
        MID
    }

    public enum CarouselState {
        IDLE,
        RIGHT_FAST,
        RIGHT_SLOW,
        LEFT_FAST,
        LEFT_SLOW
    }

    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        state = State.IDLE;
        mode = ScoreMode.HIGH;
        timer = new ElapsedTime();
        gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx1 = new GamepadEx(gamepad1);

        carouselState = CarouselState.IDLE;
        ElapsedTime carouselTimer = new ElapsedTime();


        robot.init();
        robot.cap.setPosition(0.65);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {

            //anti-tip + regular teleop code - ONLY ON PITCH RIGHT NOW
            double pitch = robot.drive.getOrientation().secondAngle;

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepadEx1.getLeftY() * multi,
                            -gamepadEx1.getLeftX() * multi,
                            -gamepadEx1.getRightX() * multi
                    )
            );

            if(gamepadEx2.getRightX() != 0) {
                robot.lift.setTurretPosition(robot.lift.getTurretPosition() - gamepadEx2.getRightX() * turretMulti);
            }

            if(gamepadEx2.getRightY() != 0) {
                robot.lift.setArmPos(robot.lift.getArmPosition() - gamepadEx2.getRightY() * armMulti);
            }

            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                capState++;
                capState %= 5;
            }
            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                capState--;
                capState %= 5;
            }

            switch(capState) {
                case 0:
                    //nothing
                    robot.cap.setPosition(0.65);
                    break;
                case 1:
                    robot.cap.setPosition(0.11);
                    state = State.IDLE;
                    idleTransition();
                    robot.lift.setTargetHeight(0.5);
                    break;
                case 2:
                    robot.cap.setPosition(0.34);
                    state = State.IDLE;
                    idleTransition();
                    robot.lift.setTargetHeight(12);
                    break;
                case 3:
                    robot.cap.setPosition(0.2);
                    state = State.IDLE;
                    idleTransition();
                    break;
                case 4:
                    robot.cap.setPosition(0.35);

            }

//            double forward = (gamepadEx2.getLeftY()-gamepad1.left_stick_y) * multi;
//            double strafe = (gamepadEx2.getLeftX()+gamepad1.left_stick_x) * multi;
//            double turn = (gamepadEx2.getRightX()+gamepad1.right_stick_x) * 0.7 * multi;
//
//            robot.drive.leftFront.setPower(forward + strafe + turn);
//            robot.drive.leftRear.setPower(forward - strafe + turn);
//            robot.drive.rightFront.setPower(forward - strafe - turn);
//            robot.drive.rightRear.setPower(forward + strafe - turn);


//            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.X)){
//                robot.intake.setPower(0.8);;
//            }
//            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)){
//                robot.intake.setPower(-1);
//            }

            switch(carouselState) {
                case IDLE:
//                    robot.carousel.setPower(0);
//                    robot.carousel.setPower((gamepad1.right_trigger + gamepad2.right_trigger - gamepad1.left_trigger - gamepad2.right_trigger)*0.7);

                    if(gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                        carouselState = CarouselState.RIGHT_SLOW;
                        carouselTimer.reset();
//                        robot.carousel.setPower(0.5);
                    }
                    if(gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                        carouselState = CarouselState.LEFT_SLOW;
                        carouselTimer.reset();
//                        robot.carousel.setPower(-0.5);
                    }
                    break;
                case LEFT_SLOW:
                    if(carouselTimer.milliseconds() < 1000) {
                        robot.carousel.setPower(-0.28);
                    }
                    if(carouselTimer.milliseconds() > 1000) {
                        robot.carousel.setPower(-1);
                    }
                    if(carouselTimer.milliseconds() > 1500) {
                        robot.carousel.setPower(0);
                        carouselState = CarouselState.IDLE;
                    }
                    break;
                case RIGHT_SLOW:
                    if(carouselTimer.milliseconds() < 1000) {
                        robot.carousel.setPower(0.28);
                    }
                    if(carouselTimer.milliseconds() > 1000) {
                        robot.carousel.setPower(1);
                    }
                    if(carouselTimer.milliseconds() > 1500) {
                        robot.carousel.setPower(0);
                        carouselState = CarouselState.IDLE;
                    }
                    break;
            }

            if(gamepad2.dpad_up) {
                mode = ScoreMode.HIGH;
            }
            if(gamepad2.dpad_right) {
                mode = ScoreMode.RIGHT;
            }
            if(gamepad2.dpad_left) {
                mode = ScoreMode.LEFT;
            }
            if(gamepad2.dpad_down){
                mode = ScoreMode.LOW;
            }
            if(gamepad2.x) {
                mode = ScoreMode.MID;
            }



            if(gamepadEx2.getLeftY() != 0)
                robot.cap.setPosition(robot.cap.getPosition() + 0.015 * gamepadEx2.getLeftY());



            if(gamepad1.right_bumper) {
                multi = 0.5;
            } else {
                multi = 1;
            }

            switch(state) {
                case IDLE:
//                    robot.intake.setPower(0.5);
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        if(robot.lift.getDistance() < 1.0
                        ) {
                            state = State.READY_TO_EXTEND;
                            readyToExtendTransition();
                        } else {
                            state = State.INTAKING;
                            intakingTransition();
                        }
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {

                        state = State.READY_TO_EXTEND;
                        readyToExtendTransition();
                    }
                    break;
                case INTAKING:
                    robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    if(timer.milliseconds() > 400) {
                        robot.intake.setPower(0.8);;
                        robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);
                    }
                    if(robot.lift.getDistance() < 1.1) {
                        state = State.READY_TO_EXTEND;
                        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                        robot.intake.setPower(0.3);
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.READY_TO_EXTEND;
                        robot.intake.setPower(-0.2);
                        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    break;
                case READY_TO_EXTEND:
                    if(timer.milliseconds() > 1500) {
                        robot.intake.setPower(0);
                    }
                    if(timer.milliseconds() > 600) {
                        readyToExtendTransition();
                        robot.intake.setPower(-0.2);
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.INTAKING;
                        intakingTransition();
                    }
                    break;
            }

            gamepadEx2.readButtons();
            gamepadEx1.readButtons();

            addDebugInfo();
            robot.update();
        }
    }

    public void addDebugInfo() {
        telemetry.addData("State", state);
        telemetry.addData("ScoreMode", mode);
        telemetry.addData("Distance", robot.lift.getDistance());
        telemetry.addData("Orientation", robot.drive.getOrientation());
//        telemetry.addData("Gamepad2 left stick y", gamepadEx2.getLeftY());
//        telemetry.addData("Gamepad2 left stick x", gamepadEx2.getLeftX());
//        telemetry.addData("Gamepad2 right stick x", gamepadEx2.getRightX());
    }

    public void idleTransition() {
        //lift
        robot.lift.setTargetHeight(0.5);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);

        //intake
        robot.intake.setPower(0);
    }

    public void intakingTransition() {
        //lift
        robot.lift.setTargetHeight(0);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_INTAKE_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);

        //intake
        robot.intake.setPower(-1);
        timer.reset();
    }

    public void readyToExtendTransition() {
        //lift
        robot.lift.setTargetHeight(1.5);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        if(mode == ScoreMode.LEFT || mode == ScoreMode.RIGHT) {
            robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_SIDE_POS);
        } else {
            robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_BACK_POS);
        }

        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);

        //intake
        robot.intake.setPower(0.8);;

        timer.reset();
    }


}

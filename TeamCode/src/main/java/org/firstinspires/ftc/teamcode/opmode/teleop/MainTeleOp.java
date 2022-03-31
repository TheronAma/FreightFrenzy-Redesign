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
public class MainTeleOp extends LinearOpMode {

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

    private double multi = 1;

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
        LOW
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
        cap = hardwareMap.get(Servo.class, "cap");

        carouselState = CarouselState.IDLE;
        ElapsedTime carouselTimer = new ElapsedTime();

        robot.init();
        cap.setPosition(1);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepadEx1.getLeftY() * multi,
                            -gamepadEx1.getLeftX() * multi,
                            -gamepadEx1.getRightX() * multi
                    )
            );

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

                    if(gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                        carouselState = CarouselState.RIGHT_SLOW;
                        carouselTimer.reset();
//                        robot.carousel.setPower(0.5);
                    }
                    if(gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        carouselState = CarouselState.LEFT_SLOW;
                        carouselTimer.reset();
//                        robot.carousel.setPower(-0.5);
                    }
                    break;
                case LEFT_SLOW:
                    if(carouselTimer.milliseconds() < 1300) {
                        robot.carousel.setPower(-0.5);
                    }
                    if(carouselTimer.milliseconds() > 1300) {
                        robot.carousel.setPower(-0.5);
                    }
                    if(carouselTimer.milliseconds() > 1700) {
                        robot.carousel.setPower(0);
                        carouselState = CarouselState.IDLE;
                    }
                    break;
                case RIGHT_SLOW:
                    if(carouselTimer.milliseconds() < 1300) {
                        robot.carousel.setPower(0.5);
                    }
                    if(carouselTimer.milliseconds() > 1300) {
                        robot.carousel.setPower(0.5);
                    }
                    if(carouselTimer.milliseconds() > 1700) {
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

//            if(gamepad2.right_bumper) {
//                cap.setPosition(0.6);
//            }
//            if(gamepad2.left_bumper) {
//                cap.setPosition(0.23);
//            }

            cap.setPosition(cap.getPosition() + 0.015 * gamepadEx2.getLeftY());

            if(gamepad1.right_bumper) {
                multi = 0.5;
            } else {
                multi = 1;
            }

            switch(state) {
                case IDLE:
//                    robot.intake.setPower(0.5);
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.INTAKING;
                        intakingTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.RETRACT_VERTICAL_AND_TURRET;
                        retractVerticalAndTurretTransition();
                    }
                    break;
                case INTAKING:
                    if(timer.milliseconds() > 300) {
                        robot.intake.setPower(0.8);;
                        if(robot.lift.getDistance() < 1.5) {
                            state = State.READY_TO_EXTEND;
                            robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                        }
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.READY_TO_EXTEND;
                        robot.intake.setPower(-0.5);
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
                        robot.intake.setPower(-0.5);
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.VERTICAL_AND_ROTATE_TURRET;
                        verticalAndRotateTurretTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.INTAKING;
                        intakingTransition();
                    }
                    break;
                case VERTICAL_AND_ROTATE_TURRET:
                    switch(mode){
                        case LOW:
                        case HIGH:
                            if(timer.milliseconds() > 400) {
                                state = State.EXTEND_ARM;
                                extendArmTransition();
                            }
                            break;
                        case LEFT:
                        case RIGHT:
                            if(timer.milliseconds() > 300) {
                                extendArmTransition();
                                state = State.EXTEND_ARM;
                            }
                            break;
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.EXTEND_ARM;
                        extendArmTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.READY_TO_EXTEND;
                        readyToExtendTransition();
                    }
                    break;
                case EXTEND_ARM:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.OPEN_DOOR;
                        openDoorTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.VERTICAL_AND_ROTATE_TURRET;
                        verticalAndRotateTurretTransition();
                    }
                    break;
                case OPEN_DOOR:
                    switch(mode){
                        case LOW:
                        case HIGH:
                            if(timer.milliseconds() > 400) {
                                state = State.IDLE;
                                idleTransition();
                            }
                            break;
                        case RIGHT:
                        case LEFT:
                            if(timer.milliseconds() > 300) {
                                state = State.RETRACT_ARM;
                                retractArmTransition();
                            }
                            break;
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.RETRACT_ARM;
                        retractArmTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.EXTEND_ARM;
                        extendArmTransition();
                    }
                    break;
                case RETRACT_ARM:
                    switch(mode) {
                        case RIGHT:
                        case LEFT:
                            if(timer.milliseconds() > 250) {
                                state = State.RETRACT_VERTICAL_AND_TURRET;
                                retractVerticalAndTurretTransition();
                            }
                            break;
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.RETRACT_VERTICAL_AND_TURRET;
                        retractVerticalAndTurretTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.OPEN_DOOR;
                        openDoorTransition();
                    }
                    break;
                case RETRACT_VERTICAL_AND_TURRET:
                    switch(mode) {
                        case RIGHT:
                        case LEFT:
                            if (timer.milliseconds() > 400) {
                                state = State.IDLE;
                                idleTransition();
                            }
                            break;
                    }
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A) || gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B) || gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.RETRACT_ARM;
                        retractArmTransition();
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
        robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);

        //intake
        robot.intake.setPower(-1);
        timer.reset();
    }

    public void readyToExtendTransition() {
        //lift
        robot.lift.setTargetHeight(0.5);
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

    public void verticalAndRotateTurretTransition() {
        //lift
        switch(mode) {
            case HIGH:
                robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_TELE_OP_EXTEND_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LOW:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LEFT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_LEFT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_SIDE_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case RIGHT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_RIGHT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_SIDE_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
        }


        //intake
        robot.intake.setPower(0);

        timer.reset();
    }

    public void extendArmTransition() {
        switch(mode) {
            case HIGH:
                robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_TELE_OP_EXTEND_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LOW:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LEFT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_LEFT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_SHARED_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case RIGHT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_RIGHT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_SHARED_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
        }
        timer.reset();
        robot.intake.setPower(0);
    }

    public void openDoorTransition() {
        switch(mode) {
            case HIGH:
                robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_TELE_OP_EXTEND_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_BACK_OUTPUT_SCORE_POS);
                break;
            case LOW:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_HUB_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_BACK_OUTPUT_SCORE_POS);
                break;
            case LEFT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_LEFT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_SHARED_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_SIDE_POS);
                break;
            case RIGHT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_RIGHT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_SCORE_SHARED_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_OPEN_SIDE_POS);
                break;
        }
        timer.reset();
        robot.intake.setPower(0);
    }

    public void retractArmTransition() {
        switch(mode) {
            case HIGH:
                robot.lift.setTargetHeight(LiftConstants.HIGH_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LOW:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case LEFT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_LEFT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
            case RIGHT:
                robot.lift.setTargetHeight(LiftConstants.LOW_HUB_HEIGHT);
                robot.lift.setTurretPosition(LiftConstants.TURRET_RIGHT_POS);
                robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
                robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_LEFT_SHARED_POS);
                robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
                break;
        }
        timer.reset();
        robot.intake.setPower(0);
    }

    public void retractVerticalAndTurretTransition() {
        robot.lift.setTargetHeight(0.5);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_CLOSE_POS);
        timer.reset();
        robot.intake.setPower(0);
    }


}

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class MainTeleOp extends LinearOpMode {

    private Robot robot;
    private ScoreMode mode;
    private State state;
    private ElapsedTime timer;

    private GamepadEx gamepadEx2;

    private boolean lastAPress=false;
    private boolean lastBPress=false;


    public enum State {
        IDLE,
        INTAKING,
        READY_TO_EXTEND,
        VERTICAL_AND_ROTATE_TURRET,
        EXTEND_ARM,
        OPEN_DOOR,
        RETRACT_ARM,
        RETRACT_VERTICAL_AND_TURRET
    }

    public enum ScoreMode {
        BACK,
        LEFT,
        RIGHT
    }

    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        state = State.IDLE;
        mode = ScoreMode.BACK;

        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {

//            robot.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            gamepadEx2.getLeftY(),
//                            gamepadEx2.getLeftX(),
//                            gamepadEx2.getRightX()
//                    )
//            );

            double forward = gamepadEx2.getLeftY();
            double strafe = gamepadEx2.getLeftX();
            double turn = gamepadEx2.getRightX();

            robot.drive.leftFront.setPower(forward + strafe + turn);
            robot.drive.leftRear.setPower(forward - strafe + turn);
            robot.drive.rightFront.setPower(forward - strafe - turn);
            robot.drive.rightRear.setPower(forward + strafe - turn);

            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.X)){
                robot.intake.setPower(1);
            }
            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)){
                robot.intake.setPower(-1);
            }

            switch(state) {
                case IDLE:
//                    robot.intake.setPower(0.5);
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.INTAKING;
                        intakingTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.RETRACT_VERTICAL_AND_TURRET;
                        retractVerticalAndTurretTransition();
                    }
                    break;
                case INTAKING:
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                        state = State.READY_TO_EXTEND;
                        readyToExtendTransition();
                    }
                    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                        state = State.IDLE;
                        idleTransition();
                    }
                    break;
                case READY_TO_EXTEND:
                    break;
                case VERTICAL_AND_ROTATE_TURRET:
                    break;
                case EXTEND_ARM:
                    break;
                case OPEN_DOOR:
                    break;
                case RETRACT_ARM:
                    break;
                case RETRACT_VERTICAL_AND_TURRET:
                    break;
            }
            gamepadEx2.readButtons();

            addDebugInfo();
            robot.update();
        }
    }

    public void addDebugInfo() {
        telemetry.addData("State", state);
        telemetry.addData("ScoreMode", mode);
//        telemetry.addData("Gamepad2 left stick y", gamepadEx2.getLeftY());
//        telemetry.addData("Gamepad2 left stick x", gamepadEx2.getLeftX());
//        telemetry.addData("Gamepad2 right stick x", gamepadEx2.getRightX());
    }

    public void idleTransition() {
        //lift
        robot.lift.setTargetHeight(0);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_READY_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_RETRACT_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);

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
        robot.intake.setPower(1);
    }

    public void readyToExtendTransition() {
        //lift
        robot.lift.setTargetHeight(1.5);
        robot.lift.setTurretPosition(LiftConstants.TURRET_CENTER_POS);
        robot.lift.setArmPos(LiftConstants.ARM_UP_POS);
        robot.lift.setHorizontalPos(LiftConstants.HORIZONTAL_READY_POS);
        robot.lift.setDoorPos(LiftConstants.DOOR_READY_POS);

        //intake
        robot.intake.setPower(-0.5);
    }

    public void verticalAndRotateTurretTransition() {

    }

    public void extendArmTransition() {

    }

    public void openDoorTransition() {

    }

    public void retractArmTransition() {

    }

    public void retractVerticalAndTurretTransition() {

    }


}

package org.firstinspires.ftc.teamcode.hw;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public enum LiftPosition {
        DOWN, HIGH_BASKET, MOVING
    }

    public enum SlidePosition {
        DOWN, BASKET,
    }

    public static LiftPosition liftPosition = LiftPosition.DOWN;
    public static SlidePosition slidePosition = SlidePosition.DOWN;
    public Telemetry telemetry;
    public static boolean liftAtTargetPosition = false;
    public static boolean slideAtTargetPosition = false;
    public static int liftTargetPosition = 0;
    public static int slideTargetPosition = 0;
    public static int liftEncoderValue = 0;
    public static int slideEncoderValue = 0;
    public static double liftPower = 0;
    public static double slidePower = 0;
    public static boolean liftAtBottom = false;
    public static boolean slideAtBottom = false;
    private DcMotor rightLift;
    private Servo slide;
    private TouchSensor bottomSwitch;
    private TouchSensor slideSwitch;

    public Lift(DcMotor rightLift, Servo slide, TouchSensor bottomSwitch, TouchSensor slideSwitch, Telemetry telemetry) {

        this.rightLift = rightLift;
        this.slide = slide;
        this.bottomSwitch = bottomSwitch;
        this.slideSwitch = slideSwitch;
        this.telemetry = telemetry;

        initMotor(rightLift, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateState(Gamepad gamepad1, Gamepad gamepad2) {

        if(liftAtBottom || gamepad2.x) {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad2.b) {
            rightLift.setPower(-0.8);
        }

        if(gamepad2.left_stick_y < -0.1) {
            moveLift(LiftPosition.HIGH_BASKET, 1);
        } else if(gamepad2.left_stick_y > 0.1) {
            moveLift(LiftPosition.DOWN, 1);
        }

        if(gamepad2.dpad_up) {
            slide.setPosition(0.85);
        } else if(gamepad2.dpad_right) {
            slide.setPosition(0);
        } else if(gamepad2.dpad_down) {
            slide.setPosition(0.12);
        }

        if(gamepad2.left_stick_button) {
            moveLift(LiftPosition.DOWN, 1);
            slide.setPosition(0.12);
        }

        telemetry.addData("Lift position", liftPosition);
        telemetry.addData("Lift target position", liftTargetPosition);
        telemetry.addData("Lift encoder value", liftEncoderValue);
        telemetry.addData("Lift at target position", liftAtTargetPosition);
        telemetry.addData("Lift at bottom", liftAtBottom);
        telemetry.addData("Lift power", liftPower);
        telemetry.addData("Slide position", slidePosition);
        telemetry.addData("Slide encoder value", slideEncoderValue);
        telemetry.addData("slide target position", slideTargetPosition);
        telemetry.addData("Slide at target position", slideAtTargetPosition);
        telemetry.addData("Slide power", slidePower);
        telemetry.addData("slide position", slide.getPosition());
        telemetry.addData("lift busy", rightLift.isBusy());
        telemetry.addData("lift run mode", rightLift.getMode());
        // telemetry.addData("slide distance to target", getSlideDistanceToTarget());

        updateLift();
    }

    public void moveLift(LiftPosition position, double power) {

        if(position == LiftPosition.HIGH_BASKET) {
            rightLift.setTargetPosition(2250);
        } else if(position == LiftPosition.DOWN && Intake.getIntakePosition() == Intake.IntakePosition.DOWN) {
            rightLift.setTargetPosition(0);
        }

        rightLift.setPower(power);
    }

    public void moveSlideTo(SlidePosition position) {

        switch(position) {
            case BASKET: slide.setPosition(0.8);
            break;
            case DOWN: slide.setPosition(0.12);
            break;
        }
    }

    public void updateLift() {
        slideAtBottom = slideSwitch.isPressed();
        liftAtBottom = bottomSwitch.isPressed();
        liftEncoderValue = rightLift.getCurrentPosition();
    }

    private void initMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode runMode) {

        motor.setTargetPosition(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }


}

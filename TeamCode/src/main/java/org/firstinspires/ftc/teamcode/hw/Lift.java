package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public enum LiftPosition {
        DOWN, ASCENT, HIGH_BASKET, MOVING
    }

    public enum SlidePosition {
        DOWN, INTAKE, ASCENT, BASKET, MOVING
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
    public static DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor slide;
    private TouchSensor bottomSwitch;
    private TouchSensor slideSwitch;

    public Lift(DcMotor leftLift, DcMotor rightLift, DcMotor slide, TouchSensor bottomSwitch, TouchSensor slideSwitch, Telemetry telemetry) {

        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.slide = slide;
        this.bottomSwitch = bottomSwitch;
        this.slideSwitch = slideSwitch;
        this.telemetry = telemetry;

        initMotor(leftLift, DcMotor.Direction.REVERSE);
        initMotor(rightLift, DcMotor.Direction.FORWARD);
        initMotor(slide, DcMotor.Direction.REVERSE);
    }

    public void updateState(Gamepad gamepad1, Gamepad gamepad2) {

        updateLift();

        if(slideAtBottom) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(liftAtBottom) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad2.b) {
            leftLift.setPower(-0.8);
            rightLift.setPower(-0.8);
            slide.setPower(-0.3);
        }

        if(gamepad2.x) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad2.left_stick_y < 0) {
            moveLiftTo(LiftPosition.HIGH_BASKET, -gamepad2.left_stick_y);
        } else if(gamepad2.left_stick_y >= 0 && !gamepad2.b) {
            moveLiftTo(LiftPosition.DOWN, gamepad2.left_stick_y);
        }

        if(gamepad2.right_stick_y < 0) {
            moveSlideTo(SlidePosition.BASKET, -gamepad2.right_stick_y * 0.6);
        } else if(gamepad2.right_stick_y >= 0 && !gamepad2.b) {
            moveSlideTo(SlidePosition.DOWN, gamepad2.right_stick_y * 0.6);
        }

        telemetry.addData("Lift position", liftPosition);
        telemetry.addData("Lift target position", liftTargetPosition);
        telemetry.addData("Lift encoder value", liftEncoderValue);
        telemetry.addData("Lift at target position", liftAtTargetPosition);
        telemetry.addData("Lift at bottom", liftAtBottom);
        telemetry.addData("Lift power", liftPower);
        telemetry.addData("Lift distance to target", getLiftDistanceToTarget());
        telemetry.addData("Slide position", slidePosition);
        telemetry.addData("Slide encoder value", slideEncoderValue);
        telemetry.addData("slide target position", slideTargetPosition);
        telemetry.addData("Slide at target position", slideAtTargetPosition);
        telemetry.addData("Slide power", slidePower);
        telemetry.addData("slide distance to target", getSlideDistanceToTarget());

        updateLift();
    }

    public double setLiftPower(double power) {

        double direction = Math.signum(power);

        if(getLiftDistanceToTarget() < 10) {
            power = 0;
        } else if (getLiftDistanceToTarget() < 300) {
            power *= 0.4;
        }

        leftLift.setPower(power);
        rightLift.setPower(power);

        return power;
    }

    public void moveLiftTo(LiftPosition targetPosition, double power) {

        switch(targetPosition) {
            case DOWN:
                leftLift.setTargetPosition(0);
                liftTargetPosition = 0;
                break;
            case ASCENT:
                leftLift.setTargetPosition(1000);// 1000 is estimate
                liftTargetPosition = 1000;
                break;
            case HIGH_BASKET:
                leftLift.setTargetPosition(2300);
                liftTargetPosition = 2300;
                break;
        }

        if(liftEncoderValue > liftTargetPosition) {
            power *= -1;
        }

        liftPower = setLiftPower(power);

        if(getLiftDistanceToTarget() < 10) {
            liftAtTargetPosition = true;
            liftPosition = targetPosition;
        } else {
            liftAtTargetPosition = false;
            liftPosition = LiftPosition.MOVING;
        }
    }

    public double setSlidePower(double power) {

        double direction = Math.signum(power);

        if(getSlideDistanceToTarget() < 20) {
            power = 0;
        } else if(getSlideDistanceToTarget() < 300) {
            power *= getSlideDistanceToTarget() / 1000.0;
            if(Math.abs(power) < 0.2) {power = 0.2 * direction;}
        }

        slide.setPower(power);

        return power;
    }

    public void moveSlideTo(SlidePosition targetPosition, double power) {

        switch(targetPosition) {
            case DOWN:
                slide.setTargetPosition(0);
                slideTargetPosition = 0;
                break;
            case INTAKE:
                slide.setTargetPosition(150);
                slideTargetPosition = 150;
                break;
            case ASCENT:
                slide.setTargetPosition(700); // 700 is estimate
                slideTargetPosition = 700;
                break;
            case BASKET:
                slide.setTargetPosition(1400);
                slideTargetPosition = 1400;
                break;
        }

        if(slide.getCurrentPosition() > slide.getTargetPosition()) {
            power *= -1;
        }

        slidePower = setSlidePower(power);

        if(getSlideDistanceToTarget() < 50) {
            slideAtTargetPosition = true;
            slidePosition = targetPosition;
        } else {
            slideAtTargetPosition = false;
            slidePosition = SlidePosition.MOVING;
        }
    }

    public void updateLift() {
        slideAtBottom = slideSwitch.isPressed();
        liftAtBottom = bottomSwitch.isPressed();
        liftEncoderValue = leftLift.getCurrentPosition();
        slideEncoderValue = slide.getCurrentPosition();
    }


    public int getLiftDistanceToTarget() {
        return Math.abs(leftLift.getCurrentPosition() - leftLift.getTargetPosition());
    }

    public int getSlideDistanceToTarget() {
        return Math.abs(slide.getCurrentPosition() - slide.getTargetPosition());
    }

    public int getSlideEncoderValue() {
        return slide.getCurrentPosition();
    }


    private void initMotor(DcMotor motor, DcMotor.Direction direction) {

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);
    }


}

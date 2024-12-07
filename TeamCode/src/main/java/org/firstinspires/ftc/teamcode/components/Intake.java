package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotComponent{

    public Telemetry telemetry;

    public Intake(CRServo intakeLift, CRServo spinner, TouchSensor upStop, TouchSensor bottomStop, Servo topLeftExtender, Servo topRightExtender, Servo bottomLeftExtender, Servo bottomRightExtender, Telemetry telemetry) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;
        this.upStop = upStop;
        this.bottomStop = bottomStop;
        this.telemetry = telemetry;
        this.topLeftExtender = topLeftExtender;
        this.topRightExtender = topRightExtender;
        this.bottomLeftExtender = bottomLeftExtender;
        this.bottomRightExtender = bottomRightExtender;
        intakeLift.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public enum IntakePosition {
        DOWN, MOVING, UP
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad.y) {
            moveIntake(1);
        } else if(gamepad.a) {
            moveIntake(-1);
        } else {
            moveIntake(0);
        }

        if(gamepad2.left_bumper) {
            spinner.setPower(-1);
        } else if(gamepad2.right_bumper)  {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }

        if(gamepad2.dpad_down) {
            intakeLift.setPower(-1);
            extendIntake(0.2, 0.4, 0.35, 0.9);
            intakeLift.setPower(0);
        } else if(gamepad2.dpad_up) {
            intakeLift.setPower(1);
            extendIntake(0.35, 0.7, 0.2, 0.6);
            intakeLift.setPower(0);
        }

        telemetry.addData("top intake button", upStop.isPressed());
        telemetry.addData("bottom intake button", bottomStop.isPressed());
        telemetry.addData("spinner power: ", spinner.getPower());
        telemetry.addData("top left intake extender", topLeftExtender.getPosition());
        telemetry.addData("top right intake extender", topRightExtender.getPosition());
        telemetry.addData("bottom left intake extender", bottomLeftExtender.getPosition());
        telemetry.addData("bottom right intake extender", bottomRightExtender.getPosition());

        if(bottomStop.isPressed()) {
            intakePosition = IntakePosition.DOWN;
        } else if(upStop.isPressed()) {
            intakePosition = IntakePosition.UP;
        } else {
            intakePosition = IntakePosition.MOVING;
        }
    }

    public boolean moveIntake(double power) {

        boolean atPosition = false;

        if(upStop.isPressed() && power >= 0) {
            power = 0;
            atPosition = true;
        } else if(bottomStop.isPressed() && power < 0) {
            power = 0;
            atPosition = true;
        }

        intakeLift.setPower(power);

        intakeLift.setPower(power);
        telemetry.addData("intake lift power", intakeLift.getPower());
        // telemetry.update();

        return atPosition;

    }

    public void moveToPosition(IntakePosition targetPosition) {

        if(targetPosition == IntakePosition.DOWN) {
            moveIntake(-1);
        } else if(targetPosition == IntakePosition.UP) {
            moveIntake(1);
        }

    }

    public void extendIntake(double topLeftPosition, double topRightPosition, double bottomLeftPosition, double bottomRightPosition) {

        topLeftExtender.setPosition(topLeftPosition);
        topRightExtender.setPosition(topRightPosition);
        bottomLeftExtender.setPosition(bottomLeftPosition);
        bottomRightExtender.setPosition(bottomRightPosition);
    }

    public void spin(double power) {
        spinner.setPower(power);
    }

    private CRServo intakeLift;
    private CRServo spinner;
    private TouchSensor upStop;
    private TouchSensor bottomStop;
    private Servo topLeftExtender;
    private Servo topRightExtender;
    private Servo bottomLeftExtender;
    private Servo bottomRightExtender;
    private IntakePosition intakePosition = IntakePosition.UP;

}

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotComponent{

    public Telemetry telemetry;

    public Intake(CRServo intakeLift, CRServo spinner, TouchSensor upStop, TouchSensor bottomStop, Servo leftExtender, Servo rightExtender, Telemetry telemetry) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;
        this.upStop = upStop;
        this.bottomStop = bottomStop;
        this.telemetry = telemetry;
        this.leftExtender = leftExtender;
        this.rightExtender = rightExtender;
        intakeLift.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public enum IntakePosition {
        DOWN, MOVING, UP
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad2.y) {
            moveIntake(1);
        } else if(gamepad2.a) {
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

        if(gamepad.dpad_down) {
            intakeLift.setPower(-1);
            extendIntake(0.1, 0.7);
            intakeLift.setPower(0);
        } else if(gamepad.dpad_up) {
            intakeLift.setPower(1);
            extendIntake(0.35, 0.5);
            intakeLift.setPower(0);
        }

        telemetry.addData("top intake button", upStop.isPressed());
        telemetry.addData("bottom intake button", bottomStop.isPressed());
        telemetry.addData("left intake extender", leftExtender.getPosition());
        telemetry.addData("right intake extender", rightExtender.getPosition());

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

    public void extendIntake(double leftPosition, double rightPosition) {

        leftExtender.setPosition(leftPosition);
        rightExtender.setPosition(rightPosition);
    }

    private CRServo intakeLift;
    private CRServo spinner;
    private TouchSensor upStop;
    private TouchSensor bottomStop;
    private Servo leftExtender;
    private Servo rightExtender;
    private IntakePosition intakePosition = IntakePosition.UP;

}

package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotComponent {

    public enum IntakePosition {
        DOWN, UP, MOVING
    }

    public Telemetry telemetry;
    public static IntakePosition intakePosition = IntakePosition.UP;
    public IntakePosition intakeTargetPosition = IntakePosition.UP;
    public static double intakeLiftPower = 0;
    public static double spinnerPower = 0;
    public static double intakeExtensionPower = 0;
    public boolean intakeAtTargetPosition = false;
    private DcMotor intakeLift;
    private CRServo spinner;
    private CRServo leftExtender;
    private CRServo rightExtender;
    private TouchSensor upStop;
    private TouchSensor bottomStop;
    private TouchSensor retractStop;
    private TouchSensor extendStop;


    public Intake(DcMotor intakeLift, CRServo spinner, CRServo leftExtender, CRServo rightExtender, TouchSensor upStop, TouchSensor bottomStop, TouchSensor retractStop, TouchSensor extendStop, Telemetry telemetry) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;
        this.leftExtender = leftExtender;
        this.rightExtender = rightExtender;
        this.upStop = upStop;
        this.bottomStop = bottomStop;
        this.retractStop = retractStop;
        this.extendStop = extendStop;
        this.telemetry = telemetry;

        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtender.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updateState(Gamepad gamepad1, Gamepad gamepad2) {


        if(upStop.isPressed()) {
            intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad1.a) {
            moveIntakeTo(IntakePosition.DOWN, 0.3);
        } else if (gamepad1.y) {
            moveIntakeTo(IntakePosition.UP, 0.6);
        } else if(!gamepad2.b) {
            setIntakeLiftPower(0);
        }

        if(gamepad2.left_bumper) {
            setSpinnerPower(1);
        } else if(gamepad2.right_bumper) {
            setSpinnerPower(-1);
        } else {
            setSpinnerPower(0);
        }

        if(gamepad2.y) {
            extendIntake(1);
        } else {
            extendIntake(-1);
        }


        telemetry.addData("Intake position: ", intakePosition);
        telemetry.addData("Intake encoder value: ", intakeLift.getCurrentPosition());
        telemetry.addData("Intake at target position: ", intakeAtTargetPosition);
        telemetry.addData("Intake power: ", intakeLiftPower);
        telemetry.addData("Spinner power", spinnerPower);
        telemetry.addData("gamepad y", gamepad1.y);
        telemetry.addData("gamepad a", gamepad1.a);
        telemetry.addData("up stop pressed: ", upStop.isPressed());
        telemetry.addData("bottom stop pressed: ", bottomStop.isPressed());
        telemetry.addData("retract stop pressed: ", retractStop.isPressed());
        telemetry.addData("extend stop pressed: ", extendStop.isPressed());

    }

    private void setIntakeLiftPower(double power) {

        if(upStop.isPressed() && power < 0) {
            power = 0;
        } else if(bottomStop.isPressed() && power > 0) {
            power = 0;
        }

        intakeLift.setPower(power);
        intakeLiftPower = power;
    }

    public void moveIntakeTo(IntakePosition targetPosition, double power) {

        if(upStop.isPressed()) {
            intakePosition = IntakePosition.UP;
        } else if(bottomStop.isPressed()) {
            intakePosition = IntakePosition.DOWN;
        } else {
            intakePosition = IntakePosition.MOVING;
        }

        if (intakePosition != IntakePosition.UP && targetPosition == IntakePosition.UP) {
            power *= -1;
        }

        if(intakePosition == IntakePosition.UP && power < 0) {
            power = 0;
        } else if(intakePosition == IntakePosition.DOWN && power > 0) {
            power = 0;
        }

        setIntakeLiftPower(power);
    }

    public void extendIntake(double power) {

        if(power < 0 && retractStop.isPressed()) {
            power = 0;
        } else if(power > 0 && extendStop.isPressed()) {
            power = 0;
        }

        leftExtender.setPower(power);
        rightExtender.setPower(power);
        intakeExtensionPower = power;
    }

    public void setSpinnerPower(double power) {
        spinner.setPower(power);
        spinnerPower = power;
    }

    private int getIntakeDistanceToTarget() {
        return Math.abs(intakeLift.getCurrentPosition() - intakeLift.getTargetPosition());
    }

}

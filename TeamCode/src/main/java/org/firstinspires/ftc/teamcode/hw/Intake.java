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

        if(gamepad2.b) {
            intakeLift.setPower(-1);
        }

        if(gamepad2.x) {
            intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(upStop.isPressed()) {
            intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad1.a) {
            moveIntakeTo(IntakePosition.DOWN, 0.2);
        } else if (gamepad1.y) {
            moveIntakeTo(IntakePosition.UP, 0.7);
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
        } else if(gamepad2.a) {
            extendIntake(-1);
            intakeLift.setPower(-0.1);
        } else {
            extendIntake(0);
        }


        telemetry.addData("Intake position: ", intakePosition);
        telemetry.addData("Intake encoder value: ", intakeLift.getCurrentPosition());
        telemetry.addData("Intake at target position: ", intakeAtTargetPosition);
        telemetry.addData("Intake power: ", intakeLiftPower);
        telemetry.addData("Spinner power", spinnerPower);
        telemetry.addData("bottom stop pressed: ", bottomStop.isPressed());
        telemetry.addData("gamepad y", gamepad1.y);
        telemetry.addData("retract stop pressed: ", retractStop.isPressed());
        telemetry.addData("extend stop pressed: ", extendStop.isPressed());

    }

    private void setIntakeLiftPower(double power) {

        double direction = Math.signum(power);

        if(upStop.isPressed() && power < 0) {
            power = 0;
        } else if(bottomStop.isPressed() && power > 0) {
            power = 0;
        }

        if(intakeTargetPosition == IntakePosition.DOWN && bottomStop.isPressed()) {
            power = 0;
        } else if(intakeTargetPosition == IntakePosition.UP && upStop.isPressed()) {
            power = 0;
        }

        if(intakeLift.getCurrentPosition() > 150 && !bottomStop.isPressed() && 0 < power) {
            power = -0.1;
        }

        if(intakeLift.getCurrentPosition() < 250 && power < -0.5) {
            power = -0.45;
        }

        intakeLift.setPower(power);
        intakeLiftPower = power;
    }

    public void moveIntakeTo(IntakePosition targetPosition, double power) {

        switch(targetPosition) {
            case DOWN:
                intakeLift.setTargetPosition(5000); // 1000 is estimate
                intakeTargetPosition = IntakePosition.DOWN;
                break;
            case UP:
                intakeLift.setTargetPosition(0);
                intakeTargetPosition = IntakePosition.UP;
                break;
            default:
                intakeLift.setTargetPosition(intakeLift.getCurrentPosition());
            break;
        }

        if (intakeLift.getCurrentPosition() > intakeLift.getTargetPosition()) {
            power *= -1;
        }

        if((intakeTargetPosition == IntakePosition.DOWN && bottomStop.isPressed()) || (intakeTargetPosition == IntakePosition.UP && upStop.isPressed()) || (intakeLift.getCurrentPosition() < 10 && targetPosition == IntakePosition.UP))  {
            intakeAtTargetPosition = true;
            intakePosition = targetPosition;
        } else {
            intakeAtTargetPosition = false;
            intakePosition = IntakePosition.MOVING;
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

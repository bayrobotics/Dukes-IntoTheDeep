package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public Telemetry telemetry;

    public Lift(DcMotor leftLift,
                DcMotor rightLift,
                Telemetry telemetry) throws InterruptedException {

        this.telemetry = telemetry;
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        initMotor(this.leftLift, DcMotor.Direction.REVERSE);
        initMotor(this.rightLift, DcMotor.Direction.FORWARD);

    }

    public enum LiftState {
        MANUAL, PRESET
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if (gamepad2.dpad_left) {
            count++;
        }

        if (count % 2 == 0) {
            liftState = LiftState.MANUAL;
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            liftState = LiftState.PRESET;
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(bottomSwitch.isPressed()) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(liftState == LiftState.MANUAL) {
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if(liftState == LiftState.PRESET) {
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        moveLift(gamepad, gamepad2);
        telemetry.addData("Lift encoder value: ", leftLift.getCurrentPosition());
        telemetry.update();
    }

    public void moveLift(Gamepad gamepad, Gamepad gamepad2) {

        if (liftState == LiftState.MANUAL) {
            if (gamepad2.dpad_up && leftLift.getCurrentPosition() < 2380) {
                setLiftPower(1);
            } else if (gamepad2.dpad_down && !bottomSwitch.isPressed()) {
                setLiftPower(-1);
            } else {
                setLiftPower(0);
            }
        } else if(liftState == LiftState.PRESET) {
            if(gamepad2.dpad_up) {
                leftLift.setTargetPosition(2380);
                moveUntilInterrupt(gamepad2);
            } else if(gamepad2.dpad_right) {
                leftLift.setTargetPosition(1500); // 1500 is estimate
                moveUntilInterrupt(gamepad2);
            } else if(gamepad2.dpad_down) {
                leftLift.setTargetPosition(0);
                moveUntilInterrupt(gamepad2);
            }
        }
    }

    public void setLiftPower(double power) {

        leftLift.setPower(power);
        rightLift.setPower(power);

    }

    public void moveUntilInterrupt(Gamepad gamepad2) {
        setLiftPower(1);
        if(leftLift.isBusy()) {
            if(gamepad2.dpad_left || bottomSwitch.isPressed()) {
                setLiftPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        setLiftPower(0);
    }

    private LiftState liftState;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private int count = 0;
    private TouchSensor bottomSwitch;


    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) throws InterruptedException {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(100);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }
}
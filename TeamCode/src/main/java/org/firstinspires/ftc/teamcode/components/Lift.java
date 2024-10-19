package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    public enum LiftMode {
        MANUAL, PRESET
    }

    public enum LiftState {
        STOPPED, MOVING
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(bottomSwitch.isPressed()) leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(liftMode == LiftMode.PRESET) leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else if(liftMode == LiftMode.MANUAL) leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if(gamepad2.dpad_left) {

            if(liftMode == LiftMode.MANUAL) liftMode = LiftMode.PRESET;
            else if(liftMode == LiftMode.PRESET) liftMode = LiftMode.MANUAL;

            if(liftState == LiftState.MOVING) setLiftPower(0);

        } else if(gamepad2.dpad_up) {

            if(liftMode == LiftMode.MANUAL) setLiftPower(1);
            else if(liftMode == LiftMode.PRESET) moveTo(gamepad2, 2380);

        } else if(gamepad2.dpad_right) {

            if(liftMode == LiftMode.PRESET) moveTo(gamepad2, 1500); // 1500 is estimate

        } else if(gamepad2.dpad_down) {

            if(liftMode == LiftMode.MANUAL) setLiftPower(-1);
            else if(liftMode == LiftMode.PRESET) moveTo(gamepad2, 0);

        } else {

            if(liftMode == LiftMode.MANUAL) setLiftPower(0);

        }

        telemetry.addData("Lift encoder value: ", leftLift.getCurrentPosition());
        telemetry.addData("Lift Mode", liftMode);
        telemetry.addData("Lift State", liftState);
        telemetry.update();

    }


    public void setLiftPower(double power) {

        if(power < 0 && bottomSwitch.isPressed()) power = 0;
        else if(power > 0 && leftLift.getCurrentPosition() > 2380) power = 0;

        leftLift.setPower(power);
        rightLift.setPower(power);

        if(power != 0) liftState = liftState.MOVING;
        else liftState = liftState.STOPPED;

    }

    public void moveTo(Gamepad gamepad2, int target) {

        leftLift.setTargetPosition(target);
        setLiftPower(1);
        if(!leftLift.isBusy()) setLiftPower(0);

    }

    private LiftMode liftMode = LiftMode.MANUAL;
    private LiftState liftState = LiftState.STOPPED;
    private DcMotor leftLift;
    private DcMotor rightLift;
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
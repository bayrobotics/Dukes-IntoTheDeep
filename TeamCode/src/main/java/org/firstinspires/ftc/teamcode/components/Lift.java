package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public Telemetry telemetry;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private int count = 0;

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
        moveLift(gamepad, gamepad2);

        telemetry.addData("Lift encoder value: ", leftLift.getCurrentPosition());
        telemetry.update();
    }

    public void moveLift(Gamepad gamepad, Gamepad gamepad2) {

        LiftState liftState;

        if (gamepad2.x) {
            count++;
        }

        if (count % 2 == 0) {
            liftState = LiftState.MANUAL;
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            liftState = LiftState.PRESET;
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


        if (liftState == LiftState.MANUAL) {
            if (gamepad2.y && leftLift.getCurrentPosition() < 2380 && !gamepad2.x) {
                setLiftPower(1);
            } else if (gamepad2.b && leftLift.getCurrentPosition() > 0 && !gamepad2.x) {
                setLiftPower(-1);
            } else {
                setLiftPower(0);
            }
        } else if(liftState == LiftState.PRESET) {
            if(gamepad2.y && !gamepad2.x) {
                leftLift.setTargetPosition(2380);
                goToTargetPosition();
            } else if(gamepad2.b && !gamepad2.x) {
                leftLift.setTargetPosition(1500);
                goToTargetPosition();
            } else if(gamepad2.a && !gamepad2.x) {
                leftLift.setTargetPosition(0);
                goToTargetPosition();
            }
        }

    }

    public void setLiftPower(double power) {

        leftLift.setPower(power);
        rightLift.setPower(power);

    }

    public void goToTargetPosition() {

        if(leftLift.getCurrentPosition() > leftLift.getTargetPosition() + 10) {
            setLiftPower(-1);
        } else if(leftLift.getCurrentPosition() < leftLift.getTargetPosition() - 10) {
            setLiftPower(1);
        } else {
            setLiftPower(0);
        }

    }

    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) throws InterruptedException {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(100);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }
}
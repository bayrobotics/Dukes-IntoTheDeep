package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public Telemetry telemetry;
    private DcMotor leftLift;
    private DcMotor rightLift;

    public Lift(DcMotor leftLift,
                DcMotor rightLift,
                Telemetry telemetry) throws InterruptedException {

        this.telemetry = telemetry;
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        initMotor(this.leftLift, DcMotor.Direction.FORWARD);
        initMotor(this.rightLift, DcMotor.Direction. REVERSE);

    }

    public enum LiftState {
        DOWN, LOW, HIGH
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {
        manualMoveLift(gamepad, gamepad2);

        telemetry.addData("Lift encoder value: ", leftLift.getCurrentPosition());
        telemetry.update();
    }

    public void manualMoveLift(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad.y && leftLift.getCurrentPosition() < 800) {
            leftLift.setPower(-0.6);
            rightLift.setPower(-0.6);
        } else if (gamepad.x && leftLift.getCurrentPosition() > 0) {
            leftLift.setPower(0.6);
            rightLift.setPower(0.6);
        } else {
            leftLift.setPower(0);
            rightLift.setPower(0);
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
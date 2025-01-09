package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis extends RobotComponent {

    public Telemetry telemetry;
    private double speedMultiplier = 1.0;
    private double frontLeftPower = 0.0;
    private double frontRightPower = 0.0;
    private double backRightPower = 0.0;
    private double backLeftPower = 0.0;
    private boolean xPressedPrevious = false;
    private boolean rotationFlipped = false;
    private SpeedMode selectedSpeedMode = SpeedMode.NORMAL;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public enum SpeedMode {SLOW, NORMAL, FAST}

    public Chassis(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor,
                   Telemetry telemetry) {

        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.telemetry = telemetry;

        initMotor(frontLeftMotor, DcMotor.Direction.REVERSE);
        initMotor(frontRightMotor, DcMotor.Direction.FORWARD);
        initMotor(backLeftMotor, DcMotor.Direction.REVERSE);
        initMotor(backRightMotor, DcMotor.Direction.FORWARD);
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        double lateral = gamepad.left_stick_x;
        double forward = gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;
        if(rotationFlipped) {rotation *= -1;}

        if(gamepad.x && !xPressedPrevious) {

            changeMotorDirection(frontLeftMotor);
            changeMotorDirection(frontRightMotor);
            changeMotorDirection(backLeftMotor);
            changeMotorDirection(backRightMotor);

            if(rotationFlipped) {
                rotationFlipped = false;
            } else {
                rotationFlipped = true;
            }

            xPressedPrevious = true;
        } else if(!gamepad.x) {
            xPressedPrevious = false;
        }

        if (gamepad.left_bumper && !gamepad.right_bumper) {
            selectedSpeedMode = SpeedMode.SLOW;
        } else if (gamepad.right_bumper && !gamepad.left_bumper) {
            selectedSpeedMode = SpeedMode.FAST;
        } else {
            selectedSpeedMode = SpeedMode.NORMAL;
        }

        if(Lift.liftEncoderValue > 600) {
            selectedSpeedMode = SpeedMode.SLOW;
        }

        setSpeedMode(selectedSpeedMode);

        frontLeftPower = speedMultiplier * (forward - lateral - rotation);
        frontRightPower = speedMultiplier * (forward + lateral + rotation);
        backRightPower = speedMultiplier * (forward - lateral + rotation);
        backLeftPower = speedMultiplier * (forward + lateral - rotation);

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void setSpeedMode(SpeedMode speedMode) {

        switch (speedMode) {
            case SLOW:
                this.speedMultiplier = 0.4;
                break;
            case NORMAL:
                this.speedMultiplier = 0.7;
                break;
            case FAST:
                this.speedMultiplier = 1.0;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + speedMode);
        }
    }


    private void changeMotorDirection(DcMotor motor) {

        if (motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) {

        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);
    }
}
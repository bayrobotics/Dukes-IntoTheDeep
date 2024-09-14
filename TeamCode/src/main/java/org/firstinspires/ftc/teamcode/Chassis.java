package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Chassis extends RobotComponent {

    public enum SpeedMode { SLOW, NORMAL, FAST }

    public Chassis(DcMotor frontLeftMotor, DcMotor.Direction frontLeftDirection,
                   DcMotor frontRightMotor, DcMotor.Direction frontRightDirection,
                   DcMotor backRightMotor, DcMotor.Direction backRightDirection,
                   DcMotor backLeftMotor, DcMotor.Direction backLeftDirection) {
        this.frontLeftMotor = frontLeftMotor;
        initMotor(this.frontLeftMotor, frontLeftDirection);

        this.frontRightMotor = frontRightMotor;
        initMotor(this.frontRightMotor, frontRightDirection);

        this.backRightMotor = backRightMotor;
        initMotor(this.backRightMotor, backRightDirection);

        this.backLeftMotor = backLeftMotor;
        initMotor(this.backLeftMotor, backLeftDirection);

        return;
    }

    public void updateState(Gamepad gamepad,Gamepad gamepad2) {
        SpeedMode selectedSpeedMode = SpeedMode.NORMAL;
        double lateral = gamepad.left_stick_x;
        double forward = gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;

        // Set the speed mode to Slow, Normal, or Fast based on gamepad bumpers.

        boolean fastmode = false;

        if (gamepad.b) {
            if (fastmode) {
                selectedSpeedMode = SpeedMode.NORMAL;
            }
            else {
                selectedSpeedMode = SpeedMode.FAST;
            }
        }

        if (gamepad.y) {
            if (this.count % 2 == 0) {
                if (this.count % 4 == 0) {
                    selectedSpeedMode = SpeedMode.FAST;
                    this.count += 2;
                }
                else {
                    selectedSpeedMode = SpeedMode.SLOW;
                    this.count +=2;
                }
            }
            else {
                selectedSpeedMode = SpeedMode.NORMAL;
                this.count += 2;
            }
        }

        if(gamepad.left_bumper && !gamepad.right_bumper) {
            selectedSpeedMode = SpeedMode.SLOW;
        }
        else if (gamepad.right_bumper && !gamepad.left_bumper) {
            selectedSpeedMode = SpeedMode.FAST;
        }
        this.setSpeedMode(selectedSpeedMode);

        // Calculate the motor powers based on the stick positions
        this.frontLeftPower = forward - lateral - rotation;
        this.frontRightPower = forward + lateral + rotation;
        this.backRightPower = forward - lateral + rotation;
        this.backLeftPower = forward + lateral - rotation;

        // Set the motor powers
        this.setFrontLeftPower();
        this.setFrontRightPower();
        this.setBackRightPower();
        this.setBackLeftPower();

        return;
    }

    public void setSpeedMode(SpeedMode speedMode) {
        switch(speedMode) {
            case SLOW:
                this.speedMultiplier = 0.2;
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
        return;
    }

    private DcMotor frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor;
    private double speedMultiplier = 0.0;
    private double frontLeftPower = 0.0;
    private double frontRightPower = 0.0;
    private double backRightPower = 0.0;
    private double backLeftPower = 0.0;
    private int count = 0;

    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }

    private void setFrontLeftPower() {
        this.frontLeftMotor.setPower(this.frontLeftPower * this.speedMultiplier);
        return;
    }

    private void setFrontRightPower() {
        this.frontRightMotor.setPower(this.frontRightPower * this.speedMultiplier);
        return;
    }

    private void setBackRightPower() {
        this.backRightMotor.setPower(this.backRightPower * this.speedMultiplier);
        return;
    }

    private void setBackLeftPower() {
        this.backLeftMotor.setPower(this.backLeftPower * this.speedMultiplier);
        return;
    }
}
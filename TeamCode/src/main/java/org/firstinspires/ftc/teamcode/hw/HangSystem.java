package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HangSystem extends RobotComponent {

    public enum HangPosition {
        HANG, DOWN, TOP_BAR
    }

    public Telemetry telemetry;
    private DcMotor hanger;



    public HangSystem(DcMotor hanger, Telemetry telemetry) {
        this.hanger = hanger;
        this.telemetry = telemetry;
        initMotor(hanger, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateState(Gamepad gamepad1,Gamepad gamepad2) {

        if(gamepad2.right_stick_y < -0.1) {
            moveHanger(HangPosition.TOP_BAR, 1);
        } else if(gamepad2.right_stick_y > 0.1) {
            moveHanger(HangPosition.DOWN, 1);
        } else if(gamepad2.right_stick_button) {
            moveHanger(HangPosition.HANG, 1);
        }

        telemetry.addData("Hanger position", hanger.getCurrentPosition());
    }


    public void moveHanger(HangPosition position, double power) {

        if(position == HangPosition.TOP_BAR) {
            hanger.setTargetPosition(8000);
        } else if(position == HangPosition.HANG) {
            hanger.setTargetPosition(3000);
        } else if(position == HangPosition.DOWN) {
            hanger.setTargetPosition(0);
        }

        hanger.setPower(power);
    }

    private void initMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode runMode) {

        motor.setTargetPosition(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }
}

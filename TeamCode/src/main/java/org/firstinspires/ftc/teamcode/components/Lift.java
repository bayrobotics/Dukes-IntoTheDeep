package org.firstinspires.ftc.teamcode.components;

import android.app.usage.NetworkStats;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends RobotComponent {

    public Telemetry telemetry;

    public Lift(DcMotor leftLift,
                DcMotor rightLift,
                TouchSensor bottomSwitch,
                DcMotor slide,
                Telemetry telemetry) {

        this.telemetry = telemetry;
        this.bottomSwitch = bottomSwitch;
        this.slide = slide;
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        initMotor(this.leftLift, DcMotor.Direction.REVERSE);
        initMotor(this.rightLift, DcMotor.Direction.FORWARD);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public enum LiftState {
      LIFT_START, LIFT_WAIT_FOR_INTAKE, LIFT_EXTEND, LIFT_DUMP
    }

    public enum SlidePosition {
        DOWN, INTAKE, HIGH_BASKET
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(bottomSwitch.isPressed()) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(Math.abs(gamepad2.left_stick_y) > 0.2) {
            setLiftPower(-gamepad2.left_stick_y);
            manualLiftControl = true;
        } else if(manualLiftControl == true) {
            setLiftPower(0);
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.2) {
            slide.setPower(gamepad2.right_stick_y * 0.5);
            manualLiftControl = true;
        } else if(manualLiftControl == true) {
            slide.setPower(0);
        }

        if(gamepad2.b) {
            manualLiftControl = false;
        }

        /*
        if(manualLiftControl == false) {
            switch (liftState) {
                case LIFT_START:
                    if (gamepad2.b && !bPressedPrevious) {
                        liftState = LiftState.LIFT_WAIT_FOR_INTAKE;
                        bPressedPrevious = true;
                    } else {
                        moveBucket(SlidePosition.DOWN, 0.4);
                        moveTo(0, 0.5);
                        bPressedPrevious = false;
                    }
                    break;
                case LIFT_WAIT_FOR_INTAKE:
                    if (gamepad2.b && !bPressedPrevious) {
                        liftState = LiftState.LIFT_EXTEND;
                        bPressedPrevious = true;
                    } else {
                        moveBucket(SlidePosition.INTAKE, 0.5);
                        bPressedPrevious = false;
                    }
                    break;
                case LIFT_EXTEND:
                    if (gamepad2.b && !bPressedPrevious) {
                        liftState = LiftState.LIFT_DUMP;
                        bPressedPrevious = true;
                    } else {
                        moveBucket(SlidePosition.DOWN, 0.3);
                        moveTo(2300, 0.7);
                        bPressedPrevious = false;
                    }
                    break;
                case LIFT_DUMP:
                    if (gamepad2.b && !bPressedPrevious) {
                        liftState = LiftState.LIFT_START;
                        bPressedPrevious = true;
                    } else {
                        moveBucket(SlidePosition.HIGH_BASKET, 0.6);
                        bPressedPrevious = false;
                    }
                    break;
                default:
                    liftState = LiftState.LIFT_START;
            }
        }

        if(gamepad2.b) {
            bPressedPrevious = true;
        } else {
            bPressedPrevious = false;
        }

         */




        telemetry.addData("Slide Position ", slide.getCurrentPosition());
        telemetry.addData("Lift Encoder Value: ", leftLift.getCurrentPosition());
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Left Lift Mode", leftLift.getMode());
        telemetry.addData("Target Position", leftLift.getTargetPosition());
        telemetry.addData("Left Power", leftLift.getPower());
        telemetry.addData("Right Power", rightLift.getPower());
        telemetry.addData("Slide Power", slide.getPower());
        telemetry.addData("Slide target position", slide.getTargetPosition());
        telemetry.addData("Lift bottom button pressed", bottomSwitch.isPressed());
        telemetry.update();

    }


    public void setLiftPower(double power) {

        if(power < 0 && bottomSwitch.isPressed()) {
            power = 0;
        } else if(power > 0 && leftLift.getCurrentPosition() > 2300) {
            power = 0;
        }

        leftLift.setPower(power);
        rightLift.setPower(power);

    }

    public boolean moveTo(int target, double power) {

        boolean atPosition = false;
        leftLift.setTargetPosition(target);

        if(Math.abs(leftLift.getCurrentPosition() - leftLift.getTargetPosition()) <= 50) {
            atPosition = true;
            power = 0;
        } else if(Math.abs(leftLift.getCurrentPosition() - leftLift.getTargetPosition()) <= 100) {
            power = 0.25;
        } else if(Math.abs(leftLift.getCurrentPosition() - leftLift.getTargetPosition()) <= 300) {
            power = 0.5;
        }

        if(leftLift.getTargetPosition() >= leftLift.getCurrentPosition()) {
            setLiftPower(power);
        } else {
            setLiftPower(-power);
        }

        return atPosition;

    }

    public int getHeight() {
        return leftLift.getCurrentPosition();
    }

    public boolean moveBucket(SlidePosition position, double power) {

        boolean atPosition = false;

        if(position == SlidePosition.DOWN) {
            slide.setTargetPosition(-170);
        } else if(position == SlidePosition.INTAKE) {
            slide.setTargetPosition(-300);
        } else if(position == SlidePosition.HIGH_BASKET) {
            slide.setTargetPosition(-1200);
        }

        if(Math.abs(slide.getCurrentPosition() - slide.getTargetPosition()) <= 50) {
            slide.setPower(0);
            atPosition = true;
        } else if(Math.abs(slide.getCurrentPosition() - slide.getTargetPosition()) <= 250) {
            power *= 0.5;
            if(power < 0.4) {
                power = 0.4;
            }
        } else if(slide.getCurrentPosition() < slide.getTargetPosition()) {
            slide.setPower(power);
        } else {
            slide.setPower(-power);
        }

        return atPosition;
    }

    private LiftState liftState = LiftState.LIFT_START;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor slide;
    private TouchSensor bottomSwitch;
    private SlidePosition slidePosition = SlidePosition.DOWN;
    private boolean bPressedPrevious = false;
    private boolean manualLiftControl = true;


    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);
        return;
    }
}
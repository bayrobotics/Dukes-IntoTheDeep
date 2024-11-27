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
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        initMotor(this.leftLift, DcMotor.Direction.REVERSE);
        initMotor(this.rightLift, DcMotor.Direction.FORWARD);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public enum LiftMode {
        MANUAL, PRESET
    }

    public enum LiftState {
        STOPPED, MOVING
    }

    public enum SlidePosition {
        DOWN, INTAKE, HIGH_BASKET
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(bottomSwitch.isPressed()) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        if(liftMode == LiftMode.PRESET) {
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(liftMode == LiftMode.MANUAL) {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        if(liftMode == LiftMode.PRESET && liftState == liftState.MOVING) {
            if(!leftLift.isBusy()) {
                setLiftPower(0);
            }
        }

        if(liftMode == LiftMode.PRESET && liftState == liftState.STOPPED) {
            if(leftLift.getCurrentPosition() - leftLift.getTargetPosition() < -20) {
                moveTo(leftLift.getTargetPosition(), 0.2);
            }
        }


        if(gamepad2.dpad_left) {

            if(liftMode == LiftMode.MANUAL) {
                liftMode = LiftMode.PRESET;
            } else if(liftMode == LiftMode.PRESET) {
                liftMode = LiftMode.MANUAL;
            }

            if(liftState == LiftState.MOVING) {
                setLiftPower(0);
            }

        } else if(gamepad2.dpad_up) {

            if(liftMode == LiftMode.MANUAL) {
                setLiftPower(0.5);
            }
            else if(liftMode == LiftMode.PRESET) {
                moveTo(2300, 0.5);
            }

        } else if(gamepad2.dpad_right) {

            if(liftMode == LiftMode.PRESET) {
                moveTo(1500, 0.5); // 1500 is estimate
            }

        } else if(gamepad2.dpad_down) {

            if(liftMode == LiftMode.MANUAL) {
                setLiftPower(-0.5);
            } else if(liftMode == LiftMode.PRESET) {
                moveTo(0, 0.5);
            }

        } else {

            if(liftMode == LiftMode.MANUAL) {
                setLiftPower(0);
            }
        }


        if(gamepad2.b) {
            slide.setPower(-0.3);
        } else if(gamepad2.x) {
            slide.setPower(0.3);
        } else {
            slide.setPower(0);
        }




        telemetry.addData("Lift Encoder Value: ", leftLift.getCurrentPosition());
        telemetry.addData("Lift Mode", liftMode);
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Left Lift Mode", leftLift.getMode());
        telemetry.addData("Target Position", leftLift.getTargetPosition());
        telemetry.addData("Left Power", leftLift.getPower());
        telemetry.addData("Right Power", rightLift.getPower());
        telemetry.addData("Slide Position ", slide.getCurrentPosition());
        telemetry.addData("Slide Power", slide.getPower());
        telemetry.addData("Slide target position", slide.getTargetPosition());
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

        if(power != 0) {
            liftState = liftState.MOVING;
        } else {
            liftState = liftState.STOPPED;
        }

    }

    public boolean moveTo(int target, double power) {

        boolean atPosition = false;
        leftLift.setTargetPosition(target);

        if(Math.abs(leftLift.getCurrentPosition() - leftLift.getTargetPosition()) <= 50) {
            atPosition = true;
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
            slide.setTargetPosition(0);
        } else if(position == SlidePosition.INTAKE) {
            slide.setTargetPosition(-200);
        } else if(position == SlidePosition.HIGH_BASKET) {
            slide.setTargetPosition(-1200);
        }

        if(Math.abs(slide.getCurrentPosition() - slide.getTargetPosition()) <= 50) {
            slide.setPower(0);
            atPosition = true;
        } else if(slide.getCurrentPosition() < slide.getTargetPosition()) {
            slide.setPower(power);
        } else {
            slide.setPower(-power);
        }

        return atPosition;
    }





    private LiftMode liftMode = LiftMode.MANUAL;
    private LiftState liftState = LiftState.STOPPED;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor slide;
    private TouchSensor bottomSwitch;
    private SlidePosition slidePosition = SlidePosition.DOWN;


    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(0);
        return;
    }
}
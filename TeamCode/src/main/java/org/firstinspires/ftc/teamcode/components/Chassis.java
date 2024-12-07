package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.liftHeight;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;

public class Chassis extends RobotComponent {

    public Telemetry telemetry;
    SpeedMode selectedSpeedMode = SpeedMode.NORMAL;

    public enum SpeedMode { SLOW, NORMAL, FAST }

    public Chassis(DcMotor frontLeftMotor, DcMotor.Direction frontLeftDirection,
                   DcMotor frontRightMotor, DcMotor.Direction frontRightDirection,
                   DcMotor backRightMotor, DcMotor.Direction backRightDirection,
                   DcMotor backLeftMotor, DcMotor.Direction backLeftDirection,
                   Telemetry telemetry) {
        this.frontLeftMotor = frontLeftMotor;
        initMotor(this.frontLeftMotor, frontLeftDirection);

        this.frontRightMotor = frontRightMotor;
        initMotor(this.frontRightMotor, frontRightDirection);

        this.backRightMotor = backRightMotor;
        initMotor(this.backRightMotor, backRightDirection);

        this.backLeftMotor = backLeftMotor;
        initMotor(this.backLeftMotor, backLeftDirection);

        this.telemetry = telemetry;
        return;
    }

    public void updateState(Gamepad gamepad,Gamepad gamepad2) {


        double lateral = gamepad.left_stick_x;
        double forward = gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;
        if(rotationFlipped) {
            rotation *= -1;
        }

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



        // Set the speed mode to Slow, Normal, or Fast based on gamepad bumpers.

        boolean fastmode = false;

        if(liftHeight > 100 && selectedSpeedMode == SpeedMode.FAST) {
            selectedSpeedMode = SpeedMode.SLOW;
        }

        /*
        if (gamepad.b) {
            if (fastmode) {
                selectedSpeedMode = SpeedMode.NORMAL;
            }
            else if(liftHeight < 100){
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

         */


        if(gamepad.left_bumper && !gamepad.right_bumper) {
            selectedSpeedMode = SpeedMode.SLOW;
        }
        else if (gamepad.right_bumper && !gamepad.left_bumper && liftHeight <= 100) {
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
        return;
    }

    private DcMotor frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor;
    private double speedMultiplier = 1.0;
    private double frontLeftPower = 0.0;
    private double frontRightPower = 0.0;
    private double backRightPower = 0.0;
    private double backLeftPower = 0.0;
    private int count = 0;
    private double distanceMultiplier = 44;
    private double sidewaysMultiplier = 100;
    private double degreesMultiplier = 7;
    private double motorPower = 0;
    private boolean xPressedPrevious = false;
    private boolean rotationFlipped = false;



    private void changeMotorDirection(DcMotor motor) {
        if(motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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
    public void moveForward(int inches) {
        double frontLeftDistance = inches * distanceMultiplier;

        this.runForwardPower(frontLeftDistance);
    }

    public void moveRight(int inches) {
        double frontRightDistance = inches * sidewaysMultiplier;

        this.runStrafePower(frontRightDistance);
    }

    public void rotate(double degrees) {

        double backRightDistance = degrees * degreesMultiplier;

        telemetry.addData("motorSpeed: ", motorPower);
        telemetry.update();

        if (backRightDistance >  0) {
            runRightRotate(Math.abs(backRightDistance));
        }
        else if (backRightDistance < 0) {
            runLeftRotate(Math.abs(backRightDistance));
        }
        else return;
    }

    private void initMotor(DcMotor motor, DcMotor.Direction motorDirection) {
        motor.setDirection(motorDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return;
    }

    private void runForwardPower(double distance) {
        this.frontLeftMotor.setTargetPosition((int) distance + frontLeftMotor.getCurrentPosition());
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(motorPower);
        frontRightMotor.setPower(motorPower);
        backLeftMotor.setPower(motorPower);
        backRightMotor.setPower(motorPower);

        while(frontLeftMotor.isBusy())
        {
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(motorPower);
        }

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return;
    }

    private void runStrafePower(double distance) {
        this.frontRightMotor.setTargetPosition((int) distance + frontLeftMotor.getCurrentPosition());
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftMotor.setPower(-motorPower);
            frontRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(-motorPower);

        while(frontRightMotor.isBusy())
        {
            frontLeftMotor.setPower(-motorPower);
            frontRightMotor.setPower(motorPower);
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower( -motorPower);
        }


        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return;
    }

    private void runLeftRotate(double distance) {
        this.backRightMotor.setTargetPosition((int) distance + backRightMotor.getCurrentPosition());
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(motorPower);
        frontRightMotor.setPower(motorPower);
        backLeftMotor.setPower(-motorPower);
        backRightMotor.setPower(motorPower);

        while(backRightMotor.isBusy())
        {
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);
            backLeftMotor.setPower(-motorPower);
            backRightMotor.setPower(motorPower);
        }

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return;
    }

    private void runRightRotate(double distance) {
        this.backLeftMotor.setTargetPosition((int) distance + backLeftMotor.getCurrentPosition());
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(motorPower);
        frontRightMotor.setPower(-motorPower);
        backLeftMotor.setPower(motorPower);
        backRightMotor.setPower(-motorPower);

        while(backLeftMotor.isBusy())
        {
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(-motorPower);
            backLeftMotor.setPower(motorPower);
            backRightMotor.setPower(-motorPower);
        }

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return;
    }

    public void setChassisSpeed(double power){
        this.motorPower = power;
    }
}
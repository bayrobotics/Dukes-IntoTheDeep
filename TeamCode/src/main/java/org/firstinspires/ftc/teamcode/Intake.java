package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends RobotComponent{

    Servo intakeLift;
    DcMotor spinner;


    public Intake(Servo intakeLift, DcMotor spinner) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        moveIntake(gamepad, gamepad2);
        spin(gamepad, gamepad2);
    }

    public void moveIntake(Gamepad gamepad, Gamepad gamepad2) {
        if(gamepad.left_bumper && !gamepad.right_bumper) {
            intakeLift.setPosition(0.8);
        }

        if(gamepad.right_bumper && !gamepad.left_bumper) {
            intakeLift.setPosition(0);
        }
    }

    public void spin(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad.a) {
            spinner.setPower(1);
        } else if(gamepad.b) {
            spinner.setPower(-1);
        }

    }


}

package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends RobotComponent{

    Servo intakeLift;
    CRServo spinner;


    public Intake(Servo intakeLift, CRServo spinner) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;
    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        moveIntake(gamepad, gamepad2);
        spin(gamepad, gamepad2);
    }

    public void moveIntake(Gamepad gamepad, Gamepad gamepad2) {
        if(gamepad2.dpad_left) {
            intakeLift.setPosition(0.8);
        }

        if(gamepad2.dpad_right) {
            intakeLift.setPosition(0);
        }
    }

    public void spin(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad2.left_bumper) {
            spinner.setPower(-1);
        } else if(gamepad2.right_bumper) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }

    }


}

package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends RobotComponent{

    private CRServo intakeLift;
    private CRServo spinner;


    public Intake(CRServo intakeLift, CRServo spinner) {

        this.intakeLift = intakeLift;
        this.spinner = spinner;

    }

    public void updateState(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad2.y) {
            intakeLift.setPower(1);
        } else if(gamepad2.a) {
            intakeLift.setPower(-1);
        } else {
            intakeLift.setPower(0);
        }

        if(gamepad2.left_bumper) {
            spinner.setPower(-1);
        } else if(gamepad2.right_bumper) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }

    }

}

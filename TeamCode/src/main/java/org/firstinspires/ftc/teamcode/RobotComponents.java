package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

abstract class RobotComponents {





    public RobotComponents() {

    }


    abstract void updateState(Gamepad gamepad1, Gamepad gamepad2);



}


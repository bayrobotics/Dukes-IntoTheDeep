package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Gamepad;

abstract class RobotComponent {

    public RobotComponent() {

    }

    abstract void updateState(Gamepad gamepad1, Gamepad gamepad2);

}


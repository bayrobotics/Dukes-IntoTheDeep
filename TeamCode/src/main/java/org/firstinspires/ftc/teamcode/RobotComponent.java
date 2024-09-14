package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

abstract class RobotComponent {

    public RobotComponent() {

    }

    abstract void updateState(Gamepad gamepad1, Gamepad gamepad2);

}


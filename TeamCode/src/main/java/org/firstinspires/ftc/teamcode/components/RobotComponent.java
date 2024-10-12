package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class RobotComponent {

    public RobotComponent() {

    }

    public abstract void updateState(Gamepad gamepad1, Gamepad gamepad2);

}
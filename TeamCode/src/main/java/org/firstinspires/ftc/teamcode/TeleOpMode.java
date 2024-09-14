package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.LinkedList;
import java.util.List;

@TeleOp()
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "front_left"), DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "front_right"), DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "back_right"),DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "back_left"), DcMotor.Direction.REVERSE);

        List<RobotComponent> components = new LinkedList<RobotComponent>();
        components.add(chassis);

        waitForStart();

        while (opModeIsActive()) {
            for(RobotComponent currentComponent: components) {
                currentComponent.updateState(gamepad1, gamepad2);
            }
        }
    }
}

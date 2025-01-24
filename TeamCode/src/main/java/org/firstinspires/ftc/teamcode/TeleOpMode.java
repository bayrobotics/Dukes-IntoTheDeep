package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hw.Chassis;
import org.firstinspires.ftc.teamcode.hw.Intake;
import org.firstinspires.ftc.teamcode.hw.Lift;
import org.firstinspires.ftc.teamcode.hw.RobotComponent;

import java.util.LinkedList;
import java.util.List;

@TeleOp()
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "front_left"),
                hardwareMap.get(DcMotor.class, "front_right"),
                hardwareMap.get(DcMotor.class, "back_left"),
                hardwareMap.get(DcMotor.class, "back_right"),
                telemetry);

        Intake intake = new Intake(hardwareMap.get(DcMotor.class, "intakeLift"),
                hardwareMap.get(CRServo.class, "spinner"),
                hardwareMap.get(CRServo.class, "leftExtender"),
                hardwareMap.get(CRServo.class, "rightExtender"),
                hardwareMap.get(TouchSensor.class, "upStop"),
                hardwareMap.get(TouchSensor.class, "bottomStop"),
                hardwareMap.get(TouchSensor.class, "retractStop"),
                hardwareMap.get(TouchSensor.class, "extendStop"),
                telemetry);

        Lift lift = new Lift(hardwareMap.get(DcMotor.class, "leftLift"),
        hardwareMap.get(DcMotor.class, "rightLift"),
                hardwareMap.get(DcMotor.class, "bucket"),
                hardwareMap.get(TouchSensor.class, "bottomSwitch"),
                hardwareMap.get(TouchSensor.class, "slideSwitch"),
                telemetry);

        List<RobotComponent> components = new LinkedList<RobotComponent>();
        components.add(chassis);
        components.add(intake);
        components.add(lift);

        waitForStart();

        while (opModeIsActive()) {

            for(RobotComponent currentComponent: components) {
                currentComponent.updateState(gamepad1, gamepad2);
            }

            telemetry.update();

        }
    }
}

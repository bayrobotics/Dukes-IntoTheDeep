package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.RobotComponent;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;

import java.util.LinkedList;
import java.util.List;

@TeleOp()
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "front_left"), DcMotor.Direction.REVERSE ,
                hardwareMap.get(DcMotor.class, "front_right"), DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "back_right"),DcMotor.Direction.FORWARD ,
                hardwareMap.get(DcMotor.class, "back_left"), DcMotor.Direction.REVERSE, telemetry);

        Intake intake = new Intake(hardwareMap.get(CRServo.class, "intakeLift"),
                hardwareMap.get(CRServo.class, "spinner"),
                hardwareMap.get(TouchSensor.class, "upStop"),
                hardwareMap.get(TouchSensor.class, "bottomStop"),
                hardwareMap.get(Servo.class, "topLeftExtender"),
                hardwareMap.get(Servo.class, "topRightExtender"),
                hardwareMap.get(Servo.class, "bottomLeftExtender"),
                hardwareMap.get(Servo.class, "bottomRightExtender"),
                telemetry);

        Lift lift = new Lift(hardwareMap.get(DcMotor.class, "leftLift"),
        hardwareMap.get(DcMotor.class, "rightLift"),
        hardwareMap.get(TouchSensor.class, "bottomSwitch"),
        hardwareMap.get(DcMotor.class, "bucket"),
                telemetry);

        List<RobotComponent> components = new LinkedList<RobotComponent>();
        components.add(chassis);
        components.add(intake);
        components.add(lift);

        waitForStart();

        while (opModeIsActive()) {


            chassis.setChassisSpeed(1.0);

            for(RobotComponent currentComponent: components) {
                currentComponent.updateState(gamepad1, gamepad2);
            }
        }
    }
}

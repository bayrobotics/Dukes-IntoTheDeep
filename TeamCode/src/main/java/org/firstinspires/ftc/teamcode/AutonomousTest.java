package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Intake;


@Autonomous
public class AutonomousTest extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {

        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "front_left"), DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "front_right"), DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "back_right"),DcMotor.Direction.FORWARD,
                hardwareMap.get(DcMotor.class, "back_left"), DcMotor.Direction.REVERSE, telemetry);

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}

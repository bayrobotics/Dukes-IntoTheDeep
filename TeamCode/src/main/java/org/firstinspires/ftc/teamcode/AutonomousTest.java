package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.Chassis;
import org.firstinspires.ftc.teamcode.hw.Intake;


@Autonomous
public class AutonomousTest extends LinearOpMode{



    @Override
    public void runOpMode() throws InterruptedException {

        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "front_left"),
                hardwareMap.get(DcMotor.class, "front_right"),
                hardwareMap.get(DcMotor.class, "back_left"),
                hardwareMap.get(DcMotor.class, "back_right"),
                telemetry);

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}

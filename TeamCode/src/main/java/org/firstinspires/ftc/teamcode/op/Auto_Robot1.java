//
// PathMaker.java
//
// This is the main module showing the use of the PathMaker library that can be used
// for autonomous control of a Mecanum wheel drive train for the FTC competition.
// This implementation uses the FTC Dashboard for developing the robot path settings
// including simulating robot movements that can be displayed on the dashboard.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.Intake;
import org.firstinspires.ftc.teamcode.hw.Lift;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.pathmaker.GameSetup;
import org.firstinspires.ftc.teamcode.pathmaker.PathDetails;
import org.firstinspires.ftc.teamcode.pathmaker.PathMakerStateMachine;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;

@Config
@Autonomous
public class Auto_Robot1 extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 1;
    public static int runTest_ms = 100;
    public static int thisNumberSteps = 2;
    public static int setZone = 1;
    public static GameSetup.Terminal terminal = GameSetup.Terminal.CLOSE;

    DriveTrain driveTrain = new DriveTrain(this);


    @Override
    public void runOpMode() throws InterruptedException {

            GameSetup.terminal = GameSetup.Terminal.CLOSE;
            terminal = GameSetup.Terminal.CLOSE;



        Lift lift = new Lift(hardwareMap.get(DcMotor.class, "leftLift"),
                hardwareMap.get(DcMotor.class, "rightLift"),
                hardwareMap.get(DcMotor.class, "bucket"),
                hardwareMap.get(TouchSensor.class, "bottomSwitch"),
                hardwareMap.get(TouchSensor.class, "slideSwitch"),
                telemetry);

        Intake intake = new Intake(hardwareMap.get(DcMotor.class, "intakeLift"),
                hardwareMap.get(CRServo.class, "spinner"),
                hardwareMap.get(TouchSensor.class, "upStop"),
                hardwareMap.get(TouchSensor.class, "bottomStop"),
                telemetry);

        RobotPose.initializePose(this, driveTrain, telemetry);
            RobotPose.setPose(63, -35, 270);

        MyIMU.init(this);
        MyIMU.updateTelemetry(telemetry);

        PathMakerStateMachine.setAutonomous();
        PathDetails.initAutoPathList();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        int cycles = 0;
        int TEST_CYCLES = 4;
        telemetry.addData("thisPathNumber", thisPathNumber);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (thisPathNumber == -1) {
                WheelPowerManager.setDrivePower(driveTrain, thisForwardPower, thisStrafePower, thisTurnPower, thisHeadingDrive);
                sleep(runTest_ms);
                WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
                RobotPose.readPose();
            } else if (thisPathNumber == 0) {
                // use dashboard parameters
                PathManager.moveRobot();
                sleep(runTest_ms);
            } else if (thisPathNumber == 1) {
                cycles++;
                if (cycles > TEST_CYCLES) {
                    double t1 = timer.milliseconds() / cycles;
                    timer.reset();
                    telemetry.addData("State", PathMakerStateMachine.pm_state);
                    telemetry.addData("PathDetails.currentPath", PathMakerStateMachine.currentPath < 0? -1: PathDetails.autoPathList.get(PathMakerStateMachine.currentPath));
                    telemetry.addLine(String.format("tag %b, ID %d, in zone %b, at rest %b",
                            PathMakerStateMachine.aprilTagDetectionOn, PathMakerStateMachine.aprilTagDetectionID, PathManager.inTargetZone, RobotPose.isRobotAtRest()));
                    telemetry.addLine(String.format("PathDetails.elapsedTime_ms %.1f", PathDetails.elapsedTime_ms.milliseconds()));
                    telemetry.addLine(String.format("ave/PM/DT cycle %d /  %d (ms) / %.3f (s)", (int) t1, PathManager.PMcycleTime_ms, RobotPose.DT_seconds));

                    double [] xya = RobotPose.tagOffset(PathMakerStateMachine.aprilTagDetectionID);
                    telemetry.addLine(String.format("delta-is-should f/s/a %.1f / %.1f / %.1f",
                            PathManager.deltaIsShouldY,
                            PathManager.deltaIsShouldX,
                            PathManager.deltaIsShouldAngle));
                    telemetry.addLine(String.format("delta is-should signum f/s/a %d / %d / %d",
                            (int) Math.signum(PathManager.deltaIsShouldY),
                            (int) Math.signum(PathManager.deltaIsShouldX),
                            (int) Math.signum(PathManager.deltaIsShouldAngle)));
                    telemetry.addLine(String.format("initial power signum f/s/a %d / %d / %d",
                            (int) Math.signum(PathDetails.yInitialPowerSignum),
                            (int) Math.signum(PathDetails.xInitialPowerSignum),
                            (int) Math.signum(PathDetails.aInitialPowerSignum)));
                    telemetry.addLine(String.format("power x,y,a %.2f / %.2f / %.2f",
                            PathManager.xPower,
                            PathManager.yPower,
                            PathManager.turnPower));
                    telemetry.addLine(String.format("tagOffset f/s/a %.1f / %.1f / %.1f (in/deg)",
                            xya[0],
                            xya[1],
                            xya[2]));
                    telemetry.addLine(String.format("Path Goals f/s/t %.1f / %.1f / %.1f (in/deg)",
                            PathDetails.yFieldGoal_in,
                            PathDetails.xFieldGoal_in,
                            PathDetails.aFieldGoal_deg));
                    telemetry.addLine(String.format("RoboPose f/s/t %.1f / %.1f / %.1f (in/deg)",
                            RobotPose.getFieldY_in(),
                            RobotPose.getFieldX_in(),
                            RobotPose.getFieldAngle_deg()));
                    // telemetry velocity
                    telemetry.addLine(String.format("x/y/a velocity %.1f / %.1f / %.1f (v_r %.3f)",
                            RobotPose.getXVelocity_inPerSec(),
                            RobotPose.getYVelocity_inPerSec(),
                            RobotPose.getHeadingVelocity_degPerSec(),
                            PathManager.v_ramp));




                    telemetry.addData("chassis at right spot: ", PathManager.inTargetZone);
                    telemetry.addData("Lift at target position: ", lift.liftAtTargetPosition);
                    telemetry.addData("Lift at target position: ", lift.slideAtTargetPosition);
                    telemetry.addData("intake at target position: ", intake.intakeAtTargetPosition);
                    telemetry.addData("Target slide position", PathDetails.slidePosition);
                    telemetry.addData("Target slide power", PathDetails.slidePower);
                    telemetry.addData("slide position", Lift.slidePosition);
                    telemetry.addData("Slide target position", Lift.slideTargetPosition);
                    telemetry.addData("Slide power", Lift.slidePower);
                    telemetry.addData("Lift encoder value", Lift.leftLift.getCurrentPosition());
                    telemetry.addData("Lift power", Lift.liftPower);
                    telemetry.addData("Lift target position", Lift.liftTargetPosition);
                    telemetry.addData("Lift distance to target", lift.getLiftDistanceToTarget());
                    telemetry.addData("Control mode", PathMakerStateMachine.control_mode);
                    telemetry.addData("Elapsed path time", PathDetails.elapsedTime_ms.milliseconds());


                    MyIMU.updateTelemetry(telemetry);
                    telemetry.update();
                    cycles = 0;
                }
                PathMakerStateMachine.updateAuto(telemetry, lift, intake);
            }
        }
    }
}

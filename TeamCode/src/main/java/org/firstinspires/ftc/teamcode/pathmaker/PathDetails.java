//
// PathDetails.java
//
// The PathDetails class implements the definition of robot paths. Each path is controlled by two
// constants for each of the three degrees of freedom (DOF). From a robot centric view the DOFs are
// forward, strafe, and turn. The two constants that define each DOF are:
// a goal (e.g. forwardGoal_in = 57) that defines the endpoint to be reached at the end of the path,
// and a delay (e.g. forwardDelay_ms = 500) that defines the timing from the beginning of the path
// at which the goal will be activated. The robot will attempt to reach the goal while maximizing
// the the use of the available battery power. The robot will ramp power down when reaching the goal.
// Additionally, the PathTime constant controls the total time allowed for the path. When PathTime
// is larger than the time needed to reach the goal, it has no effect. However, if PathTime is
// reached before the goal, the path is terminated at the current conditions. In other words, if
// the robot is moving it will continue to do so. This can be useful to create a "rolling stop"
// for a smooth transition to the next robot action.
// The coordinate system (COS) that PathMaker uses is the robot centric coordinate system where
// the forward direction is Y and the strafe direction is X. Rotations count positive going
// right. Whenever the encoders are reset, the COS will be reset to the current location.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.pathmaker;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.op.RobotPose;
import org.firstinspires.ftc.teamcode.components.Lift;
import org.firstinspires.ftc.teamcode.components.Intake.IntakePosition;

import java.util.ArrayList;

@Config
public class PathDetails {
    // PathDetails defines each path goals and constraints for each DOF (x,y,a)
    // PathMaker uses a robot centric coordinate system (COS)
    // All x/y coordinates relate to the field center
    // Positive y is towards the backstage wall
    // Positive x is towards the right when looking towards the backstage wall
    // Positive turn is clockwise (tbc). Turn is also called "a" for angle
    public static double powerScaling = 1;
    public static double yFieldGoal_in, yFieldOffset_in =0;
    public static double xFieldGoal_in, xFieldOffset_in =0;
    public static double aFieldGoal_deg, aFieldOffset_deg =0;
    private static double xRelativeToTag, yRelativetoTag, aRelativetoTag;
    public static double xInitialPowerSignum = 1, yInitialPowerSignum = 1, aInitialPowerSignum = 1;
    // set initial delay
    public static double yFieldDelay_ms;
    public static double xFieldDelay_ms;
    public static double turnFieldDelay_ms;
    public static double liftFieldDelay_ms = 0;
    public static double bucketFieldDelay_ms = 0;
    public static ElapsedTime elapsedTime_ms = new ElapsedTime();
    public static double pathTime_ms = 0;
    public  static PathMakerStateMachine.PM_STATE PMSMstate;
    public static int liftHeight = 0;
    public static double liftPower = 0.5;
    public static IntakePosition intakePosition = IntakePosition.UP;
    public static Lift.SlidePosition slidePosition = Lift.SlidePosition.DOWN;
    public static double slidePower = 0.3;
    public static double spinPower = 0;

    public static double lastTurnGoal;
    public static void initializePath() {
        // initialize each new path
        pathTime_ms = 9E9;
        elapsedTime_ms.reset();
        powerScaling = 1;
        yFieldDelay_ms = 0;
        xFieldDelay_ms = 0;
        turnFieldDelay_ms = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
        PMSMstate = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathManager.powerScalingTurn = 1;
        PathManager.maxPowerStepUp = 0.1;
        PathManager.inTargetZone = false;
        PathManager.yTargetZone_in = 2;
        PathManager.xTargetZone_in = 2;
        PathManager.turnTargetZone_deg = 1;
        PathManager.yRampReach_in = 4;
        PathManager.xRampReach_in = 4;
        PathManager.turnRampReach_deg = 20;
        PathManager.yMinVelocity_InchPerSec = 2;
        PathManager.xMinVelocity_InchPerSec = 2;
        PathManager.turnMinVelocity_degPerSec = 4;
        PathManager.rampType_x = PathManager.RAMPTYPE.LINEAR;
        PathManager.rampType_y = PathManager.RAMPTYPE.LINEAR;
        PathManager.rampType_a = PathManager.RAMPTYPE.LINEAR;
        PathManager.approachPowerTurn = 0.1;
        PathManager.approachPowerXY = 0.2;
        PathManager.breakPower = 0.05;
        PathManager.breakPowerScale = 0.5;
    }
    public enum Path {
        // Each path is labeled with a name as defined in this enum
        P0, P1, P2, P3, P4,P5, P6,
        FP1, FP2, FP3, FP4,FP5, FP6, FP7,
        DRIVER_CONTROLLED,
        DONE
    }   // end enum Event
    public static ArrayList<Path> autoPathList;
    public static ArrayList<Path> autoPathListTwo;
    public static void initAutoPathList() {
        PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.AUTO_SET_PATH;
        PathMakerStateMachine.currentPath = PathMakerStateMachine.nextPath = 0;
        PathMakerStateMachine.aprilTagDetectionOn = false;
        PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
        autoPathList = new ArrayList<PathDetails.Path>();
        autoPathListTwo = new ArrayList<PathDetails.Path>();
        // autoPathList.add(PathDetails.Path.P0);
        autoPathList.add(PathDetails.Path.P1);
        // autoPathList.add(PathDetails.Path.P2);
        // autoPathList.add(PathDetails.Path.P3);
        // autoPathList.add(PathDetails.Path.P4);
        // autoPathList.add(PathDetails.Path.P6);
        // autoPathList.add(PathDetails.Path.P5);
        autoPathListTwo.add(PathDetails.Path.FP1);
        // autoPathListTwo.add(PathDetails.Path.FP2);
        // autoPathListTwo.add(PathDetails.Path.FP3);
        // autoPathListTwo.add(PathDetails.Path.FP4);
        // autoPathListTwo.add(PathDetails.Path.FP5);
        // autoPathListTwo.add(PathDetails.Path.FP6);
        // autoPathListTwo.add(PathDetails.Path.FP7);
    }   // end method initAuto

    public static void setPath(Path path, Telemetry telemetry) throws InterruptedException {
        // set path parameters
        // this method is called from PathMakerStateMachine
        // In Autonomous mode, the autoPathList is set by the PathMakerStateMachine
        // In Tele mode, the path is set to DRIVER_CONTROLLED in the PathMakerStateMachine
        initializePath();
        switch (path) {
            case DRIVER_CONTROLLED:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.TELEOP;
                PathManager.maxPowerStepUp = 0.1;
                PathManager.turnTargetZone_deg = 3;
                powerScaling = 1;
                break;
            case P0:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                slidePosition = Lift.SlidePosition.HIGH_BASKET;
                powerScaling = 0.5;
                xFieldGoal_in = 63; yFieldGoal_in = -36; aFieldGoal_deg = 270;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P1:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                pathTime_ms = 7000;
                liftPower = 1;
                liftHeight = 2300;
                powerScaling = 0.5;
                slidePower = 0.3;
                bucketFieldDelay_ms = 1000;
                slidePosition = Lift.SlidePosition.HIGH_BASKET;
                // intakePosition = IntakePosition.DOWN;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P2:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                liftPower = 1;
                slidePower = 0.3;
                bucketFieldDelay_ms = 1000;
                liftFieldDelay_ms = 2000;
                liftHeight = 0;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P3:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 0.5;
                yFieldDelay_ms = 3000;
                xFieldGoal_in = 12; yFieldGoal_in = -24; aFieldGoal_deg = 180;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P4:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                spinPower = 0;
                intakePosition = IntakePosition.UP;
                slidePosition = Lift.SlidePosition.INTAKE;
                PathManager.yRampReach_in = 24;
                PathManager.xRampReach_in = 24;
                xFieldGoal_in = 60; yFieldGoal_in = 60; aFieldGoal_deg = 270;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P5:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                liftPower = 0.5;
                powerScaling = 0.5;
                slidePower = 0.3;
                slidePosition = Lift.SlidePosition.DOWN;
                liftHeight = 2300;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case P6:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                liftPower = 0.5;
                slidePower = 0.3;
                slidePosition = Lift.SlidePosition.HIGH_BASKET;
                liftHeight = 2300;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP1:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 0.4;
                PathManager.yRampReach_in = 12;
                PathManager.xRampReach_in = 12;
                xFieldGoal_in = 60; yFieldGoal_in = 55; aFieldGoal_deg = 270;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP2:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                xFieldGoal_in = 36; yFieldGoal_in = -60; aFieldGoal_deg = 270;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP3:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP4:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                liftHeight = 2300;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP5:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP6:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                slidePosition = Lift.SlidePosition.HIGH_BASKET;
                liftFieldDelay_ms = 3000;
                liftHeight = 1200;
                xFieldGoal_in = 56; yFieldGoal_in = -60; aFieldGoal_deg = 315;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case FP7:
                PathMakerStateMachine.control_mode = PathMakerStateMachine.CONTROL_MODE.AUTONOMOUS;
                PathMakerStateMachine.pm_state = PathMakerStateMachine.PM_STATE.CONTROL_LIFT_AND_INTAKE;
                powerScaling = 1;
                liftFieldDelay_ms = 500;
                liftHeight = 0;
                slidePosition = Lift.SlidePosition.DOWN;
                xFieldGoal_in = 60; yFieldGoal_in = 60; aFieldGoal_deg = 270;
                calculateInitialPowerSignum(xFieldGoal_in, yFieldGoal_in, aFieldGoal_deg);
                break;
            case DONE:
                break;
        }
    }

    private static void calculateInitialPowerSignum(double goalX, double goalY, double goalA) {
        // calculate initial power signum (motor direction). The PathManager uses this when
        // rampReach is set to zero. In this case the robot will keep moving until with
        // full power until the goal is reached.
        xInitialPowerSignum = Math.signum( goalX - RobotPose.getFieldX_in());
        yInitialPowerSignum = Math.signum( goalY - RobotPose.getFieldY_in());
        aInitialPowerSignum = Math.signum( goalA - RobotPose.getFieldAngle_deg());
    }
}
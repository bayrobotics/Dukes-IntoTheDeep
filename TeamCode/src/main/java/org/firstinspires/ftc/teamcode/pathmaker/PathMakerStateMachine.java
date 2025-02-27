package org.firstinspires.ftc.teamcode.pathmaker;

import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.extensionFieldDelay_ms;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.intakeExtension;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.intakeExtensionPower;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.intakeFieldDelay_ms;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.intakePosition;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.intakePower;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.slideFieldDelay_ms;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.slideFieldDelay_ms;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.slidePosition;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.liftFieldDelay_ms;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.liftPower;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.slidePower;
import static org.firstinspires.ftc.teamcode.pathmaker.PathDetails.spinPower;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hw.Intake;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.op.RobotPose;
import org.firstinspires.ftc.teamcode.hw.Lift;

public class PathMakerStateMachine {
    // The PathMakerStateMachine manages the state of the robot as defined in the State enum.
    // The PathMakerStateMachine is called from the opmodes Tele_Robot1 and Auto_Robot1.
    // The PathMakerStateMachine calls the PathManager roughly every timeStep_ms.
    // The PathMakerStateMachine has a section for teleop and a section for autonomous.
    // This is the main control loop for the robot. All other states on the robot should be
    // managed by the PathMakerStateMachine. For example, based on the robot field position the
    // arm position should be controlled so that the robot does not interfere with the truss.



    enum CONTROL_MODE {
        TELEOP, AUTONOMOUS
    }   // end enum GameMode
    enum PM_STATE {
        INIT, IDLE, DONE, DRIVER_CONTROL,AUTO_SET_PATH, AUTO_NEXT_PATH, AUTO_APRILTAG_ExecutePath, AUTO_ExecutePath
    }   // end enum State

    public static PM_STATE pm_state;
    public static CONTROL_MODE control_mode;
    public static boolean aprilTagDetectionOn = false;
    public static int aprilTagDetectionID = 0;
    public static int currentPath = -1, nextPath = -1;
    public static double yPower = 0, yPowerLast = 0;
    public static double xPower = 0, xPowerLast = 0;
    public static double turnPower = 0;
    static double turnPowerLast = 0;
    static int xStoppingCounter = 1, yStoppingCounter = 1;
    public static double turnSensitivity = 0.4;
    private static ElapsedTime switchToAutonomousTimer = new ElapsedTime();
    private static boolean fromManualToAutoHeading = false;

    public PathMakerStateMachine() {
        pm_state = PM_STATE.INIT;
        control_mode = CONTROL_MODE.TELEOP;
    }   // end constructor PathMakerStateMachine
    public static void setDriverControlled() {
        pm_state = PM_STATE.DRIVER_CONTROL;
        PathManager.autonomous_x = false;
        PathManager.autonomous_y = false;
        PathManager.autonomous_a = false;
    }
    public static void setAutonomous() {
        control_mode = CONTROL_MODE.AUTONOMOUS;
        PathManager.autonomous_x = true;
        PathManager.autonomous_y = true;
        PathManager.autonomous_a = true;
    }
    //
    // Teleop section
    //
    public static void updateTele(Gamepad gamepad, Telemetry telemetry) throws InterruptedException {
        RobotPose.readPose(); // read pose once per tele loop
        // pm_state = PM_STATE.DRIVER_CONTROL;

        switch (pm_state) {
            case INIT:
                pm_state = PM_STATE.IDLE;
                break;
            case DRIVER_CONTROL:
                PathDetails.setPath(PathDetails.Path.DRIVER_CONTROLLED,telemetry);
                getGamepadInput(gamepad);
                //autoLaneKeeping();
                PathManager.moveRobot();
                break;
            default:
                break;
        }   // end switch (state)
    }   // end method updateTele
    private static void getGamepadInput(Gamepad gamepad) {
        xPower = gamepad.left_stick_x;
        yPower = -gamepad.left_stick_y;
        turnPower = gamepad.right_stick_x * turnSensitivity;

        double gamepadThreshold = 0.1;
        // check if gampad input is below threshold
        // if so, we will ramp down the input to zero
        if (Math.abs(xPower) < gamepadThreshold) {
            // this ramp doesn't help as much as I wanted
            xPower = (gamepadThreshold / xStoppingCounter++) * Math.signum(xPowerLast);
        } else {
            xStoppingCounter = 1;
        }

        if (Math.abs(yPower) < gamepadThreshold) {
            yPower = (gamepadThreshold / yStoppingCounter++) * Math.signum(yPowerLast);
        } else {
            yStoppingCounter = 1;
        }
        if (Math.abs(turnPower) < gamepadThreshold) {
            turnPower = 0;
        }
        // add ramp for x and y powers
        double maxXYPowerStep = 0.15;
        double signumXPowerChange = Math.signum(xPower - xPowerLast);
        if (Math.abs(xPower-xPowerLast) > maxXYPowerStep) {
            xPower = xPowerLast + signumXPowerChange * maxXYPowerStep;
            // set minimum power
        }
        if (Math.abs(xPower) < 0.1) {
            xPower *= 0.9;
        }
        double signumYPowerChange = Math.signum(yPower - yPowerLast);
        if (Math.abs(yPower-yPowerLast) > maxXYPowerStep) {
            yPower = yPowerLast + signumYPowerChange * maxXYPowerStep;
            // set minimum power
        }
        if (Math.abs(yPower) < 0.1) {
            yPower *= 0.9;
        }
        // if gamepadTurn is zero, we will automatically keep the robot heading the same
        // if gamepadTurn is not zero, we will turn the robot to the new heading
        PathManager.autonomous_x = false;
        PathManager.autonomous_y = false;
        PathManager.autonomous_a = true;
        if (turnPower == 0) {
            // auto heading
            // keep robot heading the same direction if no turn input
            // this counteracts rotational drifting of the robot
            if (switchToAutonomousTimer.milliseconds() < 400 && fromManualToAutoHeading) {
                // need to wait long enough to get one or heading readings
                // increase 200 to 400 ms for heavy robot (more inertia)
                // before switching to autonomous mode to avoid bounce back
                PathDetails.aFieldGoal_deg = RobotPose.getFieldAngle_deg();
            } else {
                PathManager.autonomous_a = true;
                fromManualToAutoHeading = false;
            }
        } else {
            // manual heading
            // keep track of the last actual robot heading
            // this is used when switching from driver control to autonomous for turning
            fromManualToAutoHeading = true;
            switchToAutonomousTimer.reset();
            PathManager.autonomous_a = false;
        }
        // remember last power
        xPowerLast = xPower;
        yPowerLast = yPower;
        turnPowerLast = turnPower;
    }
    //
    // Autonomous section
    //
    public static void updateAuto(Telemetry telemetry, Lift lift, Intake intake) throws InterruptedException {
        RobotPose.readPose(); // read pose once per auto loop (moved here from PathManager)
        // process state
        switch (pm_state) {
            case AUTO_SET_PATH:
                if(GameSetup.terminal == GameSetup.Terminal.CLOSE) {
                    PathDetails.setPath(PathDetails.autoPathList.get(nextPath), telemetry);
                } else if(GameSetup.terminal == GameSetup.Terminal.FAR) {
                    PathDetails.setPath(PathDetails.autoPathListTwo.get(nextPath), telemetry);
                }
                break;
            case IDLE:
                break;
            case DONE:
                powerDown();
                intake.moveIntakeTo(Intake.IntakePosition.DOWN, 0);
                lift.moveLift(Lift.LiftPosition.DOWN, 0);
                // lift.moveSlideTo(Lift.SlidePosition.DOWN, 0);
                intake.setSpinnerPower(0);
                intake.extendIntake(Intake.IntakeExtension.RETRACTED, 0);
                break;
            default:
                break;
            case AUTO_ExecutePath:
                if (PathDetails.elapsedTime_ms.milliseconds() > PathDetails.pathTime_ms) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathManager.inTargetZone && lift.liftAtTargetPosition && lift.slideAtTargetPosition && intake.intakeAtTargetPosition) {
                    pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else if (PathDetails.elapsedTime_ms.milliseconds()>1000000 && RobotPose.isRobotAtRest()) { // robot at rest after first moving (1000 ms)
                    if (PathDetails.elapsedTime_ms.milliseconds() > Math.max(Math.max(PathDetails.xFieldDelay_ms, PathDetails.yFieldDelay_ms), PathDetails.turnFieldDelay_ms) + 10.0)
                        pm_state = PM_STATE.AUTO_NEXT_PATH;
                } else {
                    if(PathDetails.elapsedTime_ms.milliseconds() > liftFieldDelay_ms) {
                        lift.moveLift(PathDetails.liftPosition, liftPower);
                    } else {
                        lift.moveLift(PathDetails.liftPosition, 0);
                    }

                    if(PathDetails.elapsedTime_ms.milliseconds() > slideFieldDelay_ms) {
                       lift.moveSlideTo(slidePosition);
                    }

                    if(PathDetails.elapsedTime_ms.milliseconds() > intakeFieldDelay_ms) {
                        intake.moveIntakeTo(intakePosition, intakePower);
                    } else {
                        intake.moveIntakeTo(intakePosition, 0);
                    }

                    if(PathDetails.elapsedTime_ms.milliseconds() > extensionFieldDelay_ms) {
                        intake.extendIntake(intakeExtension, intakeExtensionPower);
                    } else {
                        intake.extendIntake(intakeExtension, 0);
                    }

                    intake.setSpinnerPower(-1);
                    PathManager.moveRobot();

                    /*
                    if(PathDetails.autoPathList.get(currentPath) == PathDetails.Path.DEPOSIT_SAMPLE) {
                        powerDown();
                    }
                    */

                }

                lift.updateLift();
                break;
            case AUTO_NEXT_PATH:
                setNextPath();
                if (nextPath < 0) {
                    //nextPath = 0;
                    //pm_state = PM_STATE.AUTO_SET_PATH; // repeat from the beginning
                    pm_state = PM_STATE.DONE;
                } else {
                    pm_state = PM_STATE.AUTO_SET_PATH;
                }
                break;
        }   // end switch (state)
    }   // end method update
    static void powerDown() {
        DriveTrain.setMotorPowers(0, 0, 0, 0);
    }
    static void powerDown(double minPower) throws InterruptedException {
        DriveTrain.setMotorPowers(minPower, minPower, minPower, minPower);
    }
    private static void setNextPath() {
        nextPath++;
        if (nextPath >= PathDetails.autoPathList.size()) {
            nextPath = -1;
        }
        currentPath = nextPath;
    }   // end method setNextPath
}


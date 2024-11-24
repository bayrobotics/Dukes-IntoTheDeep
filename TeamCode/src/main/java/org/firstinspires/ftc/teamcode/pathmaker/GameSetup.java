//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.pathmaker;

public class GameSetup {
    public static boolean SIMULATION = false; // used to switch to simulation mode
    public static Terminal terminal = Terminal.CLOSE;

    public enum RobotModel {ROBOT1,ROBOT2}
    public static RobotModel robotModel = RobotModel.ROBOT1;
    public enum Terminal {CLOSE,FAR}
    public static Terminal thisTerminal = Terminal.CLOSE;
    public enum TeamColor {RED,BLUE}
    public static TeamColor thisTeamColor = TeamColor.BLUE;

    public int terminalSign(TeamColor teamColor){
        if (teamColor == TeamColor.RED){
            return 1;
        } else {
            return -1;
        }

    }
}

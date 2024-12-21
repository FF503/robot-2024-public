package org.frogforce503.robot2024;

import java.util.Set;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotStatus {
    private static final RobotStatus instance = new RobotStatus();
    private Bot currentRobot;
    private GameState gameState = GameState.DISABLED;
    private AllianceColor allianceColor = AllianceColor.RED;
    private boolean allianceColorBeenOverriden = false;

    public static RobotStatus getInstance() {
        return instance;
    }

    public GameState getGameState() {
        return gameState;
    }

    public void setGameState(GameState gamestate) {
        this.gameState = gamestate;
    }

    public boolean isFMSInfoAvailable() {
        Set<String> str = NetworkTableInstance.getDefault().getTable("FMSInfo").getKeys();
        String out = "FMS KEYS: ";
        for (String s : str) {
            out += s;
        }
        System.out.println(out);
        return false;
    }

    public AllianceColor getAllianceColor() {
        if (allianceColorBeenOverriden || DriverStation.getAlliance().isEmpty())
            return this.allianceColor;

        if (DriverStation.getAlliance().get() == Alliance.Red)
            return AllianceColor.RED;
        else
            return AllianceColor.BLUE;
    }

    public void overrideAllianceColor(AllianceColor color) {
        allianceColorBeenOverriden = true;
        this.allianceColor = color;
    }

    public Bot getCurrentRobot() {
        return this.currentRobot;
    }

    public void setCurrentRobot(final Bot currentRobot) {
        this.currentRobot = currentRobot;
    }

    public enum GameState {
        AUTON, TELEOP, TEST, DISABLED
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public enum Bot {
        Automatic, CompBot, PracticeBot, ProgrammingBot
    }
}
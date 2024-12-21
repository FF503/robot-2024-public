package org.frogforce503.robot2024.subsystems.drive;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.subsystems.drive.Drive.ModuleLocation;
import org.frogforce503.robot2024.subsystems.drive.modules.BaseSwerveModule;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj.Filesystem;

public class SwerveModuleLoader {

    private static SwerveModuleLoader instance = null;
    private static final ModuleLoadingMethod LOADING_METHOD = ModuleLoadingMethod.PHOENIX_TUNER;

    public static SwerveModuleLoader getInstance() {
        if (instance == null)
            instance = new SwerveModuleLoader();

        return instance;
    }

    private JSONObject moduleSet;

    private SwerveModuleLoader() {
        // try {
        //     loadModuleInfo();
        // } catch (IOException | ParseException e) {
        //     e.printStackTrace();
        // }
    }

    private void loadModuleInfo() throws FileNotFoundException, IOException, ParseException {
        moduleSet = (JSONObject) new JSONParser().parse(new FileReader(
                Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules.json"));
    }

    private SwerveModuleConfig loadConfig(String moduleName) {
        JSONObject moduleObject = (JSONObject) moduleSet.get(moduleName);

        int driveMotorID = Math.toIntExact((Long) moduleObject.get("driveMotorID"));
        int turnMotorID = Math.toIntExact((Long) moduleObject.get("turnMotorID"));
        int turnEncoderID = moduleObject.containsKey("turnEncoderID") ? Math.toIntExact((Long) moduleObject.get("turnEncoderID")) : -1;
        double turn_kP = (double) moduleObject.get("turn_kP");
        double turn_kI = (double) moduleObject.get("turn_kI");
        double turn_kD = (double) moduleObject.get("turn_kD");
        double turn_kF = (double) moduleObject.get("turn_kF");
        double drive_kP = (double) moduleObject.get("drive_kP");
        double drive_kI = (double) moduleObject.get("drive_kI");
        double drive_kD = (double) moduleObject.get("drive_kD");
        double drive_KF = (double) moduleObject.get("drive_kF");
        double absoluteZeroDegrees = (double) moduleObject.get("absoluteZeroDegrees");
        boolean driveInvert = (boolean) moduleObject.get("driveInverted");
        boolean turnInvert = (boolean) moduleObject.get("turnInverted");

        return new SwerveModuleConfig(driveMotorID, turnMotorID, turnEncoderID, turn_kP, turn_kI, turn_kD, turn_kF, drive_kP, drive_kI, drive_kD, drive_KF, absoluteZeroDegrees, driveInvert, turnInvert);
    }

    public void setupModule(BaseSwerveModule module) {
        module.configure(loadConfig(module.moduleName));
    }

    public SwerveModuleConstants loadPhoenixModule(String moduleName, ModuleLocation location) {
        SwerveModuleConfig config = loadConfig(moduleName);

        return Robot.bot.constantCreator.createModuleConstants(
            config.turnMotorID, 
            config.driveMotorID, 
            config.turnEncoderID, 
            (config.absoluteZeroDegrees / 360),
            Robot.bot.kModulePositions[location.index].getX(),
            Robot.bot.kModulePositions[location.index].getY(),
            location == ModuleLocation.FrontLeft || location == ModuleLocation.BackLeft ? Robot.bot.kInvertLeftSide : Robot.bot.kInvertRightSide
        );
    }

    public SwerveModuleConstants[] loadPhoenixModules() {
        SwerveModuleConstants[] modules = new SwerveModuleConstants[4];

        // switch (LOADING_METHOD) {
        //     case PHOENIX_TUNER: {
                modules = new SwerveModuleConstants[] { Robot.bot.frontLeftConstants, Robot.bot.frontRightConstants, Robot.bot.backLeftConstants, Robot.bot.backRightConstants };
        //         break;
        //     }
        //     case FROG_FORCE: {
        //         modules = new SwerveModuleConstants [] { 
        //             loadPhoenixModule(Robot.bot.frontLeftName, ModuleLocation.FrontLeft),
        //             loadPhoenixModule(Robot.bot.frontRightName, ModuleLocation.FrontRight),
        //             loadPhoenixModule(Robot.bot.backLeftName, ModuleLocation.BackLeft),
        //             loadPhoenixModule(Robot.bot.backRightName, ModuleLocation.BackRight)
        //         };
        //         break;
        //     }
        // }

        return modules;
    }

    public class SwerveModuleConfig {
        public int driveMotorID;
        public int turnMotorID;
        public int turnEncoderID;
        public double turn_kP;
        public double turn_kI;
        public double turn_kD;
        public double turn_kF;
        public double drive_kP;
        public double drive_kI;
        public double drive_kD;
        public double drive_kF;
        public double absoluteZeroDegrees;
        public boolean driveInvert;
        public boolean turnInvert;

        public SwerveModuleConfig(int driveMotorID, int turnMotorID, int turnEncoderID, 
            double turn_kP, double turn_kI, double turn_kD, 
            double turn_kF, double drive_kP, double drive_kI, 
            double drive_kD, double drive_kF, double absoluteZeroDegrees, 
            boolean driveInvert, boolean turnInvert) {
                this.driveMotorID = driveMotorID;
                this.turnMotorID = turnMotorID;
                this.turnEncoderID = turnEncoderID;
                this.turn_kP = turn_kP;
                this.turn_kI = turn_kI;
                this.turn_kD = turn_kD;
                this.turn_kF = turn_kF;
                this.drive_kP = drive_kP;
                this.drive_kI = drive_kI;
                this.drive_kD = drive_kD;
                this.drive_kF = drive_kF;
                this.absoluteZeroDegrees = absoluteZeroDegrees;
                this.driveInvert = driveInvert;
                this.turnInvert = turnInvert;
        }
    }

    private static enum ModuleLoadingMethod {
        PHOENIX_TUNER,
        FROG_FORCE
    }
}
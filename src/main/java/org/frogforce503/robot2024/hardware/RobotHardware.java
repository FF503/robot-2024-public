package org.frogforce503.robot2024.hardware;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import javax.xml.transform.Transformer;

import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.Bot;
import org.frogforce503.robot2024.subsystems.drive.Drive.DrivetrainType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public abstract class RobotHardware {

    private static RobotHardware instance = null;

    //Fake vals for getting multiple cameras. Replace with real valeus later
    public Transform3d FRONT_LEFT_CAMERA_TO_CENTER; /* z */
    public Transform3d FRONT_RIGHT_CAMERA_TO_CENTER;
    public Transform3d BACK_LEFT_CAMERA_TO_CENTER;
    public Transform3d BACK_RIGHT_CAMERA_TO_CENTER;
    public Transform3d BACK_CAMERA_TO_CENTER;
    // public Transform3d FRONTLEFT_CAMERA_TO_CENTER;
    // public Transform3d FRONTRIGHT_CAMERA_TO_CENTER;
    // public Transform3d BACKLEFT_CAMERA_TO_CENTER;
    // public Transform3d BACKRIGHT_CAMERA_TO_CENTER;
    public Transform3d NOTE_DETECTION_CAMERA_TO_CENTER;
    
    // Arm
    public int leftArmZeroSwitchID;
    public int rightArmZeroSwitchID;
    public int armID;
    public double armP;
    public double armI;
    public double armD;
    public double armFF;
    public double armP_SLOT1;
    public double armI_SLOT1;
    public double armD_SLOT1;
    public double armFF_SLOT1;
    public boolean armInverted;

    // Wrist
    public int leftWristZeroSwitchID;
    public int wristID;
    public double wristP;
    public double wristI;
    public double wristD;
    public double wristFF;
    public double wristP_SLOT1;
    public double wristI_SLOT1;
    public double wristD_SLOT1;
    public double wristFF_SLOT1;
    public double wristA;
    public double wristG;
    public double wristS;
    public double wristV;
    public double wristA_SLOT1;
    public double wristG_SLOT1;
    public double wristS_SLOT1;
    public double wristV_SLOT1;
    public boolean wristInverted;

    // Intake
    public int intakeID;
    public double intakeP;
    public double intakeI;
    public double intakeD;
    public double intakeFF;

    // Shooter
    public int feederID;
    public int topShooterWheelID;
    // public boolean topShooterWheel
    public int bottomShooterWheelID;
    public int entryBeamBreakID;
    public int exitBeamBreakID;
    public int actuatorID;
    public double feederP;
    public double feederI;
    public double feederD;
    public double feederFF;
    public double shooterP;
    public double shooterI;
    public double shooterD;
    public double shooterKs;
    public double shooterKv;
    public double shooterKa;
    public boolean shooterInverted;

    // Climber
    public int leftClimberID;
    public int rightClimberID;
    public int leftClimberZeroSwitchID;
    public int rightClimberZeroSwitchID;
    public double climberP;
    public double climberI;
    public double climberD;
    public double climberFF;

    // CANdle
    public int candleID;

    // Constants
    // SwerveFileNames
    public String backLeftName;
    public String backRightName;
    public String frontLeftName;
    public String frontRightName;

    public int gyroID;
    public GYRO_TYPE gyroType;

    public DrivetrainType drivetrainType;
    public SWERVE_MODULE_TYPE moduleType;
    public String swerveCANBus;

    public SwerveDrivetrainConstants phoenixConstants;
    public SwerveModuleConstants frontLeftConstants;
    public SwerveModuleConstants backLeftConstants;
    public SwerveModuleConstants frontRightConstants;
    public SwerveModuleConstants backRightConstants;
    public SwerveModuleConstantsFactory constantCreator;
    public boolean kInvertLeftSide;
    public boolean kInvertRightSide;
    

    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;
    public double wheelDiameter;

    public double fullRobotWidth; // with bumpers
    public double fullRobotLength; // with bumpers

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToFrontLeft;
    public Translation2d kVehicleToBackLeft;
    public Translation2d[] kModulePositions;

    public SwerveDriveKinematics kinematics;

    public Translation2d kCenterDrivebaseToCenterRobot = new Translation2d(); // 0 if regularly shaped
    public double kFrameAndBumperSize;

    // Auto Following PID
    public PIDController autoXController;
    public PIDController autoYController;
    public PIDController autoThetaController;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            if (RobotStatus.getInstance().getCurrentRobot().equals(Bot.Automatic)) {
                RobotStatus.getInstance().setCurrentRobot(Util.parseRobotNameToEnum(Util.readRobotName()));
            }
            switch (RobotStatus.getInstance().getCurrentRobot()) {
                case CompBot:
                    instance = new RobotHardwareCompBot();
                    break;
                case PracticeBot:
                    instance = new RobotHardwarePracticeBot();
                    break;
                case ProgrammingBot:
                    instance = new RobotHardwareProgrammingBot();
                    break;
                case Automatic:
                default:
                    System.err.println("Robot should not be set to automatic... something went wrong");
                    break;
            }
            instance.initializeConstants();
            // Util.setPseudoInverseForwardKinematicsMatrix();
        }
        return instance;
    }

    public static enum SWERVE_MODULE_TYPE {
        SDS_L1(Units.feetToMeters(13.5), 8.14), 
        SDS_L2(Units.feetToMeters(16.3), 6.75),
        SDS_L3(Units.feetToMeters(18), 6.12),
        ANDY(Units.feetToMeters(11), (12.0 / 40.0) * (20.0 / 40.0));

        public double MAX_DRIVE_SPEED_METERS_SEC;
        public double driveGearRatio;

        SWERVE_MODULE_TYPE(double max, double gs) {
            MAX_DRIVE_SPEED_METERS_SEC = max;
            driveGearRatio = gs;
        }
    }

    public static enum GYRO_TYPE {
        PIGEON_1, PIGEON_2
    }

    public abstract void initializeConstants();

    protected void calculateDimensions() {
        this.fullRobotLength = this.kWheelbaseLength + (kFrameAndBumperSize)*2;
        this.fullRobotWidth = this.kWheelbaseWidth + (kFrameAndBumperSize)*2;
    }
}
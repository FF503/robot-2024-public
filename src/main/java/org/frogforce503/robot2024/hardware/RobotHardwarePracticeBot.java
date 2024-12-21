package org.frogforce503.robot2024.hardware;

import org.frogforce503.robot2024.hardware.tunerconstants.TunerConstantsPracticeBot;
import org.frogforce503.robot2024.subsystems.drive.Drive.DrivetrainType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotHardwarePracticeBot extends RobotHardware {

    @Override
    public void initializeConstants() {

        FRONT_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.2246), Units.inchesToMeters(11.569), Units.inchesToMeters(11.792)), /* z 20.25 */
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(15))
        );

        FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.2246), Units.inchesToMeters(-11.569), Units.inchesToMeters(11.792)), /* z 20.25 */
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(-15))
        );

        BACK_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.24342), Units.inchesToMeters(7.39966), Units.inchesToMeters(12.5279)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(90 + 30))
        );

        BACK_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.24342), Units.inchesToMeters(-7.39966), Units.inchesToMeters(12.5279)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-26), Units.degreesToRadians(-90 - 30))
        );

        BACK_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.585), Units.inchesToMeters(0), Units.inchesToMeters(16.543)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180))
        );
        
        NOTE_DETECTION_CAMERA_TO_CENTER = new Transform3d(
                new Translation3d(Units.inchesToMeters(17.625), Units.inchesToMeters(1.25), Units.inchesToMeters(15.75)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-19) /* positive value up for this camera */, Units.degreesToRadians(0)));
        
        // Arm
        leftArmZeroSwitchID = 0;
        rightArmZeroSwitchID = 1;
        armID = 5; 
        armP = 0.05;
        armI = 0.0;
        armD = 0.005;
        armFF = 0.0;
        armInverted = true;

        // Wrist
        leftWristZeroSwitchID = 6;
        wristID = 6; 
        wristP = 0.016;
        wristI = 0.00001;
        wristD = 0.0;
        wristFF = 0.0;
        wristP_SLOT1 = 0.03;
        wristI_SLOT1 = 0.00005;
        wristD_SLOT1 = 0.0;
        wristFF_SLOT1 = 0.0;
        wristA = 0.0;
        wristG = 0.001;
        wristS = 0.0;
        wristV = 0.0;
        wristA_SLOT1 = 0.0;
        wristG_SLOT1 = 0.02;
        wristS_SLOT1 = 0.0;
        wristV_SLOT1 = 0.0;
        wristInverted = false;

        // Intake
        intakeID = 3;
        intakeP = 0.0001;
        intakeI = 0.0;
        intakeD = 0.0;
        intakeFF = 0.00019;

        // Shooter
        feederID = 4;
        topShooterWheelID = 7; 
        bottomShooterWheelID = 8;
        entryBeamBreakID = 4;
        exitBeamBreakID = 9;
        actuatorID = 1;
        feederP = 0.0;              
        feederI = 0.0;
        feederD = 0.0;
        feederFF = 0.0;
        shooterP = 0.0004;
        shooterI = 0.0;
        shooterD = 0.000;
        shooterKs = 0.88247;
        shooterKv = 0.0002;
        shooterKa = 0.0;
        shooterInverted = false;

        // Climber
        leftClimberID = 9;
        rightClimberID = 10;
        leftClimberZeroSwitchID = 2;
        rightClimberZeroSwitchID = 3;
        climberP = 0.0;
        climberI = 0.0;
        climberD = 0.0;
        climberFF = 0.0;

        // CANdle
        candleID = 11;

        // this.backLeftName = "SDS12"; // SDS8
        // this.backRightName = "SDS13"; // SDS1
        // this.frontLeftName = "SDS11"; // SDS10
        // this.frontRightName = "SDS14"; // SDS3

        this.drivetrainType = DrivetrainType.PHOENIX_SWERVE;
        this.moduleType = SWERVE_MODULE_TYPE.SDS_L3;
        this.swerveCANBus = "CANivore";

        this.gyroID = 9;
        this.gyroType = GYRO_TYPE.PIGEON_2;

        this.phoenixConstants = TunerConstantsPracticeBot.DrivetrainConstants;
        this.frontLeftConstants = TunerConstantsPracticeBot.FrontLeft;
        this.frontRightConstants = TunerConstantsPracticeBot.FrontRight;
        this.backLeftConstants = TunerConstantsPracticeBot.BackLeft;
        this.backRightConstants = TunerConstantsPracticeBot.BackRight;

        this.constantCreator = TunerConstantsPracticeBot.ConstantCreator;
        this.kInvertLeftSide = TunerConstantsPracticeBot.kInvertLeftSide;
        this.kInvertRightSide = TunerConstantsPracticeBot.kInvertRightSide;

        // full robot size is 32.625 x 36.625 inches (with bumpers)
        this.wheelDiameter = Units.inchesToMeters(4);

        // Swerve Module Positions (relative to the center of the drive base)
        this.kVehicleToFrontLeft = new Translation2d(this.frontLeftConstants.LocationX, this.frontLeftConstants.LocationY);
        this.kVehicleToFrontRight = new Translation2d(this.frontRightConstants.LocationX, this.frontRightConstants.LocationY);
        this.kVehicleToBackRight = new Translation2d(this.backRightConstants.LocationX, this.backRightConstants.LocationY);
        this.kVehicleToBackLeft = new Translation2d(this.backLeftConstants.LocationX, this.backLeftConstants.LocationY);

        this.kWheelbaseLength = kVehicleToFrontLeft.getDistance(kVehicleToBackLeft); 
        this.kWheelbaseWidth = kVehicleToFrontLeft.getDistance(kVehicleToFrontRight); 

        this.kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight,
                kVehicleToBackLeft, kVehicleToBackRight };

        this.kinematics = new SwerveDriveKinematics(kModulePositions);

        this.kCenterDrivebaseToCenterRobot = new Translation2d(Units.inchesToMeters(2.25), 0);
        this.kFrameAndBumperSize = Units.inchesToMeters(2.625 + 3); // 2.626 from center of wheel to frame and then 3 inch bumpers

        this.fullRobotLength = Units.inchesToMeters(33.5);
        this.fullRobotWidth = Units.inchesToMeters(33.5);
        
        // this.autoXController = new PIDController(4.0, 0.0, 0.0); //TODO
        // this.autoYController = new PIDController(4.0, 0.0, 0.0);
        // this.autoThetaController = new PIDController(4.0, 0.0, 0.0);

        this.autoXController = new PIDController(6.0, 0.0, 0.0);
        this.autoYController = new PIDController(6.0, 0.0, 0.0);
        this.autoThetaController = new PIDController(6.0, 0.0, 0.0);

        this.calculateDimensions();
    }    
}
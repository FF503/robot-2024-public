package org.frogforce503.robot2024.hardware;

import org.frogforce503.robot2024.hardware.tunerconstants.TunerConstantsCompBot;
import org.frogforce503.robot2024.subsystems.drive.Drive.DrivetrainType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotHardwareCompBot extends RobotHardware {

    @Override
    public void initializeConstants() {

        FRONT_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.2246 + 0.5) + Units.inchesToMeters(0.5), Units.inchesToMeters(11.569), Units.inchesToMeters(11.792)), /* z 20.25 */
            new Rotation3d(0, Units.degreesToRadians(-26), 0)
                .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(15)))
        );

        FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.2246 + 0.5), Units.inchesToMeters(-11.569), Units.inchesToMeters(11.792)), /* z 20.25 */
            new Rotation3d(0, Units.degreesToRadians(-26), 0)
                .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(-15)))
        );

        BACK_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.24342), Units.inchesToMeters(7.39966), Units.inchesToMeters(12.5279)),
            new Rotation3d(0, Units.degreesToRadians(-26), 0)
                .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(90 + 30)))
        );

        BACK_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.24342), Units.inchesToMeters(-7.39966), Units.inchesToMeters(12.5279)),
            new Rotation3d(0, Units.degreesToRadians(-26), 0)
                .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(-90 - 30)))
        );

        BACK_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.976757), Units.inchesToMeters(0), Units.inchesToMeters(16.18954)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180))
        );
     
        NOTE_DETECTION_CAMERA_TO_CENTER = new Transform3d(
                new Translation3d(Units.inchesToMeters(17.009), Units.inchesToMeters(0), Units.inchesToMeters(16.197)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-19) /* positive value up for this camera */, Units.degreesToRadians(0)));
        
        // Arm
        leftArmZeroSwitchID = 0;
        rightArmZeroSwitchID = 1;
        armID = 5; 
        armP = 0.1;
        armI = 0.0;
        armD = 0.0;
        armFF = 0.0;
        armP_SLOT1 = 0.05;
        armI_SLOT1 = 0.0;
        armD_SLOT1 = 0.005;
        armFF_SLOT1 = 0.0;
        armInverted = true;

        // Wrist
        leftWristZeroSwitchID = 6;
        wristID = 6; 
        wristP = 0.0155; // 0.008; // 0.016 // 0.0275; // 0.04
        wristI = 0.00001;//0.00004;
        wristD = 0.0;//0.0015; // 0.064
        wristFF = 0.0;
        wristP_SLOT1 = 0.0225; //0.03;
        wristI_SLOT1 = 0.00005;
        wristD_SLOT1 = 0.0;
        wristFF_SLOT1 = 0.0;
        wristA = 0.0;
        wristG = 0.00; // 0.001
        wristS = 0.0;
        wristV = 0.0;
        wristA_SLOT1 = 0.0;
        wristG_SLOT1 = 0.0; // 0.02
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
        entryBeamBreakID = 4;   //was 7
        exitBeamBreakID = 9;    //was 8, before 4
        actuatorID = 1;
        feederP = 0.0;
        feederI = 0.0;
        feederD = 0.0;
        feederFF = 0.0;
        shooterP = 0.0004;
        shooterI = 0.0;
        shooterD = 0.000;
        shooterKs = 0.88247;
        shooterKv = 0.0002; // 0.17325
        shooterKa = 0.0; // 0.059657
        shooterInverted = true;

        // Climber
        leftClimberID = 9; 
        rightClimberID = 10; // wrong can 
        leftClimberZeroSwitchID = 2;
        rightClimberZeroSwitchID = 3;
        climberP = 0.0;
        climberI = 0.0;
        climberD = 0.0;
        climberFF = 0.0;

        // CANdleP  
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

        this.phoenixConstants = TunerConstantsCompBot.DrivetrainConstants;
        this.frontLeftConstants = TunerConstantsCompBot.FrontLeft;
        this.frontRightConstants = TunerConstantsCompBot.FrontRight;
        this.backLeftConstants = TunerConstantsCompBot.BackLeft;
        this.backRightConstants = TunerConstantsCompBot.BackRight;

        this.constantCreator = TunerConstantsCompBot.ConstantCreator;
        this.kInvertLeftSide = TunerConstantsCompBot.kInvertLeftSide;
        this.kInvertRightSide = TunerConstantsCompBot.kInvertRightSide;

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

        this.autoXController = new PIDController(6.0, 0.0, 0.3);
        this.autoYController = new PIDController(6.0, 0.0, 0.3);
        this.autoThetaController = new PIDController(10.0, 0.0, 0.0);

        this.calculateDimensions();
    }    
}
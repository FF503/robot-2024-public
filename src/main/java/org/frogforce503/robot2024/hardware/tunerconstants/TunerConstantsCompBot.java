package org.frogforce503.robot2024.hardware.tunerconstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

public class TunerConstantsCompBot {
   // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0);
        // .withKP(69.158).withKI(0).withKD(5.1575)
        // .withKS(1.9236).withKV(0.44212).withKA(0.44212);
        // .withKP(0.73029).withKI(0).withKD(0.0)  
        // // .withKP(100).withKI(0).withKD(0.2)
        // .withKS(0.15217).withKV(2.4164).withKA(0.069301);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
        // .withKP(3).withKI(0).withKD(0)
        .withKP(0.28486).withKI(0).withKD(0) // 0.4156, 0, 0
        // .withKS(0).withKV(0).withKA(0);
        .withKS(1.5202).withKV(0.15396).withKA(0.47802); // 0, 0.27228, 0.53928

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double kSlipCurrentA = 650.0; //300

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 6.5;//Units.feetToMeters(19);// 6.5; // 5.59;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.125;

    public static final double kDriveGearRatio = 5.357142857142857;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final double kWheelRadiusInches = 1.875;

    public static final boolean kSteerMotorReversed = true;
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final String kCANbusName = "Default Name";
    public static final int kPigeonId = 9;


    // These are only used for simulation
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public static final double kSteerFrictionVoltage = 0.25;
    public static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);



    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = 0.453125;

    private static final double kFrontLeftXPosInches = 10.37675;
    private static final double kFrontLeftYPosInches = 10.37675;

    // Front Right
    private static final int kFrontRightDriveMotorId = 13;
    private static final int kFrontRightSteerMotorId = 14;
    private static final int kFrontRightEncoderId = 15;
    private static final double kFrontRightEncoderOffset = 0.09033203125;

    private static final double kFrontRightXPosInches = 10.37675;
    private static final double kFrontRightYPosInches = -10.37675;

    // Back Left
    private static final int kBackLeftDriveMotorId = 16;
    private static final int kBackLeftSteerMotorId = 17;
    private static final int kBackLeftEncoderId = 18;
    private static final double kBackLeftEncoderOffset = 0.44580078125;

    private static final double kBackLeftXPosInches = -10.37675;
    private static final double kBackLeftYPosInches = 10.37675;

    // Back Right
    private static final int kBackRightDriveMotorId = 19;
    private static final int kBackRightSteerMotorId = 20;
    private static final int kBackRightEncoderId = 21;
    private static final double kBackRightEncoderOffset = 0.394287109375;

    private static final double kBackRightXPosInches = -10.37675;
    private static final double kBackRightYPosInches = -10.37675;

    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
}

package org.frogforce503.robot2024.subsystems.drive.modules;

import org.frogforce503.lib.drivers.TalonFXWrapper;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.hardware.RobotHardware.SWERVE_MODULE_TYPE;
import org.frogforce503.robot2024.subsystems.drive.SwerveModuleLoader.SwerveModuleConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class FalconSwerveModule extends BaseSwerveModule {

    private TalonFXWrapper driveMotor, turnMotor;
    private CANCoder turnEncoder;
    private SwerveModuleConfig config;

    private static final double kTurnEncoderClicksperRevolution = 2048;
    private static final double turnGearRatio = 7.0 / 150.0;
    private static final double kTurnMotorEncoderPositionCoefficientRadiansPerCount = (2.0 * Math.PI
            / kTurnEncoderClicksperRevolution)
            * turnGearRatio;
    private static final double kTurnMotorEncoderVelocityCoefficient = kTurnMotorEncoderPositionCoefficientRadiansPerCount
            * 10;

    public static final double MAX_DRIVE_SPEED_METERS_SEC = Robot.bot.moduleType.MAX_DRIVE_SPEED_METERS_SEC;
    private static final int kSlotIdx = 0;
    private static final int kTimeoutMs = 100;

    private final double driveGearRatio = Robot.bot.moduleType.driveGearRatio; //6.12; // Unitless (FOR L1 on SDS Mk4i. FF 6 is 7.125) //FIXME
    private static final double kWheelDiameter = Robot.bot.wheelDiameter;
    private final double CIRCUMFERENCE = (Math.PI * kWheelDiameter); // meters for 1 rotation
    private final double CLICKS_PER_REV_DRIVE_MOTOR = 2048;
    private final double metersToDriveClicks = ((CLICKS_PER_REV_DRIVE_MOTOR / CIRCUMFERENCE) * driveGearRatio);
    private final double metersPerSecondToDriveClicksPer100Millisecond = metersToDriveClicks / 10; // ((1 rotation / circumference (neters)) * 2048) / 10

    double resetIteration = 0;
    double preResetCanCoder = 0;
    private static final int ENCODER_RESET_ITERATIONS = (int) (50);

    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    public FalconSwerveModule(String moduleName, ModuleLocation location) {
        super(moduleName, location, Robot.bot.moduleType);

        driveMotor = new TalonFXWrapper(config.driveMotorID, Robot.bot.swerveCANBus);
        turnMotor = new TalonFXWrapper(config.turnMotorID,  Robot.bot.swerveCANBus);
        turnEncoder = new CANCoder(config.turnEncoderID, Robot.bot.swerveCANBus);

        // driveFeedforward = new SimpleMotorFeedforward((0.32 / 12), (1.51 / 12), (0.27 / 12));
        // driveFeedforward = new SimpleMotorFeedforward(0.204596, 2.276200);

        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        turnEncoder.configFactoryDefault();

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Coast);

        // configure drive motor
        driveMotor.setInverted(config.driveInvert);
        // driveMotor.setSensorPhase(driveEncInvert);

        // setup velocity pid
        driveMotor.configNeutralDeadband(0.001);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        // Config the peak and nominal outputs
        driveMotor.configNominalOutputForward(0, kTimeoutMs);
        driveMotor.configNominalOutputReverse(0, kTimeoutMs);
        driveMotor.configPeakOutputForward(1, kTimeoutMs);
        driveMotor.configPeakOutputReverse(-1, kTimeoutMs);

        // Config the Velocity closed loop gains in slot0
        driveMotor.config_kP(kSlotIdx, config.drive_kP, kTimeoutMs);
        driveMotor.config_kI(kSlotIdx, config.drive_kI, kTimeoutMs);
        driveMotor.config_kD(kSlotIdx, config.drive_kD, kTimeoutMs);
        driveMotor.config_kF(kSlotIdx, config.drive_kF, kTimeoutMs);
        
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 85, 1.0));

        // driveMotor.configClosedloopRamp(0.05);
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 85, 1.0));

        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        turnMotor.selectProfileSlot(kSlotIdx, 0);
        turnMotor.config_kF(kSlotIdx, config.turn_kF, kTimeoutMs);
        turnMotor.config_kP(kSlotIdx, config.turn_kP, kTimeoutMs);
        turnMotor.config_kI(kSlotIdx, config.turn_kI, kTimeoutMs);
        turnMotor.config_kD(kSlotIdx, config.turn_kD, kTimeoutMs);
        turnMotor.setInverted(true);
        turnMotor.setSensorPhase(false);
        turnMotor.configAllowableClosedloopError(0,
                Math.toRadians(0.5) / kTurnMotorEncoderPositionCoefficientRadiansPerCount,
                kTimeoutMs); // previously 50 for mk6 then 36 for mk4i

        turnMotor.configMotionCruiseVelocity(15000, kTimeoutMs);
        turnMotor.configMotionAcceleration(6000, kTimeoutMs);

        turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));

        turnEncoder.configMagnetOffset(0, 0);

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // System.out.println(moduleName);
        // System.out.println(turnEncoder.getAbsolutePosition());

        turnEncoder.configMagnetOffset(config.absoluteZeroDegrees, 0);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // System.out.println(turnEncoder.getAbsolutePosition());
        turnEncoder.configSensorDirection(false);

        turnMotor.setSelectedSensorPosition(
                (Math.toRadians(turnEncoder.getAbsolutePosition()))
                        / kTurnMotorEncoderPositionCoefficientRadiansPerCount);

        // turnEncoder.setPosition(turnEncoder.getAbsolutePosition());

        if (RobotBase.isSimulation())
            this.simulationInit();
    }

    private void simulationInit() {}

    @Override
    public void configure(SwerveModuleConfig config) {
        this.config = config;
    }

    @Override
    public void initialize() { }

    @Override
    public void update() {
        if (RobotBase.isSimulation())
            this.simulationPeriodic();        
    }

    @Override
    public void setRotationSetpoint(Rotation2d referenceAngle) {
        double currentAngleRadians = turnMotor.getSelectedSensorPosition()
                * kTurnMotorEncoderPositionCoefficientRadiansPerCount;
        double referenceAngleRadians = referenceAngle.getRadians();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.

        if (Math.abs(turnMotor.getSelectedSensorVelocity())
                * kTurnMotorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngleRadians = Math.toRadians(turnEncoder.getAbsolutePosition());
                preResetCanCoder = turnEncoder.getAbsolutePosition();
                turnMotor.setSelectedSensorPosition(
                        absoluteAngleRadians / kTurnMotorEncoderPositionCoefficientRadiansPerCount);
                currentAngleRadians = absoluteAngleRadians;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
        // above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        turnMotor.set(ControlMode.Position,
                adjustedReferenceAngleRadians / kTurnMotorEncoderPositionCoefficientRadiansPerCount);
    }

    @Override
    public void setDriveSetpoint(double speedMetersPerSecond) {
        // if (controlMode == ControlMode.Velocity) {
            speedMetersPerSecond *= metersPerSecondToDriveClicksPer100Millisecond;
            driveMotor.set(ControlMode.Velocity, speedMetersPerSecond/*, DemandType.ArbitraryFeedForward, driveFeedforward.calculate(outputValue)*/);   
        // }
        // driveMotor.set(controlMode, outputValue);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public Rotation2d getAngle() {
        double outputAngleRad = (turnMotor.getSelectedSensorPosition()
                * kTurnMotorEncoderPositionCoefficientRadiansPerCount);

        // if (RobotBase.isSimulation())
        //     outputAngleRad = _simTurnEncoderPosition;

        outputAngleRad %= Math.PI * 2.0;

        if (outputAngleRad < 0.0)
            outputAngleRad += Math.PI * 2.0;

        return new Rotation2d(outputAngleRad);
    }

    @Override
    public double getVelocity() {
        return driveMotor.getSelectedSensorVelocity() / metersPerSecondToDriveClicksPer100Millisecond;
    }

    @Override
    public double getDistance() {
        return driveMotor.getSelectedSensorPosition() / metersToDriveClicks;
    }
    
}

package org.frogforce503.robot2024.subsystems.drive.modules;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.TalonSRXWrapper;
import org.frogforce503.robot2024.hardware.RobotHardware;
import org.frogforce503.robot2024.hardware.RobotHardware.SWERVE_MODULE_TYPE;
import org.frogforce503.robot2024.subsystems.drive.SwerveModuleLoader.SwerveModuleConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class AndymarkSwerveModule extends BaseSwerveModule {

    private TalonSRXWrapper turnMotor;
    private CANSparkMaxWrapper driveMotor;

    private SwerveModuleConfig config;

    private static final double kTurnEncoderClicksperRevolution = 4096;
    private static final double kWheelDiameter = Units.inchesToMeters(4.0);
    private static final double kAzimuthClicksPerDegree = kTurnEncoderClicksperRevolution / 360.0;
    // private final int countsPerRotation = 42; // Counts/Rotation
    private final double driveGearRatio = 1.0 / 5.67; // Unitless
    private final double CIRCUMFERENCE = (Math.PI * kWheelDiameter); // Rotations/inch
    private final double REVS_PER_INCH = (1.0 / driveGearRatio) / CIRCUMFERENCE; // Counts/Inch
    private final double driveVelocityConversionFactor = (REVS_PER_INCH * 60.0); // 60 Inches/Rotation

    private final int ZERO_POSITION;

    public AndymarkSwerveModule(String moduleName, ModuleLocation location) {
        super(moduleName, location, SWERVE_MODULE_TYPE.ANDY);

        turnMotor = new TalonSRXWrapper(config.turnMotorID);
        driveMotor = new CANSparkMaxWrapper(config.driveMotorID, MotorType.kBrushless);

        ZERO_POSITION = (int) config.absoluteZeroDegrees;

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        turnMotor.configFeedbackNotContinuous(true, 20);

        turnMotor.config_kP(0, config.turn_kP, 20);
        turnMotor.config_kI(0, config.turn_kI, 20);
        turnMotor.config_kD(0, config.turn_kD, 20);
        turnMotor.config_kF(0, config.turn_kF, 20);

        driveMotor.setInverted(config.driveInvert);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        
        driveMotor.setP(0, config.drive_kP);
        driveMotor.setFF(0, config.drive_kF);

        driveMotor.setSmartCurrentLimit(50);
        turnMotor.enableCurrentLimit(true);
        turnMotor.configContinuousCurrentLimit(30);

        turnMotor.setInverted(config.turnInvert);

        System.out.println("STARTING AZIMUTH CONFIG");

        int absolutePosition = turnMotor.getSensorCollection().getPulseWidthPosition();
        turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        turnMotor.setSelectedSensorPosition(absolutePosition, 0, 100);
        turnMotor.getSensorCollection().setQuadraturePosition(absolutePosition, 100);

        System.out.println("ENDING AZIMUTH CONFIG " + this.locationName + " abs " + absolutePosition);
    }

    @Override
    public void configure(SwerveModuleConfig config) {
        this.config = config;
    }

    @Override
    public void initialize() { }

    @Override
    public void update() { }

    @Override
    public void setRotationSetpoint(Rotation2d angle) {
        double trueAngle = angle.getDegrees();

        int desiredclicks = (int) Math.round(trueAngle * kAzimuthClicksPerDegree) + ZERO_POSITION;
        
        double pos = turnMotor.getSelectedSensorPosition(); // sensor units
        int rotationNumber = (int) (pos / kTurnEncoderClicksperRevolution);
        if (pos < 0) {
            rotationNumber--;
        }
        int target;
        desiredclicks += rotationNumber * kTurnEncoderClicksperRevolution;

        int desiredclicksPrevRotation = desiredclicks - (int) kTurnEncoderClicksperRevolution;
        int desiredClicksNextRotation = desiredclicks + (int) kTurnEncoderClicksperRevolution;
        target = getClosestPointToTarget((int) pos, desiredclicks, desiredClicksNextRotation,
                desiredclicksPrevRotation);

        turnMotor.set(ControlMode.Position, target);
    }

    private int absoluteDistanceToTarget(int target, int pos) {
        return Math.abs(target - pos);
    }

    private int getClosestPointToTarget(int pos, int t1, int t2, int t3) {
        int d1 = absoluteDistanceToTarget(t1, pos), d2 = absoluteDistanceToTarget(t2, pos),
                d3 = absoluteDistanceToTarget(t3, pos);
        int minD = Math.min(d1, Math.min(d2, d3));
        if (d1 == minD) {
            return t1;
        } else if (d2 == minD) {
            return t2;
        }
        return t3;
    }

    @Override
    public void setDriveSetpoint(double speedMetersPerSecond) {
        driveMotor.set(CANSparkMaxWrapper.ControlMode.PercentOutput, (speedMetersPerSecond / Units.feetToMeters(11.5)));
        // driveMotor.set(CANSparkMaxWrapper.ControlMode.Velocity, Units.metersToInches(speedMetersPerSecond) * driveVelocityConversionFactor);        
    }

    @Override
    public void setBrakeMode(boolean enable) {
        driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition() / kAzimuthClicksPerDegree);
    }

    @Override
    public double getVelocity() {
        return Units.inchesToMeters(driveMotor.getEncoderVelocity() / driveVelocityConversionFactor);
    }

    @Override
    public double getDistance() {
        return Units.inchesToMeters(driveMotor.getEncoderPosition() / REVS_PER_INCH);
    }

}

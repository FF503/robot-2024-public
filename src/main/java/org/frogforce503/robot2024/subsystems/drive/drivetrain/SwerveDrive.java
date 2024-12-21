package org.frogforce503.robot2024.subsystems.drive.drivetrain;

import java.util.function.Consumer;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.swerve.SwerveDriveState;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.subsystems.drive.gyro.BaseGyro;
import org.frogforce503.robot2024.subsystems.drive.gyro.Pigeon2Gyro;
import org.frogforce503.robot2024.subsystems.drive.gyro.PigeonGyro;
import org.frogforce503.robot2024.subsystems.drive.modules.AndymarkSwerveModule;
import org.frogforce503.robot2024.subsystems.drive.modules.BaseSwerveModule;
import org.frogforce503.robot2024.subsystems.drive.modules.FalconSwerveModule;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import org.frogforce503.robot2024.subsystems.drive.modules.BaseSwerveModule.ModuleLocation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive implements BaseDrivetrain {

    private BaseSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private BaseSwerveModule[] modules;
    private BaseGyro gyro;

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private Consumer<SwerveDriveState> telemetryMethod;

    public SwerveDrive() {
        switch (Robot.bot.moduleType) {
            case SDS_L1:
            case SDS_L2:
            case SDS_L3:
                this.frontLeftModule = new FalconSwerveModule(Robot.bot.frontLeftName, ModuleLocation.FrontLeft);
                this.frontRightModule = new FalconSwerveModule(Robot.bot.frontRightName, ModuleLocation.FrontRight);
                this.backLeftModule = new FalconSwerveModule(Robot.bot.backLeftName, ModuleLocation.BackLeft);
                this.backRightModule = new FalconSwerveModule(Robot.bot.backRightName, ModuleLocation.BackRight);
                break;
            case ANDY:
                this.frontLeftModule = new AndymarkSwerveModule(Robot.bot.frontLeftName, ModuleLocation.FrontLeft);
                this.frontRightModule = new AndymarkSwerveModule(Robot.bot.frontRightName, ModuleLocation.FrontRight);
                this.backLeftModule = new AndymarkSwerveModule(Robot.bot.backLeftName, ModuleLocation.BackLeft);
                this.backRightModule = new AndymarkSwerveModule(Robot.bot.backRightName, ModuleLocation.BackRight);
                break;
        }
        
        this.modules = new BaseSwerveModule[] {
            this.frontLeftModule,
            this.frontRightModule,
            this.backLeftModule,
            this.backRightModule
        };

        switch (Robot.bot.gyroType) {
            case PIGEON_1:
                this.gyro = new PigeonGyro(Robot.bot.gyroID);
                break;
            case PIGEON_2:
                this.gyro = new Pigeon2Gyro(Robot.bot.gyroID);
                break;
        }

        this.gyro.setYaw(new Rotation2d());

        kinematics = Robot.bot.kinematics;


















        
        odometry = new SwerveDrivePoseEstimator(
            kinematics,
            getAngleRotation2d(),
            getSwerveModulePositions(),
            new Pose2d()
        );
    }

    @Override
    public void drive(SwerveCommand request) {
        SwerveModuleState[] moduleStates = request.getDriven(getDefaultParameters());
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Robot.bot.moduleType.MAX_DRIVE_SPEED_METERS_SEC);
    
        for (int i = 0; i < 4; i++) {
            this.modules[i].setSwerveModuleState(moduleStates[i]);
        }
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {frontLeftModule.getSwerveModulePosition(),
            frontRightModule.getSwerveModulePosition(), backLeftModule.getSwerveModulePosition(),
            backRightModule.getSwerveModulePosition()} ;
    }

    private SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {frontLeftModule.getSwerveModuleState(),
            frontRightModule.getSwerveModuleState(), backLeftModule.getSwerveModuleState(),
            backRightModule.getSwerveModuleState()} ;
    }

    @Override
    public Pose2d getPoseMeters() {
        return this.odometry.getEstimatedPosition();
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return this.gyro.getYaw();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.odometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.gyro.setYaw(angle);
    }

    @Override
    public void stop() {
        this.drive(new SwerveCommand.SwerveDriveBrake());
    }

    @Override
    public void periodic() {
        this.odometry.update(getAngleRotation2d(), getSwerveModulePositions());
    }

    @Override
    public SwerveDriveState getCurrentState() {
        SwerveDriveState currentState = new SwerveDriveState();
        currentState.SuccessfulDaqs = 0;
        currentState.FailedDaqs = 0;
        currentState.Pose = getPoseMeters();
        currentState.ModuleStates = getSwerveModuleStates();
        currentState.OdometryPeriod = 0.02;
        return currentState;
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    @Override
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {

    }
}
